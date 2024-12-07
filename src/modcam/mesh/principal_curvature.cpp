/*
 * This file is part of modCAM, open source software for Computer Aided
 * Manufacturing research.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
 * SPDX-License-Identifier: MPL-2.0
 */

#include "modcam/mesh/principal_curvature.h"

#include "modcam/mesh/per_vertex_basis.h"
#include "modcam/mesh/per_vertex_normals.h"
#include "modcam/mesh/voronoi_area.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <igl/local_basis.h>
#include <igl/principal_curvature.h>

#include <cmath>

namespace modcam::mesh {
Curvature principal_curvature_rus2004(const Eigen::MatrixX3d &vertices,
                                      const Eigen::MatrixX3i &faces) {

	Curvature principal_curvature;
	auto &pv1 = std::get<0>(principal_curvature);
	auto &pv2 = std::get<1>(principal_curvature);
	auto &pd1 = std::get<2>(principal_curvature);
	auto &pd2 = std::get<3>(principal_curvature);

	if (faces.size() == 0 || vertices.size() == 0) {
		pv1.resize(0, 1);
		pv2.resize(0, 1);
		pd1.resize(0, 2);
		pd2.resize(0, 2);
		return principal_curvature;
	}

	Eigen::Index vertex_dim = vertices.cols();
	if (vertex_dim != 3) {
		throw std::invalid_argument(
			"Vertices must be three-dimensional, i.e. the vertices array "
			"should have three columns.");
	}

	Eigen::Index vertices_per_face = faces.cols();
	if (vertices_per_face != 3) {
		throw std::invalid_argument(
			"There should be three vertices per face, i.e. the faces array "
			"should have three columns.");
	}

	Eigen::MatrixX3d edge0{vertices(faces.col(2).array(), Eigen::all) -
	                       vertices(faces.col(1).array(), Eigen::all)};
	Eigen::MatrixX3d edge1{vertices(faces.col(0).array(), Eigen::all) -
	                       vertices(faces.col(2).array(), Eigen::all)};
	Eigen::MatrixX3d edge2{vertices(faces.col(1).array(), Eigen::all) -
	                       vertices(faces.col(0).array(), Eigen::all)};

	Eigen::MatrixX3d face_basis0;
	Eigen::MatrixX3d face_basis1;
	Eigen::MatrixX3d face_basis2;
	igl::local_basis(vertices, faces, face_basis0, face_basis1, face_basis2);

	Eigen::VectorXd e0_b0{(edge0.cwiseProduct(face_basis0)).rowwise().sum()};
	Eigen::VectorXd e0_b1{(edge0.cwiseProduct(face_basis1)).rowwise().sum()};
	Eigen::VectorXd e1_b0{(edge1.cwiseProduct(face_basis0)).rowwise().sum()};
	Eigen::VectorXd e1_b1{(edge1.cwiseProduct(face_basis1)).rowwise().sum()};
	Eigen::VectorXd e2_b0{(edge2.cwiseProduct(face_basis0)).rowwise().sum()};
	Eigen::VectorXd e2_b1{(edge2.cwiseProduct(face_basis1)).rowwise().sum()};

	Eigen::MatrixX3d vertex_normals{mesh::per_vertex_normals(vertices, faces)};
	Eigen::MatrixX3d normal0{vertex_normals(faces.col(0).array(), Eigen::all)};
	Eigen::MatrixX3d normal1{vertex_normals(faces.col(1).array(), Eigen::all)};
	Eigen::MatrixX3d normal2{vertex_normals(faces.col(2).array(), Eigen::all)};
	// [a, b, c] where the second fundamental form is [[a, b],
	//                                                 [b, c]]
	Eigen::Index num_faces{faces.rows()};
	Eigen::MatrixXd second_fundamental_fb(num_faces, vertices_per_face);
	for (Eigen::Index i = 0; i < num_faces; i++) {
		Eigen::MatrixXd A{Eigen::MatrixXd{{e0_b0(i), e0_b1(i), 0.0},
		                                  {0.0, e0_b0(i), e0_b1(i)},
		                                  {e1_b0(i), e1_b1(i), 0.0},
		                                  {0.0, e1_b0(i), e1_b1(i)},
		                                  {e2_b0(i), e2_b1(i), 0.0},
		                                  {0.0, e2_b0(i), e2_b1(i)}}};
		Eigen::VectorXd b{Eigen::VectorXd{
			{(normal2.row(i) - normal1.row(i)).dot(face_basis0.row(i)),
		     (normal2.row(i) - normal1.row(i)).dot(face_basis1.row(i)),
		     (normal0.row(i) - normal2.row(i)).dot(face_basis0.row(i)),
		     (normal0.row(i) - normal2.row(i)).dot(face_basis1.row(i)),
		     (normal1.row(i) - normal0.row(i)).dot(face_basis0.row(i)),
		     (normal1.row(i) - normal0.row(i)).dot(face_basis1.row(i))}}};
		second_fundamental_fb.row(i) =
			(A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b))
				.transpose();
	}

	std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d>
		vertex_basis = mesh::per_vertex_basis(vertex_normals);
	auto &vertex_basis0 = std::get<0>(vertex_basis);
	auto &vertex_basis1 = std::get<1>(vertex_basis);
	auto &vertex_basis2 = std::get<2>(vertex_basis);

	// Compute the second fundamental form in the vertex basis frame.
	Eigen::MatrixX3d weights{voronoi_area(vertices, faces)};
	Eigen::Index num_vertices{vertices.rows()};
	Eigen::ArrayXd sum_weights{Eigen::ArrayXd::Zero(num_vertices)};
	Eigen::MatrixXd second_fundamental_vb{
		Eigen::MatrixXd::Zero(num_vertices, vertex_dim)};
	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			int v_idx{faces(row, col)};

			// Rotate the face basis to align its z-axis with the vertex basis
			// z-axis.
			double cos_angle{
				face_basis2.row(row).dot(vertex_basis2.row(v_idx))};
			double angle{std::acos(cos_angle)};
			Eigen::RowVector3d fb0{face_basis0.row(row)};
			Eigen::RowVector3d fb1{face_basis1.row(row)};
			if (angle > 1.0e-6) {
				double sin_angle{std::sin(angle)};
				Eigen::RowVector3d rotation_axis{
					(face_basis2.row(row).cross(vertex_basis2.row(v_idx)))
						.normalized()};
				fb0 = fb0 * cos_angle + rotation_axis.cross(fb0) * sin_angle +
				      rotation_axis * rotation_axis.dot(fb0) * (1 - cos_angle);
				fb1 = fb1 * cos_angle + rotation_axis.cross(fb1) * sin_angle +
				      rotation_axis * rotation_axis.dot(fb1) * (1 - cos_angle);
			}

			double vb0_fb0{vertex_basis0.row(v_idx).dot(fb0)};
			double vb0_fb1{vertex_basis0.row(v_idx).dot(fb1)};
			double vb1_fb0{vertex_basis1.row(v_idx).dot(fb0)};
			double vb1_fb1{vertex_basis1.row(v_idx).dot(fb1)};

			double a{second_fundamental_fb(row, 0) * vb0_fb0 * vb0_fb0 +
			         second_fundamental_fb(row, 1) * 2.0 * vb0_fb0 * vb0_fb1 +
			         second_fundamental_fb(row, 2) * vb0_fb1 * vb0_fb1};
			double b{second_fundamental_fb(row, 0) * vb0_fb0 * vb1_fb0 +
			         second_fundamental_fb(row, 1) * vb0_fb0 * vb1_fb1 +
			         second_fundamental_fb(row, 1) * vb0_fb1 * vb1_fb0 +
			         second_fundamental_fb(row, 2) * vb0_fb1 * vb1_fb1};
			double c{second_fundamental_fb(row, 0) * vb1_fb0 * vb1_fb0 +
			         second_fundamental_fb(row, 1) * vb1_fb0 * vb1_fb1 +
			         second_fundamental_fb(row, 2) * vb1_fb1 * vb1_fb1};
			double weight{weights(row, col)};
			second_fundamental_vb(v_idx, 0) += weight * a;
			second_fundamental_vb(v_idx, 1) += weight * b;
			second_fundamental_vb(v_idx, 2) += weight * c;
			sum_weights(v_idx) += weight;
		}
	}

	// Compute per-vertex principal curvature from the second fundamental form.
	pv1.resize(num_vertices, 1);
	pv2.resize(num_vertices, 1);
	pd1.resize(num_vertices, 2);
	pd2.resize(num_vertices, 2);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es;
	for (Eigen::Index row = 0; row < num_vertices; row++) {
		double a{second_fundamental_vb(row, 0) / sum_weights(row)};
		double b{second_fundamental_vb(row, 1) / sum_weights(row)};
		double c{second_fundamental_vb(row, 2) / sum_weights(row)};
		es.compute(Eigen::Matrix2d{{a, b}, {b, c}});
		Eigen::Vector2d eigval{es.eigenvalues()};
		pv1(row) = eigval(0);
		pv2(row) = eigval(1);
		Eigen::Matrix2d eigvec{es.eigenvectors().transpose()};
		pd1.row(row) = eigvec.row(0);
		pd1.row(row) = eigvec.row(1);
	}

	return principal_curvature;
}
} // namespace modcam::mesh
