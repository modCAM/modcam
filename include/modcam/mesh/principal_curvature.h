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

#ifndef PRINCIPAL_CURVATURE_H
#define PRINCIPAL_CURVATURE_H

#include "modcam/mesh/concepts.h"
#include "modcam/mesh/per_vertex_basis.h"
#include "modcam/mesh/per_vertex_normals.h"
#include "modcam/mesh/voronoi_area.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <igl/local_basis.h>
#include <igl/principal_curvature.h>

#include <cassert>
#include <cmath>
#include <limits>

namespace modcam::mesh {

/**
 * Compute the (vertex) principal curvature using the algorithm described in
 * @cite Rusinkiewicz2004.
 *
 * @param[in] vertices V-by-3 matrix of mesh vertex coordinates
 * @param[in] faces F-by-3 matrix of face (triangle) indices
 * @param[out] pd1 V-by-3 maximal curvature direction for each vertex.
 * @param[out] pd2 V-by-3 minimal curvature direction for each vertex.
 * @param[out] pv1 V-by-1 maximal curvature value for each vertex.
 * @param[out] pv2 V-by-1 minimal curvature value for each vertex.
 */
template <Vertices3D DerivedV, TriangleFaces DerivedF, Vectors3D DerivedPD,
          typename DerivedPV>
requires std::floating_point<typename DerivedPV::Scalar> &&
         (DerivedPV::ColsAtCompileTime == 1 ||
          DerivedPV::ColsAtCompileTime == Eigen::Dynamic)
void principal_curvature_rus2004(const Eigen::MatrixBase<DerivedV> &vertices,
                                 const Eigen::MatrixBase<DerivedF> &faces,
                                 Eigen::PlainObjectBase<DerivedPD> &pd1,
                                 Eigen::PlainObjectBase<DerivedPD> &pd2,
                                 Eigen::PlainObjectBase<DerivedPV> &pv1,
                                 Eigen::PlainObjectBase<DerivedPV> &pv2)
{
	assert(vertices.cols() == 3 && "vertices must have 3 columns");
	assert(faces.cols() == 3 && "faces must have 3 columns");

	if (vertices.size() == 0) {
		pd1.derived().resize(0, 3);
		pd2.derived().resize(0, 3);
		pv1.derived().resize(0, 1);
		pv2.derived().resize(0, 1);
		return;
	}

	if (faces.size() == 0) {
		pd1.setConstant(
			std::numeric_limits<typename DerivedPD::Scalar>::quiet_NaN());
		pd2.setConstant(
			std::numeric_limits<typename DerivedPD::Scalar>::quiet_NaN());
		pv1.setConstant(
			std::numeric_limits<typename DerivedPV::Scalar>::quiet_NaN());
		pv2.setConstant(
			std::numeric_limits<typename DerivedPV::Scalar>::quiet_NaN());
		return;
	}

	Eigen::Index vertex_dim = vertices.cols();
	Eigen::Index vertices_per_face = faces.cols();

	using RowMatrixF3 =
		Eigen::Matrix<typename DerivedV::Scalar, DerivedF::RowsAtCompileTime, 3,
	                  Eigen::RowMajor>;

	RowMatrixF3 edge0{vertices(faces.col(2).array(), Eigen::all) -
	                  vertices(faces.col(1).array(), Eigen::all)};
	RowMatrixF3 edge1{vertices(faces.col(0).array(), Eigen::all) -
	                  vertices(faces.col(2).array(), Eigen::all)};
	RowMatrixF3 edge2{vertices(faces.col(1).array(), Eigen::all) -
	                  vertices(faces.col(0).array(), Eigen::all)};

	using MatrixF3 =
		Eigen::Matrix<typename DerivedV::Scalar, DerivedF::RowsAtCompileTime, 3,
	                  Eigen::RowMajor>;

	RowMatrixF3 face_basis0;
	RowMatrixF3 face_basis1;
	RowMatrixF3 face_basis2;
	igl::local_basis(vertices, faces, face_basis0, face_basis1, face_basis2);

	using VectorF =
		Eigen::Vector<typename DerivedV::Scalar, DerivedF::RowsAtCompileTime>;

	VectorF e0_b0{(edge0.cwiseProduct(face_basis0)).rowwise().sum()};
	VectorF e0_b1{(edge0.cwiseProduct(face_basis1)).rowwise().sum()};
	VectorF e1_b0{(edge1.cwiseProduct(face_basis0)).rowwise().sum()};
	VectorF e1_b1{(edge1.cwiseProduct(face_basis1)).rowwise().sum()};
	VectorF e2_b0{(edge2.cwiseProduct(face_basis0)).rowwise().sum()};
	VectorF e2_b1{(edge2.cwiseProduct(face_basis1)).rowwise().sum()};

	using RowMatrixV3 =
		Eigen::Matrix<typename DerivedV::Scalar, DerivedV::RowsAtCompileTime, 3,
	                  Eigen::RowMajor>;

	RowMatrixV3 vertex_normals;
	mesh::per_vertex_normals(vertices, faces, vertex_normals);

	RowMatrixF3 normal0{vertex_normals(faces.col(0).array(), Eigen::all)};
	RowMatrixF3 normal1{vertex_normals(faces.col(1).array(), Eigen::all)};
	RowMatrixF3 normal2{vertex_normals(faces.col(2).array(), Eigen::all)};
	// [a, b, c] where the second fundamental form is [[a, b],
	//                                                 [b, c]]
	auto num_faces = faces.rows();
	RowMatrixF3 second_fundamental_fb;
	second_fundamental_fb.resize(num_faces, 3);
	for (Eigen::Index i = 0; i < num_faces; i++) {
		Eigen::Matrix<typename DerivedV::Scalar, 6, 3> A{
			{e0_b0(i), e0_b1(i), 0.0}, {0.0, e0_b0(i), e0_b1(i)},
			{e1_b0(i), e1_b1(i), 0.0}, {0.0, e1_b0(i), e1_b1(i)},
			{e2_b0(i), e2_b1(i), 0.0}, {0.0, e2_b0(i), e2_b1(i)}};
		Eigen::Vector<typename DerivedV::Scalar, 6> b{
			{(normal2.row(i) - normal1.row(i)).dot(face_basis0.row(i)),
		     (normal2.row(i) - normal1.row(i)).dot(face_basis1.row(i)),
		     (normal0.row(i) - normal2.row(i)).dot(face_basis0.row(i)),
		     (normal0.row(i) - normal2.row(i)).dot(face_basis1.row(i)),
		     (normal1.row(i) - normal0.row(i)).dot(face_basis0.row(i)),
		     (normal1.row(i) - normal0.row(i)).dot(face_basis1.row(i))}};
		Eigen::CompleteOrthogonalDecomposition<
			typename Eigen::Matrix<typename DerivedV::Scalar, 6, 3>>
			cod(A);
		second_fundamental_fb.row(i) = cod.solve(b).transpose();
	}

	RowMatrixV3 vertex_basis0;
	RowMatrixV3 vertex_basis1;
	RowMatrixV3 vertex_basis2;
	mesh::per_vertex_basis(vertex_normals, vertex_basis0, vertex_basis1,
	                       vertex_basis2);

	// Compute the second fundamental form in the vertex basis frame.
	RowMatrixF3 weights;
	mesh::voronoi_area(vertices, faces, weights);
	auto num_vertices = vertices.rows();
	Eigen::Array<typename DerivedV::Scalar, DerivedV::RowsAtCompileTime, 1>
		sum_weights{
			Eigen::Array<typename DerivedV::Scalar, DerivedV::RowsAtCompileTime,
	                     1>::Zero(num_vertices)};
	Eigen::Matrix<typename DerivedV::Scalar, DerivedV::RowsAtCompileTime, 3,
	              Eigen::RowMajor>
		second_fundamental_vb{
			Eigen::Matrix<typename DerivedV::Scalar,
	                      DerivedV::RowsAtCompileTime, 3,
	                      Eigen::RowMajor>::Zero(num_vertices, 3)};
	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			auto v_idx = faces(row, col);

			// Rotate the face basis to align its z-axis with the vertex basis
			// z-axis.
			auto cos_angle = face_basis2.row(row).dot(vertex_basis2.row(v_idx));
			auto angle = std::acos(cos_angle);
			Eigen::RowVector3<typename DerivedV::Scalar> fb0{
				face_basis0.row(row)};
			Eigen::RowVector3<typename DerivedV::Scalar> fb1{
				face_basis1.row(row)};
			if (angle > 1.0e-6) {
				auto sin_angle = std::sin(angle);
				Eigen::RowVector3<typename DerivedV::Scalar> rotation_axis{
					(face_basis2.row(row).cross(vertex_basis2.row(v_idx)))
						.normalized()};
				fb0 = fb0 * cos_angle + rotation_axis.cross(fb0) * sin_angle +
				      rotation_axis * rotation_axis.dot(fb0) * (1 - cos_angle);
				fb1 = fb1 * cos_angle + rotation_axis.cross(fb1) * sin_angle +
				      rotation_axis * rotation_axis.dot(fb1) * (1 - cos_angle);
			}

			auto vb0_fb0 = vertex_basis0.row(v_idx).dot(fb0);
			auto vb0_fb1 = vertex_basis0.row(v_idx).dot(fb1);
			auto vb1_fb0 = vertex_basis1.row(v_idx).dot(fb0);
			auto vb1_fb1 = vertex_basis1.row(v_idx).dot(fb1);

			auto a = second_fundamental_fb(row, 0) * vb0_fb0 * vb0_fb0 +
			         second_fundamental_fb(row, 1) * 2.0 * vb0_fb0 * vb0_fb1 +
			         second_fundamental_fb(row, 2) * vb0_fb1 * vb0_fb1;
			auto b = second_fundamental_fb(row, 0) * vb0_fb0 * vb1_fb0 +
			         second_fundamental_fb(row, 1) * vb0_fb0 * vb1_fb1 +
			         second_fundamental_fb(row, 1) * vb0_fb1 * vb1_fb0 +
			         second_fundamental_fb(row, 2) * vb0_fb1 * vb1_fb1;
			auto c = second_fundamental_fb(row, 0) * vb1_fb0 * vb1_fb0 +
			         second_fundamental_fb(row, 1) * vb1_fb0 * vb1_fb1 +
			         second_fundamental_fb(row, 2) * vb1_fb1 * vb1_fb1;
			auto weight = weights(row, col);
			second_fundamental_vb(v_idx, 0) += weight * a;
			second_fundamental_vb(v_idx, 1) += weight * b;
			second_fundamental_vb(v_idx, 2) += weight * c;
			sum_weights(v_idx) += weight;
		}
	}

	// Compute per-vertex principal curvature from the second fundamental form.
	pv1.derived().resize(num_vertices, 1);
	pv2.derived().resize(num_vertices, 1);
	pd1.derived().resize(num_vertices, 3);
	pd2.derived().resize(num_vertices, 3);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2<typename DerivedV::Scalar>> es;
	for (Eigen::Index row = 0; row < num_vertices; row++) {
		auto a = second_fundamental_vb(row, 0) / sum_weights(row);
		auto b = second_fundamental_vb(row, 1) / sum_weights(row);
		auto c = second_fundamental_vb(row, 2) / sum_weights(row);
		es.compute(Eigen::Matrix2<typename DerivedV::Scalar>{{a, b}, {b, c}});
		Eigen::Vector2<typename DerivedV::Scalar> eigval{es.eigenvalues()};
		pv1(row) = eigval(0);
		pv2(row) = eigval(1);
		Eigen::Matrix2<typename DerivedV::Scalar> eigvec{
			es.eigenvectors().transpose()};
		Eigen::Matrix<typename DerivedPD::Scalar, 2, 3> pd =
			(eigvec * (Eigen::Matrix<typename DerivedV::Scalar, 2, 3>()
		                   << vertex_basis0.row(row),
		               vertex_basis1.row(row))
		                  .finished())
				.template cast<typename DerivedPD::Scalar>();
		pd1.row(row) = pd.row(0);
		pd2.row(row) = pd.row(1);
	}
}

} // namespace modcam::mesh

#endif
