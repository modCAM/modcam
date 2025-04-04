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

#ifndef PER_VERTEX_NORMALS_H
#define PER_VERTEX_NORMALS_H

#include "modcam/utility/modulus.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <igl/edge_lengths.h>

#include <limits>

namespace modcam::mesh {

/**
 * Compute the per-vertex normal vectors for a set of vertices and faces, as
 * described in @cite Max1999
 *
 * @param[in] vertices #V-by-3 matrix of mesh vertex coordinates
 * @param[in] faces #F-by-3 matrix of face (triangle) indices
 * @param[out] normals #V-by-3 matrix of mesh vertex 3D normals
 */
template <typename DerivedV, typename DerivedF, typename DerivedN>
void per_vertex_normals(const Eigen::MatrixBase<DerivedV> &vertices,
                        const Eigen::MatrixBase<DerivedF> &faces,
                        Eigen::MatrixBase<DerivedN> &normals) {

	if (faces.size() == 0 || vertices.size() == 0) {
		normals.derived().resize(0, 3);
		return;
	}

	using MatrixT = Eigen::Matrix<typename DerivedV::Scalar,
	                              DerivedV::RowsAtCompileTime, 3>;
	using RowVectorT = Eigen::Matrix<typename DerivedV::Scalar, 1, 3>;

	MatrixT edge_squared;
	igl::edge_lengths(vertices, faces, edge_squared);
	edge_squared = edge_squared.cwiseProduct(edge_squared);

	auto num_vertices = vertices.rows();
	auto vertex_dim = vertices.cols();
	normals.derived().resize(num_vertices, vertex_dim);
	normals = MatrixT::Zero(num_vertices, vertex_dim);
	auto num_faces = faces.rows();
	auto vertices_per_face = faces.cols();
	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			auto i = utility::mod(col - 1, vertices_per_face);
			auto j = utility::mod(col + 1, vertices_per_face);
			RowVectorT edge_i = vertices.row(faces(row, j)).array() -
			                    vertices.row(faces(row, col)).array();
			RowVectorT edge_j = vertices.row(faces(row, i)).array() -
			                    vertices.row(faces(row, col)).array();
			normals.row(faces(row, col)) +=
				edge_i.cross(edge_j) /
				(edge_squared(row, i) * edge_squared(row, j));
		}
	}
	for (auto row : normals.rowwise()) {
		if (row.isZero()) {
			row =
				RowVectorT::Constant(std::numeric_limits<double>::quiet_NaN());
		}
	}
	normals.rowwise().normalize();
}

} // namespace modcam::mesh

#endif
