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

#ifndef MODCAM_MESH_PER_VERTEX_NORMALS_MAX1999_H
#define MODCAM_MESH_PER_VERTEX_NORMALS_MAX1999_H

#include "modcam/mesh/concepts.h"
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
 * Vertex normals are computed using a weighted sum of their surrounding face
 * (triangle) normals.
 *
 * @param[out] normals V-by-3 matrix of mesh vertex 3D normals. If the
 * @p vertices array is empty, then an empty array is returned. If the @p faces
 * array is empty, then @p normals values are set to NaN.
 * @param[in] vertices V-by-3 matrix of mesh vertex Cartesian coordinates
 * @param[in] faces F-by-3 matrix of face (triangle) indices. Each row
 * represents a triangle by indexing three vertices (rows) from the \p vertices
 * array.
 */
template <Vertices3D DerivedV, TriangleFaces DerivedF, Vectors3D DerivedN>
void per_vertex_normals_max1999(Eigen::PlainObjectBase<DerivedN> &normals,
                                const Eigen::MatrixBase<DerivedV> &vertices,
                                const Eigen::MatrixBase<DerivedF> &faces)
{
	assert(vertices.cols() == 3 && "vertices must have 3 columns");
	assert(faces.cols() == 3 && "faces must have 3 columns");

	if (vertices.size() == 0) {
		normals.derived().resize(0, 3);
		return;
	}

	using RowMatrixX3 = Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic,
	                                  3, Eigen::RowMajor>;
	using RowVector3 = Eigen::RowVector3<typename DerivedV::Scalar>;

	// Although this is technically a #V-by-3 matrix, igl::edge_lengths requires
	// that it have a dynamic number of rows.
	RowMatrixX3 edge_squared;
	igl::edge_lengths(vertices, faces, edge_squared);
	edge_squared = edge_squared.cwiseProduct(edge_squared);

	auto num_vertices = vertices.rows();
	auto vertex_dim = vertices.cols();
	normals.setZero(num_vertices, vertex_dim);
	auto num_faces = faces.rows();
	auto vertices_per_face = faces.cols();
	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			auto i = utility::mod(col - 1, vertices_per_face);
			auto j = utility::mod(col + 1, vertices_per_face);
			const RowVector3 edge_i = vertices.row(faces(row, j)).array() -
			                          vertices.row(faces(row, col)).array();
			const RowVector3 edge_j = vertices.row(faces(row, i)).array() -
			                          vertices.row(faces(row, col)).array();
			normals.row(faces(row, col)) +=
				(edge_i.cross(edge_j) /
			     (edge_squared(row, i) * edge_squared(row, j)))
					.template cast<typename DerivedN::Scalar>();
		}
	}
	for (auto row : normals.rowwise()) {
		if (row.isZero()) {
			row = Eigen::RowVector3<typename DerivedN::Scalar>::Constant(
				std::numeric_limits<typename DerivedN::Scalar>::quiet_NaN());
		}
	}
	normals.rowwise().normalize();
}

} // namespace modcam::mesh

#endif
