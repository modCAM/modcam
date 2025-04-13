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

#ifndef PER_VERTEX_BASIS_H
#define PER_VERTEX_BASIS_H

#include "modcam/utility/random_orthonormal.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace modcam::mesh {

/**
 * Compute an orthonormal set of basis vectors for each vertex in a mesh.
 *
 * Each vertex in the mesh is assigned a local coordinate system, where the
 * z-axis is aligned with the vertex normal.
 *
 * @param[in] vertex_normals #V-by-3 matrix of mesh vertex 3D normals
 * @param[out] b0 #V-by-3 matrix of basis vectors, corresponding to the
 * vertex-local x-axis
 * @param[out] b1 #V-by-3 matrix of basis vectors, corresponding to the
 * vertex-local y-axis
 * @param[out] b2 #V-by-3 matrix of basis vectors, corresponding to the
 * vertex-local z-axis (aligned with the vertex normal)
 */
template <typename DerivedN, typename DerivedB>
void per_vertex_basis(const Eigen::MatrixBase<DerivedN> &vertex_normals,
                      Eigen::PlainObjectBase<DerivedB> &b0,
                      Eigen::PlainObjectBase<DerivedB> &b1,
                      Eigen::PlainObjectBase<DerivedB> &b2) {
	auto num_vectors = vertex_normals.rows();
	b2.derived().resize(num_vectors, 3);
	b2 = vertex_normals.rowwise()
	         .normalized()
	         .template cast<typename DerivedB::Scalar>();
	utility::random_orthonormal(b2, b0);
	b1.derived().resize(num_vectors, 3);
	for (Eigen::Index row = 0; row < num_vectors; row++) {
		b1.row(row) = b2.row(row).cross(b0.row(row));
	}
}

} // namespace modcam::mesh

#endif
