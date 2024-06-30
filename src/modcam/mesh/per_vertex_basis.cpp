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

#include "modcam/mesh/per_vertex_basis.h"

#include "modcam/utility/random_orthonormal.h"

#include <Eigen/Geometry>

namespace modcam::mesh {
std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d>
per_vertex_basis(const Eigen::MatrixX3d &vertex_normals) {
	std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d> basis;
	auto &b0 = std::get<0>(basis);
	auto &b1 = std::get<1>(basis);
	auto &b2 = std::get<2>(basis);

	b2 = vertex_normals.rowwise().normalized();
	Eigen::Index num_vectors = b2.rows();
	Eigen::Index dim = b2.cols();
	b0.resize(num_vectors, dim);
	b1.resize(num_vectors, dim);
	for (Eigen::Index row = 0; row < num_vectors; row++) {
		b0.row(row) = utility::random_orthonormal(b2.row(row));
		b1.row(row) = b2.row(row).cross(b0.row(row));
	}

	return basis;
}
} // namespace modcam::mesh
