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

#include "modcam/mesh/voronoi_area.h"

#include "modcam/utility/modulus.h"

#include <igl/cotmatrix_entries.h>
#include <igl/doublearea.h>
#include <igl/edge_lengths.h>
#include <igl/internal_angles.h>

#include <numbers>
#include <stdexcept>

namespace modcam::mesh {
Eigen::MatrixXd voronoi_area(const Eigen::MatrixXd &vertices,
                             const Eigen::MatrixXi &faces) {

	if (faces.size() == 0) {
		return Eigen::MatrixXd(0, 0);
	}

	Eigen::Index vertices_per_face = faces.cols();
	if (vertices_per_face != 3) {
		throw std::invalid_argument(
			"There should be three vertices per face, i.e. the faces array "
			"should have three columns.");
	}

	Eigen::Index num_faces = faces.rows();
	if (vertices.size() == 0) {
		return Eigen::MatrixXd::Zero(num_faces, 3);
	}

	Eigen::ArrayXd area;
	igl::doublearea(vertices, faces, area);
	area /= 2.0;

	Eigen::ArrayXXd angles;
	igl::internal_angles(vertices, faces, angles);

	Eigen::MatrixXd half_cot;
	igl::cotmatrix_entries(vertices, faces, half_cot);

	Eigen::MatrixXd edge_squared;
	igl::edge_lengths(vertices, faces, edge_squared);
	edge_squared = edge_squared.cwiseProduct(edge_squared);

	double right_angle = std::numbers::pi / 2.0;
	Eigen::Array<bool, Eigen::Dynamic, 1> nonobtuse =
		(angles <= right_angle).rowwise().all();

	Eigen::MatrixXd v_area(num_faces, 3);

	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			if (nonobtuse(row)) {
				Eigen::Index i = utility::mod(col - 1, vertices_per_face);
				Eigen::Index j = utility::mod(col + 1, vertices_per_face);
				v_area(row, col) =
					0.25 * (edge_squared(row, i) * half_cot(row, i) +
				            edge_squared(row, j) * half_cot(row, j));
			} else {
				if (angles(row, col) > right_angle) {
					v_area(row, col) = area(row) / 2.0;
				} else {
					v_area(row, col) = area(row) / 4.0;
				}
			}
		}
	}

	return v_area;
}
} // namespace modcam::mesh
