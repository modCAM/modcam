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

#include "modcam/mesh/per_vertex_normals.h"

#include <Eigen/Core>
#include <doctest/doctest.h>

#include <cmath>

namespace modcam {
TEST_CASE("Test per-vertex normals function") {
	SUBCASE("Partial icosahedron") {
		constexpr auto phi = (1.0 + std::sqrt(5.0)) / 2.0;
		Eigen::MatrixX3d vertices{
			{phi, 1.0, 0.0},   {phi, -1.0, 0.0},  {-phi, -1.0, 0.0},
			{-phi, 1.0, 0.0},  {1.0, 0.0, phi},   {-1.0, 0.0, phi},
			{-1.0, 0.0, -phi}, {1.0, 0.0, -phi},  {0.0, phi, 1.0},
			{0.0, phi, -1.0},  {0.0, -phi, -1.0}, {0.0, -phi, 1.0}};
		vertices.rowwise()
			.normalize(); // This will make it easier to compare calculated
		                  // vertex normals to the "true" normals.
		Eigen::MatrixX3i faces{{5, 4, 8},  {4, 5, 11},  {8, 4, 0},   {4, 1, 0},
		                       {4, 11, 1}, {11, 10, 1}, {11, 2, 10}, {5, 2, 11},
		                       {1, 10, 7}, {0, 1, 7}};
		Eigen::MatrixX3d vertex_normals;
		mesh::per_vertex_normals(vertices, faces, vertex_normals);
		CHECK(vertex_normals.rows() == vertices.rows());
		CHECK(vertex_normals.cols() == vertices.cols());
		CHECK((vertex_normals.row(4).array() == vertices.row(4).array()).all());
		CHECK(
			(vertex_normals.row(11).array() == vertices.row(11).array()).all());
		CHECK((vertex_normals.row(1).array() == vertices.row(1).array()).all());
		CHECK(vertex_normals.row(3).array().isNaN().all());
		CHECK(vertex_normals.row(6).array().isNaN().all());
		CHECK(vertex_normals.row(9).array().isNaN().all());
	}
	SUBCASE("Empty face array") {
		constexpr auto phi = (1.0 + std::sqrt(5.0)) / 2.0;
		Eigen::MatrixX3d vertices{
			{phi, 1.0, 0.0},   {phi, -1.0, 0.0},  {-phi, -1.0, 0.0},
			{-phi, 1.0, 0.0},  {1.0, 0.0, phi},   {-1.0, 0.0, phi},
			{-1.0, 0.0, -phi}, {1.0, 0.0, -phi},  {0.0, phi, 1.0},
			{0.0, phi, -1.0},  {0.0, -phi, -1.0}, {0.0, -phi, 1.0}};
		Eigen::MatrixX3i faces(0, 3);
		Eigen::MatrixX3d vertex_normals;
		mesh::per_vertex_normals(vertices, faces, vertex_normals);
		CHECK(vertex_normals.size() == 0);
	}
	SUBCASE("Empty vertex array") {
		const Eigen::MatrixX3d vertices(0, 3);
		Eigen::MatrixX3i faces{{0, 1, 2}, {0, 2, 3}, {0, 3, 4},
		                       {0, 4, 5}, {0, 5, 6}, {0, 6, 1}};
		Eigen::MatrixX3d vertex_normals;
		mesh::per_vertex_normals(vertices, faces, vertex_normals);
		CHECK(vertex_normals.size() == 0);
	}
}
} // namespace modcam
