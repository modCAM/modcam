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

#include <Eigen/Core>
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace modcam {
TEST_CASE("Test Voronoi area function") {
	SUBCASE("Equilateral triangle") {
		const Eigen::MatrixX3d vertices{{0.0, 0.0, 0.0},
		                                {1.0, 0.0, 0.0},
		                                {0.5, std::numbers::sqrt3 / 2.0, 0.0}};
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		const double one_third_area = std::numbers::sqrt3 / 12.0;
		CHECK(weights(0) == doctest::Approx(one_third_area));
		CHECK(weights(1) == doctest::Approx(one_third_area));
		CHECK(weights(2) == doctest::Approx(one_third_area));
	}
	SUBCASE("Obtuse triangle") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.1, 0.0}};
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0125);
		CHECK(weights(1) == 0.0125);
		CHECK(weights(2) == 0.025);
	}
	SUBCASE("Multiple triangles") {
		const Eigen::MatrixX3d vertices{{0.0, 0.0, 0.0},
		                                {1.0, 0.0, 0.0},
		                                {0.5, std::numbers::sqrt3 / 2.0, 0.0},
		                                {0.5, 0.1, 0.0}};
		const Eigen::MatrixX3i faces{{0, 1, 2}, {0, 1, 3}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights.rows() == faces.rows());
		CHECK(weights.cols() == 3);
		const double one_third_area = std::numbers::sqrt3 / 12.0;
		CHECK(weights(0, 0) == doctest::Approx(one_third_area));
		CHECK(weights(0, 1) == doctest::Approx(one_third_area));
		CHECK(weights(0, 2) == doctest::Approx(one_third_area));
		CHECK(weights(1, 0) == 0.0125);
		CHECK(weights(1, 1) == 0.0125);
		CHECK(weights(1, 2) == 0.025);
	}
	SUBCASE("Colocated vertices") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Face singularity") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.1, 0.0}};
		const Eigen::MatrixX3i faces{{0, 0, 0}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Colinear vertices") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Empty face array") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
		const Eigen::MatrixXi faces(0, 0);
		Eigen::MatrixXd weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights.size() == 0);
	}
	SUBCASE("Empty vertex array") {
		const Eigen::MatrixXd vertices(0, 0);
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixXd weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("2D vertex array") {
		const Eigen::MatrixXd vertices{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.1}};
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d weights;
		mesh::voronoi_area(vertices, faces, weights);
		CHECK(weights(0) == 0.0125);
		CHECK(weights(1) == 0.0125);
		CHECK(weights(2) == 0.025);
	}
	SUBCASE("Improperly sized face array") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.1, 0.0}};
		const Eigen::MatrixXi faces{{0, 1}};
		Eigen::MatrixX3d weights;
		CHECK_THROWS_AS(mesh::voronoi_area(vertices, faces, weights),
		                std::invalid_argument);
	}
}
} // namespace modcam
