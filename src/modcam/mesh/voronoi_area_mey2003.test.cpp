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

#include "modcam/mesh/voronoi_area_mey2003.h"

#include <Eigen/Core>

#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

namespace modcam {

using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

TEST_CASE("Test Voronoi area function")
{
	SUBCASE("Equilateral triangle")
	{
		const RowMatrixX3d vertices{{0.0, 0.0, 0.0},
		                            {1.0, 0.0, 0.0},
		                            {0.5, std::numbers::sqrt3 / 2.0, 0.0}};
		const RowMatrixX3i faces{{0, 1, 2}};
		constexpr double one_third_area = std::numbers::sqrt3 / 12.0;
		SUBCASE("Same numeric types")
		{
			RowMatrixX3d weights;
			mesh::voronoi_area_mey2003(weights, vertices, faces);
			CHECK(weights(0) == doctest::Approx(one_third_area));
			CHECK(weights(1) == doctest::Approx(one_third_area));
			CHECK(weights(2) == doctest::Approx(one_third_area));
		}
		SUBCASE("Different numeric types")
		{
			RowMatrixX3f weights;
			mesh::voronoi_area_mey2003(weights, vertices, faces);
			CHECK(weights(0) == doctest::Approx(one_third_area));
			CHECK(weights(1) == doctest::Approx(one_third_area));
			CHECK(weights(2) == doctest::Approx(one_third_area));
		}
	}
	SUBCASE("Obtuse triangle")
	{
		const RowMatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.1, 0.0}};
		const RowMatrixX3i faces{{0, 1, 2}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0125);
		CHECK(weights(1) == 0.0125);
		CHECK(weights(2) == 0.025);
	}
	SUBCASE("Multiple triangles")
	{
		const RowMatrixX3d vertices{{0.0, 0.0, 0.0},
		                            {1.0, 0.0, 0.0},
		                            {0.5, std::numbers::sqrt3 / 2.0, 0.0},
		                            {0.5, 0.1, 0.0}};
		const RowMatrixX3i faces{{0, 1, 2}, {0, 1, 3}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights.rows() == faces.rows());
		CHECK(weights.cols() == 3);
		constexpr double one_third_area = std::numbers::sqrt3 / 12.0;
		CHECK(weights(0, 0) == doctest::Approx(one_third_area));
		CHECK(weights(0, 1) == doctest::Approx(one_third_area));
		CHECK(weights(0, 2) == doctest::Approx(one_third_area));
		CHECK(weights(1, 0) == 0.0125);
		CHECK(weights(1, 1) == 0.0125);
		CHECK(weights(1, 2) == 0.025);
	}
	SUBCASE("Multiple triangles - Fixed size, column-major matrices")
	{
		const Eigen::Matrix<double, 4, 3> vertices{
			{0.0, 0.0, 0.0},
			{1.0, 0.0, 0.0},
			{0.5, std::numbers::sqrt3 / 2.0, 0.0},
			{0.5, 0.1, 0.0}};
		const Eigen::Matrix<int, 2, 3> faces{{0, 1, 2}, {0, 1, 3}};
		Eigen::Matrix<double, 2, 3> weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights.rows() == faces.rows());
		CHECK(weights.cols() == 3);
		constexpr double one_third_area = std::numbers::sqrt3 / 12.0;
		CHECK(weights(0, 0) == doctest::Approx(one_third_area));
		CHECK(weights(0, 1) == doctest::Approx(one_third_area));
		CHECK(weights(0, 2) == doctest::Approx(one_third_area));
		CHECK(weights(1, 0) == 0.0125);
		CHECK(weights(1, 1) == 0.0125);
		CHECK(weights(1, 2) == 0.025);
	}
	SUBCASE("Colocated vertices")
	{
		const RowMatrixX3d vertices{
			{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
		const RowMatrixX3i faces{{0, 1, 2}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Face singularity")
	{
		const RowMatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.1, 0.0}};
		const RowMatrixX3i faces{{0, 0, 0}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Colinear vertices")
	{
		const RowMatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
		const RowMatrixX3i faces{{0, 1, 2}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("Empty face array")
	{
		const RowMatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
		const Eigen::MatrixXi faces(0, 3);
		Eigen::MatrixXd weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights.size() == 0);
	}
	SUBCASE("Empty vertex array")
	{
		const Eigen::MatrixXd vertices(0, 3);
		const RowMatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixXd weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0);
		CHECK(weights(1) == 0.0);
		CHECK(weights(2) == 0.0);
	}
	SUBCASE("2D vertex array")
	{
		const Eigen::MatrixXd vertices{{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.1}};
		const RowMatrixX3i faces{{0, 1, 2}};
		RowMatrixX3d weights;
		mesh::voronoi_area_mey2003(weights, vertices, faces);
		CHECK(weights(0) == 0.0125);
		CHECK(weights(1) == 0.0125);
		CHECK(weights(2) == 0.025);
	}
}
} // namespace modcam
