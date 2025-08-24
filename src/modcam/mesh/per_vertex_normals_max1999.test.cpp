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

#include "modcam/mesh/per_vertex_normals_max1999.h"

#include <doctest/doctest.h>

#include <Eigen/Core>

#include <cmath>
#include <numbers>

namespace modcam {

using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

TEST_CASE("Test per-vertex normals function")
{
	constexpr auto pi = std::numbers::pi;
	const auto r = 1.0 / (2.0 * std::sin(pi / 5.0));
	using Array12d = Eigen::Array<double, 12, 1>;
	using Array5d = Eigen::Array<double, 5, 1>;
	Array12d phi = Array12d::Zero();
	phi(Eigen::seq(1, 5)) = Array5d::Constant(std::asin(r));
	phi(Eigen::seq(6, 10)) = Array5d::Constant(-std::asin(r));
	phi(11) = pi;
	Array12d theta = Array12d::Zero();
	theta(Eigen::seq(1, 5)) = Array5d::LinSpaced(5, 0.0, 8.0 * pi / 5.0);
	theta(Eigen::seq(6, 10)) = Array5d::LinSpaced(5, pi / 5.0, 9.0 * pi / 5.0);

	SUBCASE("Icosahedron")
	{
		RowMatrixX3d vertices = RowMatrixX3d::Zero(12, 3);
		vertices.col(0) = Eigen::sin(phi) * Eigen::cos(theta);
		vertices.col(1) = Eigen::sin(phi) * Eigen::sin(theta);
		vertices.col(2) = Eigen::cos(phi);
		// vertices.rowwise().normalize(); // Set radius to 1.0
		constexpr auto radius = 1.0;
		vertices *= radius;
		const RowMatrixX3i faces{
			{0, 1, 2},  {0, 2, 3},  {0, 3, 4},  {0, 4, 5},   {0, 5, 1},
			{2, 1, 6},  {3, 2, 7},  {4, 3, 8},  {5, 4, 9},   {1, 5, 10},
			{6, 7, 2},  {7, 8, 3},  {8, 9, 4},  {9, 10, 5},  {10, 6, 1},
			{11, 7, 6}, {11, 8, 7}, {11, 9, 8}, {11, 10, 9}, {11, 6, 10}};
		RowMatrixX3d vertex_normals;
		mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
		CHECK(vertex_normals.rows() == vertices.rows());
		CHECK(vertex_normals.cols() == vertices.cols());
		for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
			for (Eigen::Index j = 0; j < vertices.cols(); ++j) {
				CHECK(vertex_normals(i, j) == doctest::Approx(vertices(i, j)));
			}
		}
	}
	SUBCASE("Partial icosahedron")
	{
		RowMatrixX3d vertices = RowMatrixX3d::Zero(12, 3);
		vertices.col(0) = Eigen::sin(phi) * Eigen::cos(theta);
		vertices.col(1) = Eigen::sin(phi) * Eigen::sin(theta);
		vertices.col(2) = Eigen::cos(phi);
		vertices.rowwise().normalize(); // Set radius to 1.0
		constexpr auto radius = 1.0;
		vertices *= radius;
		const RowMatrixX3i faces{
			{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 1}};

		SUBCASE("Same numeric types")
		{
			RowMatrixX3d vertex_normals;
			mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
			CHECK(vertex_normals.rows() == vertices.rows());
			CHECK(vertex_normals.cols() == vertices.cols());
			CHECK_FALSE(vertex_normals(Eigen::seq(0, 5), Eigen::all)
			                .array()
			                .isNaN()
			                .all());
			CHECK(vertex_normals(Eigen::seq(6, 11), Eigen::all)
			          .array()
			          .isNaN()
			          .all());
		}
		SUBCASE("Different numeric types")
		{
			RowMatrixX3f vertex_normals;
			mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
			CHECK(vertex_normals.rows() == vertices.rows());
			CHECK(vertex_normals.cols() == vertices.cols());
			CHECK_FALSE(vertex_normals(Eigen::seq(0, 5), Eigen::all)
			                .array()
			                .isNaN()
			                .all());
			CHECK(vertex_normals(Eigen::seq(6, 11), Eigen::all)
			          .array()
			          .isNaN()
			          .all());
		}
	}
	SUBCASE("Partial icosahedron - Fixed size, column-major matrices")
	{
		Eigen::Matrix<double, 12, 3> vertices;
		vertices.setZero();
		vertices.col(0) = Eigen::sin(phi) * Eigen::cos(theta);
		vertices.col(1) = Eigen::sin(phi) * Eigen::sin(theta);
		vertices.col(2) = Eigen::cos(phi);
		vertices.rowwise().normalize(); // Set radius to 1.0
		constexpr auto radius = 1.0;
		vertices *= radius;
		const Eigen::Matrix<int, 5, 3> faces{
			{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 1}};
		Eigen::Matrix<double, 12, 3> vertex_normals;
		vertex_normals.setZero();
		mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
		CHECK(vertex_normals.rows() == vertices.rows());
		CHECK(vertex_normals.cols() == vertices.cols());
		CHECK_FALSE(
			vertex_normals(Eigen::seq(0, 5), Eigen::all).array().isNaN().all());
		CHECK(vertex_normals(Eigen::seq(6, 11), Eigen::all)
		          .array()
		          .isNaN()
		          .all());
	}
	SUBCASE("Empty face array")
	{
		RowMatrixX3d vertices = RowMatrixX3d::Zero(12, 3);
		vertices.col(0) = Eigen::sin(phi) * Eigen::cos(theta);
		vertices.col(1) = Eigen::sin(phi) * Eigen::sin(theta);
		vertices.col(2) = Eigen::cos(phi);
		vertices.rowwise().normalize(); // Set radius to 1.0
		const RowMatrixX3i faces(0, 3);
		RowMatrixX3d vertex_normals;
		mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
		CHECK(vertex_normals.size() == vertices.size());
	}
	SUBCASE("Empty vertex array")
	{
		const RowMatrixX3d vertices(0, 3);
		const RowMatrixX3i faces{
			{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 1}};
		RowMatrixX3d vertex_normals;
		mesh::per_vertex_normals_max1999(vertex_normals, vertices, faces);
		CHECK(vertex_normals.size() == 0);
	}
}
} // namespace modcam
