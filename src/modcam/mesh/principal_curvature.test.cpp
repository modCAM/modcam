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

#include "modcam/mesh/principal_curvature.h"

#include <Eigen/Core>
#include <cmrc/cmrc.hpp>
#include <doctest/doctest.h>
#include <igl/readSTL.h>
#include <igl/remove_duplicate_vertices.h>

#include <cmath>
#include <numbers>
#include <strstream>

using namespace std::string_literals;

CMRC_DECLARE(modcam);

namespace modcam {
TEST_CASE("Test principal curvature function") {
	SUBCASE("Sphere") {
		auto fs = cmrc::modcam::get_filesystem();
		auto sphere_path = "data/mesh/sphere.stl"s;
		REQUIRE(fs.is_file(sphere_path));
		auto sphere_rc = fs.open(sphere_path);
		std::istrstream sphere_stl(
			sphere_rc.begin(),
			sphere_rc.size()); // TODO: Change this to spanstream when I switch
		                       // to C++23.
		Eigen::MatrixX3d tmp_vertices;
		Eigen::MatrixX3i tmp_faces;
		Eigen::MatrixX3d tmp_normals;
		igl::readSTL(sphere_stl, tmp_vertices, tmp_faces, tmp_normals);
		Eigen::MatrixX3d vertices;
		Eigen::MatrixX3i faces;
		Eigen::VectorXi tmpi;
		Eigen::VectorXi tmpj;
		igl::remove_duplicate_vertices(tmp_vertices, tmp_faces, 1.0e-7,
		                               vertices, tmpi, tmpj, faces);
		Eigen::RowVector3d centers{
			(vertices.colwise().maxCoeff() - vertices.colwise().minCoeff()) /
			2.0};
		for (auto i = 0; i < 3; i++) {
			vertices.col(i).array() -=
				centers(i); // Center the sphere on the origin
		}
		vertices.rowwise().normalize(); // Set radius to 1.0
		constexpr auto radius = 11.37;
		vertices *= radius; // Make the sphere radius variable
		Eigen::MatrixX3d pd1;
		Eigen::MatrixX3d pd2;
		Eigen::VectorXd pv1;
		Eigen::VectorXd pv2;
		mesh::principal_curvature_rus2004(vertices, faces, pd1, pd2, pv1, pv2);

		Eigen::Index num_vertices{vertices.rows()};
		CHECK(pv1.rows() == num_vertices);
		CHECK(pv1.cols() == 1);
		CHECK(pv2.rows() == num_vertices);
		CHECK(pv2.cols() == 1);
		CHECK(pd1.rows() == num_vertices);
		CHECK(pd1.cols() == 3);
		CHECK(pd2.rows() == num_vertices);
		CHECK(pd2.cols() == 3);

		auto tolerance = 1.0e-6;
		CHECK(((pv1.array() - 1.0 / radius).abs() < tolerance).all());
		CHECK(((pv2.array() - 1.0 / radius).abs() < tolerance).all());
	}
	SUBCASE("Empty face array") {
		const Eigen::MatrixX3d vertices{
			{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
		const Eigen::MatrixX3i faces(0, 3);
		Eigen::MatrixX3d pd1;
		Eigen::MatrixX3d pd2;
		Eigen::VectorXd pv1;
		Eigen::VectorXd pv2;
		mesh::principal_curvature_rus2004(vertices, faces, pd1, pd2, pv1, pv2);
		CHECK(pv1.size() == 0);
		CHECK(pv2.size() == 0);
		CHECK(pd1.size() == 0);
		CHECK(pd2.size() == 0);
	}
	SUBCASE("Empty vertex array") {
		const Eigen::MatrixX3d vertices(0, 3);
		const Eigen::MatrixX3i faces{{0, 1, 2}};
		Eigen::MatrixX3d pd1;
		Eigen::MatrixX3d pd2;
		Eigen::VectorXd pv1;
		Eigen::VectorXd pv2;
		mesh::principal_curvature_rus2004(vertices, faces, pd1, pd2, pv1, pv2);
		CHECK(pv1.size() == 0);
		CHECK(pv2.size() == 0);
		CHECK(pd1.size() == 0);
		CHECK(pd2.size() == 0);
	}
	SUBCASE("Icosahedron") {
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
		theta(Eigen::seq(6, 10)) =
			Array5d::LinSpaced(5, pi / 5.0, 9.0 * pi / 5.0);
		Eigen::MatrixX3d vertices = Eigen::MatrixX3d::Zero(12, 3);
		vertices.col(0) = Eigen::sin(phi) * Eigen::cos(theta);
		vertices.col(1) = Eigen::sin(phi) * Eigen::sin(theta);
		vertices.col(2) = Eigen::cos(phi);
		// vertices.rowwise().normalize(); // Set radius to 1.0
		constexpr auto radius = 1.0;
		vertices *= radius;
		const Eigen::MatrixX3i faces{
			{0, 1, 2},  {0, 2, 3},  {0, 3, 4},  {0, 4, 5},   {0, 5, 1},
			{2, 1, 6},  {3, 2, 7},  {4, 3, 8},  {5, 4, 9},   {1, 5, 10},
			{6, 7, 2},  {7, 8, 3},  {8, 9, 4},  {9, 10, 5},  {10, 6, 1},
			{11, 7, 6}, {11, 8, 7}, {11, 9, 8}, {11, 10, 9}, {11, 6, 10}};
		Eigen::MatrixX3d pd1;
		Eigen::MatrixX3d pd2;
		Eigen::VectorXd pv1;
		Eigen::VectorXd pv2;
		mesh::principal_curvature_rus2004(vertices, faces, pd1, pd2, pv1, pv2);
		Eigen::Index num_vertices{vertices.rows()};
		CHECK(pv1.rows() == num_vertices);
		CHECK(pv1.cols() == 1);
		CHECK(pv2.rows() == num_vertices);
		CHECK(pv2.cols() == 1);
		CHECK(pd1.rows() == num_vertices);
		CHECK(pd1.cols() == 3);
		CHECK(pd2.rows() == num_vertices);
		CHECK(pd2.cols() == 3);
		for (auto inner_vertices : {1, 4, 11}) {
			CHECK(pv1(inner_vertices) == doctest::Approx(1.0 / radius));
			CHECK(pv2(inner_vertices) == doctest::Approx(1.0 / radius));
		}
		for (auto excluded_vertices : {3, 6, 9}) {
			CHECK(pd1.row(excluded_vertices).array().isNaN().all());
		}
	}
	SUBCASE("Cylinder") {
		const double radius = 0.5;
		const double x0 = radius;
		const double x1 = radius * std::cos(1.0 * 2.0 * std::numbers::pi / 6.0);
		const double x2 = radius * std::cos(2.0 * 2.0 * std::numbers::pi / 6.0);
		const double x3 = radius * std::cos(3.0 * 2.0 * std::numbers::pi / 6.0);
		const double x4 = radius * std::cos(4.0 * 2.0 * std::numbers::pi / 6.0);
		const double x5 = radius * std::cos(5.0 * 2.0 * std::numbers::pi / 6.0);
		const double y0 = 0.0;
		const double y1 = radius * std::sin(1.0 * 2.0 * std::numbers::pi / 6.0);
		const double y2 = radius * std::sin(2.0 * 2.0 * std::numbers::pi / 6.0);
		const double y3 = radius * std::sin(3.0 * 2.0 * std::numbers::pi / 6.0);
		const double y4 = radius * std::sin(4.0 * 2.0 * std::numbers::pi / 6.0);
		const double y5 = radius * std::sin(5.0 * 2.0 * std::numbers::pi / 6.0);
		const Eigen::MatrixX3d vertices{
			{x0, y0, 0.0}, {x0, y0, 1.0}, {x1, y1, 0.0}, {x1, y1, 1.0},
			{x2, y2, 0.0}, {x2, y2, 1.0}, {x3, y3, 0.0}, {x3, y3, 1.0},
			{x4, y4, 0.0}, {x4, y4, 1.0}, {x5, y5, 0.0}, {x5, y5, 1.0}};
		const Eigen::MatrixX3i faces{{1, 0, 2},   {2, 3, 1},   {3, 2, 4},
		                             {4, 5, 3},   {5, 4, 6},   {6, 7, 5},
		                             {7, 6, 8},   {8, 9, 7},   {9, 8, 10},
		                             {10, 11, 9}, {11, 10, 0}, {0, 1, 11}};
		Eigen::MatrixX3d pd1;
		Eigen::MatrixX3d pd2;
		Eigen::VectorXd pv1;
		Eigen::VectorXd pv2;
		mesh::principal_curvature_rus2004(vertices, faces, pd1, pd2, pv1, pv2);
		Eigen::Index num_vertices{vertices.rows()};
		CHECK(pv1.rows() == num_vertices);
		CHECK(pv1.cols() == 1);
		CHECK(pv2.rows() == num_vertices);
		CHECK(pv2.cols() == 1);
		CHECK(pd1.rows() == num_vertices);
		CHECK(pd1.cols() == 3);
		CHECK(pd2.rows() == num_vertices);
		CHECK(pd2.cols() == 3);
		for (int i = 0; i < vertices.cols(); i++) {
			CHECK(pv1(i) == doctest::Approx(0.0));
			CHECK(pv2(i) == doctest::Approx(1.0 / radius));
		}
	}
}
} // namespace modcam
