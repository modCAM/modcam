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

#include <Eigen/Geometry>
#include <doctest/doctest.h>

#include <cmath>
#include <tuple>

namespace modcam {
TEST_CASE("Test per-vertex basis function") {
	SUBCASE("Unit normal along the x-axis") {
		// Expect z, -y, x
		const Eigen::MatrixX3d normal_vectors{{1.0, 0.0, 0.0}};
		std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d> basis =
			mesh::per_vertex_basis(normal_vectors);
		auto &b0 = std::get<0>(basis);
		auto &b1 = std::get<1>(basis);
		auto &b2 = std::get<2>(basis);
		CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
		          .all());
		CHECK((b0.row(0).cross(b1.row(0)).array() == b2.row(0).array()).all());
		CHECK((b1.row(0).cross(b2.row(0)).array() == b0.row(0).array()).all());
	}
	SUBCASE("Unit normal along the y-axis") {
		// Expect -z, -x, y
		const Eigen::MatrixX3d normal_vectors{{0.0, 1.0, 0.0}};
		std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d> basis =
			mesh::per_vertex_basis(normal_vectors);
		auto &b0 = std::get<0>(basis);
		auto &b1 = std::get<1>(basis);
		auto &b2 = std::get<2>(basis);
		CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
		          .all());
		CHECK((b0.row(0).cross(b1.row(0)).array() == b2.row(0).array()).all());
		CHECK((b1.row(0).cross(b2.row(0)).array() == b0.row(0).array()).all());
	}
	SUBCASE("Unit normal along the z-axis") {
		// Expect y, -x, z
		const Eigen::MatrixX3d normal_vectors{{0.0, 0.0, 1.0}};
		std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d> basis =
			mesh::per_vertex_basis(normal_vectors);
		auto &b0 = std::get<0>(basis);
		auto &b1 = std::get<1>(basis);
		auto &b2 = std::get<2>(basis);
		CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
		          .all());
		CHECK((b0.row(0).cross(b1.row(0)).array() == b2.row(0).array()).all());
		CHECK((b1.row(0).cross(b2.row(0)).array() == b0.row(0).array()).all());
	}
	SUBCASE("Non-unit normal vector") {
		const Eigen::MatrixX3d normal_vectors{{1.0, 1.0, 1.0}};
		std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d> basis =
			mesh::per_vertex_basis(normal_vectors);
		auto &b0 = std::get<0>(basis);
		auto &b1 = std::get<1>(basis);
		auto &b2 = std::get<2>(basis);
		CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
		          .all());
		Eigen::MatrixX3d b0_b1 = b0.row(0).cross(b1.row(0));
		CHECK(b0_b1(0, 0) == doctest::Approx(b2(0, 0)));
		CHECK(b0_b1(0, 1) == doctest::Approx(b2(0, 1)));
		CHECK(b0_b1(0, 2) == doctest::Approx(b2(0, 2)));
		Eigen::MatrixX3d b1_b2 = b1.row(0).cross(b2.row(0));
		CHECK(b1_b2(0, 0) == doctest::Approx(b0(0, 0)));
		CHECK(b1_b2(0, 1) == doctest::Approx(b0(0, 1)));
		CHECK(b1_b2(0, 2) == doctest::Approx(b0(0, 2)));
	}
}
} // namespace modcam
