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

#include "modcam/mesh/per_vertex_basis_pro2024.h"

#include <doctest/doctest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace modcam {

using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

TEST_CASE("Test per-vertex basis function")
{
	SUBCASE("Unit normals along the coordinate axes")
	{
		// Expect ( z, -y, x)
		//        (-z, -x, y)
		//        ( y, -x, z)
		RowMatrixX3d normal_vectors{
			{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
		SUBCASE("Same numeric type")
		{
			RowMatrixX3d b0;
			RowMatrixX3d b1;
			RowMatrixX3d b2;
			mesh::per_vertex_basis_pro2024(b0, b1, b2, normal_vectors);
			CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
			          .all());
			for (Eigen::Index r = 0; r < b2.rows(); r++) {
				CHECK((b0.row(r).cross(b1.row(r)).array() == b2.row(r).array())
				          .all());
				CHECK((b1.row(r).cross(b2.row(r)).array() == b0.row(r).array())
				          .all());
			}
		}
		SUBCASE("Different numeric types")
		{
			RowMatrixX3f b0;
			RowMatrixX3f b1;
			RowMatrixX3f b2;
			mesh::per_vertex_basis_pro2024(b0, b1, b2, normal_vectors);
			CHECK((b2.array() ==
			       normal_vectors.rowwise().normalized().array().cast<float>())
			          .all());
			for (Eigen::Index r = 0; r < b2.rows(); r++) {
				CHECK((b0.row(r).cross(b1.row(r)).array() == b2.row(r).array())
				          .all());
				CHECK((b1.row(r).cross(b2.row(r)).array() == b0.row(r).array())
				          .all());
			}
		}
	}
	SUBCASE("Non-unit normal vector")
	{
		const Eigen::RowVector3d normal_vector{{1.0, 1.0, 1.0}};
		Eigen::RowVector3d b0;
		Eigen::RowVector3d b1;
		Eigen::RowVector3d b2;
		mesh::per_vertex_basis_pro2024(b0, b1, b2, normal_vector);
		CHECK((b2.array() == normal_vector.normalized().array()).all());
		Eigen::RowVector3d b0_x_b1 = b0.cross(b1);
		for (Eigen::Index i = 0; i < b0_x_b1.size(); i++) {
			CHECK(b0_x_b1(i) == doctest::Approx(b2(i)));
		}
		Eigen::RowVector3d b1_x_b2 = b1.cross(b2);
		for (Eigen::Index i = 0; i < b1_x_b2.size(); i++) {
			CHECK(b1_x_b2(i) == doctest::Approx(b0(i)));
		}
	}
	SUBCASE("Fixed size, column major matrices")
	{
		Eigen::Matrix<double, 3, 3> normal_vectors{
			{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
		Eigen::Matrix<double, 3, 3> b0;
		Eigen::Matrix<double, 3, 3> b1;
		Eigen::Matrix<double, 3, 3> b2;
		mesh::per_vertex_basis_pro2024(b0, b1, b2, normal_vectors);
		CHECK((b2.array() == normal_vectors.rowwise().normalized().array())
		          .all());
		for (Eigen::Index r = 0; r < b2.rows(); r++) {
			CHECK((b0.row(r).cross(b1.row(r)).array() == b2.row(r).array())
			          .all());
			CHECK((b1.row(r).cross(b2.row(r)).array() == b0.row(r).array())
			          .all());
		}
	}
}
} // namespace modcam
