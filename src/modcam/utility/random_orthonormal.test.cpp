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

#include "modcam/utility/random_orthonormal.h"

#include <Eigen/Core>
#include <doctest/doctest.h>

#include <cmath>

namespace modcam {
using RowMatrixX3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
TEST_CASE("Test random orthonormal")
{
	SUBCASE("Dynamic size, row-major matrix")
	{
		const RowMatrixX3f vectors{{1.0F, 0.0F, 0.0F},
		                           {0.0F, 1.0F, 0.0F},
		                           {0.0F, 0.0F, 1.0F},
		                           {2.0F, 1.0F, -1.0F}};
		RowMatrixX3f ortho_vectors;
		utility::random_orthonormal(ortho_vectors, vectors);
		const Eigen::VectorXf dot_prod =
			(vectors.cwiseProduct(ortho_vectors)).rowwise().sum();
		for (auto v : dot_prod) {
			CHECK(v == doctest::Approx(0.0F));
		}
	}
	SUBCASE("Fixed size, column-major matrix")
	{
		const Eigen::Matrix<double, 4, 3> vectors{{1.0F, 0.0F, 0.0F},
		                                          {0.0F, 1.0F, 0.0F},
		                                          {0.0F, 0.0F, 1.0F},
		                                          {2.0F, 1.0F, -1.0F}};
		Eigen::Matrix<double, 4, 3> ortho_vectors;
		utility::random_orthonormal(ortho_vectors, vectors);
		const Eigen::Vector<double, 4> dot_prod =
			(vectors.cwiseProduct(ortho_vectors)).rowwise().sum();
		for (auto v : dot_prod) {
			CHECK(v == doctest::Approx(0.0F));
		}
	}
}
} // namespace modcam
