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
TEST_CASE("Test random orthonormal") {
	Eigen::MatrixX3f vectors{{1.0f, 0.0f, 0.0f},
	                         {0.0f, 1.0f, 0.0f},
	                         {0.0f, 0.0f, 1.0f},
	                         {2.0f, 1.0f, -1.0f}};
	Eigen::MatrixX3f ortho_vectors;
	utility::random_orthonormal(vectors, ortho_vectors);
	Eigen::VectorXf dot_prod =
		(vectors.cwiseProduct(ortho_vectors)).rowwise().sum();
	for (auto v : dot_prod) {
		CHECK(v == doctest::Approx(0.0f));
	}
}
} // namespace modcam
