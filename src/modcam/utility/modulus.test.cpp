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

#include "modcam/utility/modulus.h"

#include <doctest/doctest.h>

namespace modcam {
TEST_CASE("Test modulus function")
{
	for (auto i = -6; i < 4; i += 3) {
		CHECK(utility::mod(i, 3) == 0);
		CHECK(utility::mod(i + 1, 3) == 1);
		CHECK(utility::mod(i + 2, 3) == 2);
	}
}
} // namespace modcam
