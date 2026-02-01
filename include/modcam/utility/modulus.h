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

#ifndef MODCAM_UTILITY_MODULUS_H
#define MODCAM_UTILITY_MODULUS_H

#include <concepts>

namespace modcam::utility {
/**
 * Returns the positive remainder from the division of the first argument by the
 * second argument. This remainder will be in the range [0, N-1] where N is the
 * modulus (the second argument).
 *
 * Unlike the standard library modulus operator, this version of the modulus
 * operator allows us to loop backward through array indices. For example,
 * mod(-1, 3) == 2, whereas in the standard library -1 % 3 == -1.
 *
 * @param a the dividend
 * @param b the divisor
 * @return the positive remainder r, such that r == a - x * b, where x is
 * an integer
 */
template <typename T>
requires std::signed_integral<T> || std::floating_point<T>
auto mod(T a, T b) -> T
{
	return ((a %= b) < 0) ? a + b : a;
}
} // namespace modcam::utility

#endif
