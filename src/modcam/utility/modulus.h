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

#ifndef MODULUS_H
#define MODULUS_H

namespace modcam::utility {
// Modulo function for looping backward
// For example, mod(-1, 3) == 2, whereas -1 % 3 == -1
template <typename T> T mod(T k, T n) { return ((k %= n) < 0) ? k + n : k; }
} // namespace modcam::utility

#endif
