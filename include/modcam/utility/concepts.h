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

#ifndef MODCAM_UTILITY_CONCEPTS_H
#define MODCAM_UTILITY_CONCEPTS_H

#include <Eigen/Core>

#include <concepts>

namespace modcam::utility {

template <typename V>
concept Vectors3D =
	std::floating_point<typename V::Scalar> &&
	(V::ColsAtCompileTime == 3 || V::ColsAtCompileTime == Eigen::Dynamic);

}

#endif
