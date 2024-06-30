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

#ifndef RANDOM_ORTHONORMAL_H
#define RANDOM_ORTHONORMAL_H

#include <Eigen/Core>

namespace modcam::utility {
// Create a random normalized 3D vector orthogonal to the input vector.
Eigen::RowVector3d random_orthonormal(const Eigen::RowVector3d &vector);
} // namespace modcam::utility

#endif
