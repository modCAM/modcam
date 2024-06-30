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

#include <Eigen/Geometry>

#include <cmath>

namespace modcam::utility {
Eigen::RowVector3d random_orthonormal(const Eigen::RowVector3d &vector) {
	Eigen::RowVector3d perturb_vec = vector.normalized();
	double perturbation = 1.0;
	bool x_aligned = (1.0 - std::abs(perturb_vec(0))) < 1.0e-2;
	if (x_aligned) {
		perturb_vec(1) += perturbation;
	} else {
		perturb_vec(0) += perturbation;
	}
	Eigen::RowVector3d random_ortho_vector = vector.cross(perturb_vec);
	random_ortho_vector.rowwise().normalize();
	return random_ortho_vector;
}
} // namespace modcam::utility
