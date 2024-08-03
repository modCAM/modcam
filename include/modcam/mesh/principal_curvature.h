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

#ifndef CURVATURE_H
#define CURVATURE_H

#include "modcam_mesh_export.h"

#include <Eigen/Core>

#include <tuple>
#include <vector>

namespace modcam::mesh {

using Curvature = std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd,
                             Eigen::MatrixXd>;

/**
 * Compute the (vertex) principal curvature using the algorithm described in
 * @cite Rusinkiewicz2004.
 */
MODCAM_MESH_EXPORT Curvature principal_curvature_rus2004(
	const Eigen::MatrixX3d &vertices, const Eigen::MatrixX3i &faces);

} // namespace modcam::mesh

#endif
