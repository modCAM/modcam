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

#ifndef VORONOI_AREA_H
#define VORONOI_AREA_H

#include "modcam_mesh_export.h"

#include <Eigen/Core>

namespace modcam::mesh {

/**
 * Compute the Voronoi cell areas for the triangles in a mesh as described in
 * @cite Meyer2003.
 */
MODCAM_MESH_EXPORT Eigen::MatrixXd voronoi_area(const Eigen::MatrixXd &vertices,
                                                const Eigen::MatrixXi &faces);

} // namespace modcam::mesh

#endif
