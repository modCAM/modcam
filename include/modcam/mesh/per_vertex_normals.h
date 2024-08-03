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

#ifndef PER_VERTEX_NORMALS_H
#define PER_VERTEX_NORMALS_H

#include "modcam_mesh_export.h"

#include <Eigen/Core>

namespace modcam::mesh {

/**
 * Compute the per-vertex normal vectors for a set of vertices and faces, as
 * described in @cite Max1999
 */

MODCAM_MESH_EXPORT Eigen::MatrixX3d
per_vertex_normals(const Eigen::MatrixX3d &vertices,
                   const Eigen::MatrixX3i &faces);

} // namespace modcam::mesh

#endif
