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

#ifndef PER_VERTEX_BASIS_H
#define PER_VERTEX_BASIS_H

#include "modcam_mesh_export.h"

#include <Eigen/Core>

#include <tuple>

namespace modcam::mesh {

/**
 * Compute an orthonormal set of basis vectors where the z-axis is the vertex
 * normal.
 */

MODCAM_MESH_EXPORT
	std::tuple<Eigen::MatrixX3d, Eigen::MatrixX3d, Eigen::MatrixX3d>
	per_vertex_basis(const Eigen::MatrixX3d &vertex_normals);

} // namespace modcam::mesh

#endif
