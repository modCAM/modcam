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

#ifndef MODCAM_MESH_CONCEPTS_H
#define MODCAM_MESH_CONCEPTS_H

#include <Eigen/Core>

#include <concepts>

namespace modcam::mesh {

/**
 * An array in which each row represents a triangle by indexing into an array of
 * vertices. For example, [3, 1, 7] represents the triangle formed by the
 * vertices in (zero-indexed) rows 3, 1, and 7 of a vertex coordinates array.
 */
template <typename F>
concept TriangleFaces =
	std::integral<typename F::Scalar> &&
	(F::ColsAtCompileTime == 3 || F::ColsAtCompileTime == Eigen::Dynamic);

/**
 * An array in which each row represents a three-dimensional vector. The first
 * column is the x-coordinate, the second column is the y-coordinate, and the
 * third column is the z-coordinate.
 */
template <typename V>
concept Vectors3D =
	std::floating_point<typename V::Scalar> &&
	(V::ColsAtCompileTime == 3 || V::ColsAtCompileTime == Eigen::Dynamic);

/**
 * An array in which each row contains the three-dimensional coordinates for a
 * vertex. The first column is the x-coordinate, the second column is the
 * y-coordinate, and the third column is the z-coordinate.
 */
template <typename V>
concept Vertices3D =
	std::floating_point<typename V::Scalar> &&
	(V::ColsAtCompileTime == 3 || V::ColsAtCompileTime == Eigen::Dynamic);

} // namespace modcam::mesh

#endif