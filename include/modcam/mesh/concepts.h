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

#ifndef MESH_CONCEPTS_H
#define MESH_CONCEPTS_H

#include <Eigen/Core>

#include <concepts>

namespace modcam::mesh {

template <typename V>
concept Vectors3D =
	std::floating_point<typename V::Scalar> &&
	(V::ColsAtCompileTime == 3 || V::ColsAtCompileTime == Eigen::Dynamic);

template <typename V>
concept Vertices3D =
	std::floating_point<typename V::Scalar> &&
	(V::ColsAtCompileTime == 3 || V::ColsAtCompileTime == Eigen::Dynamic);

template <typename F>
concept TriangleFaces =
	std::integral<typename F::Scalar> &&
	(F::ColsAtCompileTime == 3 || F::ColsAtCompileTime == Eigen::Dynamic);

} // namespace modcam::mesh

#endif