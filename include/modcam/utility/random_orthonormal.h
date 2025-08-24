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

#ifndef MODCAM_UTILITY_RANDOM_ORTHONORMAL_H
#define MODCAM_UTILITY_RANDOM_ORTHONORMAL_H

#include "modcam/utility/concepts.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace modcam::utility {

/**
 * Create normalized 3D vectors orthogonal to the input vectors.
 *
 * Given a vector in 3D space, one can create infinitely many vectors orthogonal
 * to it. This function, while not strictly random, creates, for all intents and
 * purposes, a random vector that is orthogonal to the given vector.
 *
 * @param[out] ortho_vectors V-by-3 matrix of unit vectors orthogonal to the
 * input vectors
 * @param[in] vectors V-by-3 matrix, where each row represents a 3D vector
 */
template <Vectors3D DerivedVec, Vectors3D DerivedOrtho>
void random_orthonormal(Eigen::PlainObjectBase<DerivedOrtho> &ortho_vectors,
                        const Eigen::MatrixBase<DerivedVec> &vectors)
{
	assert(vectors.cols() == 3 && "vectors must have 3 columns");

	ortho_vectors.derived().resize(vectors.rows(), 3);
	ortho_vectors = vectors.template cast<typename DerivedOrtho::Scalar>();
	ortho_vectors.rowwise().normalize();

	Eigen::RowVector3<typename DerivedOrtho::Scalar> perturb_vec;
	constexpr auto perturbation =
		static_cast<typename DerivedOrtho::Scalar>(1.0);
	constexpr auto one = static_cast<typename DerivedOrtho::Scalar>(1.0);
	constexpr auto tolerance =
		static_cast<typename DerivedOrtho::Scalar>(1.0e-2);
	for (auto vec : ortho_vectors.rowwise()) {
		perturb_vec = vec;
		auto x_aligned = (one - std::abs(vec(0))) < tolerance;
		if (x_aligned) {
			perturb_vec(1) += perturbation;
		} else {
			perturb_vec(0) += perturbation;
		}
		vec = vec.cross(perturb_vec);
	}
	ortho_vectors.rowwise().normalize();
}

} // namespace modcam::utility

#endif
