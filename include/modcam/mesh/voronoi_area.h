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

#include "modcam/mesh/concepts.h"
#include "modcam/utility/modulus.h"

#include <Eigen/Core>

#include <igl/cotmatrix_entries.h>
#include <igl/doublearea.h>
#include <igl/edge_lengths.h>
#include <igl/internal_angles.h>

#include <cassert>
#include <concepts>
#include <numbers>

namespace modcam::mesh {

/**
 * Compute the Voronoi cell areas for the triangles in a mesh as described in
 * @cite Meyer2003.
 *
 * @param[out] v_area F-by-3 matrix of the Voronoi area of each vertex in each
 * triangle
 * @param[in] vertices V-by-3 matrix of mesh vertex coordinates
 * @param[in] faces F-by-3 matrix of face (triangle) indices
 */
template <Vertices3D DerivedV, TriangleFaces DerivedF, typename DerivedVA>
requires std::floating_point<typename DerivedVA::Scalar> &&
         (DerivedVA::ColsAtCompileTime == 3 ||
          DerivedVA::ColsAtCompileTime == Eigen::Dynamic)
void voronoi_area(Eigen::PlainObjectBase<DerivedVA> &v_area,
                  const Eigen::MatrixBase<DerivedV> &vertices,
                  const Eigen::MatrixBase<DerivedF> &faces)
{
	assert(vertices.cols() == 2 ||
	       vertices.cols() == 3 && "vertices must have 3 columns");
	assert(faces.cols() == 3 && "faces must have 3 columns");

	if (faces.size() == 0) {
		v_area.derived().resize(0, 3);
		return;
	}

	auto num_faces = faces.rows();
	auto vertices_per_face = faces.cols();
	v_area.derived().resize(num_faces, vertices_per_face);

	if (vertices.size() == 0) {
		v_area.setZero();
		return;
	}

	Eigen::ArrayX<typename DerivedVA::Scalar> area;
	igl::doublearea(vertices, faces, area);
	area /= static_cast<typename DerivedVA::Scalar>(
		2.0); // NOLINT(cppcoreguidelines-avoid-magic-numbers)

	Eigen::ArrayXX<typename DerivedV::Scalar> angles;
	igl::internal_angles(vertices, faces, angles);

	Eigen::MatrixX<typename DerivedVA::Scalar> half_cot;
	igl::cotmatrix_entries(vertices, faces, half_cot);

	Eigen::MatrixX<typename DerivedVA::Scalar> edge_squared;
	igl::edge_lengths(vertices, faces, edge_squared);
	edge_squared = edge_squared.cwiseProduct(edge_squared);

	constexpr auto right_angle = static_cast<typename DerivedV::Scalar>(
		std::numbers::pi /
		2.0); // NOLINT(cppcoreguidelines-avoid-magic-numbers)
	Eigen::Array<bool, Eigen::Dynamic, 1> nonobtuse =
		(angles <= right_angle).rowwise().all();

	for (Eigen::Index row = 0; row < num_faces; row++) {
		for (Eigen::Index col = 0; col < vertices_per_face; col++) {
			if (nonobtuse(row)) {
				Eigen::Index i = utility::mod(col - 1, vertices_per_face);
				Eigen::Index j = utility::mod(col + 1, vertices_per_face);
				v_area(row, col) =
					0.25 * // NOLINT(cppcoreguidelines-avoid-magic-numbers)
					(edge_squared(row, i) * half_cot(row, i) +
				     edge_squared(row, j) * half_cot(row, j));
			} else {
				if (angles(row, col) > right_angle) {
					v_area(row, col) =
						area(row) /
						static_cast<typename DerivedV::Scalar>(
							2.0); // NOLINT(cppcoreguidelines-avoid-magic-numbers)
				} else {
					v_area(row, col) =
						area(row) /
						static_cast<typename DerivedV::Scalar>(
							4.0); // NOLINT(cppcoreguidelines-avoid-magic-numbers)
				}
			}
		}
	}
}

} // namespace modcam::mesh

#endif
