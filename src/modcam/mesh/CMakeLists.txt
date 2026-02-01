# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

add_library(modCAM_mesh INTERFACE)
add_library(modCAM::mesh ALIAS modCAM_mesh)
target_link_libraries(
    modCAM_mesh INTERFACE
        Eigen3::Eigen
        igl::igl_core
		modCAM_utility
)
set_target_properties(
	modCAM_mesh PROPERTIES
	EXPORT_NAME mesh
)

target_sources(
	modCAM_mesh INTERFACE
		FILE_SET HEADERS
		BASE_DIRS "${modCAM_SOURCE_DIR}/include"
		FILES
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/concepts.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/per_vertex_basis_pro2024.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/per_vertex_normals_max1999.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/principal_curvature_rus2004.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/voronoi_area_mey2003.h"
)

install(
    TARGETS modCAM_mesh
    EXPORT modCAM_Targets
    FILE_SET HEADERS
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Unit tests
if(MODCAM_ENABLE_TESTING)
	target_sources(
		test_modcam PRIVATE
			per_vertex_basis_pro2024.test.cpp
			per_vertex_normals_max1999.test.cpp
			principal_curvature_rus2004.test.cpp
			voronoi_area_mey2003.test.cpp
	)
	target_link_libraries(
		test_modcam PRIVATE
			modCAM::data
			modCAM::mesh
	)
endif()
