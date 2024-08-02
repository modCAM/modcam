# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

add_library(modCAM_mesh)
add_library(modCAM::mesh ALIAS modCAM_mesh)
target_link_libraries(
    modCAM_mesh 
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        igl::igl_core
		$<BUILD_INTERFACE:modCAM_utility>
)
set_target_properties(
	modCAM_mesh PROPERTIES
	OUTPUT_NAME modCAM_mesh
	EXPORT_NAME mesh
)

generate_export_header(
	modCAM_mesh
	BASE_NAME ${PROJECT_NAME}
)

target_sources(
	modCAM_mesh
	PUBLIC
		FILE_SET HEADERS
		BASE_DIRS "${CMAKE_CURRENT_BINARY_DIR}" "${modCAM_SOURCE_DIR}/include"
		FILES 
			"${CMAKE_CURRENT_BINARY_DIR}/modcam_export.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/per_vertex_basis.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/per_vertex_normals.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/principal_curvature.h"
			"${modCAM_SOURCE_DIR}/include/modcam/mesh/voronoi_area.h"
)
target_sources(
	modCAM_mesh
	PRIVATE
		per_vertex_basis.cpp
		per_vertex_normals.cpp
		principal_curvature.cpp
		voronoi_area.cpp
)

install(
    TARGETS modCAM_mesh 
    EXPORT modCAM_Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            COMPONENT modCAM_Runtime
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            COMPONENT modCAM_Runtime
            NAMELINK_COMPONENT modCAM_Development
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            COMPONENT modCAM_Development
	FILE_SET HEADERS
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Unit tests
if(MODCAM_ENABLE_TESTING)
	target_sources(
		test_modcam
		PRIVATE
			per_vertex_basis.test.cpp
			per_vertex_normals.test.cpp
			principal_curvature.test.cpp
			voronoi_area.test.cpp
	)
	target_link_libraries(
		test_modcam 
		PRIVATE
			modCAM::data
			modCAM::mesh
	)
endif()
