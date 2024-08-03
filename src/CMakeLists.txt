# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

configure_file("modcam/modcam_version.cpp.in" "modcam/modcam_version.cpp" @ONLY)
add_library(modCAM_version)
add_library(modCAM::version ALIAS modCAM_version)
set_target_properties(
	modCAM_version PROPERTIES
	OUTPUT_NAME modCAM_version
	EXPORT_NAME version
)

generate_export_header(modCAM_version)

target_sources(
	modCAM_version
	PUBLIC
		FILE_SET HEADERS
		BASE_DIRS "${CMAKE_CURRENT_BINARY_DIR}" "${modCAM_SOURCE_DIR}/include"
		FILES
			"${CMAKE_CURRENT_BINARY_DIR}/modcam_version_export.h"
			"${modCAM_SOURCE_DIR}/include/modcam/modcam_version.h"
)
target_sources(
	modCAM_version
	PRIVATE
		"${CMAKE_CURRENT_BINARY_DIR}/modcam/modcam_version.cpp"
)

install(
    TARGETS modCAM_version
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

add_subdirectory("modcam/utility")
add_subdirectory("modcam/mesh")
