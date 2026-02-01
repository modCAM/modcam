# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

add_subdirectory("modcam/utility")
add_subdirectory("modcam/mesh")

# Each library added via add_subdirectory above should be linked to this
# target.
add_library(modCAM_all INTERFACE)
add_library(modCAM::all ALIAS modCAM_all)
target_link_libraries(
	modCAM_all INTERFACE
		modCAM_utility
		modCAM_mesh
)
set_target_properties(
	modCAM_all PROPERTIES
	EXPORT_NAME all
)
install(
    TARGETS modCAM_all
    EXPORT modCAM_Targets
    FILE_SET HEADERS
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
