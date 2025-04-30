# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# These are small helper functions that are not intended to be used outside 
# this project.
add_library(modCAM_utility INTERFACE)
add_library(modCAM::utility ALIAS modCAM_utility)
target_link_libraries(
    modCAM_utility INTERFACE
        Eigen3::Eigen
)
set_target_properties(
	modCAM_utility PROPERTIES
	EXPORT_NAME utillity
)

target_sources(
	modCAM_utility INTERFACE
		FILE_SET HEADERS
		BASE_DIRS "${modCAM_SOURCE_DIR}/include"
		FILES
			"${modCAM_SOURCE_DIR}/include/modcam/utility/concepts.h"
			"${modCAM_SOURCE_DIR}/include/modcam/utility/modulus.h"
			"${modCAM_SOURCE_DIR}/include/modcam/utility/random_orthonormal.h"
)

install(
    TARGETS modCAM_utility
    EXPORT modCAM_Targets
    FILE_SET HEADERS
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Unit tests
if(MODCAM_ENABLE_TESTING)
	target_sources(
		test_modcam PRIVATE
			modulus.test.cpp
			random_orthonormal.test.cpp
	)
	target_link_libraries(
		test_modcam PRIVATE
			modCAM::utility
	)
endif()
