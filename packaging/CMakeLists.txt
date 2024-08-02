# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	modCAMConfigVersion.cmake
	COMPATIBILITY AnyNewerVersion
)

install(
	FILES
		modCAMConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/modCAMConfigVersion.cmake"
	DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
	COMPONENT modCAM_Development
)

install(
    EXPORT modCAM_Targets
    DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
    NAMESPACE modCAM::
    FILE modCAM-Targets.cmake
    COMPONENT modCAM_Development
)

set(CPACK_PACKAGE_NAME modCAM)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Free and open source CAM software")
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_VERBATIM_VARIABLES YES)

if(WIN32)
	set(CPACK_GENERATOR ZIP)
else()
	set(CPACK_GENERATOR TGZ)
endif()

set(CPACK_SOURCE_IGNORE_FILES
	/\\.git/
	\\.swp
	\\.orig
	/CMakeLists\\.txt\\.user
)

include(CPack)

cpack_add_component(modCAM_Runtime
	DISPLAY_NAME Runtime
	DESCRIPTION "Shared libraries and executables"
	REQUIRED
	INSTALL_TYPES Full Developer Minimal
)
cpack_add_component(modCAM_Development
	DISPLAY_NAME "Developer pre-requisites"
	DESCRIPTION "Headers/static libs needed for building"
	DEPENDS modCAM_Runtime
	INSTALL_TYPES Full Developer
)
