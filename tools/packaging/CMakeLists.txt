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
	ARCH_INDEPENDENT
)

install(
	FILES
	    modCAMConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/modCAMConfigVersion.cmake"
	DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
)

# Install the VERSION file for other projects to reference
install(
	FILES "${modCAM_SOURCE_DIR}/VERSION"
	DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(
    EXPORT modCAM_Targets
    DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
    NAMESPACE modCAM::
    FILE modCAM-Targets.cmake
)

set(CPACK_PACKAGE_NAME modCAM)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Free and open source CAM software")
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_VERBATIM_VARIABLES YES)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}")
if(CPACK_PACKAGE_VERSION_MINOR)
	string(APPEND CPACK_PACKAGE_FILE_NAME ".${CPACK_PACKAGE_VERSION_MINOR}")
endif()

set(CPACK_GENERATOR TGZ ZIP)

set(CPACK_SOURCE_IGNORE_FILES
	/\\.git/
	\\.swp
	\\.orig
	/CMakeLists\\.txt\\.user
)

include(CPack)
