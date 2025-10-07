# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# This should work, but it throws a bunch of warnings with CMake 4.0...
# FetchContent_Declare(
# 	modcam-data
# 	GIT_REPOSITORY https://github.com/modCAM/modcam-data.git
# 	GIT_TAG 31e9a2a4a921674875e298b8017e53bc2faec47d
# 	CONFIGURE_COMMAND ""
# 	BUILD_COMMAND ""
# 	INSTALL_COMMAND ""
# )
# FetchContent_MakeAvailable(modcam-data)
# ... so instead I download the data this way:
FetchContent_GetProperties(
	modcam-data
	POPULATED data_populated
)
if(NOT data_populated)
	message(STATUS "Downloading data files")
	FetchContent_Populate(
		modcam-data
		GIT_REPOSITORY https://github.com/modCAM/modcam-data.git
		GIT_TAG 31e9a2a4a921674875e298b8017e53bc2faec47d
	)
	message(STATUS "Downloading data files - done")
endif()

cmrc_add_resource_library(
	modCAM_data 
	ALIAS modCAM::data 
	NAMESPACE modcam 
	WHENCE "${modcam-data_SOURCE_DIR}"
	PREFIX data
	"${modcam-data_SOURCE_DIR}/mesh/sphere.stl"
)

if(MODCAM_ENABLE_LINTING)
	add_dependencies(run-clang-tidy modCAM::data)
	configure_file(.clang-tidy .clang-tidy COPYONLY)
endif()
