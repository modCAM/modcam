# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

FetchContent_Declare(
	modcam-data
	GIT_REPOSITORY https://github.com/modCAM/modcam-data.git
	GIT_TAG 31e9a2a4a921674875e298b8017e53bc2faec47d
	CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(modcam-data)

set(DATA_PREFIX_PATH data)

cmrc_add_resource_library(
	modCAM_data 
	ALIAS modCAM::data 
	NAMESPACE modcam 
	WHENCE "${modcam-data_SOURCE_DIR}"
	PREFIX ${DATA_PREFIX_PATH}
	"${modcam-data_SOURCE_DIR}/mesh/sphere.stl"
)

if(MODCAM_ENABLE_LINTING)
	configure_file(.clang-tidy .clang-tidy COPYONLY)
endif()
