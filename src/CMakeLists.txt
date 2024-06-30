# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

configure_file("${modCAM_SOURCE_DIR}/src/modcam/modcam_version.cpp.in" "${modCAM_SOURCE_DIR}/src/modcam/modcam_version.cpp" @ONLY)

add_subdirectory("modcam/utility")
add_subdirectory("modcam/mesh")
