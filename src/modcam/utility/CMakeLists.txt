# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# The utility library MUST be linked privately by other libraries.
# 
# These are small helper functions that are not intended to be used outside 
# this project.
add_library(modCAM_utility OBJECT)
add_library(modCAM::utility ALIAS modCAM_utility)
target_link_libraries(
    modCAM_utility 
    PUBLIC
        Eigen3::Eigen
)
target_sources(
	modCAM_utility
	PUBLIC
		FILE_SET HEADERS
		BASE_DIRS "${modCAM_SOURCE_DIR}/src"
		FILES
			"${modCAM_SOURCE_DIR}/src/modcam/utility/modulus.h"
			"${modCAM_SOURCE_DIR}/src/modcam/utility/random_orthonormal.h"
)
target_sources(
	modCAM_utility
	PRIVATE
		random_orthonormal.cpp
)
