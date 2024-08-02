# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# modCAM does not have components. No components should be specified.
if(${CMAKE_FIND_PACKAGE_NAME}_FIND_COMPONENTS)
	set(
		${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
		"modCAM does not have components. The following components were specified: ${${CMAKE_FIND_PACKAGE_NAME}_FIND_COMPONENTS}"
	)
	set(${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
	return()
endif()

include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(libigl)

include(${CMAKE_CURRENT_LIST_DIR}/modCAM-Targets.cmake)
