# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 3.23)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(modcam_version)

project(
    modCAM
    DESCRIPTION "Open source software for Computer Aided Manufacturing (CAM) research"
	VERSION ${modCAM_VRSN}
    LANGUAGES CXX
)

# Helpful CMake modules
include(CMakeDependentOption)
include(CMakeRC)
include(FetchContent)
include(GNUInstallDirs)

# Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(PROJECT_IS_TOP_LEVEL AND EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build directory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

# Project options
option(MODCAM_BUILD_DOC "Build documentation" OFF)
option(MODCAM_ENABLE_TESTING "Build tests" OFF)
cmake_dependent_option(
	MODCAM_BUILD_PROJECT
	"Compile the code. Turn off to build only the documentation." ON
	"NOT MODCAM_ENABLE_TESTING" ON
)

# Documentation
if(MODCAM_BUILD_DOC)
    add_subdirectory(docs)
	if(NOT MODCAM_BUILD_PROJECT)
		return() # Build only the documentation and nothing else.
	endif()
endif()

# Require C++20, but let a parent project ask for something higher
if(DEFINED CMAKE_CXX_STANDARD)
	if(CMAKE_CXX_STANDARD EQUAL 98 OR CMAKE_CXX_STANDARD LESS 20)
		message(FATAL_ERROR "modCAM requires at least C++20")
	endif()
else()
	set(CMAKE_CXX_STANDARD 20)
endif()

# Always enforce the language constraint
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# We don't need compiler extensions, but let a parent ask for them
if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# All dependencies here
include(modcam_dependencies)

# Project data
# The data may include rather large files such as triangle meshes, so they are 
# only included if necessary.
if(MODCAM_ENABLE_TESTING) # Further conditions may be added in the future.
	add_subdirectory(data)
endif()

# Tests
# The test_modcam target is available for adding unit tests.
if(MODCAM_ENABLE_TESTING)
	enable_testing()
    add_subdirectory(tests)
endif()

# Project files
add_subdirectory(src)

# Packaging
if(PROJECT_IS_TOP_LEVEL)
	# Allow package maintainers to freely override the path for the configs
	set(MODCAM_INSTALL_CMAKEDIR "share/cmake/modCAM"
		CACHE PATH "CMake package config location"
	)
	mark_as_advanced(MODCAM_INSTALL_CMAKEDIR)
	add_subdirectory(packaging)
endif()
