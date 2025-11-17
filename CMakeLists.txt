# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

cmake_minimum_required(VERSION 4.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools/cmake")
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
option(MODCAM_ENABLE_LINTING "Run static analysis checks" OFF)
cmake_dependent_option(
	MODCAM_BUILD_PROJECT
	"Compile the code. Turn off to build only the documentation." ON
	"MODCAM_BUILD_DOC AND NOT MODCAM_ENABLE_TESTING AND NOT MODCAM_ENABLE_LINTING" ON
)
cmake_dependent_option(
	MODCAM_FORCE_VIRTUAL_ENV
	"Throw an error if a Python virtual environment is not active." ON
	"MODCAM_BUILD_DOC" ON
) # The documentation is built by Sphinx, which requires certain Python
  # packages. We only force the installation of those packages when building the
  # documentation. See cmake/modcam_dependencies.cmake.

# Documentation
if(MODCAM_BUILD_DOC)
    add_subdirectory(docs)
endif()
if(NOT MODCAM_BUILD_PROJECT)
	return() # Build only the documentation and nothing else.
endif()

# Require a minimum C++ standard, but let a parent project ask for something
# higher.
set(MODCAM_MIN_CXX_STANDARD 20)
if(DEFINED CMAKE_CXX_STANDARD)
	if(CMAKE_CXX_STANDARD EQUAL 98 OR CMAKE_CXX_STANDARD LESS MODCAM_MIN_CXX_STANDARD)
		message(FATAL_ERROR "modCAM requires at least C++${MODCAM_MIN_CXX_STANDARD}")
	endif()
else()
	set(CMAKE_CXX_STANDARD ${MODCAM_MIN_CXX_STANDARD})
endif()

# Always enforce the language constraint
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# We don't need compiler extensions, but let a parent ask for them
if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# All dependencies here
include(modcam_dependencies)

# Linting/static analysis
if(MODCAM_ENABLE_LINTING)
	set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

	if(NOT CLANG_TIDY_EXECUTABLE)
		find_program(
			CLANG_TIDY_EXECUTABLE
			NAMES
				clang-tidy
				clang-tidy-21
				clang-tidy-20
				clang-tidy-19
				clang-tidy-18
				clang-tidy-17
				clang-tidy-16
				clang-tidy-15
		)
		if (NOT CLANG_TIDY_EXECUTABLE)
			string(
				CONCAT CLANG_TIDY_ERR_MSG
				"clang-tidy was not found. Please specify the path to "
				"clang-tidy as a command line argument:\n"
				"    -D CLANG_TIDY_EXECUTABLE=/path/to/clang-tidy"
			)
			message(FATAL_ERROR ${CLANG_TIDY_ERR_MSG})
		endif()
	endif()

	# set(CMAKE_C_CLANG_TIDY ${CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR})
	set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR})
	configure_file(.clang-tidy .clang-tidy COPYONLY)

	if(NOT RUN_CLANG_TIDY_EXECUTABLE)
		find_program(
			RUN_CLANG_TIDY_EXECUTABLE
			NAMES
				run-clang-tidy
				run-clang-tidy-21
				run-clang-tidy-20
				run-clang-tidy-19
				run-clang-tidy-18
				run-clang-tidy-17
				run-clang-tidy-16
				run-clang-tidy-15
		)
		if (NOT RUN_CLANG_TIDY_EXECUTABLE)
			string(
				CONCAT RUN_CLANG_TIDY_ERR_MSG
				"run-clang-tidy was not found. Please specify the path to "
				"run-clang-tidy as a command line argument:\n"
				"    -D RUN_CLANG_TIDY_EXECUTABLE=/path/to/run-clang-tidy"
			)
			message(FATAL_ERROR ${RUN_CLANG_TIDY_ERR_MSG})
		endif()
	endif()

	add_custom_target(
		run-clang-tidy ALL
		COMMAND ${RUN_CLANG_TIDY_EXECUTABLE}
			-clang-tidy-binary ${CLANG_TIDY_EXECUTABLE}
			-p ${CMAKE_BINARY_DIR}
			-warnings-as-errors '*'
	)
endif()

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
# Allow package maintainers to freely override the path for the configs
set(MODCAM_INSTALL_CMAKEDIR "share/cmake/modCAM"
	CACHE PATH "CMake package config location"
)
mark_as_advanced(MODCAM_INSTALL_CMAKEDIR)

install(
	EXPORT modCAM_Targets
	DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
	NAMESPACE modCAM::
	FILE modCAM-Targets.cmake
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/modCAMConfigVersion.cmake"
	COMPATIBILITY AnyNewerVersion
	ARCH_INDEPENDENT
)

install(
	FILES
		"${CMAKE_CURRENT_LIST_DIR}/tools/cmake/modCAMConfig.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/modCAMConfigVersion.cmake"
	DESTINATION "${MODCAM_INSTALL_CMAKEDIR}"
)

# Install the VERSION file for other projects to reference
install(
	FILES "${modCAM_SOURCE_DIR}/VERSION"
	DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(PROJECT_IS_TOP_LEVEL)
	add_subdirectory(tools/packaging)
else()
	mark_as_advanced(MODCAM_BUILD_DOC)
	mark_as_advanced(MODCAM_ENABLE_TESTING)
	mark_as_advanced(MODCAM_ENABLE_LINTING)
	mark_as_advanced(MODCAM_BUILD_PROJECT)
	mark_as_advanced(MODCAM_FORCE_VIRTUAL_ENV)
endif()
