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
option(MODCAM_FORCE_VIRTUAL_ENV "Throw an error if a Python virtual environment is not active." ON)
option(MODCAM_BUILD_DOC "Build documentation" OFF)
option(MODCAM_ENABLE_TESTING "Build tests" OFF)
option(MODCAM_ENABLE_LINTING "Run static analysis checks" OFF)
cmake_dependent_option(
	MODCAM_BUILD_PROJECT
	"Compile the code. Turn off to build only the documentation." ON
	"NOT MODCAM_ENABLE_TESTING AND NOT MODCAM_ENABLE_LINTING" ON
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

# Linting/static analysis
if(MODCAM_ENABLE_LINTING)
	# Make sure that you set the following variables in a toolchain file, in a 
	# preset, or on the command line:
	#     CMAKE_EXPORT_COMPILE_COMMANDS = TRUE
	#     CLANG_TIDY_EXECUTABLE = <path/to/clang-tidy>, for example /usr/bin/clang-tidy
	#     CMAKE_CXX_CLANG_TIDY = ${CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR}
	#     [Optional] CMAKE_C_CLANG_TIDY = ${CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR}

	if(NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
		string(
			CONCAT ERR_MSG
			"MODCAM_ENABLE_LINTING is ON, but CMAKE_EXPORT_COMPILE_COMMANDS is not \n"
			"defined. CMAKE_EXPORT_COMPILE_COMMANDS should be set to ON in a \n"
			"toolchain file, in a preset, or on the command line."
		)
		message(FATAL_ERROR ${ERR_MSG})
	endif()
	if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
		string(
			CONCAT ERR_MSG
			"MODCAM_ENABLE_LINTING is ON, but CMAKE_EXPORT_COMPILE_COMMANDS is OFF. \n"
			"CMAKE_EXPORT_COMPILE_COMMANDS should be set to ON in a toolchain file, \n"
			"in a preset, or on the command line."
		)
		message(FATAL_ERROR ${ERR_MSG})
	endif()
	if(NOT DEFINED CLANG_TIDY_EXECUTABLE)
		string(
			CONCAT ERR_MSG
			"MODCAM_ENABLE_LINTING is ON, but CLANG_TIDY_EXECUTABLE is undefined. \n"
			"CLANG_TIDY_EXECUTABLE should be set in a toolchain file, in a preset, \n"
			"or on the command line. It should be the clang-tidy filepath, such as \n"
			"/usr/bin/clang-tidy. Using a toolchain file, you can find clang-tidy \n"
			"via the following CMake command: \n"
			"    find_program(CLANG_TIDY_EXECUTABLE clang-tidy REQUIRED)"
		)
		message(FATAL_ERROR ${ERR_MSG})
	endif()
	if(NOT DEFINED CMAKE_CXX_CLANG_TIDY)
		string(
			CONCAT ERR_MSG
			"MODCAM_ENABLE_LINTING is ON, but CMAKE_CXX_CLANG_TIDY is undefined. \n"
			"CMAKE_CXX_CLANG_TIDY should be set in a toolchain file, in a preset, \n"
			"or on the command line. It should contain the clang-tidy command. Using \n"
			"a toolchain file, you can set it via the following CMake command: \n"
			"    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR})"
		)
		message(FATAL_ERROR ${ERR_MSG})
	endif()

	configure_file(.clang-tidy .clang-tidy COPYONLY)

	find_program(RUN_CLANG_TIDY_EXECUTABLE run-clang-tidy REQUIRED)
	add_custom_target(
		run-clang-tidy ALL
		COMMAND ${RUN_CLANG_TIDY_EXECUTABLE}
			-clang-tidy-binary ${CLANG_TIDY_EXECUTABLE}
			-p ${CMAKE_BINARY_DIR}
	)
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
