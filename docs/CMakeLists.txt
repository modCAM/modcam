# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

find_package(Doxygen REQUIRED)

set(DOXYGEN_CITE_BIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/modCAM.bib")
set(DOXYGEN_EXCLUDE_PATTERNS "*.test.cpp")
set(DOXYGEN_GENERATE_XML YES)
# set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/doxygen")
set(DOXYGEN_STRIP_FROM_PATH "${modCAM_SOURCE_DIR}")

doxygen_add_docs(
	modCAM_doxygen_docs
	"${modCAM_SOURCE_DIR}/include"
	"${modCAM_SOURCE_DIR}/src/modcam"
	ALL
)

find_package(Sphinx REQUIRED)

set(SPHINX_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
sphinx_build(
	modCAM_sphinx_docs ALL
	OPTIONS
		"-b html"
		"-Dbreathe_projects.modCAM=${DOXYGEN_OUTPUT_DIRECTORY}/xml"
	SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
	OUTPUT_DIR "${SPHINX_BUILD_DIR}"
	SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/conf.py
		${CMAKE_CURRENT_SOURCE_DIR}/index.rst
		${CMAKE_CURRENT_SOURCE_DIR}/bibliography.rst
		${CMAKE_CURRENT_SOURCE_DIR}/reference/mesh.rst
)
add_dependencies(modCAM_sphinx_docs modCAM_doxygen_docs)

# Add an install target to install the docs
install(
	DIRECTORY ${SPHINX_BUILD_DIR}
	DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
