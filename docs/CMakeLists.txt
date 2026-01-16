# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# conf.py looks for this environment variable to determine the output location
# for the generated docs.
set(ENV{MODCAM_DOCS_BUILD_DIR} "${CMAKE_CURRENT_BINARY_DIR}")

find_package(Sphinx REQUIRED)

set(SPHINX_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
sphinx_build(
	modCAM_sphinx_docs ALL
	OPTIONS
		"-b html"
	SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
	OUTPUT_DIR "${SPHINX_BUILD_DIR}"
	SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/conf.py
		${CMAKE_CURRENT_SOURCE_DIR}/index.rst
		${CMAKE_CURRENT_SOURCE_DIR}/bibliography.rst
		${CMAKE_CURRENT_SOURCE_DIR}/reference/mesh.rst
)

# Add an install target to install the docs
install(
	DIRECTORY ${SPHINX_BUILD_DIR}
	DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
