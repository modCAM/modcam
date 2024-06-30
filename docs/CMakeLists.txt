# Copyright 2023 prosilio
# 
# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# modCAM is free software: you can redistribute it and/or modify it under the 
# terms of the GNU General Public License as published by the Free Software 
# Foundation, either version 3 of the License, or (at your option) any later 
# version.
# 
# modCAM is distributed in the hope that it will be useful, but WITHOUT ANY 
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with 
# modCAM. If not, see <https://www.gnu.org/licenses/>. 

find_package(Doxygen REQUIRED)
if (NOT DOXYGEN_FOUND)
	message("Doxygen needs to be installed to build the documentation.")
	return()
endif ()

set(DOXYGEN_EXCLUDE_PATTERNS "*.test.cpp")
set(DOXYGEN_CITE_BIB_FILES "${modCAM_SOURCE_DIR}/docs/modCAM.bib")

doxygen_add_docs(
	modCAM_docs
	"${modCAM_SOURCE_DIR}/include"
	"${modCAM_SOURCE_DIR}/src/modcam"
	ALL
)
