# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# Run this script to update the version in the VERSION file:
#   cmake -P ./cmake/update_version.cmake

# Attempt to sanitize the input, to the extent that CMake allows it.
file(
    STRINGS VERSION MODCAM_VERSION
    LIMIT_COUNT 1
    REGEX "[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9](\\.([0-9]+))?"
)
if(NOT MODCAM_VERSION)
	message(FATAL_ERROR "Something is wrong with the VERSION file! It should have the format YYYYMMDD[.minor]")
endif()

# Separate major (UTC year-month-day) and minor parts of the version
string(REPLACE "." ";" MODCAM_VERSION ${MODCAM_VERSION})
list(LENGTH MODCAM_VERSION VERSION_LENGTH)
list(GET MODCAM_VERSION 0 VERSION_MAJOR)
if(VERSION_LENGTH GREATER 1)
	list(GET MODCAM_VERSION 1 VERSION_MINOR)
endif()

string(TIMESTAMP UTC_DATE %Y%m%d UTC)

# Update the version
if(${VERSION_MAJOR} VERSION_EQUAL ${UTC_DATE})
	if(NOT VERSION_MINOR)
		set(VERSION_MINOR 0)
	endif()
	MATH(EXPR VERSION_MINOR "${VERSION_MINOR}+1")
	set(MODCAM_VERSION ${UTC_DATE}.${VERSION_MINOR})
else()
	set(MODCAM_VERSION ${UTC_DATE})
endif()

file(WRITE VERSION ${MODCAM_VERSION})
