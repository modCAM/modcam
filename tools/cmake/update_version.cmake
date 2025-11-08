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
    REGEX "^[0-9][0-9][0-9][0-9]\\.[0-9][0-9]\\.[0-9][0-9](\\.([0-9]+))?$"
)
if(NOT MODCAM_VERSION)
	message(FATAL_ERROR "Something is wrong with the VERSION file! It should have the format YYYY.MM.DD[.tweak]")
endif()

# Separate major (UTC year), minor (UTC month), patch (UTC day), and (optionally) tweak parts of the version
string(REPLACE "." ";" MODCAM_VERSION ${MODCAM_VERSION})
list(LENGTH MODCAM_VERSION VERSION_LENGTH)
list(GET MODCAM_VERSION 0 VERSION_YEAR)
list(GET MODCAM_VERSION 1 VERSION_MONTH)
list(GET MODCAM_VERSION 2 VERSION_DAY)
if(VERSION_LENGTH GREATER 3)
	list(GET MODCAM_VERSION 3 VERSION_SAME_DAY)
endif()

string(TIMESTAMP UTC_DATE %Y.%m.%d UTC)

# Update the version
set(MODCAM_VERSION ${UTC_DATE})
if("${VERSION_YEAR}.${VERSION_MONTH}.${VERSION_DAY}" VERSION_EQUAL ${UTC_DATE})
	if(NOT VERSION_SAME_DAY)
		set(VERSION_SAME_DAY 0)
	endif()
	MATH(EXPR VERSION_SAME_DAY "${VERSION_SAME_DAY}+1")
	string(APPEND MODCAM_VERSION ".${VERSION_SAME_DAY}")
endif()

file(WRITE VERSION ${MODCAM_VERSION})
