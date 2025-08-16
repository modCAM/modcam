# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

# Attempt to sanitize the input, to the extent that CMake allows it.
file(
    STRINGS VERSION modCAM_VRSN
    LIMIT_COUNT 1
    REGEX "[0-9][0-9][0-9][0-9]\\.[0-9][0-9]\\.[0-9][0-9](\\.([0-9]+))?"
)
if(NOT modCAM_VRSN)
	message(FATAL_ERROR "Something is wrong with the VERSION file! It should have the format YYYY.MM.DD[.tweak]")
endif()
