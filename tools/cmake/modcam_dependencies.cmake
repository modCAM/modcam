# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(libigl CONFIG REQUIRED)

if(NOT MODCAM_BUILD_DOC)
	# Don't force Python package installation if it's not needed.
	return()
endif()

message(STATUS "Installing required Python packages")

# Force CMake to search for Python every time to ensure that the current Python
# environment is found. Otherwise, a previously activated environment may be
# used.
set(Python3_ARTIFACTS_PREFIX "_VENV")
unset(_Python3${Python3_ARTIFACTS_PREFIX}_EXECUTABLE CACHE)

cmake_language(GET_MESSAGE_LOG_LEVEL log_level)
set(find_python_quiet QUIET)
if(log_level MATCHES "VERBOSE|DEBUG|TRACE")
	set(find_python_quiet)
endif()
find_package(Python3 COMPONENTS Interpreter REQUIRED ${find_python_quiet})
if(NOT Python3${Python3_ARTIFACTS_PREFIX}_Interpreter_FOUND)
	message(FATAL_ERROR "The Python 3 interpreter could not be found.")
endif()

set(py3_exe ${Python3${Python3_ARTIFACTS_PREFIX}_EXECUTABLE})
execute_process(
	COMMAND ${py3_exe} ${modCAM_SOURCE_DIR}/tools/is_virtual_env.py
	OUTPUT_VARIABLE is_virtual_env
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ECHO_ERROR_VARIABLE
	COMMAND_ERROR_IS_FATAL ANY
)
set(pip_args "--quiet")
if(log_level MATCHES "VERBOSE|DEBUG|TRACE")
	set(pip_args)
endif()
if(NOT is_virtual_env)
	string(
		CONCAT warn_msg
		"No active Python virtual environment. Required packages will be "
		"installed at the user level for the following Python installation:\n"
		"    ${py3_exe}\n"
	)
	set(msg_level NOTICE)
	if(MODCAM_FORCE_VIRTUAL_ENV)
		string(
			APPEND warn_msg
			"If this is alright, then set MODCAM_FORCE_VIRTUAL_ENV to FALSE.\n"
		)
		set(msg_level FATAL_ERROR)
	endif()
	message(${msg_level} ${warn_msg})
	string(JOIN " " pip_args ${pip_args} "--user")
endif()
execute_process(
	COMMAND ${py3_exe} -m pip install ${pip_args} -r "${modCAM_SOURCE_DIR}/requirements.txt"
	ECHO_ERROR_VARIABLE
	COMMAND_ERROR_IS_FATAL ANY
)
unset(Python3_ARTIFACTS_PREFIX)
message(STATUS "Installing required Python packages - done")
