# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

find_package(doctest REQUIRED)

add_executable(test_modcam
    "${modCAM_SOURCE_DIR}/tests/run_tests.cpp"
)
target_link_libraries(
    test_modcam
    PRIVATE
        doctest::doctest
        Eigen3::Eigen
        igl::igl_core
        modCAM::mesh
)

include(doctest)
doctest_discover_tests(test_modcam)
