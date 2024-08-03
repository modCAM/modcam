/*
 * This file is part of modCAM, open source software for Computer Aided
 * Manufacturing research.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef MODCAM_VERSION_H
#define MODCAM_VERSION_H

#include "modcam_version_export.h"

#include <string>

namespace modcam {

/**
 * Get the full version of modCAM.
 *
 * @returns The full verions of modCAM (major.minor)
 */
MODCAM_VERSION_EXPORT std::string modcam_version();

/**
 * Get the major version of modCAM.
 *
 * @returns The major version of modCAM
 */
MODCAM_VERSION_EXPORT unsigned modcam_version_major();

/**
 * Get the minor version of modCAM.
 *
 * @returns The minor version of modCAM
 */
MODCAM_VERSION_EXPORT unsigned modcam_version_minor();

} // namespace modcam

#endif
