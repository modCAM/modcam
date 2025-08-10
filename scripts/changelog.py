# This file is part of modCAM, open source software for Computer Aided
# Manufacturing research.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# SPDX-FileCopyrightText: Copyright contributors to the modCAM project.
# SPDX-License-Identifier: MPL-2.0

"""When a release is created, set the version in CHANGELOG.md.

Each release should update the changelog with the latest version, which is based
on the release date. Due to timing uncertainties in the GitHub workflows, we
cannot know exactly when a release will be created. If the code is updated near
midnight UTC, then the release may not be created until the following day.
Hence, we use this Python script to insert the version info into the changelog
at the time of release.

This script looks for the "## [LATEST]" placeholder in the changelog and inserts
the latest version number. It also creates a link to the corresponding release
on GitHub.
"""

with open("VERSION", "r") as version_file:
	version = version_file.readline().rstrip()

print(f"Version: {version}")

with open("CHANGELOG.md", "r+") as changelog:
	contents = changelog.read()
	contents = contents.replace("## [LATEST]", f"## [{version}]")

	changelog.seek(0)
	changelog.write(contents)

	release_link = f"\n[{version}]: https://github.com/modCAM/modcam/releases/tag/{version}"
	changelog.write(release_link)
