# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# Much of this code was taken from the Breathe project, which is licensed under
# the BSD license, reproduced below. The Breathe project can be found at
# https://github.com/breathe-doc/breathe
#
# BSD license, modified to remove the organisation as there isn't one.
#
# Copyright (c) 2009, Michael Jones
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 	 * Redistributions of source code must retain the above copyright notice,
# 	   this list of conditions and the following disclaimer.
# 	 * Redistributions in binary form must reproduce the above copyright notice,
# 	   this list of conditions and the following disclaimer in the documentation
# 	   and/or other materials provided with the distribution.
# 	 * The names of its contributors may not be used to endorse or promote
# 	   products derived from this software without specific prior written
# 	   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -- Project information -------------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from pathlib import Path
import re
import sphinx
import subprocess
import sys
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sphinx.application import Sphinx
    from sphinx.util.typing import ExtensionMetadata


project = "modCAM"
author = ""
copyright = "contributors to the modCAM project"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
print(f"PROJECT_ROOT = {PROJECT_ROOT}")

git_tag = subprocess.run(
    ["git", "describe", "--tags"],
    capture_output=True,
    encoding="utf-8",
    cwd=PROJECT_ROOT,
)
version = release = "latest"
if re.match(r"^\d+\.\d+\.\d+(\.d+)?$", git_tag.stdout):
    version = release = git_tag.stdout


# -- General configuration -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["breathe", "sphinx_copybutton", "sphinx_design"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_book_theme"

# -- Bibliography configuration ------------------------------------------------
bibtex_bibfiles = ["modCAM.bib"]
bibtex_default_style = "unsrt"

# -- Breathe configuration -----------------------------------------------------
breathe_default_project = "modCAM"

try:
    doxygen_test = subprocess.check_output(["doxygen", "--version"], encoding="utf-8")
except subprocess.CalledProcessError as err:
    msg = f"doxygen --version reported an error: {err.stderr}"
    raise RuntimeError(msg) from err
else:
    print(f"Using Doxygen v{doxygen_test}")
    del doxygen_test


def generate_doxyfile(
    doxyfile_tmpl: Path, doxyfile_out: Path, find_and_replace: dict[str]
) -> None:
    """Generate a Doxygen config file from a template.

    Parameters
    ----------
    doxyfile_tmpl : pathlib.Path
        The Doxygen config file template.
    doxyfile_out : pathlib.Path
        The Doxygen config file to create from the template.
    find_and_replace : dict[str]
        A dict in which the keys are the strings to find, and the values are
        the replacement strings.
    """
    doxyfile_out.parent.mkdir(exist_ok=True, parents=True)
    with open(doxyfile_tmpl, "r") as in_file, open(doxyfile_out, "w") as out_file:
        template = in_file.read()
        for find, replace in find_and_replace.items():
            template = template.replace(find, replace)
        template = re.sub(r"@[\w]+@", "", template)
        out_file.write(template)


def run_doxygen(doxyfile: Path) -> None:
    """Run the doxygen command with the specified configuration file.

    Parameters
    ----------
    doxyfile : pathlib.Path
        The Doxygen configuration file
    """
    try:
        subprocess.run(["doxygen", str(doxyfile)], check=True, cwd=PROJECT_ROOT)
    except subprocess.CalledProcessError as e:
        print(f"doxygen terminated by signal {-e.returncode}", file=sys.stderr)
    except OSError as e:
        print(f"doxygen execution failed: {e}", file=sys.stderr)


def generate_doxygen_xml(app: Sphinx) -> None:
    """Generate Doxygen XML files if we're on the ReadTheDocs server"""
    doxyfile_tmpl = PROJECT_ROOT / "docs" / "Doxyfile.in"
    doxyfile = Path(app.config.modcam_doxygen_output_dir) / "Doxyfile"
    doxyfile_find_and_replace = {
        "@DOXYGEN_CITE_BIB_FILES@": (PROJECT_ROOT / "docs" / "modCAM.bib").as_posix(),
        "@_DOXYGEN_INPUT@": f"{PROJECT_ROOT.as_posix()}/include {PROJECT_ROOT.as_posix()}/src/modcam",
        "@DOXYGEN_OUTPUT_DIRECTORY@": app.config.modcam_doxygen_output_dir,
        "@DOXYGEN_STRIP_FROM_PATH@": (PROJECT_ROOT).as_posix(),
    }
    generate_doxyfile(
        doxyfile_tmpl,
        doxyfile,
        doxyfile_find_and_replace,
    )
    run_doxygen(doxyfile)


def setup(app: sphinx.application) -> ExtensionMetadata:
    app.add_config_value(
        "modcam_doxygen_output_dir",
        (PROJECT_ROOT / "build" / "docs" / "doxygen").as_posix(),
        "env",
        [str],
        "Where to put the Doxygen output.",
    )
    app.config.breathe_projects = {
        "modCAM": Path(app.config.modcam_doxygen_output_dir) / "xml"
    }
    print(f"Placing Doxygen output in {app.config.modcam_doxygen_output_dir}")

    # Add hook for building doxygen xml when needed
    app.connect("builder-inited", generate_doxygen_xml)

    return {
        "version": version,
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
