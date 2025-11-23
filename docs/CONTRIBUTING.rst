************
Contributing
************

Welcome, and thank you for your interest in making a difference! ModCAM relies on bug fixes and new algorithms from contributors like you.

If you have a bug fix or an enhancement you'd like to make, this document will guide you through the process. Here you'll find

* :ref:`what sort of contributions we do and don't want <project-scope>`
* :ref:`where to find information and ask questions <comms-channels>`
* :ref:`how to contribute <how-to-contribute>`
* :ref:`how to style your code <style-guide>`

Please review these guidelines and the code of conduct before making your contribution so that we can maintain consistent, high quality code.

.. _project-scope:

Project scope
=============

The focus of modCAM is the control of machine tools for manufacturing. We welcome algorithms for additive and subtractive manufacturing, such as workpiece setup or tool path planning. Plant management operations, such as scheduling and logistics, fall outside the scope of modCAM.

.. _comms-channels:

Communication channels
======================

Questions about usage can be asked in `Discussions`_.

For bug reports or enhancement suggestions, create an `Issue`_.

For private communication, email modCAM.github@proton.me.

.. _Discussions: https://github.com/modCAM/modcam/discussions

.. _Issue: https://github.com/modCAM/modcam/issues

.. _how-to-contribute:

How to
======

Submit a bug report or suggest an enhancement
---------------------------------------------

If you find a bug, or you want to suggest an enhancement, please start by opening an `issue`_ to raise awareness. However, before opening an issue...

* Make sure that you are using the latest version.
* Read the documentation carefully to check if the functionality already exists.
* Perform a search to see if the enhancement or bug fix has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.
* For new features, make sure your idea fits within the :ref:`scope <project-scope>` of the project.

If you want to work on the improvements yourself, thank you! You can fork the repository, make your changes, and open a `pull request`_.

.. _issue: https://github.com/modCAM/modcam/issues

.. _pull request: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork

Compile and test the code
-------------------------

There is a `development container <https://github.com/modCAM/devcontainer>`__ with everything you need pre-installed, though it is not a requirement to use the container. To run the container::

	docker pull ghcr.io/modcam/devcontainer:latest
	docker run --rm -it ghcr.io/modcam/devcontainer:latest

ModCAM has the following dependencies:

- `Eigen <https://libeigen.gitlab.io/>`__
- `libigl <https://libigl.github.io/>`__
- `doctest <https://github.com/doctest/doctest>`__ (unit tests)
- `Doxygen <https://www.doxygen.nl/>`__ (documentation)
- `Breathe <https://breathe.readthedocs.io/>`__ (documentation)
- `Sphinx <https://www.sphinx-doc.org/>`__ (documentation)

The CMake presets supplied with this project use `vcpkg <https://vcpkg.io/>`__ to install the C++ dependencies.

You can use the following presets and CMake options to configure your build:

+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Description                         | Preset                 | Equivalent options                                                             |
+=====================================+========================+==========================+==========================+==========================+
| Build only the library              | build-minimal          | N/A                                                                            |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Build the library with unit tests   | build-tests            | MODCAM_ENABLE_TESTING=ON                                                       |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Build the library and documentation | build-docs             | MODCAM_BUILD_DOC=ON                                                            |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Unit tests and static analysis      | linting                | MODCAM_ENABLE_TESTING=ON | MODCAM_ENABLE_LINTING=ON                            |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Build library with tests and docs   | build-all              | MODCAM_ENABLE_TESTING=ON | MODCAM_BUILD_DOC=ON                                 |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Tests, docs, and static analysis    | build-all-with-linting | MODCAM_BUILD_DOC=ON      | MODCAM_ENABLE_TESTING=ON | MODCAM_ENABLE_LINTING=ON |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+
| Only the documentation, no library  | build-docs-only        | MODCAM_BUILD_DOC=ON      | MODCAM_BUILD_PROJECT=OFF                            |
+-------------------------------------+------------------------+--------------------------+--------------------------+--------------------------+

As an example, to build modCAM and run the unit tests, do ::

	cmake -S . -D CMAKE_BUILD_TYPE=Release --preset build-tests
	cmake --build build/
	ctest --test-dir build/

Build the documentation
-----------------------

ModCAM uses Python-based tools to build the documentation. The necessary Python packages are in requirements.txt. To build the docs::

	python -m venv .venv                  # Create a Python virtual environment
	source .venv/bin/activate             # Activate the environment
	pip install -r requirements.txt       # Install the necessary Python packages
	cmake -S . --preset build-docs-only   # Tell CMake you want to build only the docs
	cmake --build build --config Release  # Build the docs

.. _style-guide:

Style guide
===========

Code style and quality are enforced as much as possible with `clang-format`_ and `clang-tidy`_.

Note that the static analysis checks defined in .clang-tidy are meant to be helpful and not a burden. If you think that one of the checks is overly burdensome, then please open an issue to discuss removing it.

Below are the style guidelines that cannot be enforced with clang-format.

.. _clang-format: https://clang.llvm.org/docs/ClangFormat.html

.. _clang-tidy: https://clang.llvm.org/extra/clang-tidy/

Code organization
-----------------

The code is organized according to the `Pitchfork Layout`_. The major points of the Pitchfork Layout are as follows:

* The ``src/`` directory contains the project source (.cpp) files and private header files. This directory also contains unit tests, which are denoted by the ``.test.cpp`` suffix.
* The ``include/`` directory contains the public header (.h) files.
* The ``tools/`` directory contains development utilities, such as Python scripts, CMake scripts, and packaging utilities.
* The ``data/`` directory contains project data files, such as images or data files needed by unit tests.
* Documentation is in the ``docs/`` directory.
* The ``build/`` directory is reserved for build artifacts.
* Namespaces should match directories. For example, the namespaced function ``foo::bar::baz()`` should be declared in ``include/foo/bar/baz.h``.

.. _Pitchfork Layout: https://github.com/vector-of-bool/pitchfork

Functions
---------

Function and variable names should use `underscore style`_.

When possible, `use return values`_ rather than output parameters. When output parameters are necessary, they should come before inputs in the function declaration so that default input values can be specified. For example: ``void foo(Out_Type1 output1, Out_Type2 output2, const In_Type1 input1, const In_Type2 input2)``

Since there can be multiple approaches to the same problem, functions should be suffixed with the first three letters of the lead author's surname and the year of publication (similar to some inline citation styles). For example: ``principal_curvature_rus2004``

If you're writing a new implementation of an existing function, then try to match the existing function's interface. For example, if implementing a new principal curvature function, try to match the interface of ``modcam::mesh::principal_curvature_rus2004``. This will make it easier for users to swap out different algorithms.

.. _underscore style: https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rl-camel

.. _use return values: https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rf-out

Documentation
-------------

Functions should be documented using `Doxygen`_ in the `Javadoc style`_. The documentation should follow `Diataxis reference principles`_. Namely, it should be a neutral description of what the function does and how to use it. It should not attempt to instruct or explain why. How-to guides and tutorials cover instruction and explanation elsewhere.

Citations are strongly encouraged. Citation information should go in the BibTeX file docs/modCAM.bib. They can be referenced with the Doxygen `@cite`_ command.

.. _Doxygen: https://www.doxygen.nl/

.. _Javadoc style: https://www.doxygen.nl/manual/docblocks.html

.. _Diataxis reference principles: https://diataxis.fr/reference/

.. _@cite: https://www.doxygen.nl/manual/commands.html#cmdcite

Commit messages
---------------

Git commit messages should follow the usual formatting rules. From https://cbea.ms/git-commit/:

* Separate subject from body with a blank line
* Limit the subject line to 50 characters
* Capitalize the subject line
* Do not end the subject line with a period
* Use the imperative mood in the subject line
* Wrap the body at 72 characters
* Use the body to explain what and why vs. how

See the following resources for more information:

* https://cbea.ms/git-commit/
* https://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html
* https://git-scm.com/book/en/v2/Distributed-Git-Contributing-to-a-Project#_commit_guidelines
