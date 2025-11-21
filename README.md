# modCAM

[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/modCAM/modcam/badge)](https://scorecard.dev/viewer/?uri=github.com/modCAM/modcam)

ModCAM is free and open source software for computer aided manufacturing (CAM) research. It is a library of algorithms for the control of manufacturing machinery, such as CNC lathes and mills and 3D printers.

## The vision

ModCAM's goal is to make the development of new CAM algorithms easier by providing a library of existing algorithms to build from and by giving researchers a place to share their code. The goal is to automate additive and, especially, subtractive manufacturing, making these fields more accessible for both professionals and hobbyists.

## Features

### Algorithms

At this early stage, there's not much here. You can help by [contributing](docs/CONTRIBUTING.rst) algorithms that you want. Planned features include

  - algorithms for 2- to 5-axis machines
  - manufacturability analysis
  - workpiece setup
  - machining feature extraction
  - roughing machining tool path planning
  - finish machining tool path planning
  - machine control/G-code generation

### Bindings to other languages

Not everyone loves to write, compile, and (inevitably) debug C++ code, so modCAM has Python bindings to enable easier prototyping of your own algorithms (which we hope you [share](docs/CONTRIBUTING.rst) with the community). If you want to add other language bindings it would be greatly appreciated! Please open an [issue](https://github.com/modCAM/modcam/issues) pointing to your bindings.

## Documentation

For tutorials and further information about the project in general, see the project main page on [readthedocs]().

For a technical reference guide to the modCAM functions, see the [C++ reference page]().

## Getting started

To use modCAM in your own project, see the [example project](https://github.com/modCAM/cmake-example), which shows you how to build a CMake-based project that retrieves modCAM with FetchContent.

## Build from source

ModCAM has the following dependencies:

- [Eigen](https://libeigen.gitlab.io/)
- [libigl](https://libigl.github.io/)
- [doctest](https://github.com/doctest/doctest) (unit tests)
- [Doxygen](https://www.doxygen.nl/) (documentation)
- [Sphinx](https://www.sphinx-doc.org/) (documentation)

The CMake presets supplied with this project use [vcpkg](https://vcpkg.io/) to install the dependencies.

To build and install modCAM, do

```
git clone 
cd modcam
cmake -S . -D CMAKE_BUILD_TYPE=Release --preset build-minimal
cmake --build build/
cmake --install build/ --prefix build/modcam
```

For more build options, see [CONTRIBUTING.rst](docs/CONTRIBUTING.rst).

## Communication channels

Questions about usage can be asked in [Discussions](https://github.com/modCAM/modcam/discussions).

For bug reports or enhancement suggestions, create an [Issue](https://github.com/modCAM/modcam/issues).

To report vulnerabilities, see [SECURITY.md](https://github.com/modCAM/modcam?tab=security-ov-file).

For private communication, email modCAM.github@proton.me.

All interactions with the modCAM community are expected to follow the [Code of conduct](https://github.com/modCAM/modcam?tab=coc-ov-file#).

## Contributing

Enhancements and bug fixes are welcome. For instructions on how to contribute, see [CONTRIBUTING.rst](docs/CONTRIBUTING.rst).

## License

ModCAM is licensed under the [Mozilla Public License Version 2.0](LICENSE).
