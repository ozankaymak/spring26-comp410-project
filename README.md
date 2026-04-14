# Hyperbolica

Interactive curved-space rendering experiment in C++17.

Current pieces:

- hyperboloid-model math utilities
- regular `{p,q}` tiling metric helpers
- a GLFW/OpenGL application shell
- shader and mesh wrappers
- CTest tests

## Build

Dependencies: CMake, pkg-config, GLFW, GLM, and OpenGL.

On macOS with Homebrew:

```bash
brew install cmake pkg-config glfw glm
```

Then build:

```bash
cmake -S . -B build
cmake --build build
```

## Test

```bash
ctest --test-dir build --output-on-failure
```

## Run

```bash
./build/hyperbolica
```

## Layout

- `src/`: app, rendering, and math source files
- `shaders/`: GLSL shader sources
- `tests/`: CTest executables and small shared helpers
- `docs/`: geometry, testing, and architecture notes
- `external/`: local compatibility code

## Notes

- [Architecture](docs/architecture.md)
- [Testing](docs/testing.md)
- [Hyperbolic Geometry](docs/hyperbolic_geometry.md)
