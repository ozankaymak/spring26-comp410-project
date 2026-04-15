# Hyperbolica

Interactive curved-space rendering experiment in C++17.

Current pieces:

- hyperboloid-model math utilities
- regular `{p,q}` tiling metric helpers
- tile-local crossing and rebasing for walking across generated tiles
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

Progress-demo controls:

- `WASD` or arrow keys: move across the `{4,6}` patch
- Mouse drag: look around (when captured)
- `[` / `-`: decrease movement speed
- `]` / `=`: increase movement speed
- `Q` / `E`: zoom out/in
- `Esc`: release mouse and toggle Debug Settings Panel

The window title reports the current tile, tile depth, origin distance, speed,
zoom, and generated tile count. The ImGui Debug Settings panel offers sliders
for fog density, speed adjustment, tiling radial bands and geodesic subdivision,
and toggles for showing the grid and wireframe models.

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
- [Progress Demo Smoke Checklist](docs/progress_demo_smoke_checklist.md)
- [Progress Report Draft](docs/progress_report_draft.md)
