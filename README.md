# HyperGL

Interactive 3D curved-space rendering experiment in C++17.

Current pieces:

- hyperboloid-model math utilities
- regular `{p,q}` tiling metric helpers
- tile-local crossing and rebasing for walking across generated tiles
- a GLFW/OpenGL perspective renderer for H3-embedded tilings
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
- Mouse: look around in first person (when captured)
- `Space` / `Shift`: raise/lower the camera height
- `1`: render `{4,6}` with six squares meeting at each corner
- `2`: render `{3,7}` with seven triangles meeting at each corner
- `3`: render `{5,4}`
- `4`: render `{7,3}`
- `[` / `-`: decrease movement speed
- `]` / `=`: increase movement speed
- `Q` / `E`: zoom out/in by changing perspective field of view
- `F1`: show or hide the debug overlay
- `G`: toggle floor grid
- `F`: toggle wireframe
- `Z` / `X`: decrease/increase fog density
- `C` / `V`: decrease/increase edge segments
- `B` / `N`: decrease/increase radial bands
- `Esc`: release or recapture the mouse

The window title reports the current tile, tile depth, origin distance, speed,
zoom, and generated tile count. The built-in debug overlay mirrors the current
runtime settings and avoids an external ImGui dependency.

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
