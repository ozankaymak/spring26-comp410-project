# HyperGL

Interactive 3D curved-space rendering experiment in C++17.

Current pieces:

- hyperbolic-plane math utilities using the hyperboloid model
- an experimental spherical (2-sphere) mode that mirrors the hyperbolic pipeline
- regular `{p,q}` tiling metric helpers for both curvatures
- tile-local crossing and rebasing for walking across generated tiles
- a GLFW/OpenGL perspective renderer for embedded tilings
- static and camera-local minimaps
- small demo props: houses, center-start laser loops, and minimap laser paths
- atmospheric background shading
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

Runtime controls:

- `WASD` or arrow keys: move across the current tiling patch
- Mouse: look around in first person (when captured)
- `Space` / `Shift`: raise/lower the camera height
- `1`: render hyperbolic `{4,6}` with six squares meeting at each corner
- `2`: render hyperbolic `{3,7}` with seven triangles meeting at each corner
- `3`: render hyperbolic `{5,4}` with four pentagons meeting at each corner
- `4`: render hyperbolic `{7,3}` with three heptagons meeting at each corner
- `5`: render spherical `{4,3}` (the cube)
- `6`: render spherical `{3,4}` (the octahedron)
- `7`: render spherical `{5,3}` (the dodecahedron)
- `8`: render spherical `{3,5}` (the icosahedron)
- `[` / `-`: decrease movement speed
- `]` / `=`: increase movement speed
- `Q` / `E`: zoom in/out by changing perspective field of view
- `F1`: show or hide the debug overlay
- `G`: toggle floor grid
- `F`: toggle wireframe
- `M`: toggle minimaps
- `H`: toggle house props
- `L`: toggle center-start laser loops and minimap laser paths (`q` turns at `360 / p` degrees for `{p,q}`)
- `C` / `V`: decrease/increase edge segments
- `B` / `N`: decrease/increase radial bands
- `Esc`: release or recapture the mouse

The window title reports the active geometry, tiling parameters, current tile,
tile depth, origin distance, speed, zoom, camera height, and generated tile
count. The built-in debug overlay mirrors the current runtime settings and
avoids an external ImGui dependency.

## Layout

- `src/`: app, rendering, and math source files
- `shaders/`: GLSL shader sources
- `tests/`: CTest executables and small shared helpers
- `docs/`: geometry, testing, and architecture notes
- `external/`: vendored GLAD loader and unused ImGui sources

## Notes

- [Architecture](docs/architecture.md)
- [Testing](docs/testing.md)
- [Hyperbolic Geometry](docs/hyperbolic_geometry.md)
- [Spherical Geometry](docs/spherical_geometry.md)
