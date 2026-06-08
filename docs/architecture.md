# Architecture

Current source layout:

```text
main.cpp
  -> app.cpp
       -> tiling_core.cpp
       -> shader.cpp
       -> mesh.cpp
       -> src/math/hyperbolic.cpp
       -> src/math/spherical.cpp

tests/math_tests.cpp
  -> src/math/hyperbolic.cpp

tests/spherical_tests.cpp
  -> src/math/spherical.cpp

tests/tiling_tests.cpp
  -> tiling_core.cpp

tests/render_tests.cpp
  -> shader.cpp
  -> mesh.cpp
```

`src/math/spherical.cpp` (namespace `hyper::math::sphere`) is the spherical
analogue of the hyperbolic math layer. It reuses the shared data types but works
on the unit 2-sphere. `src/math/geometry_mode.h` defines the `GeometryMode`
flag the tiling and app layers use to dispatch between the two paths; the
hyperbolic API itself is unchanged.

`main.cpp` is just startup and error reporting.

`app.cpp` creates the GLFW window, handles input, generates the current tiling
patch, updates the camera matrix, binds the shader, and draws the uploaded
mesh. The camera frame is stored in the current tile's local coordinates.
When movement crosses a tile side, the app rebases the frame through the linked
neighbor and converts back to global coordinates only for rendering and status
output.

`shader.cpp` reads GLSL files, compiles shaders, links a program, and sets
uniforms.

`mesh.cpp` contains the OpenGL buffer wrapper and a small triangle helper used
by render tests.

`src/math/hyperbolic.cpp` is independent from rendering. It handles the
hyperboloid model, distance, projections, and regular tiling metrics.

`src/tiling_core.cpp` uses the math layer to build finite `{p,q}` patches from
base polygon vertices and side reflections. It also owns tile-side crossing
detection in the Klein disk and the local-frame rebasing operation used by the
demo.

The important boundary is that math code does not include OpenGL or GLFW. The
app/rendering code can use GLAD, GLM, GLFW, and OpenGL.
