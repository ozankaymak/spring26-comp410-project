# Architecture

Current source layout:

```text
main.cpp
  -> app.cpp
       -> tiling_core.cpp
       -> shader.cpp
       -> mesh.cpp
       -> src/math/hyperbolic.cpp

tests/math_tests.cpp
  -> src/math/hyperbolic.cpp

tests/tiling_tests.cpp
  -> tiling_core.cpp

tests/render_tests.cpp
  -> shader.cpp
  -> mesh.cpp
```

`main.cpp` is just startup and error reporting.

`app.cpp` creates the GLFW window, handles input, generates the current tiling
patch, updates the camera matrix, binds the shader, and draws the uploaded
mesh.

`shader.cpp` reads GLSL files, compiles shaders, links a program, and sets
uniforms.

`mesh.cpp` contains the OpenGL buffer wrapper and a small triangle helper used
by render tests.

`src/math/hyperbolic.cpp` is independent from rendering. It handles the
hyperboloid model, distance, projections, and regular tiling metrics.

`src/tiling_core.cpp` uses the math layer to build finite `{p,q}` patches from
base polygon vertices and side reflections.

The important boundary is that math code does not include OpenGL or GLFW. The
app/rendering code can use GLAD, GLM, GLFW, and OpenGL.
