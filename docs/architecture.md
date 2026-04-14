# Architecture

Current source layout:

```text
main.cpp
  -> app.cpp
       -> shader.cpp
       -> mesh.cpp
       -> src/math/hyperbolic.cpp

tests/math_tests.cpp
  -> src/math/hyperbolic.cpp

tests/render_tests.cpp
  -> shader.cpp
  -> mesh.cpp
```

`main.cpp` is just startup and error reporting.

`app.cpp` creates the GLFW window, handles input, clears the frame, updates the
camera matrix, binds the shader, and draws the current mesh.

`shader.cpp` reads GLSL files, compiles shaders, links a program, and sets
uniforms.

`mesh.cpp` contains the current triangle data and the OpenGL buffer wrapper.

`src/math/hyperbolic.cpp` is independent from rendering. It handles the
hyperboloid model, distance, projections, and regular tiling metrics.

The important boundary is that math code does not include OpenGL or GLFW. The
app/rendering code can use GLAD, GLM, GLFW, and OpenGL.
