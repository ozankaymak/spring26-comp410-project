cmake_minimum_required(VERSION 3.20)
project(hyperbolica LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)
find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
pkg_check_modules(GLFW3 REQUIRED IMPORTED_TARGET glfw3)
pkg_check_modules(GLM REQUIRED IMPORTED_TARGET glm)
pkg_check_modules(IMGUI_PKG QUIET IMPORTED_TARGET imgui)

option(HYPER_ENABLE_WARNINGS "Enable extra compiler warnings for project targets" ON)
add_library(project_warnings INTERFACE)

if(HYPER_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(project_warnings INTERFACE /W4)
    else()
        target_compile_options(project_warnings INTERFACE
            -Wall
            -Wextra
            -Wpedantic
        )
    endif()
endif()

add_library(glad
    external/glad/src/glad.cpp
)
target_include_directories(glad PUBLIC external/glad/include)
target_link_libraries(glad PUBLIC OpenGL::GL)
target_compile_definitions(glad PUBLIC GL_SILENCE_DEPRECATION)

add_library(hyperbolica_math
    src/math/hyperbolic.cpp
)
target_include_directories(hyperbolica_math PUBLIC src)
target_link_libraries(hyperbolica_math PUBLIC project_warnings)

add_library(hyperbolica_tiling
    src/tiling_core.cpp
    src/tiling_minimap.cpp
)
target_include_directories(hyperbolica_tiling PUBLIC src)
target_link_libraries(hyperbolica_tiling PUBLIC hyperbolica_math PkgConfig::GLM project_warnings)

set(HYPER_HAS_IMGUI 0)
set(HYPER_IMGUI_TARGET "")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/imgui.cpp")
    add_library(imgui
        external/imgui/imgui.cpp
        external/imgui/imgui_draw.cpp
        external/imgui/imgui_tables.cpp
        external/imgui/imgui_widgets.cpp
        external/imgui/backends/imgui_impl_glfw.cpp
        external/imgui/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        external/imgui
        external/imgui/backends
    )
    target_link_libraries(imgui PUBLIC glad PkgConfig::GLFW3)
    set(HYPER_HAS_IMGUI 1)
    set(HYPER_IMGUI_TARGET imgui)
elseif(IMGUI_PKG_FOUND)
    set(HYPER_HAS_IMGUI 1)
    set(HYPER_IMGUI_TARGET PkgConfig::IMGUI_PKG)
endif()

add_library(hyperbolica_render
    src/shader.cpp
    src/mesh.cpp
)
target_include_directories(hyperbolica_render PUBLIC src)
target_compile_definitions(hyperbolica_render PUBLIC HYPER_HAS_IMGUI=${HYPER_HAS_IMGUI})
target_link_libraries(hyperbolica_render
    PUBLIC
        glad
        PkgConfig::GLM
    PRIVATE
        project_warnings
)
if(HYPER_HAS_IMGUI)
    target_link_libraries(hyperbolica_render PUBLIC ${HYPER_IMGUI_TARGET})
endif()

add_executable(hyperbolica
    src/main.cpp
    src/app.cpp
)
target_include_directories(hyperbolica PRIVATE src)
target_link_libraries(hyperbolica
    PRIVATE
        hyperbolica_tiling
        hyperbolica_render
        PkgConfig::GLFW3
        project_warnings
)

if(BUILD_TESTING)
    add_executable(hyperbolica_smoke
        tests/smoke.cpp
    )
    target_link_libraries(hyperbolica_smoke PRIVATE project_warnings)
    add_test(NAME hyperbolica_smoke COMMAND hyperbolica_smoke)

    add_executable(hyperbolica_math_tests
        tests/math_tests.cpp
    )
    target_link_libraries(hyperbolica_math_tests PRIVATE hyperbolica_math project_warnings)
    add_test(NAME hyperbolica_math_tests COMMAND hyperbolica_math_tests)

    add_executable(hyperbolica_tiling_tests
        tests/tiling_tests.cpp
    )
    target_link_libraries(hyperbolica_tiling_tests PRIVATE hyperbolica_tiling project_warnings)
    add_test(NAME hyperbolica_tiling_tests COMMAND hyperbolica_tiling_tests)

    add_executable(hyperbolica_render_tests
        tests/render_tests.cpp
    )
    target_compile_definitions(hyperbolica_render_tests PRIVATE HYPER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(hyperbolica_render_tests PRIVATE hyperbolica_render project_warnings)
    add_test(NAME hyperbolica_render_tests COMMAND hyperbolica_render_tests)
endif()
