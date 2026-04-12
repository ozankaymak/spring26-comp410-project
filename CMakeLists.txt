cmake_minimum_required(VERSION 3.20)
project(hyperbolica LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

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

add_library(hyperbolica_math
    src/math/hyperbolic.cpp
)
target_include_directories(hyperbolica_math PUBLIC src)
target_link_libraries(hyperbolica_math PUBLIC project_warnings)

add_executable(hyperbolica
    src/main.cpp
    src/app.cpp
)
target_include_directories(hyperbolica PRIVATE src)
target_link_libraries(hyperbolica PRIVATE hyperbolica_math project_warnings)

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
endif()
