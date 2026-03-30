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

add_executable(hyperbolica
    src/main.cpp
    src/app.cpp
)
target_include_directories(hyperbolica PRIVATE src)
target_link_libraries(hyperbolica PRIVATE project_warnings)

if(BUILD_TESTING)
    add_executable(hyperbolica_smoke
        tests/smoke.cpp
    )
    target_link_libraries(hyperbolica_smoke PRIVATE project_warnings)
    add_test(NAME hyperbolica_smoke COMMAND hyperbolica_smoke)
endif()
