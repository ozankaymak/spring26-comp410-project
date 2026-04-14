#pragma once

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void* (*GLADloadproc)(const char* name);

int gladLoadGLLoader(GLADloadproc load);

#ifdef __cplusplus
}
#endif

