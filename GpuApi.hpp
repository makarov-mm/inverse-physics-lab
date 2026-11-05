#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <gl/GL.h>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace inverse {
constexpr GLenum SSBO = 0x90D2, COMPUTE = 0x91B9, VERTEX = 0x8B31, FRAGMENT = 0x8B30;
constexpr GLbitfield STORAGE_BARRIER = 0x2000, READBACK_BARRIER = 0x200;
template <class T> T load(const char *name) {
    auto p = wglGetProcAddress(name);
    auto address = reinterpret_cast<std::intptr_t>(p);
    if (address == 0 || address == 1 || address == 2 || address == 3 || address == -1)
        throw std::runtime_error(std::string("Missing OpenGL entry point: ") + name);
    return reinterpret_cast<T>(p);
}
#define FN(ret, name, ...)                                                                                   \
    using name##T = ret(APIENTRY *)(__VA_ARGS__);                                                            \
    name##T name {}
class GL {
  public:
    FN(GLuint, CreateShader, GLenum);
    FN(void, ShaderSource, GLuint, GLsizei, const char *const *, const GLint *);
    FN(void, CompileShader, GLuint);
    FN(void, GetShaderiv, GLuint, GLenum, GLint *);
    FN(void, GetShaderInfoLog, GLuint, GLsizei, GLsizei *, char *);
    FN(void, DeleteShader, GLuint);
    FN(GLuint, CreateProgram);
    FN(void, AttachShader, GLuint, GLuint);
    FN(void, LinkProgram, GLuint);
    FN(void, GetProgramiv, GLuint, GLenum, GLint *);
    FN(void, GetProgramInfoLog, GLuint, GLsizei, GLsizei *, char *);
    FN(void, DeleteProgram, GLuint);
    FN(void, UseProgram, GLuint);
    FN(void, GenBuffers, GLsizei, GLuint *);
    FN(void, DeleteBuffers, GLsizei, const GLuint *);
    FN(void, BindBuffer, GLenum, GLuint);
    FN(void, BufferData, GLenum, std::ptrdiff_t, const void *, GLenum);
    FN(void, GetBufferSubData, GLenum, std::ptrdiff_t, std::ptrdiff_t, void *);
    FN(void, BufferSubData, GLenum, std::ptrdiff_t, std::ptrdiff_t, const void *);
    FN(void, BindBufferBase, GLenum, GLuint, GLuint);
    FN(void, GenVertexArrays, GLsizei, GLuint *);
    FN(void, BindVertexArray, GLuint);
    FN(void, EnableVertexAttribArray, GLuint);
    FN(void, VertexAttribPointer, GLuint, GLint, GLenum, GLboolean, GLsizei, const void *);
    FN(void, DeleteVertexArrays, GLsizei, const GLuint *);
    FN(void, DispatchCompute, GLuint, GLuint, GLuint);
    FN(void, MemoryBarrier, GLbitfield);
    FN(void, DrawArraysInstanced, GLenum, GLint, GLsizei, GLsizei);
    FN(GLint, GetUniformLocation, GLuint, const char *);
    FN(void, Uniform1i, GLint, GLint);
    FN(void, Uniform1f, GLint, GLfloat);
    FN(void, Uniform3f, GLint, GLfloat, GLfloat, GLfloat);
    FN(void, UniformMatrix4fv, GLint, GLsizei, GLboolean, const GLfloat *);
    FN(void, GenQueries, GLsizei, GLuint *);
    FN(void, DeleteQueries, GLsizei, const GLuint *);
    FN(void, BeginQuery, GLenum, GLuint);
    FN(void, EndQuery, GLenum);
    FN(void, GetQueryObjectuiv, GLuint, GLenum, GLuint *);
    FN(void, GetQueryObjectui64v, GLuint, GLenum, unsigned long long *);
    FN(void, GenFramebuffers, GLsizei, GLuint *);
    FN(void, DeleteFramebuffers, GLsizei, const GLuint *);
    FN(void, BindFramebuffer, GLenum, GLuint);
    FN(void, FramebufferTexture2D, GLenum, GLenum, GLenum, GLuint, GLint);
    FN(GLenum, CheckFramebufferStatus, GLenum);
    FN(void, BlitFramebuffer, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);
    HWND window{};
    HDC dc{};
    HGLRC context{};
    GLuint vao{};
    std::string driver;
    std::vector<GLuint> programs, buffers;
    GL() = default;
    GL(const GL &) = delete;
    GL &operator=(const GL &) = delete;
    ~GL() {
        if (context) {
            wglMakeCurrent(dc, context);
            for (auto p : programs)
                DeleteProgram(p);
            if (!buffers.empty())
                DeleteBuffers(static_cast<GLsizei>(buffers.size()), buffers.data());
            if (vao)
                DeleteVertexArrays(1, &vao);
            wglMakeCurrent(nullptr, nullptr);
            wglDeleteContext(context);
        }
        if (dc)
            ReleaseDC(window, dc);
    }
    void initialize(HWND hwnd) {
        window = hwnd;
        dc = GetDC(hwnd);
        PIXELFORMATDESCRIPTOR p{};
        p.nSize = sizeof(p);
        p.nVersion = 1;
        p.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        p.iPixelType = PFD_TYPE_RGBA;
        p.cColorBits = 32;
        p.cDepthBits = 24;
        p.cAlphaBits = 8;
        int format = ChoosePixelFormat(dc, &p);
        if (!format || !SetPixelFormat(dc, format, &p))
            throw std::runtime_error("Cannot choose a depth-enabled OpenGL pixel format.");
        context = wglCreateContext(dc);
        if (!context || !wglMakeCurrent(dc, context))
            throw std::runtime_error("WGL bootstrap failed.");
        auto create = load<HGLRC(WINAPI *)(HDC, HGLRC, const int *)>("wglCreateContextAttribsARB");
        int attr[] = {0x2091, 4, 0x2092, 3, 0x9126, 1, 0};
        auto modern = create(dc, nullptr, attr);
        if (!modern)
            throw std::runtime_error("Inverse Physics Lab requires OpenGL 4.3 Core with compute shaders.");
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(context);
        context = modern;
        if (!wglMakeCurrent(dc, context))
            throw std::runtime_error("WGL context activation failed.");
#define LOAD(name) name = load<name##T>("gl" #name)
        LOAD(CreateShader);
        LOAD(ShaderSource);
        LOAD(CompileShader);
        LOAD(GetShaderiv);
        LOAD(GetShaderInfoLog);
        LOAD(DeleteShader);
        LOAD(CreateProgram);
        LOAD(AttachShader);
        LOAD(LinkProgram);
        LOAD(GetProgramiv);
        LOAD(GetProgramInfoLog);
        LOAD(DeleteProgram);
        LOAD(UseProgram);
        LOAD(GenBuffers);
        LOAD(DeleteBuffers);
        LOAD(BindBuffer);
        LOAD(BufferData);
        LOAD(GetBufferSubData);
        LOAD(BufferSubData);
        LOAD(BindBufferBase);
        LOAD(GenVertexArrays);
        LOAD(BindVertexArray);
        LOAD(EnableVertexAttribArray);
        LOAD(VertexAttribPointer);
        LOAD(DeleteVertexArrays);
        LOAD(DispatchCompute);
        LOAD(MemoryBarrier);
        LOAD(DrawArraysInstanced);
        LOAD(GetUniformLocation);
        LOAD(Uniform1i);
        LOAD(Uniform1f);
        LOAD(Uniform3f);
        LOAD(UniformMatrix4fv);
        LOAD(GenQueries);
        LOAD(DeleteQueries);
        LOAD(BeginQuery);
        LOAD(EndQuery);
        LOAD(GetQueryObjectuiv);
        LOAD(GetQueryObjectui64v);
        LOAD(GenFramebuffers);
        LOAD(DeleteFramebuffers);
        LOAD(BindFramebuffer);
        LOAD(FramebufferTexture2D);
        LOAD(CheckFramebufferStatus);
        LOAD(BlitFramebuffer);
#undef LOAD
        driver = std::string(reinterpret_cast<const char *>(glGetString(GL_VERSION))) + " | " +
                 reinterpret_cast<const char *>(glGetString(GL_RENDERER));
        GenVertexArrays(1, &vao);
        BindVertexArray(vao);
        try {
            load<BOOL(WINAPI *)(int)>("wglSwapIntervalEXT")(1);
        } catch (const std::runtime_error &) {
        }
        glEnable(GL_DEPTH_TEST);
    }
    GLuint program(std::initializer_list<std::pair<GLenum, std::string>> sources, const char *label) {
        GLuint p = CreateProgram();
        std::vector<GLuint> shaders;
        try {
            for (const auto &[stage, source] : sources) {
                GLuint s = CreateShader(stage);
                shaders.push_back(s);
                const char *data = source.c_str();
                ShaderSource(s, 1, &data, nullptr);
                CompileShader(s);
                GLint ok{};
                GetShaderiv(s, 0x8B81, &ok);
                if (!ok) {
                    char log[8192]{};
                    GetShaderInfoLog(s, sizeof(log), nullptr, log);
                    throw std::runtime_error(std::string(label) + ": " + log);
                }
                AttachShader(p, s);
            }
            LinkProgram(p);
            GLint ok{};
            GetProgramiv(p, 0x8B82, &ok);
            if (!ok) {
                char log[8192]{};
                GetProgramInfoLog(p, sizeof(log), nullptr, log);
                throw std::runtime_error(std::string(label) + ": " + log);
            }
            for (auto s : shaders)
                DeleteShader(s);
            programs.push_back(p);
            return p;
        } catch (...) {
            for (auto s : shaders)
                DeleteShader(s);
            DeleteProgram(p);
            throw;
        }
    }
    GLuint buffer(GLuint binding, std::ptrdiff_t bytes) {
        GLuint b{};
        GenBuffers(1, &b);
        buffers.push_back(b);
        BindBuffer(SSBO, b);
        BufferData(SSBO, bytes, nullptr, 0x88E8);
        BindBufferBase(SSBO, binding, b);
        return b;
    }
    void integer(GLuint p, const char *name, int v) {
        Uniform1i(GetUniformLocation(p, name), v);
    }
    void scalar(GLuint p, const char *name, float v) {
        Uniform1f(GetUniformLocation(p, name), v);
    }
    void vector(GLuint p, const char *name, float x, float y, float z) {
        Uniform3f(GetUniformLocation(p, name), x, y, z);
    }
    void matrix(GLuint p, const char *name, const float *data) {
        UniformMatrix4fv(GetUniformLocation(p, name), 1, GL_FALSE, data);
    }
    void dispatch(GLuint count) {
        DispatchCompute((count + 127) / 128, 1, 1);
        MemoryBarrier(STORAGE_BARRIER);
    }
};
#undef FN
} // namespace inverse
