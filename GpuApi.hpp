#pragma once
// WGL bootstrap and the OpenGL 4.3 Core entry points the lab uses, loaded directly through
// wglGetProcAddress. No GLAD/GLEW: the enumerants missing from the Windows SDK's OpenGL 1.1
// header are listed in namespace inverse::gl below.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <gl/GL.h>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace inverse::gl {
// Shader objects
constexpr GLenum COMPUTE_SHADER = 0x91B9, VERTEX_SHADER = 0x8B31, FRAGMENT_SHADER = 0x8B30;
constexpr GLenum COMPILE_STATUS = 0x8B81, LINK_STATUS = 0x8B82;
// Buffers
constexpr GLenum ARRAY_BUFFER = 0x8892, UNIFORM_BUFFER = 0x8A11, SHADER_STORAGE_BUFFER = 0x90D2;
constexpr GLenum STATIC_DRAW = 0x88E4, DYNAMIC_DRAW = 0x88E8, DYNAMIC_COPY = 0x88EA;
// Textures and images
constexpr GLenum TEXTURE0 = 0x84C0, TEXTURE_3D = 0x806F, TEXTURE_WRAP_R = 0x8072;
constexpr GLenum RGBA8 = 0x8058, RGBA32F = 0x8814;
constexpr GLenum READ_WRITE = 0x88BA, WRITE_ONLY = 0x88B9;
// Framebuffers
constexpr GLenum FRAMEBUFFER = 0x8D40, READ_FRAMEBUFFER = 0x8CA8, DRAW_FRAMEBUFFER = 0x8CA9;
constexpr GLenum COLOR_ATTACHMENT0 = 0x8CE0, FRAMEBUFFER_COMPLETE = 0x8CD5;
// Rasterization and queries
constexpr GLenum PROGRAM_POINT_SIZE = 0x8642;
constexpr GLenum TIME_ELAPSED = 0x88BF, QUERY_RESULT = 0x8866, QUERY_RESULT_AVAILABLE = 0x8867;
// glMemoryBarrier bits
constexpr GLbitfield VERTEX_ATTRIB_ARRAY_BARRIER_BIT = 0x1, TEXTURE_FETCH_BARRIER_BIT = 0x8,
                     SHADER_IMAGE_ACCESS_BARRIER_BIT = 0x20, TEXTURE_UPDATE_BARRIER_BIT = 0x100,
                     BUFFER_UPDATE_BARRIER_BIT = 0x200, SHADER_STORAGE_BARRIER_BIT = 0x2000;
// WGL_ARB_create_context
constexpr int CONTEXT_MAJOR_VERSION_ARB = 0x2091, CONTEXT_MINOR_VERSION_ARB = 0x2092,
              CONTEXT_PROFILE_MASK_ARB = 0x9126, CONTEXT_CORE_PROFILE_BIT_ARB = 0x1;
} // namespace inverse::gl

namespace inverse {
    /// One shader stage of a program: its type and the source chunks concatenated by the compiler.
    struct ShaderStage {
        GLenum type;
        std::vector<std::string_view> sources;
    };

    /// Owns the window's OpenGL 4.3 Core context and exposes the loaded entry points as members
    /// named after the GL functions without the "gl" prefix (gpu.DispatchCompute, ...).
    class GpuApi {
        template <class T> static T load(const char* name) {
            auto pointer = wglGetProcAddress(name);
            auto address = reinterpret_cast<std::intptr_t>(pointer);
            // Some drivers return 1, 2, 3 or -1 for missing entry points instead of null.
            if (address == 0 || address == 1 || address == 2 || address == 3 || address == -1)
                throw std::runtime_error(std::string("Missing OpenGL entry point: ") + name);
            return reinterpret_cast<T>(pointer);
        }

    public:
#define LAB_GL_FUNCTION(ret, name, ...) \
    using name##Function = ret(APIENTRY *)(__VA_ARGS__); \
    name##Function name {}
        LAB_GL_FUNCTION(GLuint, CreateShader, GLenum);
        LAB_GL_FUNCTION(void, ShaderSource, GLuint, GLsizei, const char* const*, const GLint*);
        LAB_GL_FUNCTION(void, CompileShader, GLuint);
        LAB_GL_FUNCTION(void, GetShaderiv, GLuint, GLenum, GLint*);
        LAB_GL_FUNCTION(void, GetShaderInfoLog, GLuint, GLsizei, GLsizei*, char*);
        LAB_GL_FUNCTION(void, DeleteShader, GLuint);
        LAB_GL_FUNCTION(GLuint, CreateProgram);
        LAB_GL_FUNCTION(void, AttachShader, GLuint, GLuint);
        LAB_GL_FUNCTION(void, LinkProgram, GLuint);
        LAB_GL_FUNCTION(void, GetProgramiv, GLuint, GLenum, GLint*);
        LAB_GL_FUNCTION(void, GetProgramInfoLog, GLuint, GLsizei, GLsizei*, char*);
        LAB_GL_FUNCTION(void, DeleteProgram, GLuint);
        LAB_GL_FUNCTION(void, UseProgram, GLuint);
        LAB_GL_FUNCTION(void, GenBuffers, GLsizei, GLuint*);
        LAB_GL_FUNCTION(void, DeleteBuffers, GLsizei, const GLuint*);
        LAB_GL_FUNCTION(void, BindBuffer, GLenum, GLuint);
        LAB_GL_FUNCTION(void, BufferData, GLenum, std::ptrdiff_t, const void*, GLenum);
        LAB_GL_FUNCTION(void, BufferSubData, GLenum, std::ptrdiff_t, std::ptrdiff_t, const void*);
        LAB_GL_FUNCTION(void, GetBufferSubData, GLenum, std::ptrdiff_t, std::ptrdiff_t, void*);
        LAB_GL_FUNCTION(void, BindBufferBase, GLenum, GLuint, GLuint);
        LAB_GL_FUNCTION(void, GenVertexArrays, GLsizei, GLuint*);
        LAB_GL_FUNCTION(void, DeleteVertexArrays, GLsizei, const GLuint*);
        LAB_GL_FUNCTION(void, BindVertexArray, GLuint);
        LAB_GL_FUNCTION(void, EnableVertexAttribArray, GLuint);
        LAB_GL_FUNCTION(void, VertexAttribPointer, GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
        LAB_GL_FUNCTION(void, DispatchCompute, GLuint, GLuint, GLuint);
        LAB_GL_FUNCTION(void, MemoryBarrier, GLbitfield);
        LAB_GL_FUNCTION(void, GenQueries, GLsizei, GLuint*);
        LAB_GL_FUNCTION(void, DeleteQueries, GLsizei, const GLuint*);
        LAB_GL_FUNCTION(void, BeginQuery, GLenum, GLuint);
        LAB_GL_FUNCTION(void, EndQuery, GLenum);
        LAB_GL_FUNCTION(void, GetQueryObjectuiv, GLuint, GLenum, GLuint*);
        LAB_GL_FUNCTION(void, GetQueryObjectui64v, GLuint, GLenum, std::uint64_t*);
        LAB_GL_FUNCTION(void, GenFramebuffers, GLsizei, GLuint*);
        LAB_GL_FUNCTION(void, DeleteFramebuffers, GLsizei, const GLuint*);
        LAB_GL_FUNCTION(void, BindFramebuffer, GLenum, GLuint);
        LAB_GL_FUNCTION(void, FramebufferTexture2D, GLenum, GLenum, GLenum, GLuint, GLint);
        LAB_GL_FUNCTION(GLenum, CheckFramebufferStatus, GLenum);
        LAB_GL_FUNCTION(void, BlitFramebuffer, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield,
            GLenum);
        LAB_GL_FUNCTION(void, ActiveTexture, GLenum);
        LAB_GL_FUNCTION(void, TexImage3D, GLenum, GLint, GLint, GLsizei, GLsizei, GLsizei, GLint, GLenum, GLenum,
            const void*);
        LAB_GL_FUNCTION(void, TexSubImage3D, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei,
            GLenum, GLenum, const void*);
        LAB_GL_FUNCTION(void, BindImageTexture, GLuint, GLuint, GLint, GLboolean, GLint, GLenum, GLenum);
        LAB_GL_FUNCTION(GLint, GetUniformLocation, GLuint, const char*);
        LAB_GL_FUNCTION(void, Uniform1i, GLint, GLint);
#undef LAB_GL_FUNCTION

        HDC dc{};
        std::string driver; // "GL_VERSION | GL_RENDERER", for reports
        bool vsync = false; // whether wglSwapIntervalEXT(1) was accepted

        GpuApi() = default;
        GpuApi(const GpuApi&) = delete;
        GpuApi& operator=(const GpuApi&) = delete;
        ~GpuApi() {
            shutdown();
        }

        /// Creates an OpenGL 4.3 Core context on the window and loads every entry point above.
        void initialize(HWND hwnd) {
            window = hwnd;
            dc = GetDC(hwnd);
            PIXELFORMATDESCRIPTOR descriptor{};
            descriptor.nSize = sizeof(descriptor);
            descriptor.nVersion = 1;
            descriptor.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
            descriptor.iPixelType = PFD_TYPE_RGBA;
            descriptor.cColorBits = 32;
            descriptor.cAlphaBits = 8;
            // The lab draws no depth-tested geometry, so no depth buffer is requested.
            int format = ChoosePixelFormat(dc, &descriptor);
            if (!format || !SetPixelFormat(dc, format, &descriptor))
                throw std::runtime_error("Cannot choose a double-buffered OpenGL pixel format.");
            // A legacy context is needed only to reach wglCreateContextAttribsARB.
            HGLRC legacy = wglCreateContext(dc);
            if (!legacy || !wglMakeCurrent(dc, legacy))
                throw std::runtime_error("WGL bootstrap failed.");
            HGLRC modern = nullptr;
            try {
                auto createContextAttribs =
                    load<HGLRC(WINAPI*)(HDC, HGLRC, const int*)>("wglCreateContextAttribsARB");
                const int attributes[] = { gl::CONTEXT_MAJOR_VERSION_ARB, 4, gl::CONTEXT_MINOR_VERSION_ARB, 3,
                                          gl::CONTEXT_PROFILE_MASK_ARB, gl::CONTEXT_CORE_PROFILE_BIT_ARB, 0 };
                modern = createContextAttribs(dc, nullptr, attributes);
            }
            catch (const std::runtime_error&) {
            }
            wglMakeCurrent(nullptr, nullptr);
            wglDeleteContext(legacy);
            if (!modern)
                throw std::runtime_error("Inverse Physics Lab requires OpenGL 4.3 Core with compute shaders.");
            context = modern;
            if (!wglMakeCurrent(dc, context))
                throw std::runtime_error("WGL context activation failed.");
#define LAB_GL_LOAD(name) name = load<name##Function>("gl" #name)
            LAB_GL_LOAD(CreateShader);
            LAB_GL_LOAD(ShaderSource);
            LAB_GL_LOAD(CompileShader);
            LAB_GL_LOAD(GetShaderiv);
            LAB_GL_LOAD(GetShaderInfoLog);
            LAB_GL_LOAD(DeleteShader);
            LAB_GL_LOAD(CreateProgram);
            LAB_GL_LOAD(AttachShader);
            LAB_GL_LOAD(LinkProgram);
            LAB_GL_LOAD(GetProgramiv);
            LAB_GL_LOAD(GetProgramInfoLog);
            LAB_GL_LOAD(DeleteProgram);
            LAB_GL_LOAD(UseProgram);
            LAB_GL_LOAD(GenBuffers);
            LAB_GL_LOAD(DeleteBuffers);
            LAB_GL_LOAD(BindBuffer);
            LAB_GL_LOAD(BufferData);
            LAB_GL_LOAD(BufferSubData);
            LAB_GL_LOAD(GetBufferSubData);
            LAB_GL_LOAD(BindBufferBase);
            LAB_GL_LOAD(GenVertexArrays);
            LAB_GL_LOAD(DeleteVertexArrays);
            LAB_GL_LOAD(BindVertexArray);
            LAB_GL_LOAD(EnableVertexAttribArray);
            LAB_GL_LOAD(VertexAttribPointer);
            LAB_GL_LOAD(DispatchCompute);
            LAB_GL_LOAD(MemoryBarrier);
            LAB_GL_LOAD(GenQueries);
            LAB_GL_LOAD(DeleteQueries);
            LAB_GL_LOAD(BeginQuery);
            LAB_GL_LOAD(EndQuery);
            LAB_GL_LOAD(GetQueryObjectuiv);
            LAB_GL_LOAD(GetQueryObjectui64v);
            LAB_GL_LOAD(GenFramebuffers);
            LAB_GL_LOAD(DeleteFramebuffers);
            LAB_GL_LOAD(BindFramebuffer);
            LAB_GL_LOAD(FramebufferTexture2D);
            LAB_GL_LOAD(CheckFramebufferStatus);
            LAB_GL_LOAD(BlitFramebuffer);
            LAB_GL_LOAD(ActiveTexture);
            LAB_GL_LOAD(TexImage3D);
            LAB_GL_LOAD(TexSubImage3D);
            LAB_GL_LOAD(BindImageTexture);
            LAB_GL_LOAD(GetUniformLocation);
            LAB_GL_LOAD(Uniform1i);
#undef LAB_GL_LOAD
            driver = std::string(reinterpret_cast<const char*>(glGetString(GL_VERSION))) + " | " +
                reinterpret_cast<const char*>(glGetString(GL_RENDERER));
            try {
                vsync = load<BOOL(WINAPI*)(int)>("wglSwapIntervalEXT")(1) != FALSE;
            }
            catch (const std::runtime_error&) {
                vsync = false;
            }
        }

        /// Releases the context. Safe to call more than once; GPU objects must be deleted before.
        void shutdown() {
            if (context) {
                wglMakeCurrent(nullptr, nullptr);
                wglDeleteContext(context);
                context = nullptr;
            }
            if (dc) {
                ReleaseDC(window, dc);
                dc = nullptr;
            }
        }

        /// Compiles and links a program; throws with the driver's log and the label on failure.
        GLuint program(std::initializer_list<ShaderStage> stages, const char* label) const {
            GLuint handle = CreateProgram();
            std::vector<GLuint> shaders;
            try {
                for (const auto& stage : stages) {
                    GLuint shader = CreateShader(stage.type);
                    shaders.push_back(shader);
                    std::vector<const char*> chunks;
                    std::vector<GLint> lengths;
                    for (auto source : stage.sources) {
                        chunks.push_back(source.data());
                        lengths.push_back(static_cast<GLint>(source.size()));
                    }
                    ShaderSource(shader, static_cast<GLsizei>(chunks.size()), chunks.data(), lengths.data());
                    CompileShader(shader);
                    GLint ok{};
                    GetShaderiv(shader, gl::COMPILE_STATUS, &ok);
                    if (!ok)
                        throw std::runtime_error(std::string(label) + ": " + infoLog(shader, GetShaderInfoLog));
                    AttachShader(handle, shader);
                }
                LinkProgram(handle);
                GLint ok{};
                GetProgramiv(handle, gl::LINK_STATUS, &ok);
                if (!ok)
                    throw std::runtime_error(std::string(label) + ": " + infoLog(handle, GetProgramInfoLog));
            }
            catch (...) {
                for (auto shader : shaders)
                    DeleteShader(shader);
                DeleteProgram(handle);
                throw;
            }
            // Shaders are reference-counted by the linked program and can be released immediately.
            for (auto shader : shaders)
                DeleteShader(shader);
            return handle;
        }

        /// Creates a buffer with `bytes` of storage and attaches it to an indexed binding point
        /// (shader storage or uniform block). The storage can later be re-specified with BufferData.
        GLuint buffer(GLenum target, GLuint binding, std::ptrdiff_t bytes, GLenum usage) const {
            GLuint handle{};
            GenBuffers(1, &handle);
            BindBuffer(target, handle);
            BufferData(target, bytes, nullptr, usage);
            BindBufferBase(target, binding, handle);
            return handle;
        }

    private:
        HWND window{};
        HGLRC context{};

        template <class Getter> static std::string infoLog(GLuint object, Getter getter) {
            std::string log(8192, '\0');
            GLsizei length{};
            getter(object, static_cast<GLsizei>(log.size()), &length, log.data());
            log.resize(static_cast<size_t>(length));
            return log;
        }
    };
}
