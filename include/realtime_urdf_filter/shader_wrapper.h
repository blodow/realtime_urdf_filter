#ifndef REALTIME_PERCEPTION_SHADER_WRAPPER_H_
#define REALTIME_PERCEPTION_SHADER_WRAPPER_H_

#include <fstream>
#include <stdexcept>
#define GL3_PROTOTYPES 1
#include <GL3/gl3.h>

namespace realtime_urdf_filter
{

// this class wraps loading, compiling and enabling shader programs
class ShaderWrapper
{

  public:
    // named constructor to compile from source
    template <int L1, int L2>
    static ShaderWrapper fromSource (GLchar const * (&v_source) [L1], GLchar const * (&f_source) [L2]);

    // named constructor to compile from files
    static ShaderWrapper fromFiles (const std::string vertex_file, const std::string fragment_file);
    static ShaderWrapper fromFiles (const char* vertex_file, const char* fragment_file);

    // make sure we delete everything upon deconstruction
    ~ShaderWrapper();

    // operator to get back the encapsulated program handle
    operator GLuint();

    // call operator enables the shader to be used in gl drawing calls
    void operator() ();
    void SetUniformVal1i(std::string name, GLint val);

  private:
    // templated constructor takes two char* arrays for vertex and fragment shader source code
    template <int L1, int L2>
    ShaderWrapper (GLchar const * (&v_source) [L1], GLchar const * (&f_source) [L2]);

    // compile function is templated on the number of lines in the shader
    template <int L>
    GLuint compile (GLuint type, char const * (&shader_source) [L]);

    // loads a text file as a string
    static std::string load_text_file (std::string file_name);

    // handles to the shaders and the linked program
    GLuint vertex_shader, fragment_shader, prog;
};

} // end namespace

#endif
