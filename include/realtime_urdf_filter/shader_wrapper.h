/*
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
    void SetUniformVal1f(std::string name, GLfloat val);

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
