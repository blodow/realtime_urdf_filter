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

#ifndef REALTIME_PERCEPTION_RENDERABLE_H_
#define REALTIME_PERCEPTION_RENDERABLE_H_

#include <GL3/gl3.h>
#include <tf/tf.h>

#include <urdf_exception/exception.h>
#include <urdf_model/color.h>

#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/scene.h>
#else
#include <assimp/aiScene.h>
#endif


// forward declare
struct aiScene;
struct aiMesh;

namespace realtime_urdf_filter
{

struct Renderable
{
  void setLinkName (std::string n);
  virtual void render () = 0;
  std::string name;

//  tf::Vector3 offset_t;
//  tf::Quaternion offset_q;
//  tf::Vector3 t;
//  tf::Quaternion q;

  tf::Transform link_offset;
  tf::Transform link_to_fixed;
  tf::Transform fixed_to_target;

  urdf::Color color;
  void applyTransform ();
  void unapplyTransform ();
};

struct RenderableBox : public Renderable
{
  RenderableBox (float dimx, float dimy, float dimz);

  virtual void render ();

  float dimx, dimy, dimz;
protected:
  void createBoxVBO ();
  GLuint vbo;
};

struct RenderableSphere : public Renderable
{
  RenderableSphere (float radius);

  virtual void render ();

  float radius;
};

struct RenderableCylinder : public Renderable
{
  RenderableCylinder (float radius, float length);

  virtual void render ();

  float radius;
  float length;
};

// meshes are a tad more complicated than boxes and spheres
struct RenderableMesh : public Renderable
{
  RenderableMesh (std::string meshname, float sx, float sy, float sz);

  virtual void render ();
  void setScale (float x, float y, float z);

private:
  struct Vertex
  {
    float x,y,z;
    float nx,ny,nz;
    Vertex (float x, float y, float z, float nx, float ny, float nz)
      : x(x), y(y), z(z), nx(nx), ny(ny), nz(nz)
    {}
  };

  struct SubMesh
  {
    enum {INVALID_VALUE = 0xFFFFFFFF};
    SubMesh ();
    ~SubMesh ();
    void init (const std::vector<Vertex>& vertices,
               const std::vector<unsigned int>& indices);
    GLuint vbo, ibo;
    unsigned int num_indices;
  };

  void fromAssimpScene (const aiScene* scene);
  void initMesh (unsigned int i, const aiMesh* mesh, const aiNode* node);

  std::string meshname_;

  std::vector<SubMesh> meshes;
  double scale_x;
  double scale_y;
  double scale_z;
};



} // end namespace

#endif

