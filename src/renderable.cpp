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

#define GL3_PROTOTYPES 1
#include <GL3/gl3.h>
#include <GL/freeglut.h>
#include <realtime_urdf_filter/renderable.h>
#include <resource_retriever/retriever.h>

#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

#include <ros/assert.h>

namespace realtime_urdf_filter
{
  // common methods
  void Renderable::setLinkName (std::string n)
  {
    name = n;
  }

  void Renderable::applyTransform ()
  {
    glPushMatrix ();

    tf::Transform transform (link_to_fixed);
    transform *= link_offset;
    tfScalar glTf[16];
    transform.getOpenGLMatrix(glTf);
    glMultMatrixd((GLdouble*)glTf);
  }

  void Renderable::unapplyTransform ()
  {
    glPopMatrix ();
  }

  // Sphere methods
  RenderableSphere::RenderableSphere (float radius)
    : radius(radius)
  {}

  void RenderableSphere::render ()
  {
    applyTransform ();
    glutSolidSphere(radius, 10, 10);
    unapplyTransform ();
  }

  // Cylinder methods
  RenderableCylinder::RenderableCylinder (float radius, float length)
    : radius(radius), length(length)
  {}

  void RenderableCylinder::render ()
  {
    applyTransform ();
    glTranslatef (0, 0, -length/2);
    glutSolidCylinder(radius, length, 10, 10);
    unapplyTransform ();
  }

  // Box methods
  RenderableBox::RenderableBox (float dimx, float dimy, float dimz)
    : dimx(dimx), dimy(dimy), dimz(dimz)
  {
    createBoxVBO ();
  }

  void RenderableBox::render ()
  {
    applyTransform ();

    glColor3f (color.r, color.g, color.b);

    // Enable Pointers
    glEnableClientState (GL_VERTEX_ARRAY);
    glEnableClientState (GL_NORMAL_ARRAY);

    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glVertexPointer (3, GL_FLOAT, 6*sizeof(GLfloat), (char *) NULL);
    glNormalPointer (GL_FLOAT, 6*sizeof(GLfloat), ((char *) NULL) + (3 * sizeof(GLfloat)) );

    // Render
    glDrawArrays (GL_QUADS, 0, 24);

    // Disable Pointers
    glDisableClientState( GL_VERTEX_ARRAY );          // Disable Vertex Arrays
    glDisableClientState( GL_NORMAL_ARRAY );       // Disable Texture Coord Arrays

    glScalef (dimx, dimy, dimz);
    glutSolidCube (dimx);
    unapplyTransform ();
  }

  void RenderableBox::createBoxVBO ()
  {
    const GLfloat boxvertices[] =
      {0.5f * dimx, 0.5f * dimy,-0.5f * dimz, 0.0f, 1.0f, 0.0f,          // Top Right Of The Quad (Top)
      -0.5f * dimx, 0.5f * dimy,-0.5f * dimz, 0.0f, 1.0f, 0.0f,          // Top Left Of The Quad (Top)
      -0.5f * dimx, 0.5f * dimy, 0.5f * dimz, 0.0f, 1.0f, 0.0f,          // Bottom Left Of The Quad (Top)
       0.5f * dimx, 0.5f * dimy, 0.5f * dimz, 0.0f, 1.0f, 0.0f,          // Bottom Right Of The Quad (Top)

       0.5f * dimx,-0.5f * dimy, 0.5f * dimz, 0.0f,-1.0f, 0.0f,          // Top Right Of The Quad (Bottom)
      -0.5f * dimx,-0.5f * dimy, 0.5f * dimz, 0.0f,-1.0f, 0.0f,          // Top Left Of The Quad (Bottom)
      -0.5f * dimx,-0.5f * dimy,-0.5f * dimz, 0.0f,-1.0f, 0.0f,          // Bottom Left Of The Quad (Bottom)
       0.5f * dimx,-0.5f * dimy,-0.5f * dimz, 0.0f,-1.0f, 0.0f,          // Bottom Right Of The Quad (Bottom)

       0.5f * dimx, 0.5f * dimy, 0.5f * dimz, 0.0f, 0.0f, 1.0f,          // Top Right Of The Quad (Front)
      -0.5f * dimx, 0.5f * dimy, 0.5f * dimz, 0.0f, 0.0f, 1.0f,          // Top Left Of The Quad (Front)
      -0.5f * dimx,-0.5f * dimy, 0.5f * dimz, 0.0f, 0.0f, 1.0f,          // Bottom Left Of The Quad (Front)
       0.5f * dimx,-0.5f * dimy, 0.5f * dimz, 0.0f, 0.0f, 1.0f,          // Bottom Right Of The Quad (Front)

       0.5f * dimx,-0.5f * dimy,-0.5f * dimz, 0.0f, 0.0f,-1.0f,          // Top Right Of The Quad (Back)
      -0.5f * dimx,-0.5f * dimy,-0.5f * dimz, 0.0f, 0.0f,-1.0f,          // Top Left Of The Quad (Back)
      -0.5f * dimx, 0.5f * dimy,-0.5f * dimz, 0.0f, 0.0f,-1.0f,          // Bottom Left Of The Quad (Back)
       0.5f * dimx, 0.5f * dimy,-0.5f * dimz, 0.0f, 0.0f,-1.0f,          // Bottom Right Of The Quad (Back)

      -0.5f * dimx, 0.5f * dimy, 0.5f * dimz,-1.0f, 0.0f, 0.0f,          // Top Right Of The Quad (Left)
      -0.5f * dimx, 0.5f * dimy,-0.5f * dimz,-1.0f, 0.0f, 0.0f,          // Top Left Of The Quad (Left)
      -0.5f * dimx,-0.5f * dimy,-0.5f * dimz,-1.0f, 0.0f, 0.0f,          // Bottom Left Of The Quad (Left)
      -0.5f * dimx,-0.5f * dimy, 0.5f * dimz,-1.0f, 0.0f, 0.0f,          // Bottom Right Of The Quad (Left)

       0.5f * dimx, 0.5f * dimy,-0.5f * dimz, 1.0f, 0.0f, 0.0f,          // Top Right Of The Quad (Right)
       0.5f * dimx, 0.5f * dimy, 0.5f * dimz, 1.0f, 0.0f, 0.0f,          // Top Left Of The Quad (Right)
       0.5f * dimx,-0.5f * dimy, 0.5f * dimz, 1.0f, 0.0f, 0.0f,          // Bottom Left Of The Quad (Right)
       0.5f * dimx,-0.5f * dimy,-0.5f * dimz, 1.0f, 0.0f, 0.0f};         // Bottom Right Of The Quad (Right)


    glGenBuffers (1, &vbo);
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glBufferData (GL_ARRAY_BUFFER, 24 * 6 * sizeof(GLfloat), boxvertices, GL_STATIC_DRAW);
  }

  // these classes are copied from RVIZ. TODO: header/license/author tags
  class ResourceIOStream : public Assimp::IOStream
  {
  public:
    ResourceIOStream(const resource_retriever::MemoryResource& res)
    : res_(res)
    , pos_(res.data.get())
    {}

    ~ResourceIOStream()
    {}

    size_t Read(void* buffer, size_t size, size_t count)
    {
      size_t to_read = size * count;
      if (pos_ + to_read > res_.data.get() + res_.size)
      {
        to_read = res_.size - (pos_ - res_.data.get());
      }

      memcpy(buffer, pos_, to_read);
      pos_ += to_read;

      return to_read;
    }

    size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

    aiReturn Seek( size_t offset, aiOrigin origin)
    {
      uint8_t* new_pos = 0;
      switch (origin)
      {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
        break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
        break;
      default:
        ROS_BREAK();
      }

      if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
      {
        return aiReturn_FAILURE;
      }

      pos_ = new_pos;
      return aiReturn_SUCCESS;
    }

    size_t Tell() const
    {
      return pos_ - res_.data.get();
    }

    size_t FileSize() const
    {
      return res_.size;
    }

    void Flush() {}

  private:
    resource_retriever::MemoryResource res_;
    uint8_t* pos_;
  };

  class ResourceIOSystem : public Assimp::IOSystem
  {
  public:
    ResourceIOSystem()
    {
    }

    ~ResourceIOSystem()
    {
    }

    // Check whether a specific file exists
    bool Exists(const char* file) const
    {
      // Ugly -- two retrievals where there should be one (Exists + Open)
      // resource_retriever needs a way of checking for existence
      // TODO: cache this
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever_.get(file);
      }
      catch (resource_retriever::Exception& e)
      {
        return false;
      }

      return true;
    }

    // Get the path delimiter character we'd like to see
    char getOsSeparator() const
    {
      return '/';
    }

    // ... and finally a method to open a custom stream
    Assimp::IOStream* Open(const char* file, const char* mode)
    {
      ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

      // Ugly -- two retrievals where there should be one (Exists + Open)
      // resource_retriever needs a way of checking for existence
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever_.get(file);
      }
      catch (resource_retriever::Exception& e)
      {
        return 0;
      }

      return new ResourceIOStream(res);
    }

    void Close(Assimp::IOStream* stream) { delete stream; }

  private:
    mutable resource_retriever::Retriever retriever_;
  };

  RenderableMesh::RenderableMesh (std::string meshname, float sx, float sy, float sz) :
    meshname_(meshname),
    scale_x(sx),
    scale_y(sy),
    scale_z(sz)
  {
    Assimp::Importer importer;
    importer.SetIOHandler(new ResourceIOSystem());
    const aiScene* scene = importer.ReadFile(meshname, aiProcess_PreTransformVertices|aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
    if (!scene)
    {
      ROS_ERROR("Could not load resource [%s]: %s", meshname.c_str(), importer.GetErrorString());
      return;
    }

    fromAssimpScene(scene);
  }

  RenderableMesh::SubMesh::SubMesh ()
  {
    vbo = INVALID_VALUE;
    ibo = INVALID_VALUE;
    num_indices = 0;
  }

  RenderableMesh::SubMesh::~SubMesh ()
  {
    if (vbo != INVALID_VALUE)
      glDeleteBuffers (1, &vbo);
    if (ibo != INVALID_VALUE)
      glDeleteBuffers (1, &ibo);
  }

  void RenderableMesh::SubMesh::init (const std::vector<Vertex>& vertices,
                                      const std::vector<unsigned int>& indices)
  {
    num_indices = indices.size ();
    glGenBuffers (1, &vbo);
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glBufferData (GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size (), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers (1, &ibo);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, &indices[0], GL_STATIC_DRAW);
  }

  void RenderableMesh::fromAssimpScene (const aiScene* scene)
  {
    meshes.resize (scene->mNumMeshes);
    for (unsigned int i = 0; i < meshes.size (); ++i)
    {
      const aiMesh* mesh = scene->mMeshes[i];
      initMesh (i, mesh, scene->mRootNode);
    }
  }

  void RenderableMesh::initMesh (unsigned int index, const aiMesh* mesh, const aiNode* node)
  {
    // TODO: mesh->mMaterialIndex
    // TODO: const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    // Make sure we have a root node
    if (!node) {
      return;
    }
    // We need to fix the orientation
    ROS_DEBUG_STREAM("Parsing mesh: "<<meshname_);
    aiMatrix4x4 transform = node->mTransformation;
    aiMatrix3x3 rotation(transform);

    ROS_DEBUG_STREAM("  transform: "<<std::endl
        <<std::fixed
        <<"[ "<<*transform[0]<<" \t"<<*transform[1]<<" \t"<<*transform[2]<<" \t"<<*transform[3]<<std::endl
        <<"  "<<*transform[4]<<" \t"<<*transform[5]<<" \t"<<*transform[6]<<" \t"<<*transform[7]<<std::endl
        <<"  "<<*transform[8]<<" \t"<<*transform[9]<<" \t"<<*transform[10]<<" \t"<<*transform[11]<<std::endl
        <<"  "<<*transform[12]<<" \t"<<*transform[13]<<" \t"<<*transform[14]<<" \t"<<*transform[15]<<std::endl);
    aiNode *pnode = node->mParent;


    // Add the verticies
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
    {
      aiVector3D pos = mesh->mVertices[i];
      aiVector3D n = mesh->mNormals[i];
      // TODO: const aiVector3D* pTexCoord = mesh->HasTextureCoords(0) ? &(mesh->mTextureCoords[0][i]) : &Zero3D;

      // FIXME: Some .dae files are not processed properly by this function
      // Transform the positions and normals appropriately
      // The following does not do the right transform
      // pos *= transform;
      // n = rotation*n;

      // Add a vertex / normal pair
      Vertex v(pos.x, pos.y, pos.z, n.x, n.y, n.z);
      vertices.push_back (v);
    }

    for (unsigned int i = 0 ; i < mesh->mNumFaces ; ++i)
    {
        const aiFace& face = mesh->mFaces[i];
        assert(face.mNumIndices == 3);
        indices.push_back(face.mIndices[0]);
        indices.push_back(face.mIndices[1]);
        indices.push_back(face.mIndices[2]);
    }

    meshes[index].init (vertices, indices);
  }

  void RenderableMesh::setScale (float x, float y, float z)
  {
    scale_x = x;
    scale_y = y;
    scale_z = z;
  }

  void RenderableMesh::render ()
  {
    applyTransform ();
    glScalef (scale_x, scale_y, scale_z);
    glEnableVertexAttribArray (0);
    glEnableVertexAttribArray (2);

    for (unsigned int i = 0 ; i < meshes.size() ; i++)
    {
      glBindBuffer (GL_ARRAY_BUFFER, meshes[i].vbo);
      glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), 0);
      glVertexAttribPointer (2, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), (const GLvoid*) (sizeof(float)*3));

      glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, meshes[i].ibo);

//      const unsigned int MaterialIndex = meshes[i].MaterialIndex;
//
//      if (MaterialIndex < textures.size() && textures[MaterialIndex])
//      {
//        textures[MaterialIndex]->Bind (GL_TEXTURE0);
//      }

      glDrawElements (GL_TRIANGLES, meshes[i].num_indices, GL_UNSIGNED_INT, 0);
    }

    glDisableVertexAttribArray (0);
    glDisableVertexAttribArray (2);
    unapplyTransform ();
  }


}


