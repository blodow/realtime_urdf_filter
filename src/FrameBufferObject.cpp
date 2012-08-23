//------------------------------------------------------------------------------
// File : FramebufferObject.h
//------------------------------------------------------------------------------
// Copyright (c) 2005 Gordon Wetzstein
//---------------------------------------------------------------------------
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any
// damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any
// purpose, including commercial applications, and to alter it and
// redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you
//    must not claim that you wrote the original software. If you use
//    this software in a product, an acknowledgment in the product
//    documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and
//    must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source
//    distribution.
//
// -----------------------------------------------------------------------------
//
// Credits:
// Original RenderTexture code: Mark J. Harris
//	parts of the code are used from the original RenderTexture  
//
// -----------------------------------------------------------------------------

#include "realtime_urdf_filter/FrameBufferObject.h"
#include <stdio.h>
#include <string.h>

using namespace std;

// -----------------------------------------------------------------------------

/// all formats for 8 bit data types rgba
int		_numInternal8AFormats		= 1;
GLenum	_internal8AColorFormats[]	= {	GL_RGBA8 };
/// all formats for 16 bit data types rgb
int		_numInternal8Formats = 1;
GLenum	_internal8ColorFormats[]	= {	GL_RGB8 };

/// all formats for 16 bit data types rgba
int			_numInternal16AFormats	= 3;
GLenum	_internal16AColorFormats[]	= {	GL_RGBA16F_ARB,
  GL_FLOAT_RGBA16_NV,
  GL_RGBA_FLOAT16_ATI	};

/// all formats for 16 bit data types rgb
int		_numInternal16Formats		= 3;
GLenum	_internal16ColorFormats[]	= {	GL_RGB16F_ARB,
  GL_FLOAT_RGB16_NV,
  GL_RGB_FLOAT16_ATI	};

/// all formats for 32 bit data types rgba
int		_numInternal32AFormats		= 3;
GLenum	_internal32AColorFormats[]	= {	GL_RGBA32F_ARB,
  GL_FLOAT_RGBA32_NV,
  GL_RGBA_FLOAT32_ATI	};

/// all formats for 32 bit data types rgb
int		_numInternal32Formats		= 3;
GLenum	_internal32ColorFormats[]	= {	GL_RGB32F_ARB,
  GL_FLOAT_RGB32_NV,
  GL_RGB_FLOAT32_ATI	};

GLenum _framebufferStatus;

// -----------------------------------------------------------------------------

FramebufferObject::FramebufferObject(const char *strMode) :

  _textureTarget(GL_TEXTURE_RECTANGLE_ARB),
  //_textureTarget(GL_TEXTURE_RECTANGLE_NV),

  _colorFormat(GL_RGBA),
  _internalColorFormat(GL_RGBA),
  _colorType(GL_UNSIGNED_BYTE),

  _colorAttachmentDepth(RGBA_8),

  _depthFormat(GL_DEPTH_COMPONENT),
  _internalDepthFormat(GL_DEPTH_COMPONENT24),
  _depthType(GL_UNSIGNED_BYTE),

  _wrapS(GL_CLAMP_TO_EDGE),
  _wrapT(GL_CLAMP_TO_EDGE),
  _minFilter(GL_LINEAR),
  _magFilter(GL_LINEAR),

  _width(512),
  _height(512),

  _bFBOSupported(false),
  _bInitialized(false),

  _bColorAttachment(false),
  _bColorAttachmentRenderTexture(false),

  _bDepthAttachment(false),
  _bDepthAttachmentRenderTexture(false),

  _bStencilAttachment(false),
  _bStencilAttachmentRenderTexture(false),
  _stencilDepth(1),

  // currently not used!
  _internalStencilFormat(GL_STENCIL_INDEX1),

  _bPassThroughProgramInitialized(false),

  _numColorAttachments(1),

  _bFloatColorBuffer(false)
{
  parseModeString(strMode);
}

// -----------------------------------------------------------------------------

FramebufferObject::~FramebufferObject() {

  if(_bInitialized) {

    if(_bColorAttachment) {

      for(int i=0; i<_numColorAttachments; i++) {
        if(_bColorAttachmentRenderTexture) 
          glDeleteTextures(1, &_colorAttachmentID[i]);
        else
          glDeleteRenderbuffers(1, &_colorAttachmentID[i]);
      }
    } 

    if(_bDepthAttachment) {
      if(_bDepthAttachmentRenderTexture) 
        glDeleteTextures(1, &_depthAttachmentID);
      else
        glDeleteRenderbuffers(1, &_depthAttachmentID);
    }

    glDeleteFramebuffers(1, &_frameBufferID);

  }

  if(_bPassThroughProgramInitialized)
    glDeleteProgramsARB(1, &_passThroughProgram); 
}

// -----------------------------------------------------------------------------

unsigned int
FramebufferObject::getWidth() {
  return _width;
}

// -----------------------------------------------------------------------------

unsigned int
FramebufferObject::getHeight() {
  return _height;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::bind(int index) {
  glEnable(_textureTarget);
  glBindTexture(_textureTarget, _colorAttachmentID[index]);
  glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
  glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
  glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
  glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::bindDepth(void) {
  glEnable(_textureTarget);
  glBindTexture(_textureTarget, _depthAttachmentID);
  glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
  glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
  glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
  glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
}

// -----------------------------------------------------------------------------

GLuint					
FramebufferObject::getColorAttachmentID(int index) {
  return _colorAttachmentID[index];
}

// -----------------------------------------------------------------------------

GLuint					
FramebufferObject::getDepthAttachmentID(void) {
  return _depthAttachmentID;
}

// -----------------------------------------------------------------------------

GLuint					
FramebufferObject::getStencilAttachmentID(void) {
  return _depthAttachmentID;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::setMinFilter(GLint	minFilter) {
  _minFilter = minFilter;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::setMagFilter(GLint	magFilter) {
  _magFilter = magFilter;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::setWrapS(GLint	wrapS) {
  _wrapS = wrapS;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::setWrapT(GLint	wrapT) {
  _wrapT = wrapT;
}

// -----------------------------------------------------------------------------

bool						
FramebufferObject::initialize(	unsigned int width, unsigned int height ) {

  if(_bInitialized) 
    return reInitialize( width, height );

  _width	= width;
  _height = height;


  //if (!glh_init_extensions(	"GL_ARB_fragment_program "
  //													"GL_ARB_vertex_program "
  //													"GL_NV_float_buffer "
  //													"GL_ARB_framebuffer_object "))
  //{
  //		printf("Unable to load the following extension(s): %s\n\nExiting...\n", 
  //						glh_get_unsupported_extensions());
  //		return false;
  //}

  if(!GLEW_EXT_framebuffer_object)
    cout << "ERROR: FramebufferObject - GL_EXT_framebuffer_object not supported!" << endl;

  if(!GLEW_EXT_packed_depth_stencil)
    cout << "ERROR: FramebufferObject - GL_EXT_packed_depth_stencil not supported!" << endl;

  if(!GLEW_EXT_multi_draw_arrays)
    cout << "ERROR: FramebufferObject - GLEW_EXT_multi_draw_arrays not supported!" << endl;

  if(!GLEW_ARB_framebuffer_object)
    cout << "ERROR: FramebufferObject - GL_ARB_framebuffer_object not supported!" << endl;

  if(!GLEW_ARB_texture_non_power_of_two)
    cout << "ERROR: FramebufferObject - GL_ARB_texture_non_power_of_two not supported!" << endl;


  //---------------------------------------------------------------------
  //	create pass-throug programs

  if(!_bPassThroughProgramInitialized) {

    glGenProgramsARB(1, &_passThroughProgram);

    const char* strPassThroughProgram = 
      "!!ARBfp1.0\n"
      "MOV result.color, fragment.color.primary;\n"
      "END\n";

    glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB, _passThroughProgram);
    glProgramStringARB(GL_FRAGMENT_PROGRAM_ARB, 
        GL_PROGRAM_FORMAT_ASCII_ARB,
        strlen(strPassThroughProgram), 
        strPassThroughProgram);

    _bPassThroughProgramInitialized = true;

  }

  glGetIntegerv(GL_MAX_DRAW_BUFFERS, &_maxNumDrawBuffers);
  if(_numColorAttachments > _maxNumDrawBuffers) {
    cout << "WARNING: FramebufferObject::initialize - this machine only supports up to " << _maxNumDrawBuffers << " draw buffers. Reset number of color attachments!" << endl; 
    _numColorAttachments = _maxNumDrawBuffers;
  }

  //---------------------------------------------------------------------

  glPushAttrib(GL_TEXTURE_BIT);

  // create FBO
  glGenFramebuffers(1, &_frameBufferID);
  glBindFramebuffer(GL_FRAMEBUFFER, _frameBufferID);

  if(_bColorAttachment) {

    if(_bColorAttachmentRenderTexture) {

      int		count		= 0;
      bool	success	= false;

      int			numFormats = 1;
      GLenum	*format;

      switch(_colorAttachmentDepth) {
        case RGBA_8:
          numFormats	= _numInternal8AFormats;
          format		= _internal8AColorFormats;
          break;
        case RGB_8:
          numFormats	= _numInternal8Formats;
          format		= _internal8ColorFormats;
          break;						
        case RGBA_16:
          numFormats	= _numInternal16AFormats;
          format		= _internal16AColorFormats;
          break;
        case RGB_16:
          numFormats	= _numInternal16Formats;
          format		= _internal16ColorFormats;
          break;	
        case RGBA_32:
          numFormats	= _numInternal32AFormats;
          format		= _internal32AColorFormats;
          break;
        case RGB_32:
          numFormats	= _numInternal32Formats;
          format		= _internal32ColorFormats;
          break;			
      }

      // initialize texture
      glEnable(_textureTarget);
      glGenTextures(1, &_colorAttachmentID[0]);

      do {

        _internalColorFormat = format[count];

        glBindTexture(_textureTarget, _colorAttachmentID[0]);

        glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
        glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
        glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
        glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

        glTexImage2D(	_textureTarget, 
            0, 
            _internalColorFormat, 
            _width, 
            _height, 
            0, 
            _colorFormat, 
            _colorType, 
            NULL);

        // attach texture to framebuffer color buffer
        glFramebufferTexture2D(	GL_FRAMEBUFFER, 
            GL_COLOR_ATTACHMENT0, 
            _textureTarget,
            _colorAttachmentID[0], 
            0);

        success = checkFramebufferStatus();
        count++;

      } while( !(success || (count > (numFormats-1))) );

      //----------------------------------------------------------
      // initialize multiple render targets

      for(int i=1; i<_numColorAttachments; i++) {

        glGenTextures(1, &_colorAttachmentID[i]);

        glBindTexture(_textureTarget, _colorAttachmentID[i]);

        glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
        glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
        glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
        glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

        glTexImage2D(	_textureTarget, 
            0, 
            _internalColorFormat, 
            _width, 
            _height, 
            0, 
            _colorFormat, 
            _colorType, 
            NULL);

        GLint colorAttachmentMacro=GL_COLOR_ATTACHMENT0;
        switch(i) {
          case 1:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT1;
            break;
          case 2:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT2;
            break;
          case 3:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT3;
            break;
          case 4:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT4;
            break;
          case 5:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT5;
            break;
          case 6:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT6;
            break;
          case 7:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT7;
            break;
          case 8:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT8;
            break;
          case 9:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT9;
            break;
          case 10:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT10;
            break;
          case 11:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT11;
            break;
          case 12:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT12;
            break;
          case 13:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT13;
            break;
          case 14:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT14;
            break;
          case 15:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT15;
            break;
        }

        // attach texture to framebuffer color buffer
        glFramebufferTexture2D(	GL_FRAMEBUFFER, 
            colorAttachmentMacro, 
            _textureTarget,
            _colorAttachmentID[i], 
            0);

        success = checkFramebufferStatus();
        GLenum err = glGetError();
        if(err != GL_NO_ERROR)
          printf("OpenGL ERROR after FBO initialization: %s\n", gluErrorString(err));

      }

      //----------------------------------------------------------

      if(!success) 
        printFramebufferStatus();

    } else {

      //----------------------------------------------------------
      // initialize multiple render targets

      for(int i=0; i<_numColorAttachments; i++) {

        glGenRenderbuffers(1, &_colorAttachmentID[i]);

        // initialize color renderbuffer
        glBindRenderbuffer(GL_RENDERBUFFER, _colorAttachmentID[i]);
        glRenderbufferStorage(	GL_RENDERBUFFER, 
            _internalColorFormat,
            _width, 
            _height);

        GLint colorAttachmentMacro=GL_COLOR_ATTACHMENT0;
        switch(i) {
          case 1:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT1;
            break;
          case 2:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT2;
            break;
          case 3:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT3;
            break;
          case 4:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT4;
            break;
          case 5:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT5;
            break;
          case 6:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT6;
            break;
          case 7:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT7;
            break;
          case 8:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT8;
            break;
          case 9:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT9;
            break;
          case 10:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT10;
            break;
          case 11:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT11;
            break;
          case 12:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT12;
            break;
          case 13:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT13;
            break;
          case 14:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT14;
            break;
          case 15:
            colorAttachmentMacro=GL_COLOR_ATTACHMENT15;
            break;
        }

        // attach renderbuffer to framebuffer color buffer
        glFramebufferRenderbuffer(	GL_FRAMEBUFFER,
            colorAttachmentMacro,
            GL_RENDERBUFFER,
            _colorAttachmentID[i] );

      }

      //----------------------------------------------------------

      if(!checkFramebufferStatus())
        std::cerr<<"ERROR: "<<__FILE__<<":"<<__LINE__<<std::endl;
        printFramebufferStatus();

    }

  }

  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  // depth attachment

  if(_bDepthAttachment) {

    if(_bStencilAttachment) {

      if(_internalDepthFormat != GL_DEPTH_COMPONENT24)
        cout << "WARNING: reset internal depth attachment format to 24 Bit [GL_DEPTH_STENCIL], otherwise stencil attachment won't work!" << endl;

      //std::cout<<"Using GL_DEPTH_COMPONENT24 for _internalDepthFormat."<<std::endl;
      _internalDepthFormat	= GL_DEPTH_COMPONENT24;
    }

    if(_bDepthAttachmentRenderTexture) {

      // initialize depth texture
      glGenTextures(1, &_depthAttachmentID);
      glBindTexture(_textureTarget, _depthAttachmentID);

      glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
      glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
      glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
      glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

      glTexImage2D(	_textureTarget, 
          0, 
          _internalDepthFormat, 
          _width, 
          _height, 
          0, 
          _depthFormat, 
          _depthType, 
          NULL);

      // attach texture to framebuffer color buffer
      glFramebufferTexture2D(
          GL_FRAMEBUFFER, 
          GL_DEPTH_ATTACHMENT, 
          _textureTarget,
          _depthAttachmentID, 0);

      if(!checkFramebufferStatus()) {
        std::cerr<<"ERROR: "<<__FILE__<<":"<<__LINE__<<std::endl;
        printFramebufferStatus();
      }

    } else {				

      glGenRenderbuffers(1, &_depthAttachmentID);

      // initialize depth renderbuffer
      glBindRenderbuffer(GL_RENDERBUFFER, _depthAttachmentID);
      glRenderbufferStorage(	GL_RENDERBUFFER, 
          _internalDepthFormat,
          _width, 
          _height);

      // attach renderbuffer to framebuffer depth buffer
      glFramebufferRenderbuffer(	GL_FRAMEBUFFER,
          GL_DEPTH_ATTACHMENT,
          GL_RENDERBUFFER,
          _depthAttachmentID );

      if(!checkFramebufferStatus()) {
        std::cerr<<"ERROR: "<<__FILE__<<":"<<__LINE__<<std::endl;
        printFramebufferStatus();
      }

    }

  }



  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  // stencil attachment

  if(_bStencilAttachment) {

    if(!_bDepthAttachment)
      cout << "ERROR: FBO stencil attachment is currently only supported in combination with a depth attachment: 24 Bit depth attachment + 8 Bit stencil attachment. See EXT_packed_depth_stencil specifications for details!" << endl;

    if(_bStencilAttachmentRenderTexture) {

      glBindTexture(_textureTarget, _depthAttachmentID);

      glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_S, _wrapS);
      glTexParameteri(_textureTarget, GL_TEXTURE_WRAP_T, _wrapT);
      glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER, _minFilter);
      glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER, _magFilter);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

      glTexImage2D(	_textureTarget, 
          0, 
          _internalDepthFormat, 
          _width, 
          _height, 
          0, 
          _depthFormat, 
          _depthType, 
          NULL);

      // glBindTexture( _textureTarget, 0);

      // attach texture to framebuffer color buffer
      glFramebufferTexture2D(
          GL_FRAMEBUFFER, 
          GL_DEPTH_ATTACHMENT, 
          _textureTarget,
          _depthAttachmentID, 0);

      if(!checkFramebufferStatus()) {
        std::cerr<<"ERROR: "<<__FILE__<<":"<<__LINE__<<std::endl;
        printFramebufferStatus();
      }

    } else {

      // initialize depth renderbuffer
      glBindRenderbuffer(GL_RENDERBUFFER, _depthAttachmentID);
      glRenderbufferStorage(	GL_RENDERBUFFER, 
          _internalDepthFormat,
          _width, 
          _height);

      // attach renderbuffer to framebuffer depth buffer
      glFramebufferRenderbuffer(	GL_FRAMEBUFFER,
          GL_STENCIL_ATTACHMENT,
          GL_RENDERBUFFER,
          _depthAttachmentID );

      if(!checkFramebufferStatus()) {
        std::cerr<<"ERROR: "<<__FILE__<<":"<<__LINE__<<std::endl;
        printFramebufferStatus();
      }

    }

  }


  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------

  // disable framebuffer again!
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  glPopAttrib();

  _bInitialized = true;
  _bFBOSupported = true;

  return true;
}

// -----------------------------------------------------------------------------

bool						
FramebufferObject::reInitialize(	unsigned int width, unsigned int height, const char *strMode ) {

  if(!_bInitialized) 
    return initialize( width, height );

  if(!_bFBOSupported)
    return false;

  // ---------------------------------------------------------
  // delete old configuration

  glDeleteFramebuffers(1, &_frameBufferID);

  if(_bColorAttachment) {
    if(_bColorAttachmentRenderTexture) 
      glDeleteTextures(1, &_colorAttachmentID[0]);
    else 
      glDeleteRenderbuffers(1, &_colorAttachmentID[0]);
  }

  if(_bDepthAttachment) {
    if(_bDepthAttachmentRenderTexture) 
      glDeleteTextures(1, &_depthAttachmentID);
    else
      glDeleteRenderbuffers(1, &_depthAttachmentID);
  }

  // ---------------------------------------------------------

  _bInitialized = false;
  parseModeString(strMode);
  initialize(width, height);

  return true;
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::beginCapture(bool bEnablePassThroughShader) {

  glGetIntegerv(GL_VIEWPORT, _viewport);
  glViewport(0, 0, _width, _height);

  if(_bInitialized)
    glBindFramebuffer(GL_FRAMEBUFFER, _frameBufferID);

  if(_bFloatColorBuffer && bEnablePassThroughShader) {
    glBindProgramARB(GL_FRAGMENT_PROGRAM_ARB, _passThroughProgram);
    glEnable(GL_FRAGMENT_PROGRAM_ARB);
  }		
}

// -----------------------------------------------------------------------------

void						
FramebufferObject::endCapture(bool bDisablePassThroughShader) {

  glViewport(_viewport[0], _viewport[1], _viewport[2], _viewport[3]);

  if(_bInitialized)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

  if(_bFloatColorBuffer && bDisablePassThroughShader) 
    glDisable(GL_FRAGMENT_PROGRAM_ARB);

}

// -----------------------------------------------------------------------------

void						
FramebufferObject::parseModeString(const char *modeString) {

  if (!modeString || strcmp(modeString, "") == 0)
    return;

  //int  iDepthBits		= 0;
  //bool bHasStencil	= false;
  //bool bBind2D		= false;
  //bool bBindRECT		= false;
  //bool bBindCUBE		= false;

  _bColorAttachment	= false;
  _bDepthAttachment	= false;
  _bStencilAttachment = false;

  _bFloatColorBuffer	= false;

  char *mode = strdup(modeString);

  vector<string> tokens;
  char *buf = strtok(mode, " ");
  while (buf != NULL)
  {
    tokens.push_back(buf);
    buf = strtok(NULL, " ");
  }


  for (unsigned int i = 0; i < tokens.size(); i++) {

    string token = tokens[i];
    KeyVal kv = getKeyValuePair(token);

    //cout << "Token: " << kv.first.c_str() << " = " << kv.second.c_str() << endl;

    // ------------------------------------------------
    // handle RGBA color attachment
    if ( strcmp(kv.first.c_str(), "rgba") == 0) {

      _bColorAttachment		= true;
      _colorFormat			= GL_RGBA;
      _internalColorFormat	= GL_RGBA;
      _colorType				= GL_UNSIGNED_BYTE;
      _colorAttachmentDepth	= RGBA_8;

      _minFilter				= GL_LINEAR;
      _magFilter				= GL_LINEAR;

      if( strchr(kv.second.c_str(), 't') != NULL ) {
        _bColorAttachmentRenderTexture = true;
      } else {
        _bColorAttachmentRenderTexture = false;
      }

      if( kv.second.find("16") != kv.second.npos ) {

        _colorAttachmentDepth = RGBA_16;

        _internalColorFormat	= GL_RGBA16F_ARB;
        _colorType				= GL_HALF_FLOAT_ARB;
        _bFloatColorBuffer		= true;

      }
      if( kv.second.find("32") != kv.second.npos ) {

        _colorAttachmentDepth = RGBA_32;

        _internalColorFormat	= GL_RGBA32F_ARB;
        _colorType				= GL_FLOAT;	

        // linear filter is not supported for 32 FBOs
        _minFilter				= GL_NEAREST;
        _magFilter				= GL_NEAREST;

        _bFloatColorBuffer		= true;
      }

      //---------------------------------------------	
      //	check for multiple render targets!

      if( kv.second.find("10x") != kv.second.npos ) {
        _numColorAttachments = 10;				
      } else if( kv.second.find("11x") != kv.second.npos ) {
        _numColorAttachments = 11;				
      } else if( kv.second.find("12x") != kv.second.npos ) {
        _numColorAttachments = 12;				
      } else if( kv.second.find("13x") != kv.second.npos ) {
        _numColorAttachments = 13;				
      } else if( kv.second.find("14x") != kv.second.npos ) {
        _numColorAttachments = 14;				
      } else if( kv.second.find("15x") != kv.second.npos ) {
        _numColorAttachments = 15;				
      } else if( kv.second.find("16x") != kv.second.npos ) {
        _numColorAttachments = 16;				
      } else if( kv.second.find("2x") != kv.second.npos ) {
        _numColorAttachments = 2;				
      } else if( kv.second.find("3x") != kv.second.npos ) {
        _numColorAttachments = 3;				
      } else if( kv.second.find("4x") != kv.second.npos ) {
        _numColorAttachments = 4;				
      } else if( kv.second.find("5x") != kv.second.npos ) {
        _numColorAttachments = 5;				
      } else if( kv.second.find("6x") != kv.second.npos ) {
        _numColorAttachments = 6;				
      } else if( kv.second.find("7x") != kv.second.npos ) {
        _numColorAttachments = 7;				
      } else if( kv.second.find("8x") != kv.second.npos ) {
        _numColorAttachments = 8;				
      } else if( kv.second.find("9x") != kv.second.npos ) {
        _numColorAttachments = 9;				
      }  

      //---------------------------------------------

      // ------------------------------------------------
      // ------------------------------------------------
      // handle RGB color attachment
    } else if ( strcmp(kv.first.c_str(), "rgb") == 0) {

      _bColorAttachment			= true;
      _colorFormat				= GL_RGB;
      _internalColorFormat		= GL_RGB;
      _colorType					= GL_UNSIGNED_BYTE;
      _colorAttachmentDepth		= RGB_8;

      _minFilter					= GL_LINEAR;
      _magFilter					= GL_LINEAR;

      if( strchr(kv.second.c_str(), 't') != NULL ) {
        _bColorAttachmentRenderTexture = true;
      } else {
        _bColorAttachmentRenderTexture = false;
      }

      if( kv.second.find("16") != kv.second.npos ) {

        _internalColorFormat	= GL_RGB16F_ARB;
        _colorType				= GL_HALF_FLOAT_ARB;

        _colorAttachmentDepth	= RGB_16;
        _bFloatColorBuffer		= true;
      }
      if( kv.second.find("32") != kv.second.npos ) {

        _internalColorFormat	= GL_RGB32F_ARB;
        _colorType				= GL_FLOAT;

        // linear filter is not supported for 32 FBOs
        _minFilter				= GL_NEAREST;
        _magFilter				= GL_NEAREST;

        _colorAttachmentDepth = RGB_32;
        _bFloatColorBuffer		= true;
      }

      //---------------------------------------------	
      //	check for multiple render targets!

      if( kv.second.find("10x") != kv.second.npos ) {
        _numColorAttachments = 10;				
      } else if( kv.second.find("11x") != kv.second.npos ) {
        _numColorAttachments = 11;				
      } else if( kv.second.find("12x") != kv.second.npos ) {
        _numColorAttachments = 12;				
      } else if( kv.second.find("13x") != kv.second.npos ) {
        _numColorAttachments = 13;				
      } else if( kv.second.find("14x") != kv.second.npos ) {
        _numColorAttachments = 14;				
      } else if( kv.second.find("15x") != kv.second.npos ) {
        _numColorAttachments = 15;				
      } else if( kv.second.find("16x") != kv.second.npos ) {
        _numColorAttachments = 16;				
      } else if( kv.second.find("2x") != kv.second.npos ) {
        _numColorAttachments = 2;				
      } else if( kv.second.find("3x") != kv.second.npos ) {
        _numColorAttachments = 3;				
      } else if( kv.second.find("4x") != kv.second.npos ) {
        _numColorAttachments = 4;				
      } else if( kv.second.find("5x") != kv.second.npos ) {
        _numColorAttachments = 5;				
      } else if( kv.second.find("6x") != kv.second.npos ) {
        _numColorAttachments = 6;				
      } else if( kv.second.find("7x") != kv.second.npos ) {
        _numColorAttachments = 7;				
      } else if( kv.second.find("8x") != kv.second.npos ) {
        _numColorAttachments = 8;				
      } else if( kv.second.find("9x") != kv.second.npos ) {
        _numColorAttachments = 9;				
      }  

      //---------------------------------------------

    }

    // ------------------------------------------------
    // ------------------------------------------------
    // handle depth attachment

    else if ( strcmp(kv.first.c_str(), "depth") == 0) {

      _bDepthAttachment		= true;
      _depthFormat			= GL_DEPTH_COMPONENT;
      _depthType				= GL_UNSIGNED_BYTE;
      /*TODO*/_depthType				= GL_FLOAT;
      _internalDepthFormat	= GL_DEPTH_COMPONENT24;

      if( kv.second.find("t") != kv.second.npos ) {
        _bDepthAttachmentRenderTexture = true;
      } else {
        _bDepthAttachmentRenderTexture = false;
      }

      if( kv.second.find("16") != kv.second.npos ) 
        _internalDepthFormat	= GL_DEPTH_COMPONENT16;
      if( kv.second.find("24") != kv.second.npos ) 
        _internalDepthFormat	= GL_DEPTH_COMPONENT24;				 
      if( kv.second.find("32") != kv.second.npos ) 
        _internalDepthFormat	= GL_DEPTH_COMPONENT32;
    }

    // ------------------------------------------------
    // ------------------------------------------------
    // handle depth attachment

    else if ( strcmp(kv.first.c_str(), "stencil") == 0) {

      _bStencilAttachment = true;

      if( kv.second.find("t") != kv.second.npos ) {
        _bStencilAttachmentRenderTexture = true;
      } else {
        _bStencilAttachmentRenderTexture = false;
      }

      /*			if( kv.second.find("16") != kv.second.npos ) {
              _stencilDepth	= 16;
              _internalStencilFormat = GL_STENCIL_INDEX16;
              } else if( kv.second.find("8") != kv.second.npos ) {
              _stencilDepth	= 8;
              _internalStencilFormat = GL_STENCIL_INDEX8;
              }	else if( kv.second.find("4") != kv.second.npos ) {
              _stencilDepth	= 4;
              _internalStencilFormat = GL_STENCIL_INDEX4;
              } else if( kv.second.find("1") != kv.second.npos ) {
              _stencilDepth	= 1;
              _internalStencilFormat = GL_STENCIL_INDEX1;
              }	*/		
    }

  }
}

// -----------------------------------------------------------------------------

FramebufferObject::KeyVal 
FramebufferObject::getKeyValuePair(string token) {

  string::size_type pos = 0;
  if ((pos = token.find("=")) != token.npos) {

    string key = token.substr(0, pos);
    string value = token.substr(pos+1, token.length()-pos+1);
    return KeyVal(key, value);
  }
  else
    return KeyVal(token, "");
}

// -----------------------------------------------------------------------------

bool 
FramebufferObject::checkFramebufferStatus(void) {

  _framebufferStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);

  if(_framebufferStatus == GL_FRAMEBUFFER_COMPLETE)
    return true;
  else
    return false;
}

// -----------------------------------------------------------------------------

void
FramebufferObject::printFramebufferStatus(void) {

  switch(_framebufferStatus) {

    case GL_FRAMEBUFFER_COMPLETE:
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_COMPLETE" << endl;
      break;

    case GL_FRAMEBUFFER_UNSUPPORTED:
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_UNSUPPORTED" << endl;
      break;

    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << endl;
      break; 

    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << endl;
      break; 

    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT" << endl;
      break; 

      //		case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT: 
      //			cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT" << endl;
      //			break; 

    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT" << endl;
      break; 

    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << endl;
      break; 

    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << endl;
      break; 

    case GL_FRAMEBUFFER_BINDING: 
      cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_BINDING" << endl;
      break; 

      //		case GL_FRAMEBUFFER_STATUS_ERROR: 
      //			cout << "FramebufferObject ERROR: GL_FRAMEBUFFER_STATUS_ERROR" << endl;
      //			break; 

    default: 
      cout << "FramebufferObject ERROR: unidentified error!" << endl;
      break;
  }
}

// -----------------------------------------------------------------------------
