//------------------------------------------------------------------------------
// File : FramebufferObject.h
//------------------------------------------------------------------------------
// Copyright (c) 2006 Gordon Wetzstein [gordon.wetzstein@medien.uni-weimar.de]
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
/*
* The pixel format for the pbuffer is controlled by the mode string passed
* into the FramebufferObject constructor. This string can have the
*	following attributes:
*
* To specify the pixel format, use the following syntax.
*   <channels>=<bits>
* <channels> must match one of the following.
*
*	rgba=n[t]		- currently only RGBA and RGB color attachments are supported,
*								n=8, 16 or 32; 16 and 32 are half-float / float buffers,
*								t for RenderTexture support
*
* The following other attributes are supported.
*
* depth=n[t]	- must have n-bit depth buffer, omit n for default (24 bits),
*								n can be 16, 24 or 32 whereas 16 did not work on my machine,
*								t for RenderTexture support!
*
* stencil[=t]	- stencil buffer attachment are currently only supported in
*								combination with depth buffer attachment [both must be either
*								textures or buffers], the depth attachment is set to 24 Bit.
*								This is because most OpenGL driver implementations have no
*								separate stencil or depth buffer - See
*								EXT_packed_depth_stencil specifications for details!
*
*	-----------------------------------------------------------------------------
*	TODO:
*
*	- support 32 Bit color attachments
*	- support more stencil buffer attachment combinations
*	- support more color attachment formats [rg, r, g, b etc.]
*	- test ATI support
*	- float depth buffer
*
*/

#ifndef __FRAMEBUFFER_OBJECT__
#define __FRAMEBUFFER_OBJECT__

// -----------------------------------------------------------------------------

#ifdef WIN32
	#include <windows.h>
#endif

//#include <gl/gl.h>
//#include <gl/glext.h>

////#define GLH_EXT_SINGLE_FILE
//#include <glh/glh_extensions.h>

#include <GL/glew.h>

#include <iostream>
#include <vector>

// -----------------------------------------------------------------------------

#define MAX_COLOR_COMPONENTS 16

// -----------------------------------------------------------------------------

class FramebufferObject {

public:

	// ---------------------------------------------------------------------------

	enum ColorAttachmentType {

		RGB_8	= 0,
		RGB_16,
		RGB_32,

		RGBA_8,
		RGBA_16,
		RGBA_32,
	};

	// ---------------------------------------------------------------------------

	FramebufferObject(const char *strMode="rgba=3x8t depth=24t");
	~FramebufferObject();

	/// initialize the FBOs and Textures
	bool						initialize(	unsigned int width, unsigned int height );
	bool						reInitialize(	unsigned int width, unsigned int height, const char *strMode="rgb=16t depth=24t" );

	/// enable and bind FBO
	///	if the color attachment is of type float a simple pass
	///	through fragment shader is automatically enabled to
	///	support 16/32 Bit depth at all, unfortunately this only
	///	works for the texture coordinates!!! these are convertex
	///	to the color in the shader, disable the pass through
	///	shader to support your own
	void						beginCapture(bool bEnablePassThroughShader=true);

	/// disable and "unbind" FBO
	///	if the color attachment is of type float a simple pass
	///	through fragment shader is automatically enabled to
	///	support 16/32 Bit depth at all, unfortunately this only
	///	works for the texture coordinates!!! these are convertex
	///	to the color in the shader, disable the pass through
	///	shader to support your own
	void						endCapture(bool bDisablePassThroughShader=true);

	/// bind the color attachment with the index
	void						bind(int index=0);

	/// bind the depth attachment
	void						bindDepth(void);

	/// get the Texture ID of the color attachment with the index
	GLuint						getColorAttachmentID(int index = 0);

	/// get the Texture ID of the depth attachment
	GLuint						getDepthAttachmentID(void);

	/// get the Texture ID of the stencil attachment
	GLuint						getStencilAttachmentID(void);

	/// get the Texture target
	GLenum						getTextureTarget(void) { return _textureTarget; }

	/// get width of the FBO
	unsigned int				getWidth(void);
	/// get height of the FBO
	unsigned int				getHeight(void);

	/// check errors using FBO extension function
	bool						checkFramebufferStatus(void);
	void						printFramebufferStatus(void);

	/// enable Texture Target
	void						enableTextureTarget(void) { glEnable(_textureTarget); }
	/// disable Texture Target
	void						disableTextureTarget(void) { glDisable(_textureTarget); }

	/// set the min texture interpolation filter
	void						setMinFilter(GLint	minFilter);

	/// set the mag texture interpolation filter
	void						setMagFilter(GLint	magFilter);

	/// set the wrap s
	void						setWrapS(GLint	wrapS);

	/// set the warp t
	void						setWrapT(GLint	wrapT);

protected:

	/// texture target, default: GL_TEXTURE_RECTANGLE_ARB
	GLenum										_textureTarget;

	/// format of the color texture, default: GL_RGBA
	GLint										_colorFormat;
	/// internal format of the depth texture, default: GL_RGBA
	GLint										_internalColorFormat;
	/// type of the color attachment, default: GL_UNSIGNED_BYTE
	GLenum										_colorType;

	/// color attachment depth
	ColorAttachmentType							_colorAttachmentDepth;

	/// format of the depth texture, default: GL_DEPTH_COMPONENT
	GLenum										_depthFormat;
	/// internal format of the depth texture, default: GL_DEPTH_COMPONENT24
	///		other options: GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT32
	GLenum										_internalDepthFormat;
	/// type of the depth attachment, default: GL_UNSIGNED_BYTE
	GLenum										_depthType;

	/// wrap s parameter for color attachments, default: GL_CLAMP_TO_EDGE
	GLint										_wrapS;
	/// wrap t parameter for color attachments, default: GL_CLAMP_TO_EDGE
	GLint										_wrapT;

	/// min filterfor texture interpolation, default: GL_LINEAR
	GLint										_minFilter;
	/// mag filterfor texture interpolation, default: GL_LINEAR
	GLint										_magFilter;

	/// width of the FBO
	int											_width;
	/// height of the FBO
	int											_height;

	/// FBO extension supported?
	bool										_bFBOSupported;

	/// FBO initialized?
	bool										_bInitialized;

	///	use color attachment?
	bool										_bColorAttachment;
	///	use color as RenderTexture?
	bool										_bColorAttachmentRenderTexture;

	///	use depth attachment?
	bool										_bDepthAttachment;
	///	use depth as RenderTexture?
	bool										_bDepthAttachmentRenderTexture;

	///	use stencil attachment?
	bool										_bStencilAttachment;
	///	use stencil as RenderTexture?
	bool										_bStencilAttachmentRenderTexture;
	/// depth of the stencil buffer
	int											_stencilDepth;
	/// internal stencil buffer format
	GLint										_internalStencilFormat;

  /// pass through shader program initialized?
	bool										_bPassThroughProgramInitialized;

	/// ID of the FBO
	GLuint										_frameBufferID;

	/// Texture IDs of the color attachment textures
	GLuint										_colorAttachmentID[MAX_COLOR_COMPONENTS];

	/// number of color attachments
	int											_numColorAttachments;

	/// number of maximum supported draw buffers|color attachments
	int											_maxNumDrawBuffers;

	/// Texture ID of the depth attachment
	GLuint										_depthAttachmentID;

	/// Texture ID of the stencil attachment
	GLuint										_stencilAttachmentID;

	/// save viewport before setting new one to restore it later
	///	avoid glPushAttrib() and glPopAttrib()!
	GLint										_viewport[4];

	/// shader for float support

  GLenum _passthrough_program;
  GLenum _passthrough_fragment_shader;
	GLuint										_passThroughProgram;

	/// indicates if color buffer is a float texture
	bool										_bFloatColorBuffer;

private:

	/// parse the mode string and set configuration
	void										parseModeString(const char *modeString);

	typedef std::pair<std::string, std::string> KeyVal;
	/// get the key=value pair of a single token from the mode string
	KeyVal										getKeyValuePair(std::string token);


};

#endif
