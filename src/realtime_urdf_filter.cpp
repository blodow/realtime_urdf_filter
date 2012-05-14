#include <pthread.h>
#include <execinfo.h>

#include "realtime_urdf_filter/FrameBufferObject.h"

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include <iostream>

#include <ros/node_handle.h>
#include "realtime_urdf_filter/urdf_renderer.h"
#include <realtime_urdf_filter/shader_wrapper.h>
#include <realtime_urdf_filter/depth_and_info_subscriber.h>

#include <GL/freeglut.h>

//#define USE_OWN_CALIBRATION

class RealtimeURDFFilter
{
  public:
    // constructor. sets up ros and reads in parameters
    RealtimeURDFFilter (ros::NodeHandle &nh, int argc, char **argv)
      : nh_(nh)
      , fbo_initialized_(false)
      , depth_image_pbo_ (GL_INVALID_VALUE)
      , far_plane_ (8)
      , near_plane_ (0.1)
      , argc_ (argc), argv_(argv)
    {
      // get fixed frame name
      XmlRpc::XmlRpcValue v;
      nh_.getParam ("fixed_frame", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "fixed_frame paramter!");
      fixed_frame_ = (std::string)v;
      ROS_INFO ("using fixed frame %s", fixed_frame_.c_str ());

      // get camera frame name 
      // TODO: read this from ROS message
      nh_.getParam ("camera_frame", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "need a camera_frame paramter!");
      cam_frame_ = (std::string)v;
      ROS_INFO ("using camera frame %s", cam_frame_.c_str ());

      // read additional camera offset (TODO: make optional)
      nh_.getParam ("camera_offset", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeStruct && "need a camera_offset paramter!");
      ROS_ASSERT (v.hasMember ("translation") && v.hasMember ("rotation") && "camera offset needs a translation and rotation parameter!");

      // translation
      XmlRpc::XmlRpcValue vec = v["translation"];
      ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 3 && "camera_offset.translation parameter must be a 3-value array!");
      ROS_INFO ("using camera translational offset: %f %f %f",
          (double)(vec[0]),
          (double)(vec[1]),
          (double)(vec[2])
          );
      camera_offset_t_ = tf::Vector3((double)vec[0], (double)vec[1], (double)vec[2]);

      // rotation
      vec = v["rotation"];
      ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 4 && "camera_offset.rotation parameter must be a 4-value array [x y z w]!");
      ROS_INFO ("using camera rotational offset: %f %f %f %f", (double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);
      camera_offset_q_ = tf::Quaternion((double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);

      // depth distance threshold (how far from the model are points still deleted?)
      nh_.getParam ("depth_distance_threshold", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeDouble && "need a depth_distance_threshold paramter!");
      depth_distance_threshold_ = (double)v;
      ROS_INFO ("using depth distance threshold %f", depth_distance_threshold_);

      // setup publishers 
      // TODO: make these topics parameters
      mask_pub_ = nh_.advertise<sensor_msgs::Image> ("output_mask", 10);
      depth_pub_ = nh_.advertise<sensor_msgs::Image> ("output", 10);
    }

    ~RealtimeURDFFilter ()
    {
      delete masked_depth_;
      delete mask_;
    }

    // loads URDF models
    void loadModels ()
    {
      XmlRpc::XmlRpcValue v;
      nh_.getParam ("models", v);
      
      if (v.getType () == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < v.size(); ++i)
        {
          XmlRpc::XmlRpcValue elem = v[i];
          ROS_ASSERT (elem.getType()  == XmlRpc::XmlRpcValue::TypeStruct);

          std::string description_param = elem["model"];
          std::string tf_prefix = elem["tf_prefix"];

          // read URDF model
          std::string content;

          if (!nh_.getParam(description_param, content))
          {
            std::string loc;
            if (nh_.searchParam(description_param, loc))
            {
              nh_.getParam(loc, content);
            }
            else
            {
              ROS_ERROR ("Parameter [%s] does not exist, and was not found by searchParam()",
                  description_param.c_str());
              continue;
            }
          }

          if (content.empty())
          {
            ROS_ERROR ("URDF is empty");
            continue;
          }

          // finally, set the model description so we can later parse it.
          ROS_INFO ("Loading URDF model: %s", description_param.c_str ());
          renderers_.push_back (new realtime_urdf_filter::URDFRenderer (content, tf_prefix, cam_frame_, fixed_frame_, tf_));
        }
      }
      else
      {
        ROS_ERROR ("models parameter must be an array!");
      }
    }

    // helper function to get current time
    double getTime ()
    {
      timeval current_time;
      gettimeofday (&current_time, NULL);
      return (current_time.tv_sec + 1e-6 * current_time.tv_usec);
    }

    // callback function that gets ROS images and does everything
    void filter_callback
         (const sensor_msgs::ImageConstPtr& ros_depth_image,
          const sensor_msgs::CameraInfo::ConstPtr& camera_info)
    {
      // convert to OpenCV cv::Mat
      cv_bridge::CvImageConstPtr orig_depth_img;
      try
      {
        orig_depth_img = cv_bridge::toCvShare (ros_depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge Exception: %s", e.what());
        return;
      }
      cv::Mat1f depth_image = orig_depth_img->image;

      // Make sure initGL is called from the same thread that calls render ()
      static bool first = true;
      if (first)
      {
        width_ = depth_image.cols;
        height_ = depth_image.rows;
        initGL ();
        first = false;
      }
      if (width_ != depth_image.cols || height_ != depth_image.rows)
      {
        // TODO: reinitialize FBO
        ROS_ERROR_ONCE ("image size has changed, please restart!");
        return;
      }

      // Timing
      static unsigned count = 0;
      static double last = getTime ();
      double now = getTime ();

      if (++count == 30 || (now - last) > 5)
      {
        std::cout << "Average framerate: " << std::setprecision(3) << double(count)/double(now - last) << " Hz" << std::endl;
        count = 0;
        last = now;
      }

      // get depth_image into OpenGL texture buffer
      textureBufferFromDepthImage (depth_image);

      // render everything
      render (camera_info);

      // publish processed depth image and image mask
      cv::Mat masked_depth_image (height_, width_, CV_32FC1, masked_depth_);
//      cv::imshow ("depth", masked_depth_image);
      cv::Mat mask_image (height_, width_, CV_8UC1, mask_);
//      cv::imshow ("mask", mask_image);
//      cv::waitKey (100);
      cv_bridge::CvImage out_masked_depth;
      out_masked_depth.header = ros_depth_image->header;
      out_masked_depth.encoding = "32FC1";
      out_masked_depth.image = masked_depth_image;
      depth_pub_.publish (out_masked_depth.toImageMsg ());

      cv_bridge::CvImage out_mask;
      out_mask.header = ros_depth_image->header;
      out_mask.encoding = "mono8";
      out_mask.image = mask_image;
      mask_pub_.publish (out_mask.toImageMsg ());
    }

    void textureBufferFromDepthImage (cv::Mat1f depth_image)
    {
      // Host buffer to hold depth pixel data
      static unsigned char* buffer = 0;

      // get pixel data from cv::Mat as one continuous buffer
      int row_size = depth_image.cols * depth_image.elemSize();
      if (depth_image.isContinuous())
      {
        buffer = depth_image.data;
      }
      else
      {
        if (buffer == 0)
          buffer = (unsigned char*) malloc (row_size * depth_image.rows);
        for (int i = 0; i < depth_image.rows; i++)
          memcpy ((void*)(buffer + i * row_size), (void*) &depth_image.data[i], row_size);
      }

      // check if we already have a PBO and Texture Buffer
      if (depth_image_pbo_ == GL_INVALID_VALUE)
      {
        glGenBuffers (1, &depth_image_pbo_);
        glGenTextures (1, &depth_texture_);
      }

      glBindBuffer (GL_ARRAY_BUFFER, depth_image_pbo_);

      // upload buffer data to GPU
      int size_in_bytes = row_size * depth_image.rows;
      glBufferData (GL_ARRAY_BUFFER, size_in_bytes, buffer, GL_DYNAMIC_DRAW);
      glBindBuffer (GL_ARRAY_BUFFER, 0);

      // assign PBO to Texture Buffer
      glBindTexture(GL_TEXTURE_BUFFER, depth_texture_);
      glTexBuffer (GL_TEXTURE_BUFFER, GL_R32F, depth_image_pbo_);
    }

    // set up OpenGL stuff
    void initGL ()
    {
      //TODO: change this to use an offscreen pbuffer, so no window is necessary
      glutInit (&argc_, argv_);

      // the window will show 3x2 grid of images
      glutInitWindowSize (960, 480);
      glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
      glutCreateWindow ("Realtime URDF Filter Debug Window");

      // initialize glew library
      GLenum err = glewInit();
      if (GLEW_OK != err)
      {
        std::cout << "ERROR: could not initialize GLEW!" << std::endl;
      }

      // set up FBO and load URDF models + meshes onto GPU
      initFrameBufferObject ();
      loadModels ();
      std::cerr << " --- Initialization done. ---" << std::endl;
      masked_depth_ = (GLfloat*) malloc(width_ * height_ * sizeof(GLfloat));
      mask_ = (GLubyte*) malloc(width_ * height_ * sizeof(GLubyte));
    }

    // set up FBO
    void initFrameBufferObject ()
    {
      fbo_ = new FramebufferObject ("rgba=4x32t depth=24t stencil=t");

      fbo_->initialize (width_, height_);
      fbo_initialized_ = true;

      GLenum err = glGetError();
      if(err != GL_NO_ERROR)
        printf("OpenGL ERROR after FBO initialization: %s\n", gluErrorString(err));
    }

    // compute Projection matrix from CameraInfo message
    void getProjectionMatrix (const sensor_msgs::CameraInfo::ConstPtr& current_caminfo, btScalar* glTf)
    {
      sensor_msgs::CameraInfo::ConstPtr info = current_caminfo;

      if (!info)
        return;

      tf::Vector3 position;
      tf::Quaternion orientation;

#ifdef USE_OWN_CALIBRATION
      float P[12];
      P[0] = 585.260; P[1] = 0.0;     P[2]  = 317.387; P[3]  = 0.0;
      P[4] = 0.0;     P[5] = 585.028; P[6]  = 239.264; P[7]  = 0.0;
      P[8] = 0.0;     P[9] = 0.0;     P[10] = 1.0;     P[11] = 0.0;

      double fx = P[0];
      double fy = P[5];
      double cx = P[2];
      double cy = P[6];
#else
      double fx = info->P[0] * 0.5;
      double fy = info->P[5] * 0.5;
      double cx = info->P[2] * 0.5;
      double cy = (info->P[6]) * 0.5 - 48;
 
      // TODO: check if this does the right thing with respect to registered depth / camera info
      // Add the camera's translation relative to the left camera (from P[3]);
      //double tx = -1 * (info->P[3] / fx);
      //tf::Vector3 right = orientation * tf::Vector3 (1,0,0);
      //position = position + (right * tx);

      //double ty = -1 * (info->P[7] / fy);
      //tf::Vector3 down = orientation * tf::Vector3 (0,1,0);
      //position = position + (down * ty);

#endif

      for (unsigned int i = 0; i < 16; ++i)
        glTf[i] = 0.0;

      // calculate the projection matrix
      // NOTE: this minus is there to flip the x-axis of the image.
      glTf[0]= -2.0 * fx / width_;
      glTf[5]= 2.0 * fy / height_;

      glTf[8]= 2.0 * (0.5 - cx / width_);
      glTf[9]= 2.0 * (cy / height_ - 0.5);

      glTf[10]= - (far_plane_ + near_plane_) / (far_plane_ - near_plane_);
      glTf[14]= -2.0 * far_plane_ * near_plane_ / (far_plane_ - near_plane_);

      glTf[11]= -1;
    }

    void render (const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
      if (!fbo_initialized_)
        return;

      static const GLenum buffers[] = {
        GL_COLOR_ATTACHMENT0_EXT,
        GL_COLOR_ATTACHMENT1_EXT,
        GL_COLOR_ATTACHMENT2_EXT,
        GL_COLOR_ATTACHMENT3_EXT
      };

      // get transformation from camera to "fixed frame"
      tf::StampedTransform t;
      try
      {
        tf_.lookupTransform (cam_frame_, fixed_frame_, ros::Time (), t);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        return;
      }

      GLenum err = glGetError();
      if(err != GL_NO_ERROR)
        printf("OpenGL ERROR after FBO initialization: %s\n", gluErrorString(err));

      glPushAttrib(GL_ALL_ATTRIB_BITS);
      glEnable(GL_NORMALIZE);

      // render into FBO
      fbo_->beginCapture();

      // create shader programs
      static realtime_urdf_filter::ShaderWrapper shader = realtime_urdf_filter::ShaderWrapper::fromFiles
        ("package://realtime_urdf_filter/include/shaders/urdf_filter.vert", 
         "package://realtime_urdf_filter/include/shaders/urdf_filter.frag");

      err = glGetError();
      if(err != GL_NO_ERROR)
        printf("OpenGL ERROR compiling shaders: %s\n", gluErrorString(err));
      
      // enable shader for this frame
      shader ();

      glDrawBuffers(sizeof(buffers) / sizeof(GLenum), buffers);

      // clear the buffers
      glClearColor(0.0, 0.0, 0.0, 1.0);
      glClearStencil(0x0);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

      glEnable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      // setup camera projection
      glMatrixMode (GL_PROJECTION);
      glLoadIdentity();

      // load camera info into OpenGL camera matrix
      btScalar glTf[16];
      getProjectionMatrix (cam_info, glTf);
      glMultMatrixd((GLdouble*)glTf);

      // setup camera position
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      // kinect has x right, y down, z into image
      gluLookAt (0,0,0, 0,0,1, 0,1,0);
      
      // draw background quad behind everything (just before the far plane)
      glBegin(GL_QUADS);
        glVertex3f(-10.0, -10.0, far_plane_*0.99);
        glVertex3f( 10.0, -10.0, far_plane_*0.99);
        glVertex3f( 10.0,  10.0, far_plane_*0.99);
        glVertex3f(-10.0,  10.0, far_plane_*0.99);
      glEnd();
     
      // apply user-defined camera offset transformation (launch file)
      tf::Transform transform (camera_offset_q_, camera_offset_t_);
      transform.inverse().getOpenGLMatrix(glTf);
      glMultMatrixd((GLdouble*)glTf);

      // apply camera to "fixed frame" transform (world coordinates)
      t.getOpenGLMatrix(glTf);
      glMultMatrixd((GLdouble*)glTf);
      
      // set up stencil buffer etc.
      // the background quad is not in the stencil buffer
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_ALWAYS, 0x1, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

      // make texture with depth image available in shader
      glActiveTexture (GL_TEXTURE0);
      GLuint depth_texture_id = 0;
      shader.SetUniformVal1i (std::string("depth_texture"), depth_texture_id);
      shader.SetUniformVal1i (std::string("width"), int(width_));
      shader.SetUniformVal1i (std::string("height"), int(height_));
      shader.SetUniformVal1f (std::string("z_far"), far_plane_);
      shader.SetUniformVal1f (std::string("z_near"), near_plane_);
      shader.SetUniformVal1f (std::string("max_diff"), float(depth_distance_threshold_));
      glBindTexture (GL_TEXTURE_BUFFER, depth_texture_);

      // render every renderable / urdf model
      BOOST_FOREACH (realtime_urdf_filter::URDFRenderer* r, renderers_)
        r->render ();

      // disable shader
      glUseProgram((GLuint)NULL);
      
      fbo_->endCapture();
      glPopAttrib();

      // -----------------------------------------------------------------------
      // -----------------------------------------------------------------------
      //	from here on folloes mainly display code 
      //	(not necessary for offscreen rendering)

      // use stencil buffer to draw a red / blue mask
      glPushAttrib(GL_ALL_ATTRIB_BITS);

      fbo_->beginCapture();
      glDrawBuffer(GL_COLOR_ATTACHMENT3_EXT);

      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL, 0x1, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      glColor3f(1.0, 0.0, 0.0);

      glBegin(GL_QUADS);
        glVertex2f(0.0, 0.0);
        glVertex2f(1.0, 0.0);
        glVertex2f(1.0, 1.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();

      glStencilFunc(GL_EQUAL, 0x0, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      glColor3f(0.0, 0.0, 1.0);

      glBegin(GL_QUADS);
        glVertex2f(0.0, 0.0);
        glVertex2f(1.0, 0.0);
        glVertex2f(1.0, 1.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      
      fbo_->endCapture();

      glPopAttrib();

      // -----------------------------------------------------------------------
      // -----------------------------------------------------------------------
      // render all color buffer attachments into window

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      // draw color buffer 0
      fbo_->bind(0);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.0, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.333, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.333, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      // draw color buffer 1
      fbo_->bind(1);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.0, 0.0);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.333, 0.0);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.333, 0.5);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 0.5);
      glEnd();

      // draw color buffer 2
      fbo_->bind(2);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.333, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.666, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.666, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.333, 1.0);
      glEnd();

      // draw color buffer 3
      fbo_->bind(3);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.333, 0.0);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.666, 0.0);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.666, 0.5);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.333, 0.5);
      glEnd();

      // draw depth buffer 
      fbo_->bindDepth();
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.666, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(1.0, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(1.0, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.666, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      
//      // copy frame buffer attachments to pbo's
//      fbo_->bind (2);
//      glBindBuffer (GL_PIXEL_PACK_BUFFER, interop_gl_buffers_[0]);
//      glGetTexImage (fbo_->getTextureTarget(), 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
//      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
//
//      // copy frame buffer attachments to pbo's
//      fbo_->bindDepth ();
//      glBindBuffer (GL_PIXEL_PACK_BUFFER, interop_gl_buffers_[1]);
//      glGetTexImage (fbo_->getTextureTarget(), 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
//      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
//
      // get device pointer to buffer
//      char4 *raw_ptr = 0;
//      cudaGLMapBufferObject((void**)&raw_ptr, interop_gl_buffers_[0]);
//      interop_cuda_pointer_normals_ = thrust::device_pointer_cast(raw_ptr);
//
//      // get device pointer to buffer
//      float *raw_ptr_d = 0;
//      cudaGLMapBufferObject((void**)&raw_ptr_d, interop_gl_buffers_[1]);
//      interop_cuda_pointer_depth_ = thrust::device_pointer_cast(raw_ptr_d);

      fbo_->bind(1);
      glGetTexImage (fbo_->getTextureTarget(), 0, GL_RED, GL_FLOAT, masked_depth_);
      fbo_->bind(3);
      glGetTexImage (fbo_->getTextureTarget(), 0, GL_RED, GL_UNSIGNED_BYTE, mask_);

      // ok, finished with all OpenGL, let's swap!
      glutSwapBuffers ();
      glutPostRedisplay();
      glutMainLoopEvent ();
      glFlush ();
    }
    
  protected:
    // ROS objects
    ros::NodeHandle &nh_;
    tf::TransformListener tf_;
    ros::Publisher mask_pub_;
    ros::Publisher depth_pub_;

    // rendering objects
    FramebufferObject *fbo_;
    bool fbo_initialized_;
    GLuint depth_image_pbo_;
    GLuint depth_texture_;

    // vector of renderables
    std::vector<realtime_urdf_filter::URDFRenderer*> renderers_;

    // parameters from launch file
    tf::Vector3 camera_offset_t_;
    tf::Quaternion camera_offset_q_;
    std::string cam_frame_;
    std::string fixed_frame_;

    // image size
    GLint width_;
    GLint height_;

    // OpenGL virtual camera setup
    double far_plane_;
    double near_plane_;
    double depth_distance_threshold_;

    // neccesary for glutInit()..
    int argc_;
    char **argv_;

    // output from rendering
    GLfloat* masked_depth_;
    GLubyte* mask_;
};

int 
main (int argc, char **argv)
{
  // set up ROS
  ros::init (argc, argv, "realtime_urdf_filter");
  ros::NodeHandle nh ("~");

  // create RealtimeURDFFilter and subcribe to ROS
  RealtimeURDFFilter f(nh, argc, argv);
  realtime_urdf_filter::DepthAndInfoSubscriber sub (nh, boost::bind (&RealtimeURDFFilter::filter_callback, &f, _1, _2));

  // spin that shit!
  ros::spin ();

  return 0;
}

