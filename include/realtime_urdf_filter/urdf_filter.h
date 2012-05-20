#ifndef REALTIME_URDF_FILTER_URDF_FILTER_H_
#define REALTIME_URDF_FILTER_URDF_FILTER_H_

#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

#include "realtime_urdf_filter/FrameBufferObject.h"
#include "realtime_urdf_filter/shader_wrapper.h"
#include "realtime_urdf_filter/urdf_renderer.h"

#include <GL/freeglut.h>

namespace realtime_urdf_filter
{

class RealtimeURDFFilter
{
  public:
    // constructor. sets up ros and reads in parameters
    RealtimeURDFFilter (ros::NodeHandle &nh, int argc, char **argv);

    ~RealtimeURDFFilter ();

    // loads URDF models
    void loadModels ();

    // helper function to get current time
    double getTime ();

    // callback function that gets ROS images and does everything
    void filter_callback
         (const sensor_msgs::ImageConstPtr& ros_depth_image,
          const sensor_msgs::CameraInfo::ConstPtr& camera_info);

    void textureBufferFromDepthImage (cv::Mat1f depth_image);

    // set up OpenGL stuff
    void initGL ();

    // set up FBO
    void initFrameBufferObject ();

    // compute Projection matrix from CameraInfo message
    void getProjectionMatrix (const sensor_msgs::CameraInfo::ConstPtr& current_caminfo, double* glTf);

    void render (const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    
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
    std::vector<URDFRenderer*> renderers_;

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

} // end namespace
#endif
