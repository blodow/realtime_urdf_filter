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

#ifndef REALTIME_URDF_FILTER_URDF_FILTER_H_
#define REALTIME_URDF_FILTER_URDF_FILTER_H_

#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "realtime_urdf_filter/FrameBufferObject.h"
#include "realtime_urdf_filter/shader_wrapper.h"
#include "realtime_urdf_filter/urdf_renderer.h"

#include <GL/freeglut.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

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

    // callback fucntion that gets ROS image camera info
    void cameraInfo_callback(const sensor_msgs::CameraInfo::ConstPtr& current_caminfo);

    // callback function that gets ROS images and does everything
    void filter_callback
         (const sensor_msgs::ImageConstPtr& ros_depth_image);

    // does virtual rendering and filtering based on depth buffer and opengl proj. matrix
    void filter (
        unsigned char* buffer, double* glTf, int width, int height, ros::Time timestamp = ros::Time());

    // copy cv::Mat1f to char buffer
    unsigned char* bufferFromDepthImage (cv::Mat1f depth_image);

    // copy char buffer to OpenGL texture
    void textureBufferFromDepthBuffer (unsigned char* buffer, int size_in_bytes);

    // set up OpenGL stuff
    void initGL ();

    // set up FBO
    void initFrameBufferObject ();

    // compute Projection matrix from CameraInfo message
    void getProjectionMatrix (double* glTf);

    void render (const double* camera_projection_matrix, ros::Time timestamp = ros::Time());

    GLfloat* getMaskedDepth()
      {return masked_depth_;}

  public:
    // ROS objects
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    image_transport::ImageTransport image_transport_;
    ros::Subscriber info_sub_;
    ros::Publisher realtime_filter_pub_;
    image_transport::CameraPublisher depth_pub_;
    image_transport::CameraPublisher mask_pub_;
    
    // subscribe camera_info separately because tf2_ros MessageFilter accepts one msg type
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    
  	// setup message filter
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    message_filters::Subscriber<sensor_msgs::Image> depth_mf_sub_;
    tf2_ros::MessageFilter<sensor_msgs::Image> tf2_filter_;

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
    bool show_gui_;

    // do we have subscribers for the mask image?
    bool need_mask_;

    // image size
    GLint width_;
    GLint height_;

    // Camera parameters
    double camera_tx_;
    double camera_ty_;

    // OpenGL virtual camera setup
    double far_plane_;
    double near_plane_;
    double depth_distance_threshold_;
    double filter_replace_value_;

    // neccesary for glutInit()..
    int argc_;
    char **argv_;

    // output from rendering
    GLfloat* masked_depth_ = NULL;
    GLubyte* mask_ = NULL;
};

} // end namespace
#endif
