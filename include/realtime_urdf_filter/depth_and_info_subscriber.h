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


#ifndef REALTIME_URDF_FILTER_DEPTH_AND_INFO_SUBSCRIBER_H_
#define REALTIME_URDF_FILTER_DEPTH_AND_INFO_SUBSCRIBER_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/function.hpp>

namespace realtime_urdf_filter
{
  typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::CameraInfo > DepthAndInfoSyncPolicy;

  class DepthAndInfoSubscriber
  {
  public:
    typedef boost::function<void (const sensor_msgs::ImageConstPtr& depth_image,
                                  const sensor_msgs::CameraInfo::ConstPtr& camera_matrix) > Callback;

    message_filters::Subscriber<sensor_msgs::Image> depth_img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;

    message_filters::Synchronizer<DepthAndInfoSyncPolicy> di_sync;

    Callback callback;

    DepthAndInfoSubscriber (ros::NodeHandle comm_nh, Callback cb)
      : depth_img_sub (comm_nh, "/camera/depth_registered/image", 10)
      , cam_info_sub (comm_nh, "/camera/rgb/camera_info", 10)
      , di_sync (DepthAndInfoSyncPolicy(100), depth_img_sub, cam_info_sub )
      , callback (cb)
    {
      di_sync.registerCallback (boost::bind (&DepthAndInfoSubscriber::depthAndInfoCb, this, _1,_2));

      ROS_INFO ("Subscribed to depth image on: %s", depth_img_sub.getTopic().c_str ());
      ROS_INFO ("Subscribed to camera info on: %s", cam_info_sub.getTopic().c_str ());
    }

    void depthAndInfoCb (const sensor_msgs::Image::ConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
    {
      callback (depth_img_msg, camera_info_msg);
    }
  };



}

#endif // REALTIME_URDF_FILTER_DEPTH_AND_INFO_SUBSCRIBER_H_
