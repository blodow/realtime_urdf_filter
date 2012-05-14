
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
