
#ifndef REALTIME_SELF_FILTER_RGBD_SUBSCRIBER_H_
#define REALTIME_SELF_FILTER_RGBD_SUBSCRIBER_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/function.hpp>

namespace realtime_self_filter {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo > RgbdSyncPolicy;

class RgbdSubscriber
{

public:
  typedef boost::function<void ( cv::Mat1f& depth_image,
  		cv::Mat3b& rgb_image,
  		cv::Mat1b& grey_image,
  		cv::Matx33d& camera_matrix,
  		std_msgs::Header header ) > Callback;

	message_filters::Subscriber<sensor_msgs::Image> intensity_img_sub;
	message_filters::Subscriber<sensor_msgs::Image> depth_img_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;

	message_filters::Synchronizer<RgbdSyncPolicy> rgbd_sync;

  Callback callback;

	RgbdSubscriber( ros::NodeHandle comm_nh, Callback cb ) :
		intensity_img_sub(comm_nh, "/camera/rgb/image_color", 1),
		depth_img_sub(comm_nh, "/camera/depth_registered/image", 1),
		cam_info_sub(comm_nh, "/camera/rgb/camera_info", 1),
		rgbd_sync( RgbdSyncPolicy(10), intensity_img_sub, depth_img_sub, cam_info_sub ),
    callback (cb)
	{
	  rgbd_sync.registerCallback( boost::bind(&RgbdSubscriber::rgbdImageCb, this, _1,_2,_3) );

		ROS_INFO ("Subscribed to intensity image on: %s", intensity_img_sub.getTopic().c_str ());
		ROS_INFO ("Subscribed to depth image on: %s", depth_img_sub.getTopic().c_str ());
		ROS_INFO ("Subscribed to camera info on: %s", cam_info_sub.getTopic().c_str ());
	}

  void prepareImages (
      const sensor_msgs::Image::ConstPtr ros_intensity_image,
      const sensor_msgs::Image::ConstPtr ros_depth_image,
      const sensor_msgs::CameraInfo::ConstPtr ros_camera_info,
      cv::Mat3b& rgb_img, cv::Mat1b& grey_img, cv::Mat1f& depth_img,
      cv::Matx33d &K )
  {
    cv_bridge::CvImageConstPtr orig_rgb_img;
    cv_bridge::CvImageConstPtr orig_depth_img;

    orig_rgb_img = cv_bridge::toCvCopy( ros_intensity_image, "bgr8" );
    orig_depth_img = cv_bridge::toCvCopy( ros_depth_image, sensor_msgs::image_encodings::TYPE_32FC1 );

//    if (scale_depth_to_rgb_resolutoin)
//    {
//      int scale_fac = orig_rgb_img->image.cols / orig_depth_img->image.cols;
//
//      // close depth gaps
//      cv::Mat1f depth_image_closed = orig_depth_img->image;
//      //cv::daft2::improveDepthMap<10>( orig_depth_img->image, depth_image_closed, 0.2f );
//
//      // Resize depth to have the same width as rgb
//      cv::Mat1f depth_img_tmp;
//      cv::resize( depth_image_closed, depth_img_tmp, cvSize(0,0), scale_fac, scale_fac, cv::INTER_NEAREST );
//      assert( depth_img_tmp.type() == CV_32F );
//
//      // Crop rgb so it has the same size as depth
//      cv::Mat3b rgb_img_tmp = cv::Mat( orig_rgb_img->image, cv::Rect( 0,0, depth_img_tmp.cols, depth_img_tmp.rows ) );
//      depth_img = depth_img_tmp;
//      rgb_img = rgb_img_tmp;
//    } else:

    depth_img = orig_depth_img->image;
    rgb_img = orig_rgb_img->image;

    K  = cv::Matx33d( ros_camera_info->K.data() );

    // Convert RGB to Grey image
    cv::cvtColor( rgb_img, grey_img, CV_BGR2GRAY );
  }

	void rgbdImageCb(const sensor_msgs::Image::ConstPtr rgb_img_msg,
						const sensor_msgs::Image::ConstPtr depth_img_msg,
						const sensor_msgs::CameraInfo::ConstPtr camera_info_msg )
	{
		cv::Mat1f depth_image;
		cv::Mat3b rgb_image;
		cv::Mat1b grey_image;
	  cv::Matx33d K;

	  prepareImages (rgb_img_msg, depth_img_msg, camera_info_msg,
	      rgb_image, grey_image, depth_image, K );

	  ROS_INFO_STREAM_ONCE( "f = " << K(0,0) << " cx = " << K(0,2) << " cy = " << K(1,2) );

	  callback( depth_image, rgb_image, grey_image, K, rgb_img_msg->header );
	}

};



}

#endif // REALTIME_SELF_FILTER_RGBD_SUBSCRIBER_H_

