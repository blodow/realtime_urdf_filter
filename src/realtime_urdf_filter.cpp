#include "realtime_urdf_filter/urdf_filter.h"
#include "realtime_urdf_filter/urdf_renderer.h"
#include "realtime_urdf_filter/depth_and_info_subscriber.h"

#include <ros/node_handle.h>

int main (int argc, char **argv)
{
  // set up ROS
  ros::init (argc, argv, "realtime_urdf_filter");
  ros::NodeHandle nh ("~");

  // create RealtimeURDFFilter and subcribe to ROS
  realtime_urdf_filter::RealtimeURDFFilter f(nh, argc, argv);
  realtime_urdf_filter::DepthAndInfoSubscriber sub (nh, boost::bind (&realtime_urdf_filter::RealtimeURDFFilter::filter_callback, &f, _1, _2));

  // spin that shit!
  ros::spin ();

  return 0;
}

