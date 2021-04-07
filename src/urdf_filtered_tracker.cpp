/*
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>, Matteo Saveriano
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

#include "realtime_urdf_filter/urdf_filter.h"
#include "realtime_urdf_filter/urdf_renderer.h"

#include <ros/node_handle.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

#include <std_msgs/String.h>

#include <sstream>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	}
		// TODO: return nRetVal;

class OpenNITrackerLoopback
{
public:
  OpenNITrackerLoopback (ros::NodeHandle &nh, int argc, char **argv)
    : g_bNeedPose (false)
    , nh_ (nh)
    , argc_ (argc), argv_(argv)
  {
    g_strPose[0] = 0x0;
    XnStatus nRetVal = XN_STATUS_OK;

    xn::EnumerationErrors errors;
    // TODO: Matteo had: "/opt/ros/diamondback/ros/BodyTracker/SamplesConfig.xml"
    std::string configFilename = ros::package::getPath("realtime_urdf_filter") + "/include/SamplesConfig.xml";
    nRetVal = g_Context.InitFromXmlFile(configFilename.c_str(), g_scriptNode, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
      XnChar strError[1024];
      errors.ToString(strError, 1024);
      printf("%s\n", strError);
      // TODO: return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK)
    {
      printf("Open failed: %s\n", xnGetStatusString(nRetVal));
      // TODO: return (nRetVal);
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    // TODO: can we remove this?
    if (nRetVal != XN_STATUS_OK)
    {
      printf("No depth generator found. Using a default one...");
      xn::MockDepthGenerator _mockDepth;
      nRetVal = mockDepth.Create(g_Context);
      CHECK_RC(nRetVal, "Create mock depth");

      // set some defaults
      XnMapOutputMode defaultMode;
      defaultMode.nXRes = 320;
      defaultMode.nYRes = 240;
      defaultMode.nFPS = 30;
      nRetVal = mockDepth.SetMapOutputMode(defaultMode);
      CHECK_RC(nRetVal, "set default mode");

      // set FOV
      XnFieldOfView fov;
      fov.fHFOV = 1.0225999419141749;
      fov.fVFOV = 0.79661567681716894;
      nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
      CHECK_RC(nRetVal, "set FOV");

      XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
      XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

      nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
      CHECK_RC(nRetVal, "set empty depth map");

      g_DepthGenerator = mockDepth;
    }

    // by Heresy, create mock node
    mockDepth.CreateBasedOn( g_DepthGenerator, "mock-depth" );
    g_DepthGenerator.GetMetaData(depthMD);

    // by Heresy, create an user generator fot mock depth node
    xn::Query xQuery;
    xQuery.AddNeededNode( "mock-depth" );
    nRetVal = g_UserGenerator.Create( g_Context, &xQuery );
    CHECK_RC(nRetVal, "Find user generator");

    XnCallbackHandle hUserCallbacks, hCalibrationComplete, hPoseDetected;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
      printf("Supplied user generator doesn't support skeleton\n");
      // TODO: return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, this, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
      g_bNeedPose = TRUE;
      if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
      {
        printf("Pose required, but not supported\n");
        // TODO: return 1;
      }
      nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, this, hPoseDetected);
      CHECK_RC(nRetVal, "Register to Pose Detected");
      g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    tf_pub_ = nh_.advertise<tf::tfMessage> ("/tf_human", 1);
    event_pub_ = nh_.advertise<std_msgs::String> ("user_event", 1);

    setupURDFSelfFilter ();
  }

  void setupURDFSelfFilter ()
  {
    filter = new realtime_urdf_filter::RealtimeURDFFilter (nh_, argc_, argv_);
    filter->width_ = 640;
    filter->height_ = 480;
    filter->initGL ();
  }

  ~OpenNITrackerLoopback ()
  {
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    mockDepth.Release();
    g_UserGenerator.Release();
    g_Player.Release();
    g_Context.Release();
  }

  void runOnce (void)
  {
    xn::SceneMetaData sceneMD;
    xn::DepthMetaData depthMD_;

    // Read next available data
    // by Heresy, wait original depth generator
    g_Context.WaitOneUpdateAll( g_DepthGenerator );

    // Process the data
    g_DepthGenerator.GetMetaData (depthMD_);
    g_DepthGenerator.GetMetaData (depthMD);
    depthMD_.MakeDataWritable();
    xn::DepthMap& depthMap_ = depthMD_.WritableDepthMap();
    xn::DepthMap& depthMap = depthMD.WritableDepthMap();

    static float *buffer = 0;
    if (buffer == 0)
    {
      std::cout << "(re)allocating depth buffer" << std::endl;
      buffer = (float*) malloc (depthMap.XRes() * depthMap.YRes() * sizeof(float));
    }
    for (XnUInt y = 0; y < depthMap.YRes(); y++)
    {
      for (XnUInt x = 0; x < depthMap.XRes(); x++)
      {
        buffer [x + y * depthMap.XRes()] = depthMap (depthMap.XRes() - x - 1, y) * 0.001;
      }
    }

    float P[12];
    P[0] = 585.260; P[1] = 0.0;     P[2]  = 317.387; P[3]  = 0.0;
    P[4] = 0.0;     P[5] = 585.028; P[6]  = 239.264; P[7]  = 0.0;
    P[8] = 0.0;     P[9] = 0.0;     P[10] = 1.0;     P[11] = 0.0;

    double fx = P[0];
    double fy = P[5];
    double cx = P[2];
    double cy = P[6];
    double far_plane_ = 8;
    double near_plane_ = 0.1;

    double glTf[16];
    for (unsigned int i = 0; i < 16; ++i)
      glTf[i] = 0.0;

    // calculate the projection matrix
    // NOTE: this minus is there to flip the x-axis of the image.
    glTf[0]= -2.0 * fx / depthMap.XRes();
    glTf[5]= 2.0 * fy / depthMap.YRes();

    glTf[8]= 2.0 * (0.5 - cx / depthMap.XRes());
    glTf[9]= 2.0 * (cy / depthMap.YRes() - 0.5);

    glTf[10]= - (far_plane_ + near_plane_) / (far_plane_ - near_plane_);
    glTf[14]= -2.0 * far_plane_ * near_plane_ / (far_plane_ - near_plane_);

    glTf[11]= -1;


    filter->filter ((unsigned char*)buffer, glTf, depthMap.XRes(), depthMap.YRes());

    GLfloat* masked_depth = filter->getMaskedDepth();

    for (XnUInt y = 0; y < depthMap.YRes(); y++)
    {
      for (XnUInt x = 0; x < depthMap.XRes(); x++)
      {
          depthMap_(x, y) = XnDepthPixel (masked_depth[x + y * depthMap.XRes()] * 1000);
      }
    }

    mockDepth.SetData(depthMD_);

	publishTransforms (std::string("openni_depth_frame"));
  }

  void publishEvent (std::string msg, XnUserID nId)
  {
      std::stringstream ss;
      ss << msg << " " << nId;
      std_msgs::String out;
      out.data = ss.str();
      event_pub_.publish (out);
  }

  static void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
  {
    OpenNITrackerLoopback* self = (OpenNITrackerLoopback*) pCookie;
    self->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection (nId);
    self->g_UserGenerator.GetSkeletonCap().RequestCalibration (nId, true);
    self->publishEvent ("calibrating", nId);
  }

  static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd (xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
  {
    OpenNITrackerLoopback* self = (OpenNITrackerLoopback*) pCookie;
    if (bSuccess)
    {
      self->g_UserGenerator.GetSkeletonCap().StartTracking (nId);
      self->publishEvent ("tracking", nId);
    }
    else
    {
      if (self->g_bNeedPose)
      {
        self->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection (self->g_strPose, nId);
        self->publishEvent ("posedetection", nId);
      }
      else
      {
        self->g_UserGenerator.GetSkeletonCap().RequestCalibration (nId, true);
        self->publishEvent ("calibrating", nId);
      }
    }
  }

  // Callback: An existing user was lost
  static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
  {
    OpenNITrackerLoopback* self = (OpenNITrackerLoopback*) pCookie;
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Lost user %d\n", epochTime, nId);
    self->publishEvent ("lost", nId);
  }

  // Callback: New user was detected
  static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
  {
    OpenNITrackerLoopback* self = (OpenNITrackerLoopback*) pCookie;
    if(nId > 15)
    {
      XnStatus res = self->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
      if (res != XN_CALIBRATION_STATUS_OK)
      {
        std::cout << "Error StopPoseDetection";
      }
      res = self->g_UserGenerator.GetSkeletonCap().AbortCalibration(nId);
      if (res != XN_CALIBRATION_STATUS_OK)
      {
        std::cout << "Error AbortCalibration";
      }
      res = self->g_UserGenerator.GetSkeletonCap().StopTracking(nId);
      if (res != XN_CALIBRATION_STATUS_OK)
      {
        std::cout << "Error StopTracking";
      }
      self->g_UserGenerator.GetPoseDetectionCap().Release();
      self->g_UserGenerator.GetSkeletonCap().Release();
      return;
    }
    // New user found
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d New User %d\n", epochTime, nId);
    self->publishEvent ("new", nId);

    if (self->g_bNeedPose)
    {
      std::cerr << "starting pose deteciton: " << __FILE__ << " : " << __LINE__ << std::endl;
      self->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(self->g_strPose, nId);
      self->publishEvent ("posedetection", nId);
    }
    else
    {
      std::cerr << "request calibration: " << __FILE__ << " : " << __LINE__ << std::endl;
      self->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
      self->publishEvent ("calibrating", nId);
    }
  }

  // Callback: Finished calibration
  static void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete (xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
  {
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    OpenNITrackerLoopback* self = (OpenNITrackerLoopback*) pCookie;
    std::cerr << "got here: " << __FILE__ << " : " << __LINE__ << std::endl;
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {
      // Calibration succeeded
      printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
      self->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
      self->publishEvent ("tracking", nId);
    }
    else
    {
      // Calibration failed
      printf("%d Calibration failed for user %d\n", epochTime, nId);
      if (self->g_bNeedPose)
      {
        self->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(self->g_strPose, nId);
        self->publishEvent ("posedetection", nId);
      }
      else
      {
        self->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        self->publishEvent ("calibrating", nId);
      }
    }
  }

  // publishes TF tree for each tracked user
  void publishTransforms (std::string const& frame_id)
  {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers (users, users_count);

    tf::tfMessage tfs;

    for (int i = 0; i < users_count; ++i)
    {
      XnUserID user = users[i];
      if (!g_UserGenerator.GetSkeletonCap().IsTracking (user))
        continue;

      std::stringstream ss;
      std::string user_suffix;
      ss << "_" << user;
      user_suffix = ss.str ();

      std::cerr << "publishTransforms: " << __FILE__ << " : " << __LINE__ << std::endl;

      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_HEAD,           frame_id, "head" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_NECK,           frame_id, "neck" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_TORSO,          frame_id, "torso" + user_suffix));

      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand" + user_suffix));

      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand" + user_suffix));

      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot" + user_suffix));

      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee" + user_suffix));
      tfs.transforms.push_back (getUserTransform (user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot" + user_suffix));
    }
    tf_pub_.publish (tfs);
  }

  // computes a stamped transform for each user joint
  geometry_msgs::TransformStamped
    getUserTransform(XnUserID const& user, XnSkeletonJoint joint, std::string const& frame_id, std::string const& child_frame_id)
  {
      XnSkeletonJointPosition joint_position;
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
      double x = -joint_position.position.X / 1000.0;
      double y = joint_position.position.Y / 1000.0;
      double z = joint_position.position.Z / 1000.0;

      XnSkeletonJointOrientation joint_orientation;
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

      XnFloat* m = joint_orientation.orientation.elements;
      tf::Matrix3x3 mat (m[0], m[1], m[2],
                         m[3], m[4], m[5],
                         m[6], m[7], m[8]);
      tf::Quaternion q;
      mat.getRotation (q);
      q.setY(-q.y());
      q.setZ(-q.z());

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(x, y, z));
      tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
      transform.setRotation(tf_q);

      geometry_msgs::TransformStamped msg;

      // see openni_tracker ticket #4994
      tf::Transform change_frame;
      change_frame.setOrigin(tf::Vector3(0, 0, 0));
      tf::Quaternion frame_rotation;
      frame_rotation.setRPY(1.5708, 0, 1.5708);
      change_frame.setRotation(frame_rotation);

      transform = change_frame * transform;

      transformStampedTFToMsg (tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id), msg);
      return msg;
  }

protected:
  xn::Context g_Context;
  xn::ScriptNode g_scriptNode;
  xn::DepthGenerator g_DepthGenerator;
  xn::MockDepthGenerator mockDepth;
  xn::UserGenerator g_UserGenerator;
  xn::Player g_Player;

  xn::DepthMetaData depthMD;

  XnBool g_bNeedPose;
  XnChar g_strPose[20];
  realtime_urdf_filter::RealtimeURDFFilter *filter;

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher tf_pub_;
  ros::Publisher event_pub_;

  // neccesary for glutInit()..
  int argc_;
  char **argv_;
};


// main function
int main (int argc, char **argv)
{
	sleep(1);

  // set up ROS
  ros::init (argc, argv, "urdf_filtered_tracker");
  ros::NodeHandle nh ("~");

  OpenNITrackerLoopback *lo = new OpenNITrackerLoopback (nh, argc, argv);

  // since OpenNI keeps giving you data even if no new data is available, #
  // we get crazy frequencies if we don't limit it artificially
  ros::Rate r(30);

	sleep(1);
  while (nh.ok())
  {
    lo->runOnce ();
    r.sleep ();
  }

  return 0;
}


