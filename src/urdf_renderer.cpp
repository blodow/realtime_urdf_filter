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


#include <GL/glew.h>
#include <GL3/gl3.h>
#include <GL/glu.h>
#include <GL/glx.h>
#undef Success  // <---- Screw Xlib for this

#include <ros/node_handle.h>

#include <realtime_urdf_filter/urdf_renderer.h>

namespace realtime_urdf_filter
{
  URDFRenderer::URDFRenderer (std::string model_description,
                              std::string tf_prefix,
                              std::string cam_frame,
                              std::string fixed_frame,
                              tf::TransformListener &tf,
                              const std::string &geometry_type,
                              double scale,
                              const std::unordered_set<std::string> &ignore)
    : model_description_(model_description)
    , tf_prefix_(tf_prefix)
    , geometry_type(geometry_type)
    , scale(scale)
    , ignore(ignore)
    , camera_frame_ (cam_frame)
    , fixed_frame_(fixed_frame)
    , tf_(tf)
  {
    initURDFModel ();
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Loads URDF model from the parameter server and parses it. call loadURDFModel */
  void
    URDFRenderer::initURDFModel ()
  {
    urdf::Model model;
    if (!model.initString(model_description_))
    {
      ROS_FATAL ("URDF failed Model parse");
      return;
    }

    ROS_INFO ("URDF parsed OK");
    loadURDFModel (model);
    ROS_INFO ("URDF loaded OK");
  }


  /// /////////////////////////////////////////////////////////////////////////////
  /// @brief load URDF model description from string and create search operations data structures
  void URDFRenderer::loadURDFModel
    (urdf::Model &model)
  {
    typedef std::vector<std::shared_ptr<urdf::Link> > V_Link;
    V_Link links;
    model.getLinks(links);

    V_Link::iterator it = links.begin();
    V_Link::iterator end = links.end();

    for (; it != end; ++it)
      process_link (*it);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Processes a single URDF link, creates renderable for it */
  void URDFRenderer::process_link (std::shared_ptr<urdf::Link> link)
  {
    std::vector<urdf::Pose> origins;
    std::vector<urdf::GeometryConstSharedPtr> geometries;
    std::vector<urdf::MaterialConstSharedPtr> materials;

    // skip links that are not to be used for mask
    if (ignore.count(link->name)) { return; }

    if (geometry_type.empty() || geometry_type == "visual")
    {
      for (const auto &visual : link->visual_array)
      {
        origins.push_back(visual->origin);
        geometries.push_back(visual->geometry);
        materials.push_back(visual->material);
      }
    }
    else if (geometry_type == "collision")
    {
      for (const auto &collision : link->collision_array)
      {
        origins.push_back(collision->origin);
        geometries.push_back(collision->geometry);
        materials.push_back(nullptr);
      }
    }
    else
    {
      ROS_FATAL_STREAM("invalid geometry type: " + geometry_type);
    }

    for (size_t i=0; i<geometries.size(); i++)
    {
      const urdf::Pose &origin = origins[i];
      const urdf::GeometryConstSharedPtr &geometry = geometries[i];
      const urdf::MaterialConstSharedPtr &material = materials[i];

      std::shared_ptr<Renderable> r;
      if (geometry->type == urdf::Geometry::BOX)
      {
        const urdf::BoxConstSharedPtr box = std::dynamic_pointer_cast<const urdf::Box> (geometry);
        r = std::make_shared<RenderableBox>(scale * box->dim.x, scale * box->dim.y, scale * box->dim.z);
      }
      else if (geometry->type == urdf::Geometry::CYLINDER)
      {
        const urdf::CylinderConstSharedPtr cylinder = std::dynamic_pointer_cast<const urdf::Cylinder> (geometry);
        r = std::make_shared<RenderableCylinder>(scale * cylinder->radius, scale * cylinder->length);
      }
      else if (geometry->type == urdf::Geometry::SPHERE)
      {
        const urdf::SphereConstSharedPtr sphere = std::dynamic_pointer_cast<const urdf::Sphere> (geometry);
        r = std::make_shared<RenderableSphere>(scale * sphere->radius);
      }
      else if (geometry->type == urdf::Geometry::MESH)
      {
        const urdf::MeshConstSharedPtr mesh = std::dynamic_pointer_cast<const urdf::Mesh> (geometry);
        r = std::make_shared<RenderableMesh>(mesh->filename, scale * mesh->scale.x, scale * mesh->scale.y, scale * mesh->scale.z);
      }
      r->setLinkName (tf_prefix_+ "/" + link->name);
      const urdf::Vector3 position = origin.position;
      const urdf::Rotation rotation = origin.rotation;
      r->link_offset = tf::Transform (
          tf::Quaternion (rotation.x, rotation.y, rotation.z, rotation.w).normalize (),
          tf::Vector3 (position.x, position.y, position.z));
      if (material)
        r->color  = material->color;
      renderables_.push_back (r);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief loops over all renderables and updates its transforms from TF */
  void URDFRenderer::update_link_transforms (ros::Time timestamp)
  {
    tf::StampedTransform t;

    std::vector<std::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++)
    {
      try
      {
        tf_.lookupTransform (fixed_frame_, (*it)->name, timestamp, t);
      }
      catch (tf::TransformException ex)
      {
        ROS_DEBUG_STREAM(ex.what());
      }
      (*it)->link_to_fixed = tf::Transform (t.getRotation (), t.getOrigin ());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief loops over all renderables and renders them to canvas */
  void URDFRenderer::render (ros::Time timestamp)
  {
    update_link_transforms (timestamp);

    std::vector<std::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++)
      (*it)->render ();
  }

}

// REGEX BASED LINK / SEARCH OPERATIONS / TARGET FRAMES SETUP
//    for (sop_it = search_operations_.begin (); sop_it != search_operations_.end (); sop_it++)
//    {
//      std::smatch res;
//      if (std::regex_match(link->name, res, sop_it->re))
//      {
//        if (!sop_it->pub_topic_re.empty ())
//        {
//          std::string publish_topic = res.format (sop_it->pub_topic_re);
//          sop_it->pub_topic = publish_topic;
//          ROS_INFO ("Regex expanded publisher topic to: %s", publish_topic.c_str());
//          publishers_ [publish_topic] = pcl_ros::Publisher<pcl::PointXYZ>
//                                          (private_nh, publish_topic, max_queue_size_);
//        }
//        ROS_INFO ("Match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
//        double r,p,y;
//
//        // handle special case of fixed links of drawers
//        std::shared_ptr<urdf::Collision> c;
//        std::regex re (".*_fixed_link");
//        if (std::regex_match(link->name, re))
//        {
//          c = link->child_links[0]->collision;
//
//          // handle special case of search_expand* params for drawers
//          std::map <std::string, XmlRpc::XmlRpcValue>::const_iterator op_it = sop_it->params.find ("operation");
//          if (op_it != sop_it->params.end())
//          {
//            if (op_it->second == std::string ("fit_drawer"))
//            {
//              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
//              sop_it->params["search_expand_distance"] = link->child_joints[0]->limits->upper
//                                                       - link->child_joints[0]->limits->lower;
//              sop_it->params["search_expand_axis"][0] = link->child_joints[0]->axis.x;
//              sop_it->params["search_expand_axis"][1] = link->child_joints[0]->axis.y;
//              sop_it->params["search_expand_axis"][2] = link->child_joints[0]->axis.z;
//              if (sop_it->params.count ("min_drawer_inliers") == 0)
//                sop_it->params["min_drawer_inliers"] = 50;
//            }
//            else if (op_it->second == std::string ("fit_door"))
//            {
//              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
//              if (sop_it->params.count ("min_drawer_inliers") == 0)
//                sop_it->params["min_drawer_inliers"] = 50;
//            }
//          }
//        }
//        else
//          c = link->collision;
//
//        c->origin.rotation.getRPY (r,p,y);
//        if (r != 0.0 || p != 0.0 || y != 0.0)
//        {
//          // we have a rotation here, so we need to add this frame to target_frames_
//          TargetFrames tfr;
//          tfr.frame = tf_prefix_ + "/" + link->name;
//          tfr.op = (*sop_it);
//          tfr.translation = (urdf::Vector3());
//          tfr.link = (link);
//          target_frames_.push_back (tfr);
//        }
//        else
//        {
//          // regex matches this link... so let's walk it up...
//          std::string found_link;
//          urdf::Vector3 null_offset;
//          if (walk_back_links (link, link, *sop_it, stop_link_, null_offset))
//          {
//            // do something ?
//          }
//        }
//
//      }
//      else
//      {
//        ROS_INFO ("No match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
//      }
//    }
