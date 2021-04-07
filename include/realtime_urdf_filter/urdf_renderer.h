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

#ifndef REALTIME_URDF_FILTER_URDF_RENDERER_H_
#define REALTIME_URDF_FILTER_URDF_RENDERER_H_

#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <realtime_urdf_filter/renderable.h>
#include <unordered_set>

// forward declares
namespace ros {class NodeHandle;}

namespace realtime_urdf_filter
{

class URDFRenderer
{
  public:
    URDFRenderer (std::string model_description, std::string tf_prefix,
                  std::string cam_frame, std::string fixed_frame, tf::TransformListener &tf,
                  const std::string &geometry_type, double scale, const std::unordered_set<std::string> &ignore);
    void render(ros::Time timestamp = ros::Time());

  protected:
    void initURDFModel ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (std::shared_ptr<urdf::Link> link);
    void update_link_transforms(ros::Time timestamp = ros::Time());

    // urdf model stuff
    std::string model_description_;
    std::string tf_prefix_;
    const std::string geometry_type;
    const double scale;
    const std::unordered_set<std::string> ignore;

    // camera stuff
    std::string camera_frame_;
    std::string fixed_frame_;

    // rendering stuff
    std::vector<std::shared_ptr<Renderable> > renderables_;
    tf::TransformListener &tf_;
};

} // end namespace

#endif
