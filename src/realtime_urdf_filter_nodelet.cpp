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

#include <realtime_urdf_filter/realtime_urdf_filter_nodelet.h>
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(realtime_urdf_filter::RealtimeURDFFilterNodelet, nodelet::Nodelet);

namespace realtime_urdf_filter
{
  RealtimeURDFFilterNodelet::RealtimeURDFFilterNodelet() :
    args_(),
    argc_(0),
    argv_(NULL)
  {

  }

  void RealtimeURDFFilterNodelet::onInit()
  {
    NODELET_DEBUG("Initializing nodelet...");

    // Free previously allocated memory
    if(argv_) {
      for(unsigned int i=0; i<argc_; i++) {
        delete argv_[i];
      }
      delete[] argv_;
      argc_ = 0;
    }

    // Convert argv to c-style argc,argv for OpenGL subsystem
    args_ = this->getMyArgv();
    argc_ = args_.size();
    argv_ = new char*[argc_];
    for(unsigned int i=0; i<argc_; i++) {
      // Yay constructing null-terminated strings...
      argv_[i] = new char[args_[i].size()+1];
      strncpy(argv_[i], args_[i].c_str(), args_[i].size()+1);
    }

    // Get nodelet node handle
    ros::NodeHandle nh = this->getPrivateNodeHandle();

    // Create the filter
    filter_.reset(new realtime_urdf_filter::RealtimeURDFFilter(nh, argc_, argv_));
  }
}
