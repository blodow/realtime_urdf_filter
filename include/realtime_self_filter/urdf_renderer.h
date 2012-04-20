#ifndef REALTIME_PERCEPTION_URDF_RENDERER_H_
#define REALTIME_PERCEPTION_URDF_RENDERER_H_

#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <realtime_self_filter/renderable.h>

// forward declares
namespace ros {class NodeHandle;}

namespace realtime_self_filter
{

class URDFRenderer
{ 
  public:
    URDFRenderer (std::string model_description, std::string tf_prefix, std::string cam_frame, std::string fixed_frame, tf::TransformListener &tf);
    void render ();

  protected:
    void initURDFModel ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (boost::shared_ptr<urdf::Link> link);
    void update_link_transforms ();

    // urdf model stuff
    std::string model_description_;
    std::string tf_prefix_;
    
    // camera stuff
    std::string camera_frame_;
    std::string fixed_frame_;
   
    // rendering stuff 
    std::vector<boost::shared_ptr<Renderable> > renderables_;
    tf::TransformListener &tf_;
};

} // end namespace

#endif
