#include <thread>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/math/Vector2i.hh>
// http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include "scarab_gazebo/getZ.h"

using namespace std;
namespace gazebo
{
class CostPlugin : public WorldPlugin
{
private:
  physics::WorldPtr world_ptr = NULL;
  physics::ModelPtr model = NULL;
  physics::HeightmapShapePtr heightmap = NULL;
  physics::CollisionPtr collision = NULL;
  ros::ServiceServer service;
  unique_ptr<ros::NodeHandle> rosNode;

public:
  CostPlugin() : WorldPlugin(){}

  // Destructor
  ~CostPlugin(){}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("scarab_gazebo"));
    service = rosNode->advertiseService("getZ", &CostPlugin::heightMapZ, this);
    ROS_INFO("WORLD PLUGIN INITIALIZED. YEA BOIII");
  }

  bool heightMapZ(scarab_gazebo::getZ::Request &req, scarab_gazebo::getZ::Response &res)
  {
    if (world_ptr == NULL)
    { // only initialize once
      // getting the pointer to the HeightmapShape
      world_ptr = physics::get_world();
      model = world_ptr->GetModel("Victoria_Crater");
      // physics::CollisionPtr collision = model->GetLink("victoria_crater_link")->GetCollision("victoria_crater_collision");
      // heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
    }

    // // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y)
    // math::Vector3 size = this->heightmap->GetSize();
    // math::Vector2i vc = this->heightmap->GetVertexCount();
    // int index_x = (((req.x + size.x / 2) / size.x) * vc.x - 1);
    // int index_y = (((-req.y + size.y / 2) / size.y) * vc.y - 1);

    // //  getting the height
    // res.z = this->heightmap->GetHeight(index_x, index_y);
    res.z = 0;
    return true;
  }
};
GZ_REGISTER_WORLD_PLUGIN(CostPlugin);
} // namespace gazebo
