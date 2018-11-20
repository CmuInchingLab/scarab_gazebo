#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/math/Vector2i.hh>
// http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
class CostPlugin: public WorldPlugin
{
private:
  physics::WorldPtr world_ptr = NULL;
  physics::ModelPtr model = NULL;
  physics::HeightmapShapePtr heightmap = NULL;
  physics::CollisionPtr collision = NULL;

public:
 CostPlugin() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    ROS_INFO("PLUGIN INITIALIZED. YEAHHHHHHH BOIIII");
  }

double heightMapZ(double x, double y){
    if (world_ptr == NULL){ // only initialize once
        // getting the pointer to the HeightmapShape
        world_ptr = physics::get_world();
        model = world_ptr->GetModel("worldHeightmap");
        physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
        heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
    }

    // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y) 
    math::Vector3 size = this->heightmap->GetSize(); 
    math::Vector2i vc = this->heightmap->GetVertexCount();
    int index_x = (((x + size.x/2)/size.x ) * vc.x - 1) ;
    int index_y = (((-y + size.y/2)/size.y ) * vc.y - 1) ;

    //  getting the height :
    double z =  this->heightmap->GetHeight( index_x , index_y ); 

    return z;
}

};
GZ_REGISTER_WORLD_PLUGIN(CostPlugin);
}
