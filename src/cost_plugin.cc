#include <thread>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/physics/physics.hh>
// #include <ignition/math/Vector3.hh>
#include <gazebo/math/Vector2i.hh>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <scarab_gazebo/PointArr.h>

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
  unique_ptr<ros::NodeHandle> rosNode;  // A node use for ROS transport
  ros::CallbackQueue rosQueue;  // A ROS callbackqueue that helps process messages
  thread rosQueueThread; //A thread the keeps running the rosQueue
  ros::Subscriber req_z_sub;
  ros::Publisher send_z_pub;
  math::Vector3 size;
  math::Vector2i vc;

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

    // service = rosNode->advertiseService("getZ", &CostPlugin::heightMapZ, this);
    // http://gazebosim.org/tutorials?tut=guided_i6
    // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("scarab_gazebo"));

    //set subscriber options and subscribe to "request_z" topic
    ros::SubscribeOptions so = ros::SubscribeOptions::create<scarab_gazebo::PointArr>(
      "/scarab_gazebo/request_z", 1, boost::bind(&CostPlugin::heightMapZ, this, _1), ros::VoidPtr(), &this->rosQueue);

    this->req_z_sub = this->rosNode->subscribe(so);
    send_z_pub = rosNode->advertise< scarab_gazebo::PointArr>("/scarab_gazebo/response_z", 10);
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&CostPlugin::QueueThread, this));
    ROS_INFO("WORLD PLUGIN INITIALIZED. YEA BOIII");
  }

  // return z value at given x and y coordaintes. 
  // ROS/Gazebo are great, aren't they?
  void heightMapZ(const scarab_gazebo::PointArrConstPtr& pointArr)
  {
    if (world_ptr == NULL)
    { // only initialize once
      world_ptr = physics::get_world();       // getting the pointer to the HeightmapShape
      model = world_ptr->GetModel("heightmap");
      physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
      heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
      this->size = this->heightmap->GetSize();
      this->vc = this->heightmap->GetVertexCount();

      // physics::ModelPtr robot = world_ptr->GetModel("scarab");
      // // math::Vector3 bot_on_surface(0,200,0);
      // cout <<"Set robot on top of the surface" << endl;
      // robot->SetWorldPose(math::Pose(0, 0, 50, 0, 0, 0));

    }

    if (pointArr->points.size() > 0){
      scarab_gazebo::PointArr resp;
      resp.id = pointArr->id;
      for (int i = 0; i < pointArr->points.size(); i++){
        float x = pointArr->points[i].x;
        float y = pointArr->points[i].y;
        geometry_msgs::Point curr_pt;
        curr_pt.x = x;
        curr_pt.y = y;
        
        // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y)
        int index_x = (((x + size.x / 2) / size.x) * vc.x - 1);
        int index_y = (((-y + size.y / 2) / size.y) * vc.y - 1);
        curr_pt.z = this->heightmap->GetHeight(index_x, index_y);  //  getting the height
        resp.points.push_back(curr_pt);
      }
      send_z_pub.publish(resp);
      }
  }

  // ROS helper function that processes messages
  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
};
GZ_REGISTER_WORLD_PLUGIN(CostPlugin);
} 
