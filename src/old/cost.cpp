#include "ros/ros.h"
// http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
// #include "std_msgs/String.h"
// #include <sstream>

using namespace std;

// http://answers.gazebosim.org/question/17167/how-to-get-the-terrain-elevation-z-at-specific-xy-location/?answer=17192#post-id-17192
double heightMapZ(double x, double y)  {
    // getting the pointer to the HeightmapShape
    // gazebo::physics::WorldPtr world = this->model->GetWorld();
    gazebo::physics::ModelPtr model = gazebo::physics::World::GetModel("worldHeightmap");
    gazebo::physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
    gazebo::physics::HeightmapShapePtr heightmap = boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(collision->GetShape());

    // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y) 
    ignition::math::Vector3 size = heightmap->GetSize(); 
    ignition::math::Vector2i vc = heightmap->GetVertexCount();
    int index_x = (( (x + size.x/2)/size.x) * vc.x - 1) ;
    int index_y = (((-y + size.y/2)/size.y) * vc.y - 1) ;

    //  getting the height :
    double z =  heightmap->GetHeight( index_x , index_y ) ; 
    return z;
}

void pose_cb(const gazebo_msgs::ModelStates& gazebo_msg){ 
    // scarab robot is 2nd element in model position array
    geometry_msgs::Point position = gazebo_msg.pose[1].position;
    cout << "In callback. X:" << position.x << " Y:" << position.y << endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_scarab");
  ros::NodeHandle n;
//   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cost", 1000);
    ros::Subscriber sub = n.subscribe("/gazebo/model_states", 100, pose_cb);
  ros::Rate loop_rate(10);

  int count = 0;


  while (ros::ok())
  {

    // chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    // ++count;
  }


  return 0;
}