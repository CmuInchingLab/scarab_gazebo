#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
<math/gzmath.hh>
namespace gazebo
{
  class CostMap : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {// do nothing
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    static double heightMapZ(double x, double y){
        // getting the pointer to the HeightmapShape
        private: physics::WorldPtr world = this->model->GetWorld();
        physics::ModelPtr model = world->GetModel("worldHeightmap");
        physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
        physics::HeightmapShapePtr heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

        // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y) 
        math::Vector3 size = this->heightmap->GetSize(); 
        math::Vector2i vc = this->heightmap->GetVertexCount();
        int index_x = (((x + size.x/2)/size.x ) * vc.x - 1) ;
        int index_y = (((-y + size.y/2)/size.y ) * vc.y - 1) ;

        //  getting the height :
        double z =  this->heightmap->GetHeight( index_x , index_y ) ; 

        return z;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CostMap)
}