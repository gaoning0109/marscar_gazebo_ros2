#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

namespace gazebo_plugins
{

class GazeboNoPhysicsPlugin : public gazebo::WorldPlugin
{
  private:
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;

  public:
    GazeboNoPhysicsPlugin();
    virtual ~GazeboNoPhysicsPlugin();

  protected:
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf);

  private:
    void onWorldUpdateBegin();
};

GZ_REGISTER_WORLD_PLUGIN(GazeboNoPhysicsPlugin)

}  // namespace gazebo_plugins