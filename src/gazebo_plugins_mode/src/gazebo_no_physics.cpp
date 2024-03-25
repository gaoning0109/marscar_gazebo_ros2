#include "gazebo_plugins/gazebo_no_physics.hpp"

namespace gazebo_plugins
{

GazeboNoPhysicsPlugin::GazeboNoPhysicsPlugin()
{
}
GazeboNoPhysicsPlugin::~GazeboNoPhysicsPlugin()
{
}

void GazeboNoPhysicsPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    printf("Gazebo Plugin: 'No World Physics' loaded...");

    world_ = world;
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboNoPhysicsPlugin::onWorldUpdateBegin, this));
}

void GazeboNoPhysicsPlugin::onWorldUpdateBegin()
{
#if GAZEBO_MAJOR_VERSION >= 8
    if (world_->PhysicsEnabled())
    {
        world_->SetPhysicsEnabled(false);
    }
#else
    if (world_->GetEnablePhysicsEngine())
        world_->EnablePhysicsEngine(false);
#endif
}

}  // namespace gazebo_plugins