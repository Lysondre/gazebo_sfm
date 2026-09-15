
#include "gazebo_sfm/GazeboSFM.hpp"

#include <gz/plugin/Register.hh>

IGNITION_ADD_PLUGIN(gazebo_sfm::GazeboSFM, gz::sim::System,
                    gazebo_sfm::GazeboSFM::ISystemConfigure,
                    gazebo_sfm::GazeboSFM::ISystemConfigure)

using namespace gazebo_sfm;

GazeboSFM::GazeboSFM()
{
}

GazeboSFM::~GazeboSFM()
{
}

void GazeboSFM::PreUpdate(const gz::sim::UpdateInfo& _info,
                          gz::sim::EntityComponentManager& ecm)
{
}

void GazeboSFM::Configure(const gz::sim::Entity& _entity,
                          const std::shared_ptr<const sdf::Element>& _sdf,
                          gz::sim::EntityComponentManager& _ecm,
                          gz::sim::EventManager& _eventMgr)
{
}
