
#ifndef GAZEBO_SFM_HPP_123
#define GAZEBO_SFM_HPP_123

#include <gz/sim/System.hh>

namespace gazebo_sfm
{

class GazeboSFM : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate
{
  public:
    GazeboSFM();
    ~GazeboSFM() override;

    void PreUpdate(const gz::sim::UpdateInfo& _info,
                   gz::sim::EntityComponentManager& ecm) override;

    void Configure(const gz::sim::Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager& _ecm,
                   gz::sim::EventManager& _eventMgr) override;
};

} // namespace gazebo_sfm

#endif
