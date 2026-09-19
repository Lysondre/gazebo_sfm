
#ifndef GAZEBO_SFM_HPP_123
#define GAZEBO_SFM_HPP_123

#include <gz/sim/Entity.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <ignition/gazebo/Actor.hh>

#include <lightsfm/sfm.hpp>

#include <memory>
#include <sdf/Actor.hh>
#include <vector>

#include <string>

namespace gazebo_sfm
{

const std::string ACTOR_FILENAME = "https://fuel.gazebosim.org/1.0/Mingfei/"
                                   "models/actor/tip/files/meshes/walk.dae";

const std::string ACTOR_STANDING_FILENAME =
    "https://fuel.gazebosim.org/1.0/Mingfei/"
    "models/actor/tip/files/meshes/stand.dae";

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

  private:
    std::unique_ptr<gz::sim::SdfEntityCreator> creator;
    std::vector<sfm::Agent> agents;
    std::vector<double> anim_cumul;
    sdf::Actor actor_sdf;

    gz::sim::Entity worldEntity{gz::sim::kNullEntity};

    void create_actor_sdf();
    void spawn_actor();
    void remove_actor();
    void ensure_n_actors(int n);
};

} // namespace gazebo_sfm

#endif
