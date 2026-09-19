
#include "gazebo_sfm/GazeboSFM.hpp"

#include <chrono>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>
#include <list>
#include <sdf/Actor.hh>

using namespace std::chrono_literals;

IGNITION_ADD_PLUGIN(gazebo_sfm::GazeboSFM, gz::sim::System,
                    gazebo_sfm::GazeboSFM::ISystemConfigure,
                    gazebo_sfm::GazeboSFM::ISystemPreUpdate)

using namespace gazebo_sfm;

GazeboSFM::GazeboSFM() : creator{nullptr}, actor_sdf()
{
    ignmsg << "constructing" << std::endl;
    create_actor_sdf();
}

GazeboSFM::~GazeboSFM()
{
}

void GazeboSFM::Configure(const gz::sim::Entity& _entity,
                          const std::shared_ptr<const sdf::Element>& _sdf,
                          gz::sim::EntityComponentManager& _ecm,
                          gz::sim::EventManager& _eventMgr)
{
    ignmsg << "Configuring GazeboSFM." << std::endl;

    worldEntity = _ecm.EntityByComponents(gz::sim::components::World());

    creator = std::make_unique<gz::sim::SdfEntityCreator>(_ecm, _eventMgr);

    ensure_n_actors(2);

    gz::math::Pose3d pose{};

    pose.SetX(1.0);

    agents[0].position.setX(1.0);

    gz::sim::Actor actor{static_cast<unsigned long>(agents[0].id)};
    actor.SetTrajectoryPose(_ecm, pose);

    sfm::Goal goal1{utils::Vector2d{0.5, 3.0}, 0.4};
    sfm::Goal goal2{utils::Vector2d{0.5, 7.0}, 0.4};

    agents[0].goals = std::list<sfm::Goal>{goal1};
    agents[1].goals = std::list<sfm::Goal>{goal2};

    ignmsg << "Configured GazeboSFM." << std::endl;
}

void GazeboSFM::PreUpdate(const gz::sim::UpdateInfo& _info,
                          gz::sim::EntityComponentManager& ecm)
{
    double delta_s = std::chrono::duration<double>(_info.dt).count();

    sfm::SFM.computeForces(agents);
    sfm::SFM.updatePosition(agents, delta_s);

    for (int i{}; i < agents.size(); i++)
    {
        auto& agent = agents[i];

        // NOTE: I don't particularly like casting from signed to unsigned
        // mind the wrapping
        gz::sim::Actor actor{static_cast<unsigned long>(agent.id)};

        double speed = agent.velocity.norm();

        gz::math::Pose3d pose{};

        pose.SetX(agent.position.getX());
        pose.SetY(agent.position.getY());
        pose.Set(agent.position.getX(), agent.position.getY(), 0.0, 0.0, 0.0,
                 agent.velocity.angle().toRadian());

        actor.SetTrajectoryPose(ecm, pose);

        // auto animTime = animTimeComp->Data() +
        //     std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        //     std::chrono::duration<double>(distanceTraveled *
        //     this->dataPtr->animationXVel));

        double distance_traveled = speed * delta_s;

        auto anim_time_comp = *actor.AnimationTime(ecm);
        // auto anim_time_comp =
        // ecm.Component<gz::sim::components::AnimationTime>(
        //     static_cast<unsigned long>(agent.id));

        if (speed < 0.05)
        {
            actor.SetAnimationName(ecm, "stand");
            ecm.SetChanged(static_cast<unsigned long>(agent.id),
                           gz::sim::components::AnimationName::typeId,
                           gz::sim::ComponentState::OneTimeChange);
        }
        else
        {
            actor.SetAnimationName(ecm, "walk");
            ecm.SetChanged(static_cast<unsigned long>(agent.id),
                           gz::sim::components::AnimationName::typeId,
                           gz::sim::ComponentState::OneTimeChange);
        }

        auto anim_time =
            anim_time_comp +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(distance_traveled * 1.5));

        actor.SetAnimationTime(ecm, anim_time);

        ecm.SetChanged(static_cast<unsigned long>(agent.id),
                       gz::sim::components::TrajectoryPose::typeId,
                       gz::sim::ComponentState::OneTimeChange);
        ecm.SetChanged(static_cast<unsigned long>(agent.id),
                       gz::sim::components::AnimationTime::typeId,
                       gz::sim::ComponentState::OneTimeChange);
    }
}

void GazeboSFM::create_actor_sdf()
{
    actor_sdf.SetSkinFilename(ACTOR_FILENAME);
    actor_sdf.SetSkinScale(1.0);

    sdf::Animation walk_anim;

    // it happens that the model I'm using has the same filename for both
    walk_anim.SetFilename(ACTOR_FILENAME);
    walk_anim.SetName("walk");
    walk_anim.SetInterpolateX(true);

    sdf::Animation stand_anim;

    stand_anim.SetFilename(ACTOR_STANDING_FILENAME);
    stand_anim.SetName("stand");
    stand_anim.SetInterpolateX(true);

    actor_sdf.AddAnimation(walk_anim);
    actor_sdf.AddAnimation(stand_anim);

    // temp name
    actor_sdf.SetName("John");

    gz::math::Pose3d pose;

    actor_sdf.SetRawPose(pose);
}

void GazeboSFM::spawn_actor()
{
    std::string actor_name = "pedestrian_";
    actor_name += std::to_string(agents.size());

    actor_sdf.SetName(actor_name);

    gz::sim::Entity new_entity = creator->CreateEntities(&actor_sdf);

    creator->SetParent(new_entity, worldEntity);

    sfm::Agent new_agent{};

    new_agent.id = new_entity;

    agents.push_back(new_agent);
    anim_cumul.push_back(0.0);
}

void GazeboSFM::remove_actor()
{
    ignlog << "Removing an actor..." << std::endl;

    if (!agents.empty())
    {
        auto entity = agents.back().id;

        creator->RequestRemoveEntity(entity, true);

        agents.pop_back();
        anim_cumul.pop_back();
    }
}

void GazeboSFM::ensure_n_actors(int n)
{
    // avoid wrapping
    int diff = n - static_cast<long>(agents.size());

    int steps = std::abs(diff);

    for (int i{}; i < steps; i++)
    {
        diff < 0 ? remove_actor() : spawn_actor();
    }
}
