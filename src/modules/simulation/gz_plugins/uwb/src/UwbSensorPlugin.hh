#pragma once
#include <random>
#include <chrono>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include "gtec_uwb/types.hh"
#include "gtec_uwb/UwbChannelModel.hh"
#include "gtec_uwb/RayObstacleSet.hh"

namespace gtec_uwb {

class UwbSensorPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override;

 private:
  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity tagLink_{gz::sim::kNullEntity};
  bool useParentAsReference_{false};

  UwbParams params_;
  std::string anchorPrefix_{"uwb_anchor"};
  std::chrono::steady_clock::duration updatePeriod_{};
  std::chrono::steady_clock::duration lastUpdate_{};
  unsigned int sequence_{0};

  UwbChannelModel model_;
  RayObstacleSet obstacles_;
  std::default_random_engine rng_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};

}  // namespace gtec_uwb
