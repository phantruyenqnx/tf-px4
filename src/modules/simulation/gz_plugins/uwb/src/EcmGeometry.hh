#pragma once
#include <string>
#include <vector>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include "gtec_uwb/RayObstacleSet.hh"

namespace gtec_uwb {

struct AnchorRef { gz::sim::Entity entity; int id; };

// Fill `out` with every box collision in the world except those belonging to
// `selfModel`. Detect a ground plane (collision plane geometry) if present.
void BuildObstacleSet(const gz::sim::EntityComponentManager &ecm,
                      gz::sim::Entity selfModel,
                      RayObstacleSet &out);

// Find anchors: models (or links) whose name starts with `prefix`.
std::vector<AnchorRef> FindAnchors(const gz::sim::EntityComponentManager &ecm,
                                   const std::string &prefix);

}  // namespace gtec_uwb
