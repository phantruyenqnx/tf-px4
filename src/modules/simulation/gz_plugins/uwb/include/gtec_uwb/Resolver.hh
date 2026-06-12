#pragma once
#include <functional>
#include <random>
#include <vector>
#include <gz/math/Pose3.hh>
#include "gtec_uwb/types.hh"
#include "gtec_uwb/UwbChannelModel.hh"

namespace gtec_uwb {

using RayCastFn = std::function<RayHit(const gz::math::Vector3d &origin,
                                       const gz::math::Vector3d &target)>;

// Builds the 61-angle sweep around currentYaw (UwbPlugin.cpp:792-820).
std::vector<double> BuildAngleSweep(double currentYaw);

// Full classification + measurement for one anchor.
// Mirrors addAnchor (UwbPlugin.cpp:855-1114) exactly, sans visualization.
Measurement ResolveAnchor(const gz::math::Pose3d &tagPose,
                          const gz::math::Pose3d &anchorPose,
                          const std::vector<double> &anglesToTest,
                          const RayCastFn &rayCast,
                          const UwbParams &params,
                          const UwbChannelModel &model,
                          std::default_random_engine &rng);

}  // namespace gtec_uwb
