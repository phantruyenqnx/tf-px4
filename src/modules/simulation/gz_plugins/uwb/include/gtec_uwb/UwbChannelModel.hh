#pragma once
#include <random>
#include "gtec_uwb/types.hh"

namespace gtec_uwb {

// Pure port of UwbPlugin.cpp:1050-1114 (table lookup + Gaussian sampling +
// power gate + optional bearing). No Gazebo runtime dependency.
class UwbChannelModel {
 public:
  // distanceAfterRebounds in METRES. losType must be LOS, NLOS_S, or NLOS_H.
  // Returns a Measurement; .valid is false if the power gate trips (-> NLOS).
  Measurement Generate(LosType losType,
                       double distanceAfterRebounds,
                       const gz::math::Pose3d &tagPose,
                       const gz::math::Pose3d &anchorPose,
                       const UwbParams &params,
                       std::default_random_engine &rng) const;

  // Bearing helper, verbatim from UwbPlugin.cpp:1163-1172 (yaw only).
  static float CalculateAngle(const gz::math::Pose3d &tagPose,
                              const gz::math::Pose3d &anchorPose);

  // Table accessors (exposed for tests).
  static double RangingStd(int idx, int scenario);
  static double RssMean(int idx, int scenario);
  static double RssStd(int idx, int scenario);
  static double RangingOffset(int idx, int col);
  static double MinPower(int scenario);
};

}  // namespace gtec_uwb
