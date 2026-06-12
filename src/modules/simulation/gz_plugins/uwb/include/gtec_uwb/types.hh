#pragma once
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace gtec_uwb {

enum class LosType { LOS, NLOS, NLOS_S, NLOS_H };

struct RayHit {
  double distance = 0.0;     // distance from origin to first obstacle
  std::string entity;        // empty => no hit (matches RayShape semantics)
};

struct UwbParams {
  double nlosSoftWallWidth = 0.25;
  double tagZOffset = 0.0;
  int    tagId = 0;
  double maxDBDistance = 14.0;
  double stepDBDistance = 0.1;
  bool   allBeaconsAreLOS = false;
  bool   returnsAngle = false;
};

struct Measurement {
  bool valid = false;        // false => no ranging published (NLOS)
  LosType losType = LosType::NLOS;
  double range = 0.0;        // millimetres
  double rss = 0.0;
  double angle = 0.0;        // radians (NaN if returnsAngle false)
};

}  // namespace gtec_uwb
