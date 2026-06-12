#pragma once
#include <string>
#include <vector>
#include <optional>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include "gtec_uwb/types.hh"

namespace gtec_uwb {

struct BoxObstacle {
  std::string name;
  gz::math::Pose3d pose;   // world pose of box center
  gz::math::Vector3d size; // full extents (x,y,z)
};

// CPU analytic replacement for Classic RayShape::GetIntersection.
class RayObstacleSet {
 public:
  void Clear();
  void AddBox(const BoxObstacle &b);
  void SetGroundPlane(double z, const std::string &name);  // optional floor
  void ClearGroundPlane();

  // Nearest hit strictly between origin and target (exclusive of endpoints
  // by a small epsilon). Empty RayHit.entity == no hit, matching RayShape.
  RayHit Intersect(const gz::math::Vector3d &origin,
                   const gz::math::Vector3d &target) const;

 private:
  std::vector<BoxObstacle> boxes_;
  std::optional<double> groundZ_;
  std::string groundName_;
};

}  // namespace gtec_uwb
