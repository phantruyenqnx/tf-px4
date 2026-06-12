#include "gtec_uwb/RayObstacleSet.hh"
#include <cmath>
#include <limits>

namespace gtec_uwb {

void RayObstacleSet::Clear() { boxes_.clear(); }
void RayObstacleSet::AddBox(const BoxObstacle &b) { boxes_.push_back(b); }
void RayObstacleSet::SetGroundPlane(double z, const std::string &n) { groundZ_ = z; groundName_ = n; }
void RayObstacleSet::ClearGroundPlane() { groundZ_.reset(); }

namespace {
// Ray-vs-OBB via slab method in the box's local frame.
// Returns t in [0,1] along (target-origin) of the entry hit, or <0 if none.
double HitBox(const gz::math::Vector3d &o, const gz::math::Vector3d &d,
              const BoxObstacle &b) {
  // transform ray into box local frame
  gz::math::Vector3d lo = b.pose.Rot().Inverse() * (o - b.pose.Pos());
  gz::math::Vector3d ld = b.pose.Rot().Inverse() * d;
  gz::math::Vector3d h = b.size * 0.5;
  double tmin = 0.0, tmax = 1.0;  // segment parameter range
  for (int i = 0; i < 3; ++i) {
    double oi = lo[i], di = ld[i], hi = h[i];
    if (std::fabs(di) < 1e-12) {
      if (oi < -hi || oi > hi) return -1.0;  // parallel and outside slab
    } else {
      double t1 = (-hi - oi) / di;
      double t2 = ( hi - oi) / di;
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) return -1.0;
    }
  }
  return tmin;  // entry point within [0,1]
}
}  // namespace

RayHit RayObstacleSet::Intersect(const gz::math::Vector3d &origin,
                                 const gz::math::Vector3d &target) const {
  RayHit best; best.distance = std::numeric_limits<double>::max();
  bool found = false;
  gz::math::Vector3d d = target - origin;
  double segLen = d.Length();
  const double eps = 1e-4;

  for (const auto &b : boxes_) {
    double t = HitBox(origin, d, b);
    if (t >= 0.0 && t <= 1.0) {
      double dist = t * segLen;
      if (dist > eps && dist < best.distance) {
        best.distance = dist; best.entity = b.name; found = true;
      }
    }
  }

  if (groundZ_) {
    double dz = d.Z();
    if (std::fabs(dz) > 1e-12) {
      double t = (*groundZ_ - origin.Z()) / dz;
      if (t >= 0.0 && t <= 1.0) {
        double dist = t * segLen;
        if (dist > eps && dist < best.distance) {
          best.distance = dist; best.entity = groundName_; found = true;
        }
      }
    }
  }

  if (!found) { best.distance = 0.0; best.entity = ""; }
  return best;
}

}  // namespace gtec_uwb
