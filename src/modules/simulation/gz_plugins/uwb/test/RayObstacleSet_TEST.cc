#include <gtest/gtest.h>
#include <cmath>
#include "gtec_uwb/RayObstacleSet.hh"
using namespace gtec_uwb;

TEST(RayObstacleSet, NoHitWhenEmpty) {
  RayObstacleSet s;
  auto h = s.Intersect({0,0,0}, {10,0,0});
  EXPECT_EQ(h.entity, "");
}

TEST(RayObstacleSet, HitsAxisAlignedBox) {
  RayObstacleSet s;
  // 1x1x1 wall centred at x=5 -> near face at x=4.5
  s.AddBox({"wall", gz::math::Pose3d(5,0,0,0,0,0), {1,1,1}});
  auto h = s.Intersect({0,0,0}, {10,0,0});
  EXPECT_EQ(h.entity, "wall");
  EXPECT_NEAR(h.distance, 4.5, 1e-6);
}

TEST(RayObstacleSet, MissesBoxOffAxis) {
  RayObstacleSet s;
  s.AddBox({"wall", gz::math::Pose3d(5,10,0,0,0,0), {1,1,1}});
  auto h = s.Intersect({0,0,0}, {10,0,0});
  EXPECT_EQ(h.entity, "");
}

TEST(RayObstacleSet, ReturnsNearestOfTwo) {
  RayObstacleSet s;
  s.AddBox({"far",  gz::math::Pose3d(8,0,0,0,0,0), {1,1,1}});
  s.AddBox({"near", gz::math::Pose3d(3,0,0,0,0,0), {1,1,1}});
  auto h = s.Intersect({0,0,0}, {10,0,0});
  EXPECT_EQ(h.entity, "near");
  EXPECT_NEAR(h.distance, 2.5, 1e-6);
}

TEST(RayObstacleSet, RespectsBoxOrientation) {
  RayObstacleSet s;
  // thin slab rotated 45deg about Z, centred at x=5
  s.AddBox({"slab", gz::math::Pose3d(5,0,0, 0,0,M_PI/4), {0.2,4,4}});
  auto h = s.Intersect({0,0,0}, {10,0,0});
  EXPECT_EQ(h.entity, "slab");
}

TEST(RayObstacleSet, HitsGroundPlane) {
  RayObstacleSet s;
  s.SetGroundPlane(0.0, "ground");
  auto h = s.Intersect({0,0,2}, {0,0,-1});  // pointing down to z=0
  EXPECT_EQ(h.entity, "ground");
  EXPECT_NEAR(h.distance, 2.0, 1e-6);
}
