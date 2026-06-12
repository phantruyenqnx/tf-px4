#include <gtest/gtest.h>
#include "gtec_uwb/Resolver.hh"
using namespace gtec_uwb;

namespace {
gz::math::Pose3d Tag(){ return {0,0,1,0,0,0}; }
gz::math::Pose3d Anchor(){ return {5,0,1,0,0,0}; } // 5 m away, LOS along +x
}

TEST(Resolver, SweepHas61Angles) {
  EXPECT_EQ(BuildAngleSweep(0.0).size(), 61u);
}

TEST(Resolver, LineOfSightWhenNoObstacle) {
  UwbParams p; UwbChannelModel m; std::default_random_engine rng(1);
  RayCastFn clear = [](auto, auto){ return RayHit{0.0, ""}; };
  auto r = ResolveAnchor(Tag(), Anchor(), BuildAngleSweep(0), clear, p, m, rng);
  EXPECT_EQ(r.losType, LosType::LOS);
}

TEST(Resolver, NlosSoftForThinWall) {
  UwbParams p; UwbChannelModel m; std::default_random_engine rng(2);
  // direct ray (tag->anchor) hits "wall" at 2.4 m; reverse ray (anchor->tag)
  // hits "wall" at 2.5 m => width = 5 - 2.4 - 2.5 = 0.1 < 0.25 => NLOS_S.
  RayCastFn rc = [](const gz::math::Vector3d &o, const gz::math::Vector3d &){
    return (o.X() < 2.5) ? RayHit{2.4, "wall"} : RayHit{2.5, "wall"};
  };
  auto r = ResolveAnchor(Tag(), Anchor(), BuildAngleSweep(0), rc, p, m, rng);
  EXPECT_EQ(r.losType, LosType::NLOS_S);
}

TEST(Resolver, NlosHardWhenReflectionReachesAnchor) {
  UwbParams p; UwbChannelModel m; std::default_random_engine rng(3);
  // Direct path blocked by a thick wall (width > 0.25): tag->wall@1, anchor->wall@1
  // (width = 5-1-1 = 3). Then for the first swept ray, the reflection probe hits
  // a surface at 2 m, and from that collision point the anchor is reachable.
  int call = 0;
  RayCastFn rc = [&](const gz::math::Vector3d &o, const gz::math::Vector3d &t){
    // The first two calls are the direct + reverse rays (between tag & anchor).
    bool betweenTagAnchor =
        (std::abs(o.X()) < 1e-6 && std::abs(t.X()-5) < 1e-6) ||
        (std::abs(o.X()-5) < 1e-6 && std::abs(t.X()) < 1e-6);
    if (betweenTagAnchor) return RayHit{1.0, "thickwall"};
    // reflection probe rays: report a rebound surface 2 m out...
    if ((call++ % 2) == 0) return RayHit{2.0, "mirror"};
    // ...and from the collision point, the path to the anchor is clear.
    return RayHit{0.0, ""};
  };
  auto r = ResolveAnchor(Tag(), Anchor(), BuildAngleSweep(0), rc, p, m, rng);
  EXPECT_EQ(r.losType, LosType::NLOS_H);
}

TEST(Resolver, NlosWhenUnreachable) {
  UwbParams p; UwbChannelModel m; std::default_random_engine rng(4);
  // Thick wall and no reflection ever reaches the anchor.
  RayCastFn rc = [](const gz::math::Vector3d &o, const gz::math::Vector3d &t){
    bool betweenTagAnchor =
        (std::abs(o.X()) < 1e-6 && std::abs(t.X()-5) < 1e-6) ||
        (std::abs(o.X()-5) < 1e-6 && std::abs(t.X()) < 1e-6);
    if (betweenTagAnchor) return RayHit{1.0, "thickwall"};
    return RayHit{0.0, ""};  // probes hit nothing -> no rebound found
  };
  auto r = ResolveAnchor(Tag(), Anchor(), BuildAngleSweep(0), rc, p, m, rng);
  EXPECT_EQ(r.losType, LosType::NLOS);
  EXPECT_FALSE(r.valid);
}

TEST(Resolver, AllLosBypassesRayCasting) {
  UwbParams p; p.allBeaconsAreLOS = true;
  UwbChannelModel m; std::default_random_engine rng(5);
  RayCastFn never = [](auto, auto){ ADD_FAILURE() << "should not ray cast"; return RayHit{}; };
  auto r = ResolveAnchor(Tag(), Anchor(), BuildAngleSweep(0), never, p, m, rng);
  EXPECT_EQ(r.losType, LosType::LOS);
}
