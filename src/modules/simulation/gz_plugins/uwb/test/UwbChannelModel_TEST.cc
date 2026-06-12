// gz_harmonic/test/UwbChannelModel_TEST.cc
#include <gtest/gtest.h>
#include <cmath>
#include "gtec_uwb/UwbChannelModel.hh"
using namespace gtec_uwb;

namespace { gz::math::Pose3d P(double x){ return {x,0,0,0,0,0}; } }

// Determinism: same seed -> identical output.
TEST(ChannelModel, Deterministic) {
  UwbChannelModel m; UwbParams p;
  std::default_random_engine a(123), b(123);
  auto r1 = m.Generate(LosType::LOS, 5.0, P(0), P(5), p, a);
  auto r2 = m.Generate(LosType::LOS, 5.0, P(0), P(5), p, b);
  EXPECT_EQ(r1.valid, r2.valid);
  EXPECT_DOUBLE_EQ(r1.range, r2.range);
  EXPECT_DOUBLE_EQ(r1.rss, r2.rss);
}

// Mean range ~= (distance + offset) in mm, std ~= table std (LOS, 5 m).
TEST(ChannelModel, RangeStatistics) {
  UwbChannelModel m; UwbParams p;
  std::default_random_engine rng(7);
  const double d = 5.0;
  const int idxOff = (int)std::round(d / p.stepDBDistance);
  const double withOff = d + UwbChannelModel::RangingOffset(idxOff, 0) / 1000.0;
  const int idx = (int)std::round(withOff / p.stepDBDistance);
  double sum = 0; int n = 0;
  for (int i = 0; i < 20000; ++i) {
    auto r = m.Generate(LosType::LOS, d, P(0), P(d), p, rng);
    if (r.valid) { sum += r.range; ++n; }
  }
  ASSERT_GT(n, 0);
  EXPECT_NEAR(sum / n, withOff * 1000.0,
              3.0 * UwbChannelModel::RangingStd(idx, 0));
}

// Power gate: forcing the sampled power below minPower yields NLOS.
// At 12.7 m the LOS RSS mean (-94.745) is below minPower (-94.5), so
// the majority of samples are gated out.
TEST(ChannelModel, PowerGateDropsWeakSignals) {
  UwbChannelModel m; UwbParams p;
  std::default_random_engine rng(11);
  int dropped = 0;
  for (int i = 0; i < 2000; ++i) {
    auto r = m.Generate(LosType::LOS, 12.7, P(0), P(12.7), p, rng);
    if (!r.valid) ++dropped;
  }
  EXPECT_GT(dropped, 0);
}

// return_angle off -> NaN angle.
TEST(ChannelModel, AngleNaNWhenDisabled) {
  UwbChannelModel m; UwbParams p; p.returnsAngle = false;
  std::default_random_engine rng(1);
  auto r = m.Generate(LosType::LOS, 3.0, P(0), P(3), p, rng);
  if (r.valid) EXPECT_TRUE(std::isnan(r.angle));
}
