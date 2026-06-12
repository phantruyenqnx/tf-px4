// gz_harmonic/src/UwbChannelModel.cc
// Port of UwbPlugin.cpp:1050-1114 and 1163-1172
#include "gtec_uwb/UwbChannelModel.hh"
#include <cmath>

namespace gtec_uwb {

// Forward helper: returns rangingOffset[idx][col] / 1000.0
// Preserves the exact UwbPlugin.cpp:1068/1072 conversion.
static double OffsetMetres(int idx, int col)
{
    return UwbChannelModel::RangingOffset(idx, col) / 1000.0;
}

float UwbChannelModel::CalculateAngle(const gz::math::Pose3d &tagPose,
                                      const gz::math::Pose3d &anchorPose)
{
    // Verbatim from UwbPlugin.cpp:1163-1172. The legacy code used the deprecated
    // Pose3::operator- (anchor expressed in the tag frame); this is the identical
    // CoordPositionSub: inverse-rotate the world delta into the tag frame.
    gz::math::Vector3d pos =
        tagPose.Rot().RotateVectorReverse(anchorPose.Pos() - tagPose.Pos());
    pos.Normalize();
    double yaw = std::atan2(pos.Y(), pos.X());
    return static_cast<float>(yaw);
}

Measurement UwbChannelModel::Generate(LosType losType,
                                      double distanceAfterRebounds,
                                      const gz::math::Pose3d &tagPose,
                                      const gz::math::Pose3d &anchorPose,
                                      const UwbParams &params,
                                      std::default_random_engine &rng) const
{
    Measurement out;
    out.losType = losType;

    // UwbPlugin.cpp:1053-1061 — scenario index
    int indexScenario = 0;
    if (losType == LosType::NLOS_S)
    {
        indexScenario = 2;
    }
    else if (losType == LosType::NLOS_H)
    {
        indexScenario = 1;
    }

    // UwbPlugin.cpp:1063-1075 — offset + index into table
    int indexRangingOffset = static_cast<int>(
        std::round(distanceAfterRebounds / params.stepDBDistance));

    double distanceWithOffset = distanceAfterRebounds;
    if (losType == LosType::LOS)
    {
        distanceWithOffset += OffsetMetres(indexRangingOffset, 0);
    }
    else if (losType == LosType::NLOS_S)
    {
        distanceWithOffset += OffsetMetres(indexRangingOffset, 1);
    }

    int indexRanging = static_cast<int>(
        std::round(distanceWithOffset / params.stepDBDistance));

    // UwbPlugin.cpp:1078-1087 — sample ranging and rss, then power gate
    std::normal_distribution<double> distributionRanging(
        distanceWithOffset * 1000.0,
        RangingStd(indexRanging, indexScenario));
    std::normal_distribution<double> distributionRss(
        RssMean(indexRanging, indexScenario),
        RssStd(indexRanging, indexScenario));

    double rangingValue = distributionRanging(rng);
    double powerValue   = distributionRss(rng);

    if (powerValue < MinPower(indexScenario))
    {
        out.losType = LosType::NLOS;
        out.valid   = false;
        return out;
    }

    // UwbPlugin.cpp:1091-1101 — angle
    float angle;
    if (params.returnsAngle)
    {
        angle = CalculateAngle(tagPose, anchorPose);
        std::normal_distribution<double> distributionAngle(
            angle, 5.0 / 3.0 * M_PI / 180.0);
        angle = static_cast<float>(distributionAngle(rng));
    }
    else
    {
        angle = std::nanf("1");
    }

    out.valid  = true;
    out.range  = rangingValue;
    out.rss    = powerValue;
    out.angle  = angle;
    return out;
}

}  // namespace gtec_uwb
