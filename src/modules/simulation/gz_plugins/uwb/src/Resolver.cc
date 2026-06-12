#include "gtec_uwb/Resolver.hh"
#include <cmath>

namespace gtec_uwb {

std::vector<double> BuildAngleSweep(double startAngle) {
  // UwbPlugin.cpp:793-820
  const double arc = 3 * M_PI / 2;
  const int numAnglesToTestBySide = 30;
  const double incrementAngle = arc / numAnglesToTestBySide;
  const int total = 1 + 2 * numAnglesToTestBySide;  // 61
  std::vector<double> a(total);
  a[0] = startAngle;
  for (int i = 1; i < total; ++i) {
    if (i % 2 == 0) a[i] = startAngle - (i / 2) * incrementAngle;
    else            a[i] = startAngle + (i - (i - 1) / 2) * incrementAngle;
  }
  return a;
}

Measurement ResolveAnchor(const gz::math::Pose3d &tagPose,
                          const gz::math::Pose3d &anchorPose,
                          const std::vector<double> &anglesToTest,
                          const RayCastFn &rayCast,
                          const UwbParams &params,
                          const UwbChannelModel &model,
                          std::default_random_engine &rng) {
  const int totalNumberAnglesToTest = (int)anglesToTest.size();
  gz::math::Vector3d currentTagPose = tagPose.Pos();

  LosType losType = LosType::LOS;
  double distance = tagPose.Pos().Distance(anchorPose.Pos());
  double distanceAfterRebounds = 0;

  if (!params.allBeaconsAreLOS) {
    // --- direct ray (UwbPlugin.cpp:871-881) ---
    RayHit first = rayCast(tagPose.Pos(), anchorPose.Pos());
    if (first.entity.empty()) {
      losType = LosType::LOS;
      distanceAfterRebounds = distance;
    } else {
      // --- reverse ray + wall width (UwbPlugin.cpp:885-901) ---
      RayHit second = rayCast(anchorPose.Pos(), tagPose.Pos());
      double wallWidth = distance - first.distance - second.distance;
      if (wallWidth <= params.nlosSoftWallWidth && first.entity == second.entity) {
        losType = LosType::NLOS_S;
        distanceAfterRebounds = distance;
      } else {
        // --- reflection search (UwbPlugin.cpp:904-1029) ---
        bool end = false;

        double maxDistance = 30;
        double distanceNlosHard = 0;

        double stepFloor = 1;
        double startFloorDistanceCheck = 2;
        int numStepsFloor = 6;

        int indexRay = 0;
        bool foundNlosH = false;

        int currentFloorDistance = 0;

        while (!end) {
          double currentAngle = anglesToTest[indexRay];

          double x = currentTagPose.X() + maxDistance * std::cos(currentAngle);
          double y = currentTagPose.Y() + maxDistance * std::sin(currentAngle);
          double z = currentTagPose.Z();

          if (currentFloorDistance > 0) {
            double tanAngleFloor =
              (startFloorDistanceCheck + stepFloor * (currentFloorDistance - 1)) / currentTagPose.Z();
            double angleFloor = std::atan(tanAngleFloor);
            double h = std::sin(angleFloor) * maxDistance;
            double horizontalDistance = std::sqrt(maxDistance * maxDistance - h * h);
            x = currentTagPose.X() + horizontalDistance * std::cos(currentAngle);
            y = currentTagPose.Y() + horizontalDistance * std::sin(currentAngle);
            z = -1 * (h - currentTagPose.Z());
          }

          gz::math::Vector3d rayPoint(x, y, z);
          RayHit reb = rayCast(currentTagPose, rayPoint);
          if (!reb.entity.empty()) {
            gz::math::Vector3d collisionPoint(
              currentTagPose.X() + reb.distance * std::cos(currentAngle),
              currentTagPose.Y() + reb.distance * std::sin(currentAngle),
              currentTagPose.Z());
            if (currentFloorDistance > 0) {
              collisionPoint.Set(
                currentTagPose.X() + reb.distance * std::cos(currentAngle),
                currentTagPose.Y() + reb.distance * std::sin(currentAngle), 0.0);
            }
            RayHit toAnchor = rayCast(collisionPoint, anchorPose.Pos());
            if (toAnchor.entity.empty()) {
              double distanceToFinalObstacle = anchorPose.Pos().Distance(collisionPoint);
              if (reb.distance + distanceToFinalObstacle <= params.maxDBDistance) {
                foundNlosH = true;
                if (distanceNlosHard < 0.1)
                  distanceNlosHard = reb.distance + distanceToFinalObstacle;
                else if (distanceNlosHard > reb.distance + distanceToFinalObstacle)
                  distanceNlosHard = reb.distance + distanceToFinalObstacle;
              }
            }
          }

          if (indexRay < totalNumberAnglesToTest - 1) {
            indexRay += 1;
          } else if (currentFloorDistance < numStepsFloor) {
            currentFloorDistance += 1; indexRay = 0;
          } else {
            end = true;
          }
        }

        if (foundNlosH) { losType = LosType::NLOS_H; distanceAfterRebounds = distanceNlosHard; }
        else            { losType = LosType::NLOS; }
      }
    }
  } else {
    losType = LosType::LOS;
    distanceAfterRebounds = distance;
  }

  // --- max-distance reclassification (UwbPlugin.cpp:1040-1048) ---
  if ((losType == LosType::LOS || losType == LosType::NLOS_S) &&
      distanceAfterRebounds > params.maxDBDistance) losType = LosType::NLOS;
  if (losType == LosType::NLOS_H && distanceAfterRebounds > params.maxDBDistance)
    losType = LosType::NLOS;

  if (losType == LosType::NLOS) { Measurement m; m.losType = LosType::NLOS; m.valid = false; return m; }

  return model.Generate(losType, distanceAfterRebounds, tagPose, anchorPose, params, rng);
}

}  // namespace gtec_uwb
