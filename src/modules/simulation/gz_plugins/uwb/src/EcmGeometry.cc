#include "EcmGeometry.hh"
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Plane.hh>

namespace gtec_uwb {

// Walk parents to find the model entity that owns `e`.
static gz::sim::Entity OwningModel(const gz::sim::EntityComponentManager &ecm,
                                   gz::sim::Entity e) {
  gz::sim::Entity cur = e;
  while (cur != gz::sim::kNullEntity) {
    if (ecm.Component<gz::sim::components::Model>(cur)) return cur;
    auto parent = ecm.Component<gz::sim::components::ParentEntity>(cur);
    if (!parent) break;
    cur = parent->Data();
  }
  return gz::sim::kNullEntity;
}

void BuildObstacleSet(const gz::sim::EntityComponentManager &ecm,
                      gz::sim::Entity selfModel,
                      RayObstacleSet &out) {
  out.Clear();
  out.ClearGroundPlane();
  ecm.Each<gz::sim::components::Collision,
           gz::sim::components::Geometry,
           gz::sim::components::Name>(
    [&](const gz::sim::Entity &e,
        const gz::sim::components::Collision *,
        const gz::sim::components::Geometry *geom,
        const gz::sim::components::Name *name) -> bool {
      if (OwningModel(ecm, e) == selfModel) return true;  // skip self
      const sdf::Geometry &g = geom->Data();
      if (g.Type() == sdf::GeometryType::BOX && g.BoxShape()) {
        BoxObstacle b;
        b.name = name->Data();
        b.pose = gz::sim::worldPose(e, ecm);
        b.size = g.BoxShape()->Size();
        out.AddBox(b);
      } else if (g.Type() == sdf::GeometryType::PLANE && g.PlaneShape()) {
        out.SetGroundPlane(gz::sim::worldPose(e, ecm).Pos().Z(), name->Data());
      }
      return true;
    });
}

std::vector<AnchorRef> FindAnchors(const gz::sim::EntityComponentManager &ecm,
                                   const std::string &prefix) {
  std::vector<AnchorRef> anchors;
  auto scan = [&](const gz::sim::Entity &e, const std::string &nm) {
    if (nm.rfind(prefix, 0) == 0) {  // starts-with
      try { anchors.push_back({e, std::stoi(nm.substr(prefix.size()))}); }
      catch (...) {}
    }
  };
  ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
    [&](const gz::sim::Entity &e, const gz::sim::components::Model *,
        const gz::sim::components::Name *n) -> bool { scan(e, n->Data()); return true; });
  ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
    [&](const gz::sim::Entity &e, const gz::sim::components::Link *,
        const gz::sim::components::Name *n) -> bool { scan(e, n->Data()); return true; });
  return anchors;
}

}  // namespace gtec_uwb
