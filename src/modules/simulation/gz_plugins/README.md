# Creating a Gazebo (gz-sim) plugin for PX4 SITL

This directory holds the custom Gazebo **system plugins** that PX4 builds for SITL
(`gz-sim` / Harmonic). This guide explains the architecture and walks through adding a
**new sensor plugin** end-to-end, using the existing plugins as worked examples:

- [`optical_flow/`](optical_flow) — a **gz Sensor** + system, world-level, publishes `px4::msgs::OpticalFlow`.
- [`uwb/`](uwb) — a **system plugin** tag, model-level, publishes `px4::msgs::Ranging`. Full worked example below.
- [`template_plugin/`](template_plugin) — the minimal skeleton to copy.
- [`moving_platform_controller/`](moving_platform_controller), [`gstreamer/`](gstreamer) — more examples.

## How the build & load works

```
src/modules/simulation/gz_plugins/<your_plugin>/   ← source + CMakeLists.txt
        │  built by target `px4_gz_plugins` (DEPENDS of every gz_<model>[_<world>] target)
        ▼
$PX4_BINARY_DIR/.../gz_plugins/lib<Name>.so        ← flat output dir = $PX4_GZ_PLUGINS
        │  gz_env.sh adds $PX4_GZ_PLUGINS to GZ_SIM_SYSTEM_PLUGIN_PATH
        ▼
gz-sim resolves <plugin filename="<Name>"> at load time
```

Key files:

| File | Role |
|---|---|
| [`CMakeLists.txt`](CMakeLists.txt) | finds gz packages, sets the **flat output dir** (`CMAKE_LIBRARY_OUTPUT_DIRECTORY`), `add_subdirectory(<plugin>)` for each, and the `px4_gz_plugins` aggregate target. |
| [`../gz_bridge/gz_env.sh.in`](../gz_bridge/gz_env.sh.in) | sets `GZ_SIM_SYSTEM_PLUGIN_PATH = …:$PX4_GZ_PLUGINS`. **Nothing else is needed for gz-sim to find your `.so`.** |
| [`../gz_bridge/server.config`](../gz_bridge/server.config) | world-level plugin list (loaded for every world). |
| [`../gz_msgs/`](../gz_msgs) | custom protobuf messages → library `px4_gz_msgs`. |

Because the output is on the plugin path automatically, you do **not** export
`GZ_SIM_SYSTEM_PLUGIN_PATH` yourself — just build and run `make px4_sitl gz_<model>`.

## Plugin types — pick one

**Where it attaches:**

- **World-level** — loaded once per world for all entities. Register in
  [`server.config`](../gz_bridge/server.config):
  ```xml
  <plugin entity_name="*" entity_type="world" filename="libYourSystem.so" name="custom::YourSystem"/>
  ```
  Use for global effects (optical flow camera system, moving platform).
- **Model-level** — attached to one model via a `<plugin>` in that model's SDF
  (see the UWB example). Use for a per-vehicle sensor/tag.

**What it derives from** (`#include <gz/sim/System.hh>`):

- A **System plugin** implements interfaces like `ISystemConfigure`, `ISystemPreUpdate`,
  `ISystemPostUpdate`. Good for reading the world (ECM) and publishing — this is what the
  UWB tag does.
- A **gz Sensor** (`gz::sensors::Sensor`) plugs into the gz-sensors framework (rendering,
  noise) and is paired with a thin system that registers it — this is what optical flow
  does (`OpticalFlowSensor` + `OpticalFlowSystem`). Heavier; use it when you need the
  sensor pipeline (cameras, lidars, rendering).

## Step-by-step: a new sensor plugin

### 1. Create the directory and source

```bash
cp -r template_plugin my_sensor
```

Implement your system class (`MySensorSystem.hpp/.cpp`). Minimum shape of a
publishing system plugin:

```cpp
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include "my_sensor.pb.h"            // px4::msgs::MySensor (from px4_gz_msgs)

class MySensorSystem : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    pub_ = node_.Advertise<px4::msgs::MySensor>("/my_sensor/topic");
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    px4::msgs::MySensor msg;
    msg.set_time_usec(
        std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count());
    // … fill fields from the ECM …
    pub_.Publish(msg);
  }
 private:
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};

GZ_ADD_PLUGIN(MySensorSystem, gz::sim::System,
              MySensorSystem::ISystemConfigure, MySensorSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(MySensorSystem, "custom::MySensorSystem")
```

### 2. Add the output message (`px4_gz_msgs`)

PX4 plugins use **plain proto3** messages in package `px4.msgs`, built into the
`px4_gz_msgs` library (see [`../gz_msgs/CMakeLists.txt`](../gz_msgs/CMakeLists.txt), which
globs every `*.proto`). Create `../gz_msgs/my_sensor.proto`:

```proto
syntax = "proto3";
package px4.msgs;

message MySensor {
  int64 time_usec = 1;   // timestamp (us since system start)
  // … your fields …
}
```

Do **not** `import "gz/msgs/*.proto"` — that pulls in the gz-msgs framework. Keep a plain
`int64 time_usec` for the timestamp (mirrors `opticalflow.proto` / `ranging.proto`). The
generated header is `my_sensor.pb.h`, type `px4::msgs::MySensor`. Publish it over
gz-transport like any protobuf message (`Node::Advertise<px4::msgs::MySensor>(topic)`).

> Trade-off: plain protobuf means `gz topic -e` can't decode the message without a
> descriptor. Subscribe from C++ instead, linking `px4_gz_msgs`.

### 3. Write the plugin `CMakeLists.txt`

The `project()` name becomes the library name → the `filename` you use in SDF. Copy from
[`uwb/CMakeLists.txt`](uwb/CMakeLists.txt) or `template_plugin/CMakeLists.txt`:

```cmake
project(MySensorSystem)                     # -> libMySensorSystem.so

add_library(${PROJECT_NAME} SHARED MySensorSystem.cpp)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs                       # your message library
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::register
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC px4_gz_msgs)                       # gives access to my_sensor.pb.h
```

The gz `${..._VERSION_MAJOR}` variables are set by the parent CMake's `find_package`
calls — use them so the plugin works across gz versions. `gz-math`/`gz-common` come in
transitively via `gz-sim`. Link `gz-plugin…::register` for `GZ_ADD_PLUGIN`.

### 4. Register it

In [`CMakeLists.txt`](CMakeLists.txt) (this directory):

```cmake
add_subdirectory(my_sensor)
# add the target to BOTH branches of px4_gz_plugins:
add_custom_target(px4_gz_plugins ALL DEPENDS … MySensorSystem)
```

### 5. Load it

- **World-level** → add a line to [`server.config`](../gz_bridge/server.config):
  ```xml
  <plugin entity_name="*" entity_type="world" filename="libMySensorSystem.so" name="custom::MySensorSystem"/>
  ```
- **Model-level** → add to the model SDF (the `filename` is the lib name **without**
  `lib`/`.so`):
  ```xml
  <plugin filename="MySensorSystem" name="custom::MySensorSystem"> … </plugin>
  ```

### 6. Build & run

```bash
make px4_sitl gz_<model>[_<world>]
```

CMake reconfigures (you added a subdir), builds `px4_gz_plugins`, and gz-sim loads your
`.so` from `$PX4_GZ_PLUGINS`. If it doesn't load, gz logs
`Failed to load system plugin … <filename>` — check the `project()`/`filename` match.

## Pitfalls

- **`-Werror`** — PX4 compiles plugins with warnings as errors. Avoid **deprecated gz
  APIs** (e.g. `gz::math::Pose3::operator-`); use the current equivalent. If a vendored
  dependency is unavoidably noisy, scope a suppression like
  `target_compile_options(<tgt> PRIVATE -Wno-error=<warning>)` (cf. `px4_gz_msgs`’s
  `-Wno-error=float-equal`).
- **Library name == `filename`** — `project(Foo)` → `libFoo.so` → `<plugin filename="Foo">`.
  The UWB plugin uses lowercase `project(gtec_uwb_plugin)` on purpose so existing SDFs
  referencing `filename="gtec_uwb_plugin"` keep working.
- **Don't forget the `px4_gz_plugins` DEPENDS** — `add_subdirectory` alone builds the lib
  only if something depends on it; the aggregate target is what the run targets pull in.
- **Tests/extra tools** — keep them behind an option so the normal SITL build is
  unaffected (see `uwb/CMakeLists.txt`’s `UWB_PLUGIN_TESTS`).
- **Sensor geometry** — if your sensor ray-traces the world, make sure its own marker
  geometry doesn't block its measurement (the UWB anchors are visual-only for this reason).

## Worked example: the UWB tag (model-level, custom message, vehicle variant)

The UWB plugin is a complete template for a **per-vehicle sensor that needs a custom
message and a dedicated SITL vehicle**:

1. **Message** — [`../gz_msgs/ranging.proto`](../gz_msgs/ranging.proto) → `px4::msgs::Ranging`.
2. **Plugin** — [`uwb/`](uwb) → `libgtec_uwb_plugin.so` (built by `px4_gz_plugins`).
3. **Vehicle variant** — `Tools/simulation/gz/models/f450-uwb/model.sdf` follows the
   `x500_lidar_2d` convention: `merge`-includes the base `f450` and adds the tag
   `<plugin>` on `base_link`.
4. **Airframe** — `ROMFS/.../airframes/4023_gz_f450-uwb` sets `PX4_SIM_MODEL=gz_f450-uwb`
   and sources `4022_gz_f450`; registered in that dir's `CMakeLists.txt`. The hyphen in
   `f450-uwb` keeps the model token underscore-free so the `gz_<model>_<world>` make
   target splits unambiguously.
5. **World** — four visual-only anchors `anchor_0..3` in
   `Tools/simulation/gz/worlds/uwb.sdf`.
6. **Run** — `make px4_sitl gz_f450-uwb_uwb`.

Verified headless: the tag publishes `px4::msgs::Ranging` for all four anchors and the
measured ranges match the geometric tag↔anchor distance (within a few mm + the modeled
UWB noise). See [`uwb/README.md`](uwb/README.md) for plugin specifics.

## Testing

PX4's gz plugins have no mandatory in-tree test harness. For pure-logic units, gate a
GTest build behind an option (UWB does this) and run `ctest`; for runtime checks, launch
SITL headless and subscribe to the topic from a small C++ tool that links `px4_gz_msgs`.
