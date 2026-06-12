# GTEC UWB Plugin (`gtec_uwb_plugin`)

A gz-sim (Harmonic / gz-sim8) **System plugin** that simulates a UWB ranging tag.
ROS-free port of the GTEC UWB sensor (Barral et al., *Sensors* 2019). It classifies
the LOS/NLOS channel from the tag to each anchor by ray-tracing the world geometry and
publishes a `px4::msgs::Ranging` message per anchor on the gz-transport topic
`/gtec/toa/ranging`.

This plugin is built by PX4 as part of `px4_gz_plugins` (see the
[gz_plugins guide](../README.md)); the library `libgtec_uwb_plugin.so` lands in
`$PX4_GZ_PLUGINS`, so models load it with no extra path setup.

## How it is wired into SITL

| Piece | Where | Role |
|---|---|---|
| Plugin | this directory | built into `$PX4_GZ_PLUGINS` by `px4_gz_plugins` |
| Message | [`gz_msgs/ranging.proto`](../../gz_msgs/ranging.proto) | `px4::msgs::Ranging`, built into `px4_gz_msgs` |
| Vehicle | `models/f450-uwb/model.sdf` | `merge`-includes `f450`, adds the `<plugin>` (tag on `base_link`) |
| Airframe | `ROMFS/.../airframes/4023_gz_f450-uwb` | `PX4_SIM_MODEL=gz_f450-uwb`, sources `4022_gz_f450` |
| Anchors | `worlds/uwb.sdf` | four visual-only `anchor_0..anchor_3` models |

Run it:

```bash
make px4_sitl gz_f450-uwb_uwb     # model f450-uwb, world uwb
```

## Attaching the tag (model SDF)

The tag is a model-level plugin — add one `<plugin>` to the vehicle model:

```xml
<plugin filename="gtec_uwb_plugin" name="gtec::UwbSensorPlugin">
  <update_rate>25</update_rate>
  <nlosSoftWallWidth>0.25</nlosSoftWallWidth>
  <tag_z_offset>0.0</tag_z_offset>
  <tag_link>base_link</tag_link>
  <anchor_prefix>anchor_</anchor_prefix>
  <all_los>false</all_los>
  <tag_id>0</tag_id>
  <return_angle>true</return_angle>
</plugin>
```

`filename` must equal the library name (`gtec_uwb_plugin` → `libgtec_uwb_plugin.so`);
`name` is the registered alias (`GZ_ADD_PLUGIN_ALIAS`).

### SDF parameters

| Parameter | Meaning |
|---|---|
| `update_rate` | publications per second |
| `nlosSoftWallWidth` | max wall thickness the signal penetrates (m) |
| `tag_z_offset` | height offset added to the tag (m) |
| `tag_link` | link used as the tag reference (falls back to the model) |
| `anchor_prefix` | anchors are models/links whose name starts with this; the suffix is the integer anchor id |
| `all_los` | treat every anchor as line-of-sight |
| `tag_id` | tag identifier |
| `return_angle` | also publish a noisy planar bearing |

## Anchors

An anchor is any model/link whose name starts with `anchor_prefix` (e.g. `anchor_0`).
Anchors must be **visual-only (no `<collision>`)** — a collision box on the anchor makes
the tag's direct ray hit the anchor itself, classify it NLOS, and drop the measurement.
Keep the indices contiguous so the parsed ids are dense.

## Output message

`px4::msgs::Ranging` on `/gtec/toa/ranging` (one message per anchor per update):
`time_usec, anchor_id, tag_id, range` (mm), `seq, rss` (dB), `error_estimation, angle`
(rad). Bridge it to ROS 2 with `ros_gz_bridge` (a custom mapping for `px4.msgs.Ranging`
is out of scope here).

## Architecture

- `UwbChannelModel` — empirical lookup tables + Gaussian noise/power model (pure).
- `RayObstacleSet` — CPU analytic ray-vs-box/plane intersection (box obstacles + ground).
- `Resolver` — LOS / NLOS-Soft / NLOS-Hard reflection-search classification (pure).
- `EcmGeometry` — reads collision geometry and discovers tag/anchors from the ECM.
- `UwbSensorPlugin` — the gz-sim8 System plugin (`Configure` + `PostUpdate`) and
  gz-transport publishing.

## Tests (opt-in)

Pure-logic unit tests live in [`test/`](test); off by default so the normal SITL build is
unaffected. Enable with:

```bash
cmake -DUWB_PLUGIN_TESTS=ON <build-dir> && ninja
ctest -R "UwbChannelModel|RayObstacleSet|Resolver|Ranging"   # needs GTest
```

To watch live output, subscribe to `/gtec/toa/ranging` from a small C++ program that links
`px4_gz_msgs` (the message is plain protobuf, so `gz topic -e` can't decode it without a
descriptor).
