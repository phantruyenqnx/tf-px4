# TF-PX4 Upstream Tracking Report

## Repo Info

| Item | Value |
|---|---|
| Fork repo | https://github.com/phantruyenqnx/tf-px4 |
| Upstream | https://github.com/PX4/PX4-Autopilot |
| Active branch | `tf-px4-1.16` |
| Reference branch | `tf-px4-1.15` (upstream only, no TF changes) |

---

## Upstream Base

The upstream PX4-Autopilot history has been squashed into a single root commit to keep the repo lean.

| Item | Value |
|---|---|
| Squashed root commit | `79566e37c944438ea631de2c359fb8b456e551d8` |
| Original upstream commit | `8178c3128d5bbb54912c288a07fb1bbd2df0566e` |
| Upstream commit subject | `drivers/gps: warn if gps_inject_data publications have been missed` |
| Upstream commit date | 2025-11-10 |
| Squash performed | 2026-05-26 |

To trace back what PX4 release/tag this base corresponds to:

```bash
# In the original PX4-Autopilot repo (not this fork):
git log --oneline 8178c3128d5bbb54912c288a07fb1bbd2df0566e | head -5
git tag --contains 8178c3128d5bbb54912c288a07fb1bbd2df0566e
```

---

## TF Custom Changes on tf-px4-1.16

13 commits on top of the upstream base (oldest → newest):

| # | Commit | Date | Description |
|---|---|---|---|
| 1 | `2dccbb18` | 2026-03-16 | tf: add custom board config, HIL simulation docs, fix submodule dirty noise |
| 2 | `893ea758` | 2026-03-18 | tf: add GZHILBridge HIL simulation plugin for Gazebo Harmonic |
| 3 | `5e3d26e2` | 2026-04-12 | fix optical flow |
| 4 | `5212bf95` | 2026-04-15 | add arg HIL bridge |
| 5 | `97b625c7` | 2026-05-05 | fix xrce dds reconnect |
| 6 | `2b866dfa` | 2026-05-05 | add docs/research controller |
| 7 | `5f4680ca` | 2026-05-09 | fix path for gzhilbridge module |
| 8 | `3f8c10e7` | 2026-05-18 | update research docs, focus ekf2 |
| 9 | `83881e31` | 2026-05-22 | update ekf2 docs research |
| 10 | `34ef21f4` | 2026-05-23 | update docs about ekf2 and uwb |
| 11 | `ebbfbb7a` | 2026-05-23 | update uwb docs |
| 12 | `4f1d9d07` | 2026-05-23 | update syntax math docs |
| 13 | `5a9e7c78` | 2026-05-23 | update syntax math docs |

### Files changed (non-doc)

| File | Change |
|---|---|
| `boards/px4/fmu-v6x/tf.px4board` | Custom board config for multicopter (strips FW/VTOL/UAVCAN/ETH) |
| `src/modules/simulation/gz_bridge/hil/` | GZHILBridge plugin (HIL via Gazebo Harmonic) |
| `src/modules/simulation/gz_bridge/hil/hil_launch.py` | Single-command HIL launcher with mavlink-routerd watchdog |
| `src/modules/simulation/gz_bridge/hil/CMakeLists.txt` | Standalone cmake target for plugin .so |
| `Makefile` | `make hil` target |
| `gz_assets/` | Bundled x500_hitl model and hitl_default world |
| `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp` | Fix optical flow |
| `src/modules/uxrce_dds_client/uxrce_dds_client.cpp` | Fix xRCE-DDS reconnect |
| `src/modules/uxrce_dds_client/dds_topics.h.em` | Fix xRCE-DDS reconnect |
| `.gitmodules` | `ignore=dirty` for sitl_gazebo-classic and NuttX submodules |

---

## How to Update from Upstream PX4

### Step 1 — Add upstream remote (first time only)

```bash
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream
```

### Step 2 — Find the new upstream base commit

Pick the upstream commit/tag you want to update to, e.g. `v1.16.x`:

```bash
git fetch upstream
git log upstream/v1.16 --oneline | head -10
# Note the commit hash you want as the new base, e.g. NEWBASE
```

### Step 3 — Create new squashed base commit

```bash
NEWBASE=<upstream-commit-hash>
NEW_TREE=$(git rev-parse ${NEWBASE}^{tree})
NEW_ROOT=$(git commit-tree $NEW_TREE -m "PX4 upstream base (squashed) — updated to <version>")
echo $NEW_ROOT
```

### Step 4 — Rebase TF commits onto new base

```bash
OLD_ROOT=79566e37c944438ea631de2c359fb8b456e551d8
git rebase --onto $NEW_ROOT $OLD_ROOT tf-px4-1.16
```

Resolve any conflicts, then:

```bash
git push origin tf-px4-1.16 --force
```

### Step 5 — Update this document

- Update the "Upstream Base" table with new commit hashes and date
- Add any new commits to the "TF Custom Changes" table

---

## Notes

- The `tf-px4-1.15` branch is kept as an unmodified upstream reference (no TF commits on it).
- `tf.px4board` is based on `fmu-v6x` default config with unused modules stripped.
- GZHILBridge requires Gazebo Harmonic and mavlink-routerd. See `docs/tf/gz_harmonic_hil.md`.
