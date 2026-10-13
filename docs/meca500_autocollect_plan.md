# Plan: Automated, collision-safe dataset collection with a digital target

## Context

Today, every microscope pipetting demo is recorded by a human with the SpaceMouse, and the target is a
sticker on a petri dish that has to be moved by hand. The goal is a recorder that needs no human:

- it drives the Meca500 to a random start pose (within limits);
- it draws a **digital target** at a random pixel (within limits) on the microscope feed;
- it drives the pipette tip along a slightly randomised path until the tip **touches the dish surface on the
  target**.

The resulting datasets train ACT and Diffusion policies. Their inputs are the joint state and the
microscope image with the target drawn on it. Their output is arm motion that ends with the tip on the
target.

The hard part is not hitting the illuminator column or the lamp above the inverted microscope. The design
agreed in conversation is:

- an obstacle model per bench, from the user's **Fusion 360 scene**, exported as STLs with the robot's
  base frame as the origin;
- the robot and microscope are bolted to a metal frame that is in the CAD, and touch points check the
  model against reality;
- candidate paths are checked against the model, and any path that comes too close is thrown away and a new
  one is sampled;
- paths go through a narrow approach corridor near the lamp;
- the robot's own torque limits act as a backstop.

**Scope of the first version.** It targets **Huanyu's setup** (bench 2, rig `meca500_basler`, Basler
camera only). The original setup is added later by adding a bench folder; no code is forked.

**Decisions taken.**
- The tip touches a measured dish-surface height.
- The target is a high-contrast marker, not a copy of the sticker.
- Recovery demos (deliberate small mistakes that get corrected) come in phase 2.

Work happens on a new branch, `feat/meca500-autocollect`. Its first commit is this plan, saved as
`docs/meca500_autocollect_plan.md`.

## Key design choices

1. **The robot runs in control mode. A scripted teleop replays a pre-planned, collision-checked joint
   path.**
   - Each step, the new `meca500_scripted` teleop returns the next joint target and its `precision.state`.
   - `Meca500.send_action` sends it with `MoveJoints`. This is the same path a deployed policy uses, so
     the recorded motion matches what happens at deployment.
   - The recorded actions are the commanded targets, not a position readback. This fixes the problem
     where 97% of SpaceMouse actions equalled the observed state.
   - The existing `convert_to_delta_actions.py` turns the recording into the per-step changes that training
     and inference already expect.
   - The dataset schema is unchanged: `joint_1..6.pos` + `precision.state` + `microscope_cam`.

2. **Plan everything before moving.**
   - For each episode, sample the start, the target and the path. Convert the path to joint angles with
     inverse kinematics.
   - Check every interpolated joint configuration (spacing ≤ 1 mm or 0.5°) against the obstacle model with
     a margin.
   - Reject the episode and resample on any failure: a collision, unreachable IK, a joint limit, or a
     dangerous wrist singularity.
   - Every motion goes through the same checker, including the unrecorded move to the next episode's start.

3. **Collision checking uses a signed-distance grid and points sampled on the robot.**
   - Load the bench's obstacle STLs with `trimesh`.
   - Turn them into voxels at about 2 mm resolution.
   - Precompute a signed distance field with `scipy.ndimage.distance_transform_edt` and cache it per bench.
   - Sample points on the robot's link collision meshes and on the user's tool STL.
   - For each configuration, transform those points with forward kinematics and look up their distances.
     The configuration passes when the smallest distance is above the margin.
   - The margin defaults to 15 mm for the links. The tool and holder get a separately configurable margin,
     because the tip must legitimately reach the stage.
   - This is a few dependency-light lines of NumPy and fast enough to check thousands of samples.

4. **Our own forward and inverse kinematics, from the URDF already in the repo.**
   - Forward kinematics come from `mecademic_description/urdf/meca_arm_only.urdf`.
   - The tool-tip transform comes from the CAD.
   - Inverse kinematics use damped least squares in NumPy, seeded from the previous waypoint so the path
     stays in one arm configuration.
   - Before anything is trusted, these are checked against the robot's own `GetPose()` (see Verification).

5. **The digital target is drawn live through an observation processor.**
   - Shared code in `overlay.py` draws the marker.
   - It is added as a `ProcessorStep` in `robot_observation_processor` (`lerobot_record.py:308`), so the
     recorded frames, the live display and the deployed policy all see the same pixels.
   - The camera driver is not touched.
   - The marker is a high-contrast ring or crosshair with configurable radius, thickness and colour.
     It is drawn the same way in recording and in inference.

6. **Pixel ↔ robot calibration for each bench.**
   - Jog the tip, at dish height, to about 12 points spread over the field of view, and click the tip in
     the image at each one.
   - Fit an affine map between pixels and robot X/Y at the dish height, plus the inverse. An affine map
     absorbs the Basler's rotation and mirroring and the 180° base rotation.
   - The fit also stores:
     - the dish-surface Z, measured by touching the dish;
     - the region of the image where targets are allowed, inset from the image edges.
   - Its residual error, in px and µm, is reported and stored. That error is the label noise of the whole
     dataset, so it is a gate before collection starts.

## Randomisation (all limits live in one per-bench YAML file)

- **Start pose.** A box in tip X/Y/Z plus a few degrees of orientation jitter around a nominal start.
  - The nominal start and the box size are set from the **first frames of the existing
    `basler_pipette_2026-09-16` demos**, using forward kinematics on their joints. That gives realistic,
    proven starts.
  - The start must be collision-free, and the tip must be out of the microscope's view.
- **Target.** A uniform pixel inside the allowed region. The planner turns it into robot X/Y with the
  calibration, at the dish-surface Z.
- **Path.** Start, then 1–2 random via-points in the free-space region, then a pre-touch point, then the
  touch.
  - The pre-touch point sits a random 1–4 mm back along the pipette's approach axis, with small lateral
    jitter.
  - The coarse part is a smooth curve through the via-points with a randomised overall speed.
  - The precision part is a slow straight line from the pre-touch point to the touch, with
    `precision.state = 1`.
  - Both phases have a randomised timing profile. Speeds and per-step changes are kept inside what the
    human demos show (bench 2: coarse p95 0.53°/step, precision p95 0.038°/step).
  - A short still pause at the end (0.5–1.5 s) matches the operators' endings.
- **Seeds.** One seed per episode, plus a sidecar `episodes_meta.jsonl` in the dataset folder recording the
  target pixel, the start pose, the seed and the closest obstacle distance. These are not dataset
  features, so nothing leaks into the policy's inputs.

## Files

New library code, kept in `src/` so it can be tested:

| File | Contents |
|---|---|
| `src/lerobot/robots/meca500/autocollect/kinematics.py` | Reads the URDF chain; `fk(q)` returns each link's pose; `tip_pose(q)`; `ik(pose, seed)` |
| `.../autocollect/bench.py` | `BenchConfig` dataclass loaded from `assets/benches/<rig>/bench.yaml`: STL paths, margins, start box, approach axis, speed limits, calibration path |
| `.../autocollect/collision.py` | Builds the distance grid from STLs (cached as `.npz`); `min_clearance(q)`; `check_path(qs)` |
| `.../autocollect/calibration.py` | Affine fit, save/load, `pixel_to_xy`, `xy_to_pixel`, residual report |
| `.../autocollect/planner.py` | `sample_episode(rng)` returns `EpisodePlan` (joint path at the recording fps, precision flags, metadata); `plan_transit(q_from, q_to)` |
| `.../autocollect/overlay.py` | `draw_target(img, px, style)` and a `TargetOverlayStep` processor step |
| `src/lerobot/teleoperators/meca500_scripted/` | `Meca500Scripted` teleop with no hardware. Same 7 action keys as the SpaceMouse; `load_plan(plan)`; `get_action()` steps through the plan and holds the last pose; `done` property. Registered like `meca500_home` |

Bench data lives in `assets/benches/meca500_basler/`: `bench.yaml`, `obstacles/*.stl`, `tool.stl`,
`registration.json` and `scope_calibration.json`.

New scripts in `scripts/meca500/`. Each has a `RIG =` constant and gets its config from `rigs.py`:

| Script | What it does |
|---|---|
| `register_bench.py` | Guided touch points: jog with the SpaceMouse, press Enter to log `GetPose()` next to the matching CAD point. Reports the per-point error, and optionally fits a small correction to where the robot base sits. |
| `calibrate_scope.py` | Live Basler view. Jog the tip, click it, log the pose; touch the dish to set the surface Z. Writes the calibration and prints the residual error. |
| `check_demos_clearance.py` | Runs the collision checker on every frame of the existing human demos. Any hit means the model is wrong. Also prints the start-pose distribution to use for the start box. |
| `plan_preview.py` | Dry run with no robot. Plans N episodes, reports the rejection rate, clearances and speeds, and shows the scene, arm and paths in a 3D `trimesh` viewer. |
| `record_auto.py` | The automated recorder (below). |

Existing files that change:
- `microscope_inference.py` gets the same `TargetOverlayStep`, with a random target each rollout episode
  and the target pixel logged, plus an end-of-episode error between tip and target (in px and µm).
- `rigs.py` gets a `BENCH_DIRS` entry for each rig.
- `pyproject.toml` gets an optional extra `meca500-autocollect = ["trimesh", "scipy"]`.

### `record_auto.py` flow

It follows the shape of `record.record_with_reset`, which builds the dataset, processors and keyboard
events, but it owns its loop, because each episode's length comes from its plan.

```
connect robot (monitor_mode=False, cameras on, relative_actions=False) and configure() it
    (SetJointVel, blending); set a conservative torque-limit backstop
for ep in range(N):
    plan = planner.sample_episode(rng)            # retries internally until it passes the checks
    transit = planner.plan_transit(current_q, plan.start_q)
    run transit through record_loop with dataset=None (not recorded)
    overlay.set_target(plan.target_px)            # the marker appears before recording starts
    teleop.load_plan(plan)
    record_loop(..., dataset, control_time_s=plan.duration_s)
    save_episode(); append to episodes_meta.jsonl
    on any robot error or clearance failure: stop, retreat along the planned path, ask before continuing
```

Keyboard events still work: the right arrow ends an episode early, Esc stops the session.

## Phases

1. **Branch and foundations.** Kinematics, bench config and collision code, with unit tests. Add the
   optional extra.
2. **Model the scene.** The user exports the Fusion STLs for Huanyu's setup. Run `plan_preview` (with an
   empty path) to view the scene and arm. Run `register_bench` for the touch points. Run
   `check_demos_clearance`.
3. **Calibration.** `calibrate_scope`. Accept it only if the residual error is below a target agreed with
   the user, for example ≤ 5 px.
4. **Planner and overlay.** Then a dry run with `plan_preview`.
5. **Recording on the robot, in three steps:**
   1. touch Z raised by 5 mm and speeds halved, a few episodes watched by hand;
   2. full touch on a dummy dish;
   3. unattended batch.
6. **Training and evaluation.** Delta conversion, then train ACT and Diffusion with
   `train_microscope.py`. Evaluate with the overlay in `microscope_inference.py`.
7. **Phase 2.**
   - Recovery demos: inject an offset or overshoot, then re-plan from the current pose.
   - Add the original setup's bench folder.
   - Optionally, correct the final approach using tip detection in the image.

## What the user needs to provide

- STLs from Fusion for Huanyu's setup, in mm, with the origin at the robot base frame: one per obstacle body,
  plus `tool.stl` (holder and pipette) in the robot flange frame.
- About 4–6 CAD reference points for touch registration: their coordinates and a photo or description of
  each.
- The Meca500 firmware version, which decides whether its work-zone safety feature is available as an extra
  backstop.

## Verification

- **Unit tests** in `tests/robots/test_meca500_autocollect.py`:
  - inverse kinematics followed by forward kinematics gets back to the starting pose;
  - a box obstacle in the path is rejected and a clear path is accepted;
  - the affine calibration fit gets back to a known transform, including a mirror;
  - the overlay draws at the requested pixel;
  - the scripted teleop's action keys match `Meca500.action_features`, it holds its last pose once done,
    and it carries `precision.state`.
  - Run with `.venv/Scripts/python.exe -m pytest tests/robots -q`.
- **Kinematics vs the robot.**
  - Move to 10 varied joint poses (monitor mode, read only), compare `GetPose()` with our forward
    kinematics, and expect ≤ 0.5 mm.
  - This also confirms the URDF base frame matches the robot's base frame. The URDF's
    `meca_base_link` places the J1 joint origin at z = 0.091 m, so check how that relates to the robot's
    base frame.
- **Model vs reality.** Touch-point errors stay well inside the margin, and `check_demos_clearance`
  finds no collisions in any human demo frame.
- **Calibration.** Residual error and a held-out check-point error in px and µm.
- **Dry run.** `plan_preview` over 500 episodes: rejection rate, the smallest clearance per segment, and
  per-step changes within the demo-derived limits. Visual check in the 3D viewer.
- **On the robot.** The cautious first runs from phase 5, then a recorded dataset that loads, converts to
  deltas and passes `train_microscope.py`'s startup check on the action representation.

## Status (2026-10-13)

**Built and tested (no robot or CAD needed):**
- `src/lerobot/robots/meca500/autocollect/` has five modules:
  - `kinematics.py`: forward kinematics from the URDF, plus damped-least-squares IK. FK reproduces the
    Meca's documented zero pose (190, 0, 308) mm with Euler (0, 90, 0).
  - `collision.py`: box, cylinder and mesh-grid obstacle fields; the registration transform; the
    tip exemption.
  - `calibration.py`: the affine pixel ↔ XY fit.
  - `overlay.py`: the marker drawing and the processor step.
  - `planner.py`.
- `meca500_scripted` teleop.
- 24 unit and integration tests in `tests/robots/test_meca500_autocollect.py`. One runs `record_loop`
  end-to-end on a mocked arm and checks that the recorded action equals the plan and the frame
  carries the marker.
- Scripts in `scripts/meca500/`: `check_kinematics.py`, `check_demos_clearance.py`,
  `calibrate_scope.py`, `register_bench.py`, `plan_preview.py`, `record_auto.py`.
  `microscope_inference.py` also gains `TARGET_OVERLAY`, which is off by default.
- `assets/benches/_template/` and `assets/benches/meca500_basler/bench.yaml`.
  - The start and touch poses come from the 51 Basler demos.
  - A pivot estimate from the demos' end poses puts the tip about 118 mm from the flange at
    [96.9, -3.2, 67.6] mm in the FRF, with the dish near z ≈ 21 mm. This is only a cross-check for
    the CAD.

**Departures from the plan above:**
- Bench folders are keyed by rig name directly, so `rigs.py` needs no `BENCH_DIRS`.
- The rollout overlay needed two small `FORK PATCH`es to upstream files, recorded in AGENTS.md:
  - `rollout()` takes an observation processor;
  - the episodic strategy calls its `reset()` at each episode start.
- `record_auto.py` discards an episode that is cut short instead of saving it.
- The robot's torque-limit and work-zone backstops are **not wired in yet**. Their thresholds need
  tuning on the arm.

**Waiting on the user, in order:**
1. Run `check_kinematics.py` on the arm.
2. Export the Fusion STLs and `tool.stl`, and fill in `tool.tip_pose_in_flange` and the registration
   points.
3. Run `check_demos_clearance.py`, then `register_bench.py`.
4. Run `calibrate_scope.py`.
5. Run `plan_preview.py`.
6. Run `record_auto.py`, first with `STAGE = "hover"`.
