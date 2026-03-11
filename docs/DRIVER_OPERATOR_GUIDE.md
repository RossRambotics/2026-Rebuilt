# 2026 Robot — Driver & Operator Guide

This guide covers what you need to drive and operate the robot effectively. Read the normal operations section first, then study the troubleshooting sections so you know what to do when things go sideways during a match.

---

## Table of Contents

1. [Quick Reference — Controls](#1-quick-reference--controls)
2. [Normal Operations (All Systems Go)](#2-normal-operations-all-systems-go)
3. [Reading the LEDs](#3-reading-the-leds)
4. [Diagnosing Problems During a Match](#4-diagnosing-problems-during-a-match)
5. [Manual Controls — Button Box Override](#5-manual-controls--button-box-override)
6. [Calibration Procedures](#6-calibration-procedures)
7. [Autonomous Mode](#7-autonomous-mode)
8. [Pre-Match Checklist](#8-pre-match-checklist)
9. [Change Log](#change-log)

---

## 1. Quick Reference — Controls

### Driver — Xbox Controller

| Input | Action |
|-------|--------|
| **Left Stick** | Drive (forward/backward) and strafe (left/right) — field-relative |
| **Right Stick X** | Rotate robot — field-relative |
| **Left Trigger (LT)** | Deploy intake arm and toggle roller — press to start, press again to stop roller (arm stays deployed) |
| **Left Bumper (LB)** | Retract intake arm |
| **Right Trigger (RT)** | **Auto Shoot** — full predictive targeting; toggle on to start, toggle off to stop |
| **Right Bumper (RB)** | **Manual Shoot** — runs kicker and spindexer only; use when manually aimed |
| **A Button** | **Turret-Servo Fallback Shoot** — hold when turret is inoperable; locks turret to 0°, rotates robot chassis to aim, fires when aligned. Left stick still drives. Release to return to normal. |
| **B Button** | **Auto Drive to Fuel** — hold to drive backward toward detected game pieces and auto-deploy intake; release to stop |
| **Y Button** | Drive to selected spot (operator selects the spot on button box first) |
| **X Button** | Lock wheels in X pattern — defensive stance |
| **Back Button** | Reset gyro heading only (keep current XY) — intake must face alliance wall |
| **Start Button** | **Reset pose to hub wall** — resets full pose and heading. **Shooter side must be flush against the hub wall** before pressing. Works while disabled. |

> **Speed note:** When RT is toggled on (Auto Shoot active), the robot automatically reduces drive speed and rotation speed to improve aiming accuracy while moving. This is normal — do not fight it. Prefer strafing to rotating.

### Operator — Button Box (Elastic Dashboard)

The button box is organized into sections. Key groups:

| Section | What It Does |
|---------|-------------|
| **Spots** | Select a field position for the driver to auto-drive to (then press Y) |
| **Shooter** | Targeting mode, manual target selection, RPM trim, turret nudge |
| **Intake** | Deploy/retract arm, roller on/off, eject (jam clear), spindexer control |
| **Climb** | Extend/retract elevator; full auto-climb to a spot |
| **Calibration** | Gyro reset, subsystem homing, vision sync — most require DISABLED mode |

### Field Widget (Button Box)

The button box displays a live field map. The operator should monitor this throughout the match.

| Indicator | What It Shows |
|-----------|--------------|
| **Green square** | Robot's current estimated position on the field |
| **Yellow circle (fuel icon)** | Where the shooter is currently targeting |
| **White X** | **During disabled:** selected auto's starting pose — use this for pre-match alignment. |
| **Green X** | **During teleop:** the auto-drive (SpotSelector) destination. |

**The Field widget is your primary odometry health check.** If the green square does not match where the robot physically is on the field, the pose estimate is wrong and shooting and auto-drive will both be unreliable. See [Diagnosing Problems](#4-diagnosing-problems-during-a-match) for how to correct this.

---

## 2. Normal Operations (All Systems Go)

This section describes what everything should look like and do when working correctly.

### Auto Drive to Fuel (Game Piece Chase)

The `intake-cam` PhotonVision coprocessor detects fuel game pieces on the floor. When B is held, the robot automatically steers its rear intake toward the nearest/densest grouping and deploys the intake.

1. **Driver holds B button** — robot drives backward toward detected game pieces and deploys the intake automatically.
2. **Release B** — robot stops immediately and driver regains full joystick control.
3. If no game pieces are visible, the robot will not move — it will not drive blind.
4. The command times out automatically after 5 seconds even if B is still held.

> **Tip:** The robot steers toward the *densest nearby cluster* — if multiple pieces are visible, it will home in on the group rather than flipping between individual pieces.

### Intake (Picking Up Game Pieces — Manual)

1. **Driver presses Left Trigger (LT)** — arm deploys and roller spins.
   - The arm releases to coast so it can flex if hit by another robot. This is intentional.
2. **Driver presses LT again** — roller stops. Arm remains deployed.
3. **Driver presses Left Bumper (LB)** — arm retracts and roller stops.
3. If the roller jams, the operator can press **Intake / Eject** on the button box to reverse it briefly.

### Shooting — Automatic Mode (Preferred)

Automatic mode is the default and preferred way to shoot. The robot handles all aiming.

**Operator responsibilities before shooting:**
- Confirm the correct target is selected. Check that the auto-target zone matches where you are on the field:
  - Alliance zone (your side) → **Hub** target
  - Neutral zone → **Neutral Left** or **Neutral Right** based on field position
  - Opponent zone → **Opposition Left** or **Opposition Right**
- If the wrong target is selected, press the correct target button on the button box, or press **Target Enable Auto** to let the robot choose based on field position.

**Driver responsibilities during shooting:**
1. Press **Right Trigger (RT)** to toggle auto shoot on.
2. Watch the LEDs — when the shooter status LEDs turn **solid green**, the robot is ready and actively firing.
3. Press RT again when the shot is done to toggle auto shoot off.
4. Drive gently while shooting — the speed limiter will help, but smoother movement = better shots.

**What the robot does automatically:**
- Turret aims at the selected target with predictive compensation for robot movement.
- Hood pitches to the calculated angle for the current distance.
- Flywheel spins up to the correct RPM.
- Kicker and spindexer only engage once all three are within tolerance (turret ±2°, hood ±2°, flywheel ±150 RPM).
- Once feeding starts, it continues even if tolerance is briefly lost — but stops immediately if the turret can no longer physically reach the target angle (e.g. robot is facing the wrong way). Feeding resumes automatically once the turret is back on target.
- Drive speed is reduced to 50% and rotation is heavily limited.

### Auto-Drive Between Intake and Shooting Position (Y Button)

The Y button is a one-press transit between the neutral zone and a shooting position. It reads the robot's current field position and automatically selects the right destination — no operator input needed.

**Selection logic (instant, on every Y press):**
- **Robot in home zone (near your hub):** selects the neutral-side hub shooting position and drives there.
- **Robot in neutral or opponent zone:** selects the shooting position on the same side of the field as the robot (top half → top corridor spot, bottom half → bottom corridor spot) and drives there.

The top/bottom split picks whichever trench corridor the robot is already aligned with, so the path naturally goes through the nearest trench.

**Typical collect-and-score cycle:**

1. Robot starts near the hub. Driver presses **Y** — robot drives through the nearest trench to the neutral-zone shooting position.
2. Driver presses **RT** to shoot. LEDs go green when ready.
3. After shooting, driver drives into the neutral zone to collect fuel (LT or B button).
4. When ready to return, driver presses **Y** again — robot picks the correct corridor based on its current Y position and drives to the shooting spot.
5. Repeat.

> **Y is hold-to-run.** Release at any time to take manual control. The robot stops smoothly mid-path.

> **Y auto-selects on every press.** If the robot crosses the field Y midline between presses, it will switch corridors automatically.

### Chassis Aim — Spinning Toward Target (A Button)

Holding **A** locks the robot chassis rotation toward the current shooting target (turret locked to 0°, robot body rotates to aim). The left stick still drives normally.

This is useful when:
- The **turret is at or near its rotation limit** — rotating the chassis opens up a better turret angle for the next shot.
- You want to **pre-orient the robot** toward the target before toggling RT, so the turret has less travel to cover.
- The **turret is inoperable** — A fires using chassis rotation as the sole aiming mechanism (see [Turret-Servo Fallback](#problem-turret-is-stuck--inoperable-fallback-shoot-mode) in troubleshooting).

The robot will fire automatically once aligned and all other systems are ready — the same readiness logic as auto shoot. Release A to return to normal free-rotation drive and standard turret tracking.

### Hood Auto-Retract in Trench Zones

The hood automatically lowers to a safe angle when the robot enters the trench zones on either side of the field. No button press needed. If the hood does not retract when entering a trench, use the **Shooter / Retract Hood** button on the button box.

### Speed Nanny

The speed limiter (SpeedNanny) is always active. During normal driving it applies no restriction. When auto shoot is toggled on (RT), it automatically cuts speed to 50% and nearly eliminates rotation. When toggled off, full speed is restored immediately.

---

## 3. Reading the LEDs

The LEDs are your most important real-time diagnostic tool during a match. Learn these patterns before your first match.

### LED Layout

The robot LED strip is divided into three sections:

```
[Top]    — Vision status (both cameras combined)
[Middle] — Shooter readiness
[Bottom] — Game piece / match timer
```

---

### Top LEDs — Vision / AprilTag Status

This tells you how well the robot can see AprilTags for pose estimation.

| Color | Meaning | Impact on Robot |
|-------|---------|----------------|
| **BLUE** | Both cameras see ≥ 1 AprilTag | Excellent — full dual-camera pose estimation |
| **LIME** | Left camera only sees ≥ 1 AprilTag | OK — single-camera; aiming may be slightly less confident |
| **ROSE** | Right camera only sees ≥ 1 AprilTag | OK — single-camera; aiming may be slightly less confident |
| **RED** | No AprilTags visible | Bad — robot is relying on wheel odometry only; aiming accuracy degrades over time |

> **Key:** If the top LEDs are RED for more than a few seconds during a match, odometry drift may begin to affect shooting accuracy and auto-driving. See [Diagnosing Problems](#4-diagnosing-problems-during-a-match).

---

### Middle LEDs — Shooter Readiness

| Color | Meaning |
|-------|---------|
| **Alliance color** (red/blue) | Robot is **disabled** |
| **AMBER** | Idle — auto shoot is not active; shooter is standing by |
| **RED** | Targeting — shoot command active, waiting for turret/hood/flywheel to align |
| **GREEN** | Ready — all systems on target; kicker is firing |

> **When holding RT**, expect the middle LEDs to go RED briefly (aligning), then GREEN when ready and firing. If they stay RED for more than 2–3 seconds, something is not aligning — see troubleshooting.

---

### Bottom LEDs — Game Piece & Match Timer

| Color | Animation | Meaning |
|-------|-----------|---------|
| **Alliance color** | Solid | Robot is **disabled** |
| **WHITE** | Solid | No game piece detected |
| **YELLOW** | Solid | Game piece detected |
| **Any color** | **Strobing** | Match shift transition warning (last 5 sec of a shift) |

> **Note:** Game piece detection uses the `intake-cam` PhotonVision coprocessor (the same one used by the B button chase command) — YELLOW when the camera sees a fuel game piece on the floor, WHITE when none are detected.

---

### LED Summary — At a Glance

```
TOP:    BLUE = both cams, LIME = left only, ROSE = right only, RED = blind
MIDDLE: AMBER = idle, RED = aligning, GREEN = ready/firing
BOTTOM: YELLOW = game piece detected, WHITE = none, STROBE = shift warning
```

---

## 4. Diagnosing Problems During a Match

### Problem: Field Widget Shows Wrong Robot Position

The field widget is your first indicator of a pose problem. If the green square doesn't match where the robot actually is, aiming and auto-drive will both be wrong.

**Check the top LEDs.**

- **Top LEDs RED (no tags):** Neither camera can see AprilTags so the pose estimator is running on wheel odometry alone. Odometry drifts during play.
  - **Fix:** Rotate the robot so the **intake faces the alliance wall**, then drive forward toward the field so the cameras can pick up AprilTags. The top LEDs will turn LIME, ROSE, or BLUE when tags are found and the pose will correct automatically.
  - If you have time pre-match: use **Calibration / Reseed from Vision** to sync the gyro heading from vision (see [Calibration Procedures](#6-calibration-procedures) for when to use this).

- **Top LEDs LIME, ROSE, or BLUE but Field widget still shows wrong position:** The pose may have drifted during a period of lost vision. Continue driving with the cameras in view of tags — the estimator will pull the pose back over the next few seconds. If it doesn't recover, use fixed-distance shots as a fallback.

- **Pose is badly wrong and you can reach the hub:** Use the **hub wall reset**.
  1. Drive the robot to the hub and push the **shooter side flush against the hub wall**.
  2. Driver press **Start button** (or operator press **Calibration / Reset Pose Hub Wall** on the dashboard).
  3. The robot's full pose — position and heading — is immediately snapped to the known hub wall location. This works **while disabled**.
  4. Top LEDs will confirm vision reacquisition once the pose is correct and tags are in view.

### Problem: Robot Is Not Shooting Accurately

**Check the Field widget first** — if the robot's green square is in the wrong place, fix the pose first (see above). Shooting accuracy directly depends on pose accuracy.

- **Top LEDs are RED:** Neither camera sees any AprilTags. The robot is aiming based on dead-reckoning (wheel odometry only). Odometry drifts during play — the longer the robot goes without vision, the worse aiming becomes.
  - **Fix:** Rotate intake toward the alliance wall and drive to expose cameras to AprilTags. Top LEDs will turn LIME, ROSE, or BLUE when tags are found.
  - **If vision cannot recover:** Use fixed-distance shots (see [Manual Controls](#5-manual-controls--button-box-override)). Drive to a known distance from target and use **Shot 1m / 2m / 3m**.

- **Top LEDs are LIME, ROSE, or BLUE but shots are still off:** The pose estimate may have briefly drifted before vision reacquired. Give it a moment while keeping tags in view — the estimator will self-correct.

- **Middle LEDs stay RED after toggling RT on:** The shooter is not ready. Possible causes:
  - Most likely the turret cannot reach desired angle because of the current robot direction.  Turret hit a hard stop or is stuck — operator press **Shooter / Turret Center** to re-home, then re-enable auto.
  - Hood did not reach target angle — operator press **Shooter / Retract Hood** then re-engage.
  - Flywheel did not reach target RPM — check if shooter was just turned ON (give it 2–3 seconds to spin up).
  - Shooter is OFF — operator press **Shooter / Normal Targeting** to re-enable the shooter.

### Problem: Turret Is Stuck / Inoperable (Fallback Shoot Mode)

If the turret cannot move (mechanical jam, motor fault), the robot can still shoot using the **A button** turret-servo fallback:

1. Driver holds **A button** — turret locks to 0° (straight ahead relative to robot), robot chassis rotates to face the target, hood and flywheel spin up normally.
2. Left stick still drives — the robot will steer toward the required heading while you translate.
3. Watch the LEDs — middle LEDs will go **GREEN** and fire once the robot heading is within 2° of the required aim angle.
4. Release **A** when done — normal turret targeting and free rotation resume.

> **Note:** In this mode the robot rotates to aim rather than the turret. Drive smoothly and avoid large rotation inputs that fight the heading servo. Turret trim (D-pad) still affects the required heading, so existing trim corrections carry over.

> **Turret servo mode does NOT work in autonomous.** It is teleop-only.

### Problem: Robot Won't Drive to Spots Correctly (Y Button)

Auto-driving uses the robot's pose estimate. If the pose is wrong, the robot will drive to the wrong place.

- Check the Field widget — does the green square match where the robot is? If not, fix the pose first (drive to see AprilTags with cameras facing toward the field).
- If the green X (auto-drive destination) appears in the wrong location, the SpotSelector may have selected a spot that doesn't match your intent. Reselect the spot on the button box.
- If odometry is severely corrupted and vision is unavailable, do not use auto-drive. Drive manually.

### Problem: Autonomous Path Not Executing Correctly

- Vision unavailable at match start can cause the robot's initial pose to be wrong, throwing off all auto paths.
- **Before the match:** Confirm top LEDs are BLUE (both cameras locked) when placed on the field. LIME or ROSE (one camera) is acceptable but less ideal. If RED at start of auto, auto may be unreliable.
- If auto goes wrong, switch to teleop mode as soon as possible and use manual controls.

### Problem: Aiming Lands Consistently Short / Long

The flywheel RPM trim may need adjustment.

- **Short shots:** Operator press **Shooter / RPM +50** (can press multiple times, each adds 50 RPM).
- **Long shots:** Operator press **Shooter / RPM -50**.
- **Back to baseline:** Operator press **Shooter / RPM Reset**.
- Trim is cumulative and persists until reset. Keep track of how many presses you've made.

### Problem: Aiming Lands Consistently Too High / Too Low (Hood)

The hood trim may need adjustment.

- **Shots too high (overshooting):** Operator press **Shooter / Hood -1deg** (can press multiple times, each removes 1°).
- **Shots too low (undershooting):** Operator press **Shooter / Hood +1deg**.
- **Back to baseline:** Operator press **Shooter / Hood Trim Reset**.
- Hood trim is clamped to the hood's full angular range and persists until reset.
- Hood retract commands (idle, trench auto-retract, **Retract Hood** button) are **not** affected by the trim — they always go to the minimum safe angle.

### Problem: Aiming Is Consistently Left / Right

The turret auto-aim may need to be nudged.

- Operator press **Shooter / Turret Left 5deg** or **Shooter / Turret Right 5deg** (each press is ±5°, cumulative).
- **Use carefully** — nudging overrides the calculated aim. If you get too far off, press **Turret Center** to reset to 0° and then re-enable auto targeting.

### Problem: Intake Roller Jammed

- Operator press **Intake / Eject** to reverse the roller briefly.
- If jam persists, retract the arm (LB) and try deploying again.

### Problem: Robot Driving Too Slowly

- This is probably intentional — the driver is holding RT (shoot mode) which cuts speed to 50%.
- If not in shoot mode and robot is still slow, check if a stuck command is reporting `isShooting() = true`. Operator press **Shooter / Off** and then re-enable the shooter.

### Problem: Gyro Heading Is Wrong After a Collision

- Driver press **Back button** to reset gyro to alliance-correct zero. **Only do this when the intake is facing the alliance wall** — that is the assumed heading for zero.
- This resets the heading only; X/Y position on the field is not changed.
- To restore X/Y accuracy, drive so the cameras can see AprilTags. The pose estimator will fuse vision measurements and pull the position back toward correct automatically.
- **Calibration / Reseed from Vision** also only corrects the gyro heading (from the vision system) — it does not directly fix X/Y position. See [Calibration Procedures](#6-calibration-procedures).

---

## 5. Manual Controls — Button Box Override

Use manual controls when automatic functions are not working reliably.

### Manual Targeting Workflow

When automatic targeting is unreliable (bad pose, vision down):

1. Operator presses one of the **Shooter / Target [name]** buttons to lock a specific target manually.
   - Auto-targeting is now **disabled** until you press **Target Enable Auto**.
   - Selected target stays locked even as the robot moves.
2. Operator uses **Turret Center**, **Turret Left 5deg**, **Turret Right 5deg** to manually adjust turret angle.
3. Operator uses **RPM +50 / -50** to trim flywheel speed; **Hood +1deg / -1deg** to trim hood angle.
4. Driver presses **Right Trigger (RT)** to toggle auto shoot on once aligned.

> **When done:** Press **Shooter / Target Enable Auto** to return to zone-based auto-targeting.

### Fixed-Distance Shots (Vision-Independent)

When pose estimation is lost entirely and you need to shoot:

1. Drive to a known, consistent distance from the target.
2. Operator presses **Shooter / Shot 1m**, **Shot 2m**, or **Shot 3m** depending on distance.
   - These commands lock the turret to 0° (straight ahead) and apply pre-tuned hood angle and RPM.
   - You must physically align the robot heading toward the target.
3. Driver presses **Right Trigger (RT)** to toggle auto shoot on, or holds **Right Bumper (RB)** to fire manually.

> These are tuned for specific distances. Use them at approximately 1 m, 2 m, or 3 m.

### Manual Shoot (RB) — Kicker and Spindexer Only

Right Bumper runs the kicker and spindexer but does NOT wait for shooter readiness. Use this when:
- You've pre-set turret, hood, and flywheel manually and just need to feed.
- Automatic readiness detection is not behaving correctly.

This mode trusts the operator to confirm the shooter is correctly aimed before pressing RB.

### Manual Spindexer Control

Operator can control the spindexer independently of the shoot command:
- **Intake / Run Spindexer** — forward at 500 RPM
- **Intake / Stop Spindexer** — stop
- **Intake / Reverse Spindexer** — reverse (jam clearing)

### Shooter Off

Operator press **Shooter / Off** to completely stop all shooter mechanisms (turret, hood, flywheel, kicker, spindexer all stop). Use to reset a misbehaving shooter state. After pressing Off, you must re-enable with **Shooter / Normal Targeting** or **Shooter / Predictive Targeting**.

### Hood Manual Control

- **Shooter / Retract Hood** — lowers hood to the minimum safe angle for driving. Use when:
  - Entering a trench and auto-retract didn't trigger.
  - Hood is stuck at an angle during a disabled period.
- Hood auto-targeting resumes when the next shoot command is issued.

---

## 6. Calibration Procedures

> **Most calibration must be done while the robot is DISABLED.** A few exceptions (homing) require ENABLED. Read each command label carefully.

### Gyro / Heading Reset

- **Driver Back button** — Resets heading only (preserves XY). Intake must face the alliance wall. Alliance-aware: Blue = 180°, Red = 0°.
- **Calibration / Reset Gyro** — Same heading-only reset from the dashboard; works while disabled.

### Hub Wall Pose Reset

Use this when the robot's full pose (position + heading) has drifted badly and vision cannot recover it on its own.

1. Drive the robot to the hub and push the **shooter side flush against the hub wall**.
2. Driver press **Start button** — or operator press **Calibration / Reset Pose Hub Wall** on the dashboard.
3. The robot's pose snaps to the known hub wall position with the correct alliance heading. Works while disabled.

> This is the fastest way to get a reliable pose in the middle of a match after a severe odometry loss. Vision will begin fusing immediately once the corrected pose is set and tags are in view.

### Pose Re-Sync (Vision)

- **Calibration / Reseed from Vision** — Updates the **gyro heading** from the vision system immediately. This does **not** directly update X/Y position — it only corrects the robot's rotational heading using what the cameras see. Intended primarily for **pre-match** use: place the robot on the field, confirm cameras see tags, and reseed to align the gyro with vision before the match starts. Can be used in a match but the effect is limited to heading only.
- **Calibration / Enable Normal Gyro** — Gyroscope only, no vision heading updates. This is the **default** mode.
- **Calibration / Enable Vision Gyro** — Vision contributes to heading estimation in addition to the gyro. Enable if you want MegaTag-assisted yaw correction.

### Turret Calibration

- **Turret Home & Calibrate** (ENABLED) — Drives turret to hard stop to find mechanical zero. Do at start of match if turret was moved while powered off.
- **Turret Calibrate In Place** (DISABLED) — Zeros encoder at the current physical position. Only use if turret is known to be centered.

### Hood Calibration

- **Hood Home & Calibrate** (ENABLED) — Drives hood to hard stop to find mechanical zero.
- **Hood Calibrate In Place** (DISABLED) — Zeros encoder at current position. Only use if hood is at minimum angle.
- **Hood Zero** — Sets hood to minimum angle (safe for driving). Use after a match if hood is stuck up.

### Intake Arm Calibration

- **Intake Arm Home & Calibrate** (ENABLED) — Homes arm to hard stop.
- **Intake Arm Calibrate In Place** (DISABLED) — Zeros encoder in place.

### Climb Calibration

- **Climb Home & Calibrate** (ENABLED) — Homes elevator to hard stop.
- **Climb Calibrate In Place** (DISABLED) — Zeros encoder in place.

---

## 7. Autonomous Mode

### Selecting an Auto Routine

1. Before the match, use the **Elastic Dashboard auto chooser dropdown** to select a routine.
2. You can change the selection while disabled — it takes effect at auto start.
3. Do not change selection after the match starts.

### Verifying Start Position with the Field Widget

When the robot is disabled, the **white X on the Field widget automatically moves to the selected auto's starting pose**. Use this to verify robot placement before the match:

- Place the robot on the field, then check the Field widget.
- The **green square** (robot's current pose) should overlap the **white X** (auto start pose).
- If they don't overlap, either the robot is in the wrong physical position, or the pose estimate is wrong.
  - If the robot is misplaced: physically move the robot to match the white X.
  - If the pose estimate is wrong: confirm top LEDs are **BLUE** (or LIME/ROSE), then use **Calibration / Reseed from Vision** to correct heading.
- Changing the auto chooser selection will immediately move the white X to reflect the new routine's starting pose.

### Available Auto Routines

** TODO ** Section needs updated.

| Routine | Description |
|---------|-------------|
| `AroundTheHub` | — |
| `Disrupt` | — |
| `Pickup Fuel` | — |
| `New Auto` | — |
| `Example Auto` | Basic path for testing |

> Routine descriptions should be filled in by drive coaches as routines are finalized.

### If Auto Goes Wrong

- Switch to teleop immediately at the end of auto.
- If the robot's position is wrong after auto (drove to wrong spot), operator press **Calibration / Reseed from Vision** as soon as teleop starts to correct the pose.
- If vision is unavailable, drive manually and use fixed-distance shots.

---

## 8. Pre-Match Checklist

Complete this before every match.

### Field Placement

- [ ] Robot placed on correct starting position per game plan
- [ ] Robot powered on and connected to FMS

### Vision (Top LEDs)

- [ ] Top LEDs are **BLUE** (both cameras seeing tags) — preferred
- [ ] If LIME or ROSE (one camera only), confirm the other camera is aligned and connected
- [ ] **Never start a match with top LEDs RED** — push robot to see tags or get field crew to check camera connections

### Shooter

- [ ] Turret moves freely — no physical obstructions
- [ ] Flywheel pre-spin is audible / middle LEDs confirm system active
- [ ] Auto-targeting mode is set correctly (check button box)
- [ ] RPM trim is reset (**Shooter / RPM Reset**)
- [ ] Hood trim is reset (**Shooter / Hood Trim Reset**)
- [ ] Turret nudge is cleared (verify turret snaps to calculated aim automatically)

### Gyro / Heading & Pose Verification

- [ ] **Position the robot with the intake facing the alliance wall** before zeroing the gyro
- [ ] Driver press **Back button** or operator press **Calibration / Reset Gyro** to zero heading
- [ ] Check the **Field widget** — green square should show the robot's correct starting position
- [ ] Compare the gyro heading displayed on the dashboard with the heading reported by vision; they should agree within a few degrees. If they disagree significantly:
  - Option A: Square the robot to the alliance wall and press **Back button** to reset the gyro
  - Option B: Use **Calibration / Reseed from Vision** to update the gyro heading from what the cameras are seeing (requires cameras to have tag visibility)
- [ ] Top LEDs should be **BLUE** (or at minimum LIME/ROSE) before accepting any heading/pose as trustworthy

### Autonomous

- [ ] Correct auto routine selected on dashboard
- [ ] **Field widget white X matches the robot's physical starting position** — the white X shows the selected auto's start pose; the green square shows the robot's current pose. They should overlap.
- [ ] If they don't overlap: either physically move the robot to match the white X, or fix the pose estimate (Reseed from Vision) until they agree

### Communication

- [ ] Driver and operator reviewed the match strategy
- [ ] Both know which manual fallback modes to use if vision is lost
- [ ] Operator knows current alliance color (affects gyro reset direction)

---

## Common Situations — Decision Tree

```
SHOTS CONSISTENTLY MISSING
│
├─ Top LEDs RED (no vision)?
│   ├─ Can you drive to see AprilTags? → Do it, wait for GREEN
│   ├─ Vision still can't recover → Use fixed-distance shots (Shot 1m/2m/3m)
│   └─ Operator: Reseed from Vision if tags briefly seen
│
├─ Middle LEDs stay RED after toggling RT on?
│   ├─ Turret stuck/inoperable? → Driver: hold A (turret-servo fallback) instead of RT
│   ├─ Turret stuck but movable? → Operator: Turret Center, then re-enable RT
│   ├─ Hood stuck? → Operator: Retract Hood, then re-engage
│   ├─ Flywheel slow? → Wait 2-3 sec for spin-up
│   └─ Shooter off? → Operator: Normal Targeting button
│
└─ LEDs GREEN but aim is off?
    ├─ Consistently short/long → Operator: RPM ±50 trim
    ├─ Consistently high/low  → Operator: Hood ±1deg trim
    ├─ Consistently left/right → Operator: Turret ±5deg nudge
    └─ Pose drifted? → Operator: Reseed from Vision


ROBOT WON'T DRIVE TO SPOTS (Y button)
│
├─ Top LEDs RED? → Reseed from Vision first
├─ White X not overlapping green square? → Fix pose (Reseed from Vision or hub wall reset)
├─ Pose badly wrong, near hub? → Push shooter side flush to hub wall → Driver: Start button
├─ Still wrong after reseed? → Vision unavailable, drive manually
└─ Gyro wrong? → Driver: Back button to reset heading (intake facing alliance wall)


INTAKE JAMMED
│
├─ Operator: Intake / Eject (reverse roller)
└─ Retract arm (LB), then press LT to re-deploy and start roller
```

---

## Change Log

| Date | Change |
|------|--------|
| 2026-03-11 | **A button** — new turret-servo fallback shoot mode: hold A when turret is inoperable; robot chassis rotates to aim with turret locked at 0° |
| 2026-03-06 | **RPM trim** step reduced from ±100 to ±50 RPM per press (`Shooter/RPM +50`, `Shooter/RPM -50`) |
| 2026-03-06 | **Hood trim** added: `Shooter/Hood +1deg`, `Shooter/Hood -1deg`, `Shooter/Hood Trim Reset` — trims the firing solution hood angle without affecting retract commands |
| 2026-03-06 | **Average RPM** (`Shooter/FlyWheelAverageRPM`) added to telemetry — EMA-smoothed flywheel RPM for stable dashboard reading |
| 2026-03-04 | **Left Trigger** changed from hold to toggle — press once to deploy arm and start roller, press again to stop roller (arm stays deployed) |
| 2026-03-04 | **Right Trigger** changed from hold to toggle — press once to start auto shoot, press again to stop |
| 2026-03-04 | **Auto Shoot** feeding now stops and automatically resumes if the turret loses feasibility (e.g. robot faces away from target) |

---

*For technical documentation see CLAUDE.md. For questions during competition, contact the programming lead.*
