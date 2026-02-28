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

---

## 1. Quick Reference — Controls

### Driver — Xbox Controller

| Input | Action |
|-------|--------|
| **Left Stick** | Drive (forward/backward) and strafe (left/right) — field-relative |
| **Right Stick X** | Rotate robot — field-relative |
| **Left Trigger (LT)** | Deploy intake arm and run roller — hold to keep active |
| **Left Bumper (LB)** | Retract intake arm |
| **Right Trigger (RT)** | **Auto Shoot** — full predictive targeting; holds until ready and fires |
| **Right Bumper (RB)** | **Manual Shoot** — runs kicker and spindexer only; use when manually aimed |
| **Y Button** | Drive to selected spot (operator selects the spot on button box first) |
| **X Button** | Lock wheels in X pattern — defensive stance |
| **Back Button** | Reset gyro to zero (alliance-aware intake side should be visibile and square.) |

> **Speed note:** When RT is held (Auto Shoot), the robot automatically reduces drive speed to 50% and nearly eliminates rotation to improve aiming accuracy while moving. This is normal — do not fight it.  Prefer straffing to rotating.

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
| **Green X** | **During disabled:** selected auto's starting pose — use this for pre-match alignment. **During teleop:** the auto-drive (SpotSelector) destination. |

**The Field widget is your primary odometry health check.** If the green square does not match where the robot physically is on the field, the pose estimate is wrong and shooting and auto-drive will both be unreliable. See [Diagnosing Problems](#4-diagnosing-problems-during-a-match) for how to correct this.

---

## 2. Normal Operations (All Systems Go)

This section describes what everything should look like and do when working correctly.

### Intake (Picking Up Game Pieces)

1. **Driver holds Left Trigger (LT)** — arm deploys and roller spins.
   - The arm releases to coast so it can flex if hit by another robot. This is intentional.
2. **Driver releases LT and presses Left Bumper (LB)** — arm retracts and roller stops.
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
1. Hold **Right Trigger (RT)**.
2. Watch the LEDs — when the shooter status LEDs turn **solid green**, the robot is ready and actively firing.
3. Continue holding RT until the shot is done. The robot feeds continuously while RT is held and the shooter is ready.
4. Drive gently while shooting — the speed limiter will help, but smoother movement = better shots.

**What the robot does automatically:**
- Turret aims at the selected target with predictive compensation for robot movement.
- Hood pitches to the calculated angle for the current distance.
- Flywheel spins up to the correct RPM.
- Kicker and spindexer only engage once all three are within tolerance (turret ±2°, hood ±2°, flywheel ±150 RPM).
- Drive speed is reduced to 50% and rotation is heavily limited.

### Hood Auto-Retract in Trench Zones

The hood automatically lowers to a safe angle when the robot enters the trench zones on either side of the field. No button press needed. If the hood does not retract when entering a trench, use the **Shooter / Retract Hood** button on the button box.

### Speed Nanny

The speed limiter (SpeedNanny) is always active. During normal driving it applies no restriction. When the driver holds RT to shoot, it automatically cuts speed to 50% and nearly eliminates rotation. When RT is released, full speed is restored immediately.

---

## 3. Reading the LEDs

The LEDs are your most important real-time diagnostic tool during a match. Learn these patterns before your first match.

### LED Layout

The robot has two side LED strips (left and right). Each strip is divided into three sections:

```
[Top]    — Limelight vision status
[Middle] — Shooter readiness
[Bottom] — Game piece / match timer
```

Both sides mirror each other, so read either one.

---

### Top LEDs — Vision / AprilTag Status

This tells you how well the robot can see AprilTags for pose estimation.

| Color | Meaning | Impact on Robot |
|-------|---------|----------------|
| **GREEN** | 2+ AprilTags visible | Excellent — pose estimation is confident |
| **BLUE** | 1 AprilTag visible | OK — single-tag is lower confidence; aiming may be slightly off |
| **RED** | No AprilTags visible | Bad — robot is relying on wheel odometry only; aiming accuracy degrades over time |

> **Key:** If the top LEDs are RED for more than a few seconds during a match, odometry drift may begin to affect shooting accuracy and auto-driving. See [Diagnosing Problems](#4-diagnosing-problems-during-a-match).

---

### Middle LEDs — Shooter Readiness

| Color | Meaning |
|-------|---------|
| **Alliance color** (red/blue) | Robot is **disabled** |
| **AMBER** | Idle — driver is not holding RT; shooter is standing by |
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

> **Note:** Game piece detection camera is not yet installed. Bottom LEDs will show WHITE (no piece) until the sensor is added.

---

### LED Summary — At a Glance

```
TOP:    GREEN = see tags, BLUE = one tag, RED = blind
MIDDLE: AMBER = idle, RED = aligning, GREEN = ready/firing
BOTTOM: YELLOW = game piece loaded, WHITE = empty, STROBE = shift warning
```

---

## 4. Diagnosing Problems During a Match

### Problem: Field Widget Shows Wrong Robot Position

The field widget is your first indicator of a pose problem. If the green square doesn't match where the robot actually is, aiming and auto-drive will both be wrong.

**Check the top LEDs.**

- **Top LEDs RED (no tags):** Cameras can't see AprilTags so the pose estimator is running on wheel odometry alone. Odometry drifts during play.
  - **Fix:** Rotate the robot so the **intake faces the alliance wall**, then drive forward toward the field so the cameras can pick up AprilTags. The top LEDs will turn blue or green when tags are found and the pose will correct automatically.
  - If you have time pre-match: use **Calibration / Reseed from Vision** to sync the gyro heading from vision (see [Calibration Procedures](#6-calibration-procedures) for when to use this).

- **Top LEDs GREEN but Field widget still shows wrong position:** The pose may have drifted during a period of lost vision. Continue driving with the cameras in view of tags — the estimator will pull the pose back over the next few seconds. If it doesn't recover, use fixed-distance shots as a fallback.

### Problem: Robot Is Not Shooting Accurately

**Check the Field widget first** — if the robot's green square is in the wrong place, fix the pose first (see above). Shooting accuracy directly depends on pose accuracy.

- **Top LEDs are RED:** Vision is down. The robot is aiming based on dead-reckoning (wheel odometry only). Odometry drifts during play — the longer the robot goes without vision, the worse aiming becomes.
  - **Fix:** Rotate intake toward the alliance wall and drive to expose cameras to AprilTags. Top LEDs will turn blue or green when tags are found.
  - **If vision cannot recover:** Use fixed-distance shots (see [Manual Controls](#5-manual-controls--button-box-override)). Drive to a known distance from target and use **Shot 1m / 2m / 3m**.

- **Top LEDs are GREEN but shots are still off:** The pose estimate may have briefly drifted before vision reacquired. Give it a moment while keeping tags in view — the estimator will self-correct.

- **Middle LEDs stay RED after holding RT:** The shooter is not ready. Possible causes:
  - Most likely the turret cannot reach desired angle because of the current robot direction.  Turret hit a hard stop or is stuck — operator press **Shooter / Turret Center** to re-home, then re-enable auto.
  - Hood did not reach target angle — operator press **Shooter / Retract Hood** then re-engage.
  - Flywheel did not reach target RPM — check if shooter was just turned ON (give it 2–3 seconds to spin up).
  - Shooter is OFF — operator press **Shooter / Normal Targeting** to re-enable the shooter.

### Problem: Robot Won't Drive to Spots Correctly (Y Button)

Auto-driving uses the robot's pose estimate. If the pose is wrong, the robot will drive to the wrong place.

- Check the Field widget — does the green square match where the robot is? If not, fix the pose first (drive to see AprilTags with cameras facing toward the field).
- If the green X (auto-drive destination) appears in the wrong location, the SpotSelector may have selected a spot that doesn't match your intent. Reselect the spot on the button box.
- If odometry is severely corrupted and vision is unavailable, do not use auto-drive. Drive manually.

### Problem: Autonomous Path Not Executing Correctly

- Vision unavailable at match start can cause the robot's initial pose to be wrong, throwing off all auto paths.
- **Before the match:** Confirm top LEDs are GREEN (vision locked) when placed on the field. If RED at start of auto, auto may be unreliable.
- If auto goes wrong, switch to teleop mode as soon as possible and use manual controls.

### Problem: Aiming Lands Consistently Short / Long

The flywheel RPM trim may need adjustment.

- **Short shots:** Operator press **Shooter / RPM +100** (can press multiple times, each adds 100 RPM).
- **Long shots:** Operator press **Shooter / RPM -100**.
- **Back to baseline:** Operator press **Shooter / RPM Reset**.
- Trim is cumulative and persists until reset. Keep track of how many presses you've made.

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
3. Operator uses **RPM +100 / -100** to trim flywheel speed.
4. Driver holds **Right Trigger (RT)** to feed balls once aligned.

> **When done:** Press **Shooter / Target Enable Auto** to return to zone-based auto-targeting.

### Fixed-Distance Shots (Vision-Independent)

When pose estimation is lost entirely and you need to shoot:

1. Drive to a known, consistent distance from the target.
2. Operator presses **Shooter / Shot 1m**, **Shot 2m**, or **Shot 3m** depending on distance.
   - These commands lock the turret to 0° (straight ahead) and apply pre-tuned hood angle and RPM.
   - You must physically align the robot heading toward the target.
3. Driver holds **Right Trigger (RT)** or **Right Bumper (RB)** to fire.

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

- **Driver Back button** — Quick reset during a match (alliance-aware).
- **Calibration / Reset Gyro** — Dashboard reset; use when disabled.

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

When the robot is disabled, the **green X on the Field widget automatically moves to the selected auto's starting pose**. Use this to verify robot placement before the match:

- Place the robot on the field, then check the Field widget.
- The **green square** (robot's current pose) should overlap the **green X** (auto start pose).
- If they don't overlap, either the robot is in the wrong physical position, or the pose estimate is wrong.
  - If the robot is misplaced: physically move the robot to match the green X.
  - If the pose estimate is wrong: confirm top LEDs are **GREEN**, then use **Calibration / Reseed from Vision** to correct heading.
- Changing the auto chooser selection will immediately move the green X to reflect the new routine's starting pose.

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

- [ ] Top LEDs are **GREEN** (both cameras seeing 2+ tags)
- [ ] If only BLUE (one tag), confirm camera alignment is OK
- [ ] **Never start a match with top LEDs RED** — push robot to see tags or get field crew to check camera connections

### Shooter

- [ ] Turret moves freely — no physical obstructions
- [ ] Flywheel pre-spin is audible / middle LEDs confirm system active
- [ ] Auto-targeting mode is set correctly (check button box)
- [ ] RPM trim is reset (**Shooter / RPM Reset**)
- [ ] Turret nudge is cleared (verify turret snaps to calculated aim automatically)

### Gyro / Heading & Pose Verification

- [ ] **Position the robot with the intake facing the alliance wall** before zeroing the gyro
- [ ] Driver press **Back button** or operator press **Calibration / Reset Gyro** to zero heading
- [ ] Check the **Field widget** — green square should show the robot's correct starting position
- [ ] Compare the gyro heading displayed on the dashboard with the heading reported by vision; they should agree within a few degrees. If they disagree significantly:
  - Option A: Square the robot to the alliance wall and press **Back button** to reset the gyro
  - Option B: Use **Calibration / Reseed from Vision** to update the gyro heading from what the cameras are seeing (requires cameras to have tag visibility)
- [ ] Top LEDs should be **GREEN** before accepting any heading/pose as trustworthy

### Autonomous

- [ ] Correct auto routine selected on dashboard
- [ ] **Field widget green X matches the robot's physical starting position** — the green X shows the selected auto's start pose; the green square shows the robot's current pose. They should overlap.
- [ ] If they don't overlap: either physically move the robot to match the green X, or fix the pose estimate (Reseed from Vision) until they agree

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
├─ Middle LEDs stay RED after holding RT?
│   ├─ Turret stuck? → Operator: Turret Center, then re-enable
│   ├─ Hood stuck? → Operator: Retract Hood, then re-engage
│   ├─ Flywheel slow? → Wait 2-3 sec for spin-up
│   └─ Shooter off? → Operator: Normal Targeting button
│
└─ LEDs GREEN but aim is off?
    ├─ Consistently short/long → Operator: RPM ±100 trim
    ├─ Consistently left/right → Operator: Turret ±5deg nudge
    └─ Pose drifted? → Operator: Reseed from Vision


ROBOT WON'T DRIVE TO SPOTS (Y button)
│
├─ Top LEDs RED? → Reseed from Vision first
├─ Still wrong after reseed? → Vision unavailable, drive manually
└─ Gyro wrong? → Driver: Back button to reset heading


INTAKE JAMMED
│
├─ Operator: Intake / Eject (reverse roller)
└─ Retract arm (LB), then re-deploy (LT)
```

---

*For technical documentation see CLAUDE.md. For questions during competition, contact the programming lead.*
