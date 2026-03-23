# SuperFastClimb

This document describes the SuperFastClimb automated climb sequence — what it does, how the
command is structured, how tilt detection works, and how to tune and wire it.

---

## Overview

SuperFastClimb is a fully automated climb designed for speed. The robot drives to a fixed approach
point in front of the tower, extends the climb arm, then drives through the tower at reduced speed
with the arm raised. When the Pigeon2 IMU detects a significant tilt (indicating the arm has hooked
the tower bar), the drive stops and the arm retracts — winching the robot up.

The command can run in both **autonomous** (as a PathPlanner named command) and **teleop** (bound
to a button box button).

---

## Command Sequence

```
┌─────────────────────────────────────────────────────────────┐
│ Phase 1 — Pathfind to approach pose                         │
│   AutoBuilder.pathfindToPose(CLIMB_LEFT / CENTER / RIGHT)   │
│   Full approach speed (3.0 m/s)                             │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Phase 2 — Extend elevator to MAX_HEIGHT                     │
│   getExtendCmd().until(at max height)                       │
│   Safety timeout: CLIMB_EXTEND_TIMEOUT_SEC (3 s)            │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Phase 3 — RACE (ends when first participant finishes)       │
│                                                             │
│   ┌─────────────────────────────┐                          │
│   │ Pass drive                  │  ← PathPlanner drives    │
│   │ pathfindToPose(1m forward)  │    1m past approach pose │
│   └─────────────────────────────┘                          │
│           vs.                                               │
│   ┌─────────────────────────────┐                          │
│   │ Tilt sentinel               │  ← waitUntil(tilt >      │
│   │ (sets tiltHooked = true)    │    threshold) → HOOK!    │
│   └─────────────────────────────┘                          │
│           vs.                                               │
│   ┌─────────────────────────────┐                          │
│   │ Arm hold                    │  ← getExtendCmd()        │
│   │ (never finishes)            │    keeps arm commanded   │
│   └─────────────────────────────┘    up throughout pass    │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Phase 4 — Stop drive unconditionally                        │
│   drive.stop()                                              │
└──────────────────────────┬──────────────────────────────────┘
                           │
              ┌────────────┴────────────┐
              │ tiltHooked == true?     │
         YES  │                    NO   │
              ▼                         ▼
   ┌──────────────────────┐     ┌──────────────────────┐
   │ Phase 5a             │     │ Phase 5b             │
   │ getRobotClimbCmd()   │     │ Commands.none()      │
   │ (lift robot off      │     │ Arm holds at max     │
   │  ground safely)      │     │ height — driver      │
   └──────────────────────┘     │ decides next action  │
                                └──────────────────────┘
```

### Phase 3 race winner determines outcome

| Race winner | `tiltHooked` | Phase 5 action |
|-------------|:------------:|----------------|
| Pass drive (1m reached, no tilt) | `false` | Hold arm at max; driver decides |
| Tilt sentinel (tilt threshold crossed) | `true` | `getRobotClimbCmd()` → lift robot off ground (stops at `ROBOT_CLIMB_HEIGHT_M`, not full retract — avoids damaging side panels) |

The tilt sentinel uses `Commands.waitUntil()`, which ends **naturally** (not interrupted) when its
condition becomes true. Its `finallyDo` only sets `tiltHooked = true` when it is NOT interrupted,
so the flag is only ever set if the sentinel wins the race.

---

## Tilt Detection

### Hardware

The Pigeon2 IMU (already used for swerve odometry) provides roll and pitch. These are read via two
additional `StatusSignal<Angle>` at 50 Hz (separate from the odometry signals at 100–250 Hz).

### GyroIO changes

Two fields were added to `GyroIOInputs` (logged automatically by `@AutoLog`):

```java
public double rollRobotRad  = 0.0;  // robot-frame roll in radians
public double pitchRobotRad = 0.0;  // robot-frame pitch in radians
```

Both `GyroIOPigeon2` and `GyroIONavX` populate these fields. The default stub (`new GyroIO() {}`)
leaves them at 0.0, which is correct for sim and replay.

### Drive.getTiltMagnitudeRad()

```java
@AutoLogOutput(key = "Drive/TiltMagnitudeRad")
public double getTiltMagnitudeRad() {
    return Math.hypot(gyroInputs.rollRobotRad, gyroInputs.pitchRobotRad);
}
```

`Math.hypot` gives the Euclidean magnitude across both axes, so a tilt in any direction (roll,
pitch, or diagonal) will trigger the sentinel equally. The value is logged every loop via
`@AutoLogOutput`.

---

## Approach Poses

The three approach poses are defined in `FieldSpot`. All coordinates are **Blue-alliance**;
`FieldSpot.flipIfRed()` converts them for Red alliance automatically.

| Spot | Blue x (m) | Blue y (m) | Heading |
|------|:----------:|:----------:|:-------:|
| `CLIMB_LEFT` | 6.0 | 5.5 | 90° |
| `CLIMB_CENTER` | 6.0 | 4.1 | 90° |
| `CLIMB_RIGHT` | 6.0 | 2.7 | 90° |

> **TODO:** Replace these placeholder coordinates with actual 2026 field measurements.

### "1m past tower" pose

The pass target is computed from the approach pose by applying a `Transform2d` of 1m forward
in the pose's own heading frame:

```java
approachPose.transformBy(new Transform2d(CLIMB_PASS_DISTANCE_M, 0.0, Rotation2d.kZero))
```

Because the approach poses face 90°, the pass target moves 1m in the +Y direction (forward
through the tower). The same `flipIfRed()` is applied before the transform.

---

## Primary Operator Commands

These three commands are the normal teleop controls for the elevator, available on the button box
under the `Climb/` group.

| Button box key | Method | Target height | Behavior on cancel |
|----------------|--------|:-------------:|--------------------|
| `Climb/Extend Arm` | `getExtendCmd()` | `MAX_HEIGHT_M` | Motor stops; brake holds position |
| `Climb/Retract Arm` | `getRetractCmd()` | `MIN_HEIGHT_M` | Motor stops; brake holds position |
| `Climb/Robot Climb` | `getRobotClimbCmd()` | `ROBOT_CLIMB_HEIGHT_M` | Motor stops; brake holds position |

### Cancel / interrupt behavior

When any of these commands is interrupted (button released, another command takes over, or the
sequence ends), the elevator's **default command** (`Climb/Idle`) immediately runs. The default
command sends duty-cycle 0, which stops active motor output. The TalonFX `BRAKE` idle mode then
shorts the motor phases, and the 20:1 gear reduction makes back-driving the mechanism extremely
difficult — the elevator holds its position without active closed-loop control.

---

## Constants

All tunable values live in `ClimbElevatorConstants.java`.

| Constant | Default | Description |
|----------|--------:|-------------|
| `MIN_HEIGHT_M` | `0.0` | Bottom hard-stop height (meters) |
| `MAX_HEIGHT_M` | `0.5` | Full extension height (meters) — TODO: tune on hardware |
| `ROBOT_CLIMB_HEIGHT_M` | `0.3` | Height that lifts the robot off the ground (meters) — TODO: tune on hardware |
| `TILT_HOOK_THRESHOLD_RAD` | `Math.toRadians(10.0)` | Tilt magnitude that triggers winch |
| `CLIMB_PASS_DISTANCE_M` | `1.0` | Meters to drive past approach pose |
| `CLIMB_EXTEND_TIMEOUT_SEC` | `3.0` | Max seconds to wait for arm to reach MAX_HEIGHT |
| `CLIMB_APPROACH_MAX_SPEED_MPS` | `3.0` | Approach phase linear speed cap |
| `CLIMB_APPROACH_MAX_ACCEL_MPS2` | `2.5` | Approach phase linear acceleration cap |
| `CLIMB_PASS_MAX_SPEED_MPS` | `1.5` | Tower-pass linear speed cap (slower for reaction time) |
| `CLIMB_PASS_MAX_ACCEL_MPS2` | `1.5` | Tower-pass linear acceleration cap |

### Tuning `TILT_HOOK_THRESHOLD_RAD`

1. Deploy and open AdvantageScope.
2. Plot `Drive/TiltMagnitudeRad` while the robot is on a flat surface — it should read near zero.
3. Manually tilt the robot to simulate the arm hooking the bar. Note the peak value.
4. Set the threshold to roughly 60–70% of that peak so it fires reliably without false triggers
   from normal driving.
5. Run the full sequence on the field and verify the retract fires at the right moment.

Start with a **large** threshold (e.g., 20°) and tighten it; a threshold that is too small will
abort the climb prematurely during the approach or pass.

---

## Wiring

### Autonomous

Three PathPlanner named commands are registered in `RobotContainer` before `buildAutoChooser()`:

| Named command | Approach spot |
|---------------|---------------|
| `"SuperFastClimb/Left"` | `FieldSpot.CLIMB_LEFT` |
| `"SuperFastClimb/Center"` | `FieldSpot.CLIMB_CENTER` |
| `"SuperFastClimb/Right"` | `FieldSpot.CLIMB_RIGHT` |

Reference them by name in any PathPlanner `.auto` file:

```json
{
  "type": "named",
  "data": { "name": "SuperFastClimb/Center" }
}
```

### Teleop (button box)

Climb mode is managed by the button box. Bind the climb button with `whileTrue()`:

```java
buttonBoxButton.whileTrue(
    ClimbCommands.superFastClimb(drive, m_climbElevator, FieldSpot.CLIMB_CENTER));
```

**Restart behavior:** releasing the button cancels the command mid-sequence. Pressing it again
restarts from Phase 1 (approach). When cancelled, the elevator's default command (`Climb/Idle`)
immediately runs — the motor stops and brake mode holds the arm at its current height.

---

## Telemetry

All keys are visible in AdvantageScope.

| Key | Type | Updated |
|-----|------|---------|
| `Drive/Gyro/rollRobotRad` | double | Every loop (`@AutoLog`) |
| `Drive/Gyro/pitchRobotRad` | double | Every loop (`@AutoLog`) |
| `Drive/TiltMagnitudeRad` | double | Every loop (`@AutoLogOutput`) |
| `Climb/SuperFast/TiltHooked` | boolean | On command start (false) and on hook detection (true) |
| `Climb/SuperFast/TiltMagAtHookRad` | double | Only when tilt sentinel fires |
| `Climb/SuperFast/ApproachTarget` | Pose2d | When Phase 1 begins |
| `Climb/SuperFast/PassTarget` | Pose2d | When Phase 3 begins |

---

## Code Locations

| File | Purpose |
|------|---------|
| `src/main/java/frc/robot/commands/ClimbCommands.java` | `superFastClimb()` factory and all phase helpers |
| `src/main/java/frc/robot/commands/FieldSpot.java` | Approach poses, `FIELD_LENGTH_M`, `FIELD_WIDTH_M`, `flipIfRed()` |
| `src/main/java/frc/robot/subsystems/climb/ClimbElevator.java` | `getExtendCmd()`, `getRetractCmd()`, `getRobotClimbCmd()`, `getHeight()`; default command `Climb/Idle` |
| `src/main/java/frc/robot/subsystems/climb/ClimbElevatorConstants.java` | All climb command constants |
| `src/main/java/frc/robot/subsystems/drive/GyroIO.java` | `rollRobotRad`, `pitchRobotRad` in `GyroIOInputs` |
| `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java` | Pigeon2 roll/pitch StatusSignals at 50 Hz |
| `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java` | NavX roll/pitch population |
| `src/main/java/frc/robot/subsystems/drive/Drive.java` | `getTiltMagnitudeRad()` |
| `src/main/java/frc/robot/RobotContainer.java` | PathPlanner named command registration |
