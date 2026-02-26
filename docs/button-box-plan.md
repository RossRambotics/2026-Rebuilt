# Button Box Plan

This document is the living design spec and TODO list for the operator button box.
The button box is built on top of the **Elastic dashboard**, which consumes commands published
to NetworkTables via `SmartDashboard.putData()` or `Shuffleboard.getTab().add()`.

---

## How Elastic Consumes Commands

Commands published via `SmartDashboard.putData("Name", command)` appear as `CommandWidget`
buttons in Elastic. **Click to start; click again to cancel.** Toggle semantics — no hold-to-run.

Implications for command design:
- **Continuous commands** (spindexer run, arm hold) run indefinitely; Elastic's cancel-on-second-click is the intended stop mechanism.
- **Sequences** like SuperFastClimb run to natural completion or are cancelled by a second click. A second click restarts from Phase 1 on the next press.
- Commands available during **disabled** must be wrapped with `.ignoringDisable(true)`.

All button box commands are published from a dedicated `configureButtonBox()` method in
`OperatorControls` (`commands/OperatorControls.java`), called from `configureControls()`.

---

## Elastic Tab Layout

```
┌──────────┬──────────┬──────────────┬──────────────┬───────────────┐
│  Spots   │  Climb   │   Shooter    │    Intake    │  Calibration  │
└──────────┴──────────┴──────────────┴──────────────┴───────────────┘
```

---

## Tab: Spots — AutoDrive Spot Selection

Clicking a spot button selects the target; the **Y button** (driver controller) then drives there
via `DriveToSpot`. All spots including climb positions are available here.

| Button label | Command |
|---|---|
| Alliance Trench Left | `spotSelector.cmdSelectSpot(ALLIANCE_TRENCH_LEFT)` |
| Alliance Trench Right | `spotSelector.cmdSelectSpot(ALLIANCE_TRENCH_RIGHT)` |
| Opponent Trench Left | `spotSelector.cmdSelectSpot(OPPONENT_TRENCH_LEFT)` |
| Opponent Trench Right | `spotSelector.cmdSelectSpot(OPPONENT_TRENCH_RIGHT)` |
| Hub Alliance | `spotSelector.cmdSelectSpot(HUB_ALLIANCE)` |
| Hub Neutral | `spotSelector.cmdSelectSpot(HUB_NEUTRAL)` |
| Climb Left | `spotSelector.cmdSelectSpot(CLIMB_LEFT)` |
| Climb Center | `spotSelector.cmdSelectSpot(CLIMB_CENTER)` |
| Climb Right | `spotSelector.cmdSelectSpot(CLIMB_RIGHT)` |
| Clear Selection | `spotSelector.cmdClearSelection()` |

> All 10 are currently in Shuffleboard "SpotSelector". Move to `OperatorControls.configureButtonBox()`.

---

## Tab: Climb

### Automated Climb (SuperFastClimb)

Click to start; click again to cancel and restart from Phase 1 on next press.

| Button label | Command |
|---|---|
| Climb Left | `ClimbCommands.superFastClimb(drive, climb, CLIMB_LEFT)` |
| Climb Center | `ClimbCommands.superFastClimb(drive, climb, CLIMB_CENTER)` |
| Climb Right | `ClimbCommands.superFastClimb(drive, climb, CLIMB_RIGHT)` |

### Manual Arm Overrides

| Button label | Command | Notes |
|---|---|---|
| Extend Arm | `climbElevator.getExtendCmd()` | Holds at MAX_HEIGHT until cancelled |
| Retract Arm | `climbElevator.getRetractCmd()` | Holds at MIN_HEIGHT until cancelled |

---

## Tab: Shooter

### Targeting Mode

| Button label | Command |
|---|---|
| Normal Targeting | `shooter.getEnableNormalTargetingModeCmd()` |
| Predictive Targeting | `shooter.getEnablePredictiveTargetingModeCmd()` |

### Target Selection

| Button label | Command |
|---|---|
| Hub | `shooter.getSetTargetCmd(ShootingTarget.HUB)` |
| Neutral Left | `shooter.getSetTargetCmd(ShootingTarget.NEUTRAL_LEFT)` |
| Neutral Right | `shooter.getSetTargetCmd(ShootingTarget.NEUTRAL_RIGHT)` |
| Opposition Left | `shooter.getSetTargetCmd(ShootingTarget.OPPOSITION_LEFT)` |
| Opposition Right | `shooter.getSetTargetCmd(ShootingTarget.OPPOSITION_RIGHT)` |

All poses are Blue-alliance coordinates in `ShootingTarget`; `getFieldPose3d()` flips to Red at
runtime. Target preset buttons have been removed from Shuffleboard — publish via `OperatorControls.configureButtonBox()`.

### Shooter Actions

| Button label | Command | Notes |
|---|---|---|
| Auto Shoot | `shooter.getAutoShootCmd()` | Fires at intervals while shooter is ready |
| Retract Hood | `shooter.getRetractHoodCmd("ButtonBox")` | Safe travel position |

### Fixed Distance Shots (Pose-Independent)

When the robot cannot rely on pose estimation, these buttons command a shot using **fixed
constants** — no LUT, no pose estimation. Hood angle and RPM are tuned per-distance and stored
in `ShooterConstants`. Turret locks to 0° robot-relative.

| Button label | Hood constant | RPM constant |
|---|---|---|
| Shot — 1 m | `FIXED_SHOT_1M_HOOD_DEG` | `FIXED_SHOT_1M_RPM` |
| Shot — 2 m | `FIXED_SHOT_2M_HOOD_DEG` | `FIXED_SHOT_2M_RPM` |
| Shot — 3 m | `FIXED_SHOT_3M_HOOD_DEG` | `FIXED_SHOT_3M_RPM` |

All six constants are TODOs — set during practice by shooting from measured distances.

**`getFixedShotCmd(Angle hoodAngle, double rpm)` design:**
```
1. Lock turret to Rotation2d.kZero (robot frame)
2. Command hood to hoodAngle (fixed constant)
3. Command flywheel to rpm (fixed constant)
4. Hold indefinitely — Elastic cancel returns to normal auto mode
```
← **not yet implemented**

### Turret Manual Controls

Nudge the turret 5° left/right or snap to center. Useful when auto-targeting is off or during
fixed distance shots.

| Button label | Command |
|---|---|
| Turret ← 5° | `shooter.getTurretNudgeCmd(Degrees.of(-5))` |
| Turret Center | `shooter.getTurretCenterCmd()` |
| Turret → 5° | `shooter.getTurretNudgeCmd(Degrees.of(5))` |

`getTurretNudgeCmd(Angle offset)` — captures current physical angle, adds offset, clamps to
±90°, holds until cancelled. Presses accumulate.

`getTurretCenterCmd()` — holds turret at 0° robot-relative until cancelled.

### In-Match RPM Trim

Accumulates a persistent RPM offset on top of whatever the shooter is targeting (auto or fixed).

| Button label | Command |
|---|---|
| RPM +100 | `shooter.getRPMTrimCmd(+100)` |
| RPM −100 | `shooter.getRPMTrimCmd(-100)` |
| RPM Reset | `shooter.getRPMTrimResetCmd()` |

`m_rpmTrimOffsetRpm` lives in `Shooter` and is added inside every flywheel velocity calculation
(both auto-targeting and fixed shot). Logged as `Shooter/RPMTrimOffsetRpm`.

---

## Tab: Intake & Spindexer

### Intake

| Button label | Command | Notes |
|---|---|---|
| Deploy Intake | `intake.getDeployCmd()` | Arm out + roller on |
| Retract Intake | `intake.getRetractCmd()` | Arm in + roller off |
| Roller On | `intake.getRollerOnCmd()` | Roller only, arm stays |
| Roller Off | `intake.getRollerOffCmd()` | Roller off, arm stays |
| Eject | `intake.getReverseCmd()` | Reverse roller to clear jam |

### Spindexer

| Button label | Command | Notes |
|---|---|---|
| Run Spindexer | `spindexer.getRunCmd()` | Feed toward shooter |
| Stop Spindexer | `spindexer.getStopCmd()` | |
| Reverse Spindexer | `spindexer.getReverseCmd()` | Clear jams |

> `runSpindexer(double)` and `stop()` are currently void methods. Add `getRunCmd()`,
> `getStopCmd()`, `getReverseCmd()` command factories to `Spindexer.java`.

---

## Tab: Calibration

All positional subsystem calibrations and vision controls in one place.
Pre-match and pit use only.

All four positional subsystems (Turret, Hood, IntakeDeployArm, ClimbElevator) follow the
**same calibration pattern**:
- `homeAndCalibrate()` — drives to hard stop, detects stall, zeroes encoder. Requires enable.
- `calibrateInPlace()` — zeroes encoder at current position, no motion. Runs while disabled.
- `[Subsystem]/IsCalibrated` — boolean logged every loop, displayable as an Elastic indicator.

### Turret

| Button label | Command | Disabled? |
|---|---|:---:|
| Home & Calibrate | `m_turret.homeAndCalibrate()` (exposed via Shooter) | No |
| Calibrate In Place | `m_turret.calibrateInPlace().ignoringDisable(true)` | Yes |

**Status:** `Turret/IsCalibrated` · `Turret/IsStubbed`

### Hood

| Button label | Command | Disabled? |
|---|---|:---:|
| Home & Calibrate | `m_hood.homeAndCalibrate()` (exposed via Shooter) | No |
| Calibrate In Place | `m_hood.calibrateInPlace().ignoringDisable(true)` | Yes |
| Zero Hood (quick) | `m_hood.getZeroHoodAngleCmd().ignoringDisable(true)` | Yes |

**Status:** `Hood/IsCalibrated` · `Hood/IsStubbed`

### Intake Arm

| Button label | Command | Disabled? |
|---|---|:---:|
| Home & Calibrate | `intake.getDeployArm().homeAndCalibrate()` (or via Intake) | No |
| Calibrate In Place | `intake.getDeployArm().calibrateInPlace().ignoringDisable(true)` | Yes |

**Status:** `IntakeDeployArm/IsCalibrated` · `IntakeDeployArm/IsStubbed`

### Climb Elevator

| Button label | Command | Disabled? |
|---|---|:---:|
| Home & Calibrate | `climbElevator.homeAndCalibrate()` | No |
| Calibrate In Place | `climbElevator.calibrateInPlace().ignoringDisable(true)` | Yes |

**Status:** `ClimbElevator/IsCalibrated` · `ClimbElevator/IsStubbed`

### Vision / Gyro

| Button label | Command | Disabled? |
|---|---|:---:|
| Reset Gyro | pose-reset command `.ignoringDisable(true)` | Yes |
| Enable Vision Gyro | `Commands.runOnce(vision::enableVisionGyro).ignoringDisable(true)` | Yes |
| Enable Normal Gyro | `Commands.runOnce(vision::enableNormalGyro).ignoringDisable(true)` | Yes |
| Reseed from Vision | `Commands.runOnce(vision::reseedFromVision).ignoringDisable(true)` | Yes |

**Status:** `Drive/Gyro/connected` (already logged via `@AutoLog`)

> Vision controls are currently on `SmartDashboard`. Move to `OperatorControls.configureButtonBox()` under the
> Calibration tab.

---

## What Is Already Published — Consolidation Plan

| Currently published | Location | Action |
|---|---|---|
| SpotSelector (10 entries) | Shuffleboard "SpotSelector" | Move to `OperatorControls.configureButtonBox()` |
| Targeting mode buttons | Shuffleboard "Shooter" | Move to `OperatorControls.configureButtonBox()` |
| Target preset buttons | ~~Shuffleboard "Shooter"~~ removed | Add to `OperatorControls.configureButtonBox()` |
| Enable/Normal/Reseed gyro | `SmartDashboard.putData()` | Move to `OperatorControls.configureButtonBox()` Calibration tab |
| Hood zero cmd | Shuffleboard "Hood" | Move to `OperatorControls.configureButtonBox()` Calibration tab |
| Hood angle test buttons | Shuffleboard "Hood" | Keep — tuning use only |
| Flywheel RPM presets | Shuffleboard "BottomFlyWheel" | Keep — tuning use only |
| Kicker / Spindexer dynamic | Shuffleboard tabs | Keep — tuning use only |
| Hood SysID | Shuffleboard "YAMS_SysID" | Keep — tuning use only |

---

## Implementation TODO

### Infrastructure
- [x] Create `configureButtonBox()` in `OperatorControls`, called from `configureControls()`
- [x] Add `getRunCmd()`, `getStopCmd()`, `getReverseCmd()` to `Spindexer.java` (RPM-based)
- [x] Expose Turret and Hood calibration commands publicly via `Shooter` wrapper methods
- [x] Expose `IntakeDeployArm` calibration commands via `Intake` wrapper methods (`getDeployArmHomeAndCalibrateCmd()`, `getDeployArmCalibrateInPlaceCmd()`)

### Spots Tab
- [x] All 10 `SpotSelector` commands published in `OperatorControls.configureButtonBox()`

### Climb Tab
- [x] `ClimbCommands.superFastClimb()` ×3 published
- [x] `climbElevator.getExtendCmd()` and `getRetractCmd()` published as manual overrides

### Shooter Tab
- [x] Targeting mode buttons published
- [x] All 5 target selection buttons published via `shooter.getSetTargetCmd(ShootingTarget.*)`
- [x] `getAutoShootCmd()` and `getRetractHoodCmd("ButtonBox")` published
- [x] **Refactor shooting targets** — `ShootingTarget` enum with Blue-alliance Pose3d values,
      `flipIfRed3d()`, and `getSetTargetCmd(ShootingTarget)` — done; Shuffleboard entries removed
- [x] **Implement `Shooter.getFixedShotCmd(Angle hoodAngle, double rpm)`** — locks turret to 0°
      robot frame, sets fixed hood angle and RPM, holds until cancelled
- [x] Add 6 fixed shot constants to `Shooter` (hood deg + RPM for 1 m, 2 m, 3 m) — all TODO: tune
- [x] Publish fixed shot buttons (1 m, 2 m, 3 m) via `getFixedShot1mCmd/2mCmd/3mCmd()`
- [x] **Implement `Shooter.getTurretNudgeCmd(Angle offset)`** — captures current angle, adds offset,
      clamps to soft limits, holds indefinitely; presses accumulate
- [x] **Implement `Shooter.getTurretCenterCmd()`** — holds turret at 0° robot-relative
- [x] Publish turret nudge buttons (← 5°, Center, → 5°)
- [x] **Implement `Shooter.getRPMTrimCmd(double deltaRpm)`** and `getRPMTrimResetCmd()`
- [x] Wire `m_rpmTrimOffsetRpm` into flywheel velocity calculation (auto and fixed shot)
- [x] Publish RPM trim buttons (+100, −100, Reset)

### Intake & Spindexer Tab
- [x] All 5 intake command factories published
- [x] Spindexer command factories added and published (Run/Stop/Reverse via `Shooter` wrappers)

### Calibration Tab
- [x] Turret: `homeAndCalibrate()` + `calibrateInPlace()` published via `Shooter` wrappers
- [x] Hood: `homeAndCalibrate()` + `calibrateInPlace()` + `getZeroHoodAngleCmd()` published
- [x] Intake Arm: `homeAndCalibrate()` + `calibrateInPlace()` published via `intake.getDeployArm()`
- [x] Climb: `homeAndCalibrate()` + `calibrateInPlace()` published
- [x] Vision/gyro commands (Enable Vision Gyro, Enable Normal Gyro, Reseed, Reset Gyro) moved from `SmartDashboard` to Calibration tab
- [ ] Wire Elastic boolean indicators for all `[Subsystem]/IsCalibrated` NT keys
