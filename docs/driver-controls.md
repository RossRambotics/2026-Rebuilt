# Driver Controls

All driver controls are configured in `RobotContainer.configureButtonBindings()`.
Controller port: **0** (`CommandXboxController`)

For shooter mode details (OFF / IDLE / Predictive Targeting / Manual) and the implementation plan, see [`shooter-modes.md`](shooter-modes.md).

---

## Control Map

| Input | Action | Status |
|-------|--------|--------|
| Left Thumbstick | Translate (field-relative) | Implemented |
| Right Thumbstick (axis 4) | Rotate | Implemented |
| Left Trigger (hold) | Deploy intake + run intake roller | TODO |
| Left Bumper | Retract intake | TODO |
| Right Trigger (hold) | Auto shoot | TODO |
| Right Bumper (hold) | Manual shoot (kicker + spindexer only) | TODO |
| Y | Auto-drive to selected spot | Implemented |
| X | Lock wheels in X pattern | Implemented |
| Back | Reset gyro heading | Implemented |

---

## Detailed Behavior

### Left Thumbstick / Right Thumbstick — Swerve Drive
Standard field-relative swerve. Left stick translates, right stick (axis 4) rotates.
Default command on the `Drive` subsystem via `DriveCommands.joystickDrive()`.
SpeedNanny dynamically reduces max speed/acceleration while the shooter is active (`isShooting()`).

---

### Left Trigger (hold) — Deploy Intake + Run Roller
- **`whileTrue`**: Deploy the intake arm to the down position AND run the intake roller.
- **On release**: Intake roller stops. Intake arm remains deployed (retract is a separate button).

---

### Left Bumper — Retract Intake
- **`onTrue`**: Retract the intake arm to the stowed position.
- Intake roller is not affected (it should already be stopped).

---

### Right Trigger (hold) — Auto Shoot (Predictive Targeting mode)
Enters **PREDICTIVE TARGETING** mode. Runs while the trigger is held.

**`whileTrue`: `m_shooter.getAutoShootCmd()`**
1. Overrides hood to firing solution angle; overrides flywheel to firing solution RPM.
2. Turret was already tracking (predictive targeting is always running).
3. Once `isReady()` is true (turret + hood + flywheel + speed all within tolerance, 100 ms debounce), engages kicker and spindexer to feed note.

**On release / interrupt:**
1. Kicker → stopped.
2. Spindexer → stopped.
3. Hood default command resumes → retracts to `HOOD_DEGREES_MIN`.
4. Flywheel default command resumes → returns to idle RPM (2 000 default).

> **TODO:** Implement the new `getAutoShootCmd()` per the plan in [`shooter-modes.md`](shooter-modes.md#step-4--rewrite-getautoshootcmd-predictive-targeting-mode).

#### Auto Pass
Auto passing uses the **same command** as auto shoot.
The only difference is the target pose passed to `Shooter.setTargetPose()`. Target selection is driven by `SpotSelector` or a separate mode toggle — TBD.

---

### Right Bumper (hold) — Manual Shoot (Manual mode)
Enters **MANUAL** mode. Assumes the hood, turret, and flywheel are already at the correct positions/speeds (set by the operator via Elastic or a prior auto-aim sequence).

**`whileTrue`: `m_shooter.getManualShootCmd()`** — runs kicker + spindexer.
**On release**: Stop kicker and spindexer.

> No targeting or flywheel control — this is a raw fire trigger. See [`shooter-modes.md`](shooter-modes.md#manual).

> **TODO:** Implement `getManualShootCmd()` per [`shooter-modes.md`](shooter-modes.md#step-5--add-getmanualshootcmd-manual-mode).

---

### Y Button — Auto-Drive to Selected Spot
- **`whileTrue`**: `DriveToSpot` — PathPlanner on-the-fly pathfinding to the spot currently selected in `SpotSelector`.
- **On release / arrival**: Drive returns to normal joystick control.
- Select a spot before pressing Y (via button box or the SpotSelector Shuffleboard tab).

---

### X Button — Wheel Lock
- **`onTrue`**: `drive.stopWithX()` — locks all four swerve modules in the X pattern to resist pushing.

---

### Back Button — Reset Gyro Heading
- **`onTrue`** (works while disabled): Resets the robot's pose heading to face away from the driver station.
  - **Red alliance**: 0°
  - **Blue alliance**: 180°
- Translation is preserved; only the rotation component is updated.
