# Shooter Modes

This document describes the four shooter operating modes, the state machine that governs them, and the implementation plan for bringing the code in line with this design.

See also: [`driver-controls.md`](driver-controls.md) for which controller buttons trigger each mode.

---

## Mode Overview

| Mode | Turret | Hood | Flywheel | Kicker | Spindexer |
|------|--------|------|----------|--------|-----------|
| **OFF** | Stopped (0 V) | Stopped (0 V) | Stopped (0 V) | Stopped | Stopped |
| **IDLE** | Predictive targeting | Retracted (`HOOD_DEGREES_MIN`) | Idle RPM (2 000 default) | Stopped | Stopped |
| **PREDICTIVE TARGETING** (Auto Shoot) | Predictive targeting | Firing solution angle | Firing solution RPM | Run when `isReady()` | Run when `isReady()` |
| **MANUAL** | Wherever it is | Wherever it is | Wherever it is | Running | Running |

---

## State Machine

```
             robot enable
                  │
                  ▼
            ┌──────────┐
            │   IDLE   │◄──────────────────────────────────────────┐
            │ (default)│                                           │
            └──────────┘                                           │
              │      │                                             │
  R-trigger   │      │  R-bumper (hold)                            │
  (hold)      │      │                      ┌────────────┐         │
              │      └──────────────────►   │   MANUAL   │─────────┘
              │                             └────────────┘   release
              │                                                R-bumper
              ▼
       ┌──────────────────┐
       │ PREDICTIVE       │
       │ TARGETING        │
       │  (Auto Shoot)    │─────── release R-trigger ──────────────►IDLE
       └──────────────────┘
```

OFF mode is entered via a dedicated dashboard/Elastic command (not driver-bound during a match).
All other transitions are driven by the right trigger and right bumper.

---

## Mode Details

### OFF
All shooter motors output zero volts. Used for pre-match power-up or emergencies.

- Not driver-controller-bound; triggered from Elastic / Shuffleboard.
- All commands requiring Turret, Hood, BottomFlyWheel, Kicker, or Spindexer are interrupted.

---

### IDLE (default)
Default behavior when no shooting command is active.

| Mechanism | Behavior |
|-----------|----------|
| Turret | Predictive targeting (continuously tracking target) |
| Hood | Held at `HoodConstants.HOOD_DEGREES_MIN` (retracted — safe for driving) |
| Flywheel | Held at `IDLE_FLYWHEEL_RPM` (default 2 000 RPM, pre-spun for fast response) |
| Kicker | Stopped |
| Spindexer | Stopped |

**Implementation:** The Hood and BottomFlyWheel **default commands** (set in `Shooter` constructor) implement IDLE behavior. The Turret default command is `getPredictiveTargetingCommand()` — same in IDLE and PREDICTIVE TARGETING.

---

### PREDICTIVE TARGETING (Auto Shoot)
Triggered by holding the **right trigger**. Overrides the hood and flywheel defaults with targeting-derived setpoints, then gates the kicker and spindexer on `isReady()`.

| Mechanism | Behavior |
|-----------|----------|
| Turret | Predictive targeting (same as IDLE — no change) |
| Hood | Firing solution angle (from `m_lastFiringSolution.hoodMechAngleRad`) |
| Flywheel | Firing solution RPM (from `m_lastFiringSolution.flywheelRPM + rpmTrim`) |
| Kicker | **Runs only when `isReady()` is true** (debounced) |
| Spindexer | **Runs only when `isReady()` is true** (debounced) |

**`isReady()` gate** — `m_isShooterReady` is true when ALL of:
- Turret within ±2° of target (100 ms debounce)
- Hood within ±2° of target (100 ms debounce)
- Flywheel within ±150 RPM of target (100 ms debounce)
- Robot translational speed under SpeedNanny limit

**On command end (trigger released or command interrupted):**
1. Kicker → stopped
2. Spindexer → stopped
3. Hood default command resumes → hood retracts to `HOOD_DEGREES_MIN`
4. Flywheel default command resumes → flywheel returns to `IDLE_FLYWHEEL_RPM`

**Auto Pass:** Uses the same command. The only difference is the active target in `ShooterTargetSelector`. When `AutoTargetSelector` is running, the neutral/opposition target is selected automatically from the robot's field zone. When auto is disabled (manual override), the operator selects the passing target via the button box before pulling the trigger.

---

### MANUAL
Triggered by holding the **right bumper**. Assumes the turret, hood, and flywheel are already at the correct setpoints (positioned by the operator via Elastic controls or a prior auto-aim sequence). Does not modify any targeting or flywheel commands.

| Mechanism | Behavior |
|-----------|----------|
| Turret | No change (whatever command is currently running) |
| Hood | No change |
| Flywheel | No change |
| Kicker | Running |
| Spindexer | Running |

**On command end (bumper released):**
1. Kicker → stopped
2. Spindexer → stopped

---

## Implementation Plan

### Step 1 — Add idle constants to `Shooter.java`

Add at the top with the other constants:

```java
// Default idle flywheel speed (pre-spun, ready for quick engagement)
public static final double IDLE_FLYWHEEL_RPM = 2000.0;
```

---

### Step 2 — Change hood default command to IDLE behavior

In `Shooter` constructor, change:
```java
m_hood.setDefaultCommand(getDynamicHoodCommand());
```
to:
```java
m_hood.setDefaultCommand(getIdleHoodCmd());
```

Add the new method:
```java
/** Default/idle command: holds hood at minimum angle (retracted, safe for driving). */
private Command getIdleHoodCmd() {
    return m_hood.setAngle(Degrees.of(HoodConstants.HOOD_DEGREES_MIN))
        .withName("Hood/Idle");
}
```

The existing `getDynamicHoodCommand()` becomes the targeting override used inside `getAutoShootCmd()`. Rename it `getTargetingHoodCmd()` or keep the existing name — just stop setting it as the default.

---

### Step 3 — Change flywheel default command to IDLE behavior

In `Shooter` constructor, change:
```java
m_bottomFlyWheel.setDefaultCommand(getDynamicFlywheelCommand());
```
to:
```java
m_bottomFlyWheel.setDefaultCommand(getIdleFlywheelCmd());
```

Add the new method:
```java
/** Default/idle command: holds flywheel at idle RPM (pre-spun). */
private Command getIdleFlywheelCmd() {
    return m_bottomFlyWheel.setVelocity(() -> RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm))
        .withName("Flywheel/Idle");
}
```

---

### Step 4 — Rewrite `getAutoShootCmd()` (PREDICTIVE TARGETING mode)

Replace the existing stub with:

```java
public Command getAutoShootCmd() {
    return Commands.parallel(
            // Override hood: targeting angle instead of retracted
            m_hood.setAngle(
                () -> m_lastFiringSolution != null
                    ? Radians.of(m_lastFiringSolution.hoodMechAngleRad)
                    : Degrees.of(HoodConstants.HOOD_DEGREES_MIN)),
            // Override flywheel: targeting RPM instead of idle
            m_bottomFlyWheel.setVelocity(
                () -> m_lastFiringSolution != null
                    ? RPM.of(m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm)
                    : RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm)),
            // Kicker + spindexer: gated on isReady()
            Commands.run(
                () -> {
                    if (m_isShooterReady) {
                        m_kicker.setVelocity(); // run at configured RPM
                        m_spindexer.setVelocity(); // run at configured RPM
                    } else {
                        m_kicker.set(0);
                        m_spindexer.set(0);
                    }
                },
                m_kicker, m_spindexer))
        .beforeStarting(() -> m_isShooting = true)
        .finallyDo(
            () -> {
                m_kicker.set(0);
                m_spindexer.set(0);
                m_isShooting = false;
            })
        .withName("AutoShoot");
}
```

> **Note on kicker/spindexer API:** `setVelocity()` and `set(0)` here should match how Kicker and Spindexer expose velocity control (check YAMS API). If a no-arg `setVelocity()` runs at a configured constant RPM that's fine; if not, add explicit RPM constants (`KICKER_RPM`, `SPINDEXER_RPM`) as Shooter-level constants.

---

### Step 5 — Add `getManualShootCmd()` (MANUAL mode)

```java
/**
 * Runs kicker and spindexer at full speed, assuming turret/hood/flywheel are
 * already at the correct setpoints. Use when the driver knows the shot is set up.
 */
public Command getManualShootCmd() {
    return Commands.run(
            () -> {
                m_kicker.setVelocity();
                m_spindexer.setVelocity();
            },
            m_kicker, m_spindexer)
        .finallyDo(
            () -> {
                m_kicker.set(0);
                m_spindexer.set(0);
            })
        .withName("ManualShoot");
}
```

---

### Step 6 — Add `getOffCmd()` (optional, Elastic/dashboard only)

```java
/** Stops all shooter motors. Interrupt with any other shooter command. */
public Command getOffCmd() {
    return Commands.parallel(
            m_turret.set(0),
            m_hood.set(0),
            m_bottomFlyWheel.set(0),
            Commands.run(() -> { m_kicker.set(0); m_spindexer.set(0); }, m_kicker, m_spindexer))
        .withName("Shooter/Off");
}
```

Register this on the Shooter Shuffleboard tab in the Shooter constructor.

---

### Step 7 — Update `RobotContainer.configureButtonBindings()`

| Binding | Command |
|---------|---------|
| `controller.leftTrigger()` `.whileTrue(...)` | `m_intake.getDeployCmd().alongWith(m_intake.getRollerOnCmd())` |
| `controller.leftBumper()` `.onTrue(...)` | `m_intake.getRetractCmd()` |
| `controller.rightTrigger()` `.whileTrue(...)` | `m_shooter.getAutoShootCmd()` |
| `controller.rightBumper()` `.whileTrue(...)` | `m_shooter.getManualShootCmd()` |
| `controller.y()` `.whileTrue(...)` | `new DriveToSpot(drive, m_spotSelector)` (already done) |

Remove the temporary `controller.b().whileTrue(m_shooter.getAutoShootCmd())` and `controller.a().whileTrue(...)` (launchFuelSim) bindings once the trigger bindings are wired.

---

## Open Questions / TODOs

- [ ] Verify kicker and spindexer `setVelocity()` / `set(0)` API — confirm the correct method signatures from the YAMS `FlyWheel` wrapper.
- [ ] Decide whether the right trigger threshold (e.g., `0.5`) is used to debounce the analog trigger before engaging auto shoot.
- [ ] Confirm turret force-stub is resolved before testing auto targeting (CAN 40 is currently force-stubbed in `Shooter.java`).
- [ ] Tune `IDLE_FLYWHEEL_RPM` (starting default: 2 000 RPM) — adjust based on how long it takes to spool up from idle to target RPM.
- [ ] Define `KICKER_RPM` and `SPINDEXER_RPM` constants (currently inferred from `getKickerRPMCmd` / `getSpindexerRPMCmd` defaults).
- [x] Target selection architecture — `ShooterTargetSelector` subsystem with `AutoTargetSelector` default command handles zone-based selection automatically; button box buttons override when needed.
