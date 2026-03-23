# Manual Shooter Controls

## Overview

The shooter normally aims automatically using a firing solution computed from the robot's pose and target position. The manual control commands let an operator override this and directly control:

- **Turret** rotation (yaw)
- **Hood** pitch angle
- **Flywheel** RPM

Each command takes a `Supplier<Double>` input (-1.0 to +1.0) and changes the setpoint at a constant rate per second. When the command ends, the subsystem's automatic control resumes.

## Command Factory Methods

All three methods live in `Shooter.java`:

| Method | What it controls | Rate | Clamp range |
|--------|-----------------|------|-------------|
| `getManualTurretCmd(Supplier<Double>)` | Turret yaw | 180 deg/s | -90 to +90 deg |
| `getManualHoodCmd(Supplier<Double>)` | Hood pitch | 90 deg/s | 15 to 45 deg |
| `getManualFlywheelCmd(Supplier<Double>)` | Flywheel speed | 1000 RPM/s | 0 to 6000 RPM |

### How they work

1. When the command starts, it captures the subsystem's **current** position/speed as the starting point.
2. Every 20ms, it reads the input supplier and adjusts: `value += input * rate * dt`.
3. The value is clamped to the safe range and sent to the subsystem's closed-loop controller (YAMS PID).
4. When the command ends, WPILib automatically resumes the default auto-targeting command.

### Input supplier

The `Supplier<Double>` should return a value from **-1.0 to +1.0**:

- **+1.0** = full speed in the positive direction (turret right, hood up, RPM increase)
- **0.0** = hold current position/speed
- **-1.0** = full speed in the negative direction (turret left, hood down, RPM decrease)

Joystick axes, buttons, and dashboard widgets all work as input sources.

## Example: Xbox Controller Binding

This example uses the left stick for turret/hood and the right stick for flywheel RPM. A bumper toggles manual mode on/off.

### In `OperatorControls.java` or `RobotContainer.java`

```java
// Manual turret: left stick X axis
// Push left = turret rotates left, push right = turret rotates right
Command manualTurret = shooter.getManualTurretCmd(() -> controller.getLeftX());

// Manual hood: left stick Y axis (inverted — pushing up should raise the hood)
// Xbox Y axis is negative when pushed forward, so negate it
Command manualHood = shooter.getManualHoodCmd(() -> -controller.getLeftY());

// Manual flywheel: right stick Y axis (inverted — pushing up should increase RPM)
Command manualFlywheel = shooter.getManualFlywheelCmd(() -> -controller.getRightY());

// Option A: Hold left bumper to enable all three manual controls at once
controller.leftBumper().whileTrue(
    manualTurret.alongWith(manualHood).alongWith(manualFlywheel)
);

// Option B: Bind each one separately to different buttons
controller.leftBumper().whileTrue(manualTurret);
controller.rightBumper().whileTrue(manualHood);
controller.leftTrigger(0.5).whileTrue(manualFlywheel);

// Option C: Toggle manual mode (press once to start, press again to stop)
controller.leftBumper().toggleOnTrue(
    manualTurret.alongWith(manualHood).alongWith(manualFlywheel)
);
```

### Notes on `alongWith`

When composing multiple manual commands with `alongWith`, each one independently requires its own subsystem. This means:

- The turret's auto-targeting is paused while `manualTurret` runs
- The hood's auto-aiming is paused while `manualHood` runs
- The flywheel's auto-speed is paused while `manualFlywheel` runs

They all start and stop together.

## Tuning the Rates

The rates are constants at the top of `Shooter.java`:

```java
private static final double MANUAL_TURRET_RATE_DEG_PER_SEC = 180.0;
private static final double MANUAL_HOOD_RATE_DEG_PER_SEC = 90.0;
private static final double MANUAL_FLYWHEEL_RATE_RPM_PER_SEC = 1000.0;
private static final double MANUAL_FLYWHEEL_MAX_RPM = 6000.0;
```

- If the turret feels too fast/slow, change `MANUAL_TURRET_RATE_DEG_PER_SEC`.
- At 180 deg/s, full stick sweeps the entire turret range (-90 to +90) in 1 second.
- At 90 deg/s, full stick sweeps the hood range (15 to 45) in about 0.33 seconds.
- At 1000 RPM/s, full stick goes from 0 to 3000 RPM in 3 seconds.

## Telemetry

While manual commands are running, the existing telemetry in `Shooter.updateTelemetry()` continues to log:

- `Shooter/TurretActualDegRobot` — current turret angle
- `Shooter/TurretTargetDegRobot` — target from the firing solution (not the manual override)
- `Shooter/HoodActualAngleDeg` — current hood angle
- `Shooter/HoodTargetAngleDeg` — firing solution angle + hood trim offset (clamped)
- `Shooter/HoodTrimOffsetDeg` — current hood trim accumulator value
- `Shooter/FlyWheelActualRPM` — current flywheel RPM (raw)
- `Shooter/FlyWheelAverageRPM` — EMA-smoothed flywheel RPM (~20-sample window at 50 Hz); use this for a stable dashboard reading
- `Shooter/FlyWheelTargetRPM` — firing solution RPM + RPM trim offset
- `Shooter/RPMTrimOffsetRpm` — current RPM trim accumulator value

The YAMS motor telemetry (`TurretMotor/*`, `HoodMotor/*`, `BottomFlyWheelMotor/*`) logs the actual setpoints being sent to the motors, which will reflect the manual values.
