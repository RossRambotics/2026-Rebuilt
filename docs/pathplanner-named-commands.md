# PathPlanner Named Commands

Named commands registered in `RobotContainer.java` for use as event markers in PathPlanner `.auto` files.

## Intake

| Command | Instant? | Description |
|---------|----------|-------------|
| `Intake.Deploy` | Yes | Bursts the arm to the deploy position, then coasts it and leaves the roller running at intake speed. Arm burst and roller spin-up are scheduled as background tasks so the auto path continues immediately. |
| `IntakeRoller.Start` | Yes | Starts the intake roller at full intake speed (4000 RPM). Does not move the arm. No-op if already running. |
| `IntakeRoller.Stop` | Yes | Stops the intake roller by cancelling the roller command. The roller's default command (off) resumes automatically. Does not move the arm. No-op if not running. |

## Shooter

| Command | Instant? | Description |
|---------|----------|-------------|
| `Shoot.Start` | Yes | Starts the AutoShoot sequence: enables predictive turret targeting, commands hood and flywheel to the firing-solution setpoints, and feeds the kicker/spindexer once the shooter reports ready. No-op if already running. |
| `Shoot.Stop` | Yes | Stops the AutoShoot sequence. Hood and flywheel return to idle defaults; kicker and spindexer stop. No-op if not running. |

## Notes

- All commands above are **instant** — they return immediately so PathPlanner can advance along the path without waiting. Background work (roller spinning, turret tracking, etc.) continues after the command finishes.
- `Shoot.Start` and `Shoot.Stop` share the same command instance as the driver's right trigger toggle. Starting shooting from auto and then cancelling via the driver trigger (or vice versa) works correctly.
- The SuperFastClimb named commands (`SuperFastClimb/Left`, `SuperFastClimb/Center`, `SuperFastClimb/Right`) are defined in `ClimbCommands.java` but are currently **commented out** in `RobotContainer.java` and not available.

## Trench Zone Warning — Shoot.Start / Shoot.Stop

The robot has an **automatic hood retract** that fires whenever the robot enters a trench zone (triggered by field position). This retract will interrupt `Shoot.Start`'s AutoShoot sequence because both require the Hood subsystem. The PathPlanner path itself continues unaffected, but the shoot sequence stops.

**Rules for any auto path that enters a trench zone:**

1. **Issue `Shoot.Stop` before the path segment that enters the trench.** Do not leave AutoShoot running while entering the trench.
2. **Do not issue `Shoot.Start` until the robot has fully exited the trench zone.** Issuing it while still in the zone will work momentarily, then be immediately interrupted by the auto-retract trigger.
3. The trench zones are approximately:
   - Blue alliance: X ≈ 3–6 m, Y ≈ 0–1.25 m (left) and Y ≈ 6.95–8.2 m (right)
   - Red alliance: X ≈ 10.5–13.5 m, Y ≈ 0–1.25 m (left) and Y ≈ 6.95–8.2 m (right)
