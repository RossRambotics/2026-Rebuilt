# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Deploy Commands

```bash
./gradlew build          # Build the project (includes Spotless formatting)
./gradlew deploy         # Deploy to RoboRIO
./gradlew test           # Run JUnit 5 tests
./gradlew replayWatch    # Watch AdvantageKit logs for replay
```

Code formatting is enforced automatically via Spotless (Google Java Format) and runs before compilation.

## Architecture

This is an FRC command-based robot using **AdvantageKit** for logging/replay. The robot extends `LoggedRobot` and operates in three modes controlled by `Constants.currentMode`:

- **REAL** — Live hardware, logs to USB (`/U/logs`) + NetworkTables
- **SIM** — Physics simulation, logs to NetworkTables
- **REPLAY** — Replays WPILOG files for offline analysis

### IO Abstraction Pattern

All hardware access uses an IO interface layer. Each subsystem defines a `*IO` interface with multiple implementations (real hardware, simulation, replay). `RobotContainer` selects the implementation based on the current mode. AdvantageKit's `@AutoLog` annotation generates input logging classes automatically.

Example: `ModuleIO` → `ModuleIOTalonFX` (real), `ModuleIOSim` (sim)

### Key Source Layout

```
src/main/java/frc/robot/
├── Robot.java / RobotContainer.java    # Lifecycle and wiring
├── Constants.java                      # Mode selection
├── Subsystems.java                     # Record bundle of all subsystems
├── generated/TunerConstants.java       # CTRE Tuner X swerve config (auto-generated, do not hand-edit)
├── commands/
│   ├── DriveCommands.java              # Drive command factories
│   ├── ClimbCommands.java              # Climb command factories (SuperFastClimb)
│   ├── OperatorControls.java           # Operator button bindings (student-extensible)
│   ├── AutoTargetSelector.java         # Auto-select shooting target by field zone
│   ├── DriveToSpot.java                # PathPlanner on-the-fly navigation to FieldSpot
│   ├── FieldSpot.java                  # Predefined field position enum
│   └── ShootingTarget.java             # Shooting target definitions with field poses
├── subsystems/
│   ├── drive/          # Swerve drivetrain (Drive, Module, GyroIO, ModuleIO, PhoenixOdometryThread)
│   ├── shooter/        # Turret + Hood + BottomFlyWheel + Kicker, coordinated by Shooter.java
│   ├── intake/         # IntakeRoller + IntakeDeployArm (YAMS), coordinated by Intake.java
│   ├── climb/          # ClimbElevator (YAMS linear mechanism)
│   ├── vision/         # Multi-camera AprilTag pose estimation (Limelight / PhotonVision)
│   ├── leds/           # LEDSubsystem (CANdle, 308 addressable LEDs)
│   ├── SpeedNanny.java         # Dynamic speed/accel limiter (reduces rates while shooting)
│   ├── Spindexer.java          # Feed mechanism (YAMS FlyWheel, TalonFX CAN 39)
│   ├── ShooterTargetSelector.java  # Default command: auto-selects target by field zone
│   └── SpotSelector.java           # Field spot selection for DriveToSpot
└── util/               # FuelSim, PhoenixUtil, LocalADStarAK, ShiftHelpers
```

### Subsystem Interactions

- **RobotContainer** instantiates all subsystems, sets default commands, and binds controller buttons. Driver controls live in `RobotContainer`; operator controls are in `OperatorControls`.
- **Drive** owns pose estimation, fusing wheel odometry (via `PhoenixOdometryThread` at 100–250 Hz) with vision updates from **Vision**.
- **Shooter** coordinates Turret, Hood, BottomFlyWheel, and Kicker — calculates firing solutions based on robot pose and target position, exposes readiness flags with 100ms debouncing.
- **Kicker** uses a **CANrange sensor (CAN 44)** for time-of-flight ball detection.
- **ShooterTargetSelector** is the default command on Shooter; auto-selects target based on robot field zone (overrideable by operator).
- **Intake** coordinates IntakeRoller (YAMS FlyWheel) and IntakeDeployArm (YAMS Pivot) as separate subsystems.
- **ClimbElevator** runs autonomously via `SuperFastClimb` (left/center/right variants) registered as PathPlanner NamedCommands.
- **LEDSubsystem** reflects shooter readiness, vision lock, and game piece status via 6 LED groups on a CANdle.
- **SpeedNanny** dynamically reduces drive speed/acceleration limits when the shooter reports `isShooting()`.
- **DriveCommands** applies SpeedNanny limits via slew rate limiters on joystick input.
- **FuelSim** provides physics-based ball simulation in SIM mode (2021 field geometry).

### Autonomous

PathPlanner handles path planning and trajectory following. Paths are stored in `src/main/deploy/pathplanner/`. Auto routines are selected via `LoggedDashboardChooser` in RobotContainer. SysID characterization routines (feedforward, wheel radius) are available as auto options. Named commands registered: `SuperFastClimb/Left`, `SuperFastClimb/Center`, `SuperFastClimb/Right`.

## Vendor Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| AdvantageKit | 26.0.0 | Logging, replay, `@AutoLog` codegen |
| Phoenix6 | 26.1.1 | CTRE TalonFX, CANcoder, Pigeon2, CANdle, CANrange |
| PathplannerLib | 2026.1.2 | Autonomous path planning/following |
| PhotonVision | v2026.2.2 | AprilTag camera processing (sim) |
| YAMS | 2026.2.23 | Motor mechanism modeling (FlyWheel, Pivot, Linear) |
| Studica | 2026.0.0 | Additional vendor hardware |

## Hardware Configuration

- **Drivetrain:** 4-module swerve with TalonFX (Kraken X60) drive/steer motors, CANcoders, Pigeon2 gyro (CAN 19)
- **Shooter:** Turret/CAN 40 (Kraken X60), Hood/CAN 24 (Kraken X44), BottomFlyWheel/CAN 35+36 (Kraken X60 ×2), Kicker/CAN 38 (Falcon500)
- **Kicker sensor:** CANrange time-of-flight sensor (CAN 44)
- **Intake:** IntakeDeployArm/CAN 41 (TalonFX), IntakeRoller/CAN 42 (TalonFX)
- **Climb:** ClimbElevator TalonFX (YAMS linear mechanism, 20:1 reduction)
- **Spindexer:** TalonFX Falcon500 (CAN 39)
- **LEDs:** CANdle (CAN 50) driving 308 addressable LEDs in 6 groups
- **Vision:** Dual Limelight cameras (`limelight-frontl` / `limelight-frontr`) with robot-to-camera transforms in `VisionConstants`; PhotonVision used in SIM mode
- **CAN Bus:** `"usb"` (all swerve, shooter, intake, sensors)

### CAN ID Reference

| CAN ID | Device | Subsystem |
|--------|--------|-----------|
| 1 | FrontRight Drive (TalonFX) | Swerve |
| 2 | FrontLeft Steer (TalonFX) | Swerve |
| 3 | FrontLeft Drive (TalonFX) | Swerve |
| 4 | FrontRight CANcoder | Swerve |
| 5 | FrontLeft CANcoder | Swerve |
| 6 | BackLeft CANcoder | Swerve |
| 7 | BackRight CANcoder | Swerve |
| 12 | BackLeft Steer (TalonFX) | Swerve |
| 13 | BackLeft Drive (TalonFX) | Swerve |
| 14 | BackRight Steer (TalonFX) | Swerve |
| 15 | BackRight Drive (TalonFX) | Swerve |
| 16 | FrontRight Steer (TalonFX) | Swerve |
| 19 | Pigeon2 gyro | Drive |
| 24 | Hood (TalonFX Kraken X44) | Shooter |
| 35 | BottomFlyWheel Left (TalonFX Kraken X60) | Shooter |
| 36 | BottomFlyWheel Right (TalonFX Kraken X60) | Shooter |
| 38 | Kicker (TalonFX Falcon500) | Shooter |
| 39 | Spindexer (TalonFX Falcon500) | Spindexer |
| 40 | Turret (TalonFX Kraken X60) | Shooter |
| 41 | IntakeDeployArm (TalonFX) | Intake |
| 42 | IntakeRoller (TalonFX) | Intake |
| 44 | CANrange (time-of-flight) | Shooter (Kicker) |
| 50 | CANdle (LED controller) | LEDs |

## Documentation

Key docs live in `docs/`. Keep them in sync when the corresponding code changes.

| File | Update when… |
|------|-------------|
| [`docs/DRIVER_OPERATOR_GUIDE.md`](docs/DRIVER_OPERATOR_GUIDE.md) | Controls change (button bindings, toggle behavior, LED patterns, auto routines) |
| [`docs/robot-subsystems.md`](docs/robot-subsystems.md) | A subsystem is added/removed, CAN IDs change, hardware is swapped, or a subsystem is stubbed/unstubbed |
| [`docs/driver-controls.md`](docs/driver-controls.md) | Driver controller bindings change in `RobotContainer` |
| [`docs/shooter-modes.md`](docs/shooter-modes.md) | Shooter state machine or targeting modes change |
| [`docs/led-subsystem.md`](docs/led-subsystem.md) | LED groups, colors, or triggers change |
| [`docs/super-fast-climb.md`](docs/super-fast-climb.md) | `ClimbCommands.superFastClimb()` sequence changes |
| [`docs/vision-pipeline.md`](docs/vision-pipeline.md) | Vision IO, camera names, or pose fusion strategy changes |

Other docs (`shakedown.md`, `shoot-while-moving-checklist.md`, `button-box-plan.md`, etc.) are process/checklist docs — update as needed but they don't track code directly.

## Conventions

- Subsystem constants live in subsystem-specific files (e.g., `HoodConstants`, `VisionConstants`, `IntakeConstants`, `ClimbElevatorConstants`) or `TunerConstants` for swerve.
- Thread safety: `PhoenixOdometryThread` runs at 100-250Hz; `Drive` uses `ReentrantLock` for odometry access.
- Telemetry uses `Logger.recordOutput()` throughout; avoid `System.out.println`.
- `TunerConstants.java` is generated by CTRE Tuner X — modify via the tool, not by hand.
- **YAMS pattern:** Subsystems using YAMS pass `isReal` (true for REAL/SIM, false for REPLAY) to select real vs. stubbed mechanisms. Use `IsStubbed` as the telemetry key (not `IsReal`) since "real" is ambiguous.
- **Unit suffixes on variable names:** Unitless numeric types (`double`, `DoubleEntry`, etc.) must include a unit suffix — e.g., `yawRad`, `speed_mps`, `angleDeg`. When using the WPILib Units library, use the `Measure` type instead (e.g., `Angle yawAngle`, `Pose2d robotPose2d`).
- **Frame of reference in variable names:** When a value is frame-dependent, include the frame before the unit/type suffix. Common frames are `Field` and `Robot`. Examples: `Angle yawField`, `double yawFieldRad`, `Angle yawRobot`, `double yawRobotRad`, `double speedFieldMps`.
