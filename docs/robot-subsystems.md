# Robot Subsystems Overview

This document summarizes all subsystems on the 2026 robot and provides a complete motor/device inventory.

---

## Subsystems

### Drive
A 4-module swerve drivetrain using CTRE Phoenix 6 (Kraken X60 motors). Owns pose estimation by fusing high-frequency wheel odometry (via `PhoenixOdometryThread` at 100–250 Hz) with vision updates from the Vision subsystem. Uses a `ReentrantLock` for thread-safe odometry access. PathPlanner handles autonomous trajectory following.

Key files: `Drive.java`, `Module.java`, `ModuleIOTalonFX.java`, `GyroIOPigeon2.java`, `PhoenixOdometryThread.java`

---

### Shooter
A coordinating subsystem (`Shooter.java`) that owns and orchestrates Turret, Hood, BottomFlyWheel, and Kicker. Computes firing solutions (turret yaw, hood pitch, flywheel RPM) using LUT-based interpolation with motion prediction, and exposes readiness flags with 100 ms debouncing.

#### Turret
Controls yaw rotation of the shooter head. Operates from −90° to +90° robot-relative. Supports auto-targeting (normal and predictive) and manual override modes.

> **Note:** The Turret motor (CAN 40) is currently force-stubbed in `Shooter.java` — it will not be commanded even on real hardware until this is resolved.

#### Hood
Controls the pitch angle of the shooter. Mechanical angle range is 15°–45° (0° = ball fires straight up). Uses an `ArmFeedforward` with a 19:1 gear reduction.

#### BottomFlyWheel
Dual-motor flywheel. CAN 35 (left) is the leader; CAN 36 (right) follows in the opposed direction. Contains LUT interpolation tables for RPM, hood angle, and time-of-flight vs. distance.

#### Kicker
A single-wheel feed roller that transfers game pieces from the Spindexer into the flywheel. Velocity-controlled via YAMS FlyWheel. Includes a CANrange proximity sensor (CAN 44) that detects balls transiting through the kicker. The falling edge of detection (ball leaving the sensor) timestamps TOF launches; the absence of detection for > 1 second during a shoot session signals `Shooter.isHopperEmpty()`. Note: the sensor can only see balls at the kicker — it has no view into the hopper.

Key files: `Shooter.java`, `Turret.java`, `Hood.java`, `BottomFlyWheel.java`, `Kicker.java`

---

### Spindexer
The intake/indexing mechanism. Ferries game pieces toward the Kicker. Single-motor, velocity-controlled via YAMS FlyWheel. Instantiated at the subsystems root level and owned by Shooter.

Key file: `Spindexer.java`

---

### Vision
Multi-camera AprilTag pose estimation using two Limelight cameras (`FR-limelight`, `FL-limelight`). Fuses pose estimates into the Drive subsystem's pose estimator using configurable standard deviation scaling. Supports MegaTag 2 (MT2) observations. Also supports PhotonVision and PhotonVision Sim IO implementations.

Key files: `Vision.java`, `VisionConstants.java`

---

### GamePieceVision
A dedicated object-detection camera subsystem using a PhotonVision coprocessor running a trained YOLO-style pipeline. Detects fuel game pieces on the floor behind the robot and computes an **area-weighted centroid yaw** across all accepted targets. Applies **Exponential Moving Average (EMA)** smoothing (α = 0.35) to prevent frame-to-frame flipping when multiple pieces are visible. Close pieces dominate by area (area ∝ 1/distance²), so the centroid naturally favors the nearest and densest grouping.

Used by `AutoDriveToFuel` to steer the rear intake toward detected game pieces while the driver holds B. The camera (`intake-cam`) is rear-facing, mounted at −0.30 m from robot center, 0.25 m above floor, pitched −30° down, yawed 180°.

- Camera name: `"intake-cam"`, static IP `10.32.01.13`
- All filtering thresholds and drive-tuning constants are in `GamePieceVisionConstants.java`
- See [`docs/gamepiece-vision-setup.md`](gamepiece-vision-setup.md) for setup, configuration, and tuning

Key files: `GamePieceVisionSubsystem.java`, `GamePieceVisionIO.java`, `GamePieceVisionIOPhoton.java`, `GamePieceVisionIOSim.java`, `GamePieceVisionConstants.java`; command: `AutoDriveToFuel.java`

---

### Intake
A coordinator subsystem (`Intake.java`) that owns and orchestrates IntakeRoller and IntakeDeployArm. Exposes compound commands (`getDeployCmd`, `getRetractCmd`) and standalone roller commands (`getRollerOnCmd`, `getRollerOffCmd`, `getReverseCmd`).

#### IntakeRoller
Spins game pieces into the robot. Velocity-controlled via YAMS FlyWheel (Kraken X60). CAN ID: **42**. Idle mode: COAST.

#### IntakeDeployArm
Deploys and retracts the intake mechanism. Position-controlled via YAMS Arm (Kraken X60). CAN ID: **41**. Idle mode: **COAST** — arm must fold freely if struck by another robot to prevent mechanical damage. Deploy uses a two-phase command: position-control until at angle, then releases to coast so the arm floats freely while the roller runs. Retract holds position with active control.

> **Constants:** Deploy angle, retract angle, gear reduction (60:1), and roller RPM are in `IntakeConstants.java` — marked TODO for hardware tuning.

Key files: `Intake.java`, `IntakeRoller.java`, `IntakeDeployArm.java`, `IntakeConstants.java`

---

### Climb Elevator
A single-motor elevator mechanism for end-game climbing. Position-controlled via YAMS `Elevator` with an `ElevatorFeedforward`. Idle mode: **BRAKE** — holds the robot's weight when hanging. Exposes `getExtendCmd()`, `getRetractCmd()`, and `getManualCmd(Supplier<Double>)` for operator control.

> **Constants:** Min/max heights, gear reduction, drum radius, and carriage mass are in `ClimbElevatorConstants.java` — marked TODO for hardware tuning.

Key files: `ClimbElevator.java`, `ClimbElevatorConstants.java`

---

### LEDSubsystem
A CTRE CANdle driving a GRB strip plus 8 onboard LEDs. Runs in all robot modes including disabled. Three active groups (indices 8–157): Group 1 shows combined vision status for both cameras (BLUE = both have targets, LIME = left only, ROSE = right only, RED = neither). Group 2 shows shooter readiness. Group 3 shows game piece presence (strobes during TeleOp shift transitions). Indices 158–307 are unused and held off. See [`docs/led-subsystem.md`](led-subsystem.md) for full details.

Key files: `LEDSubsystem.java`, `LEDConstants.java`

---

### ShooterTargetSelector
Tracks the currently selected `ShootingTarget` (Hub, Neutral Left/Right, Opposition Left/Right). Its default command, `AutoTargetSelector`, continuously picks the correct target based on the robot's field zone using flipIfRed logic — no operator input needed during normal play. Operator button box buttons call `cmdSelectManual()` to override; `cmdEnableAuto()` re-enables zone-based selection. Pushes the active target to `Shooter.setTargetPose()` whenever the selection changes.

Zone boundaries (Blue-alliance coords): alliance zone x < 4.6 m → Hub; neutral zone 4.6–11.9 m → Neutral Left/Right; opposition zone x > 11.9 m → Opposition Left/Right. Left/right split at field-center y (4.035 m).

Key files: `subsystems/ShooterTargetSelector.java`, `commands/AutoTargetSelector.java`

---

### SpeedNanny
A pure logic subsystem. Monitors `Shooter.isShooting()` and dynamically reduces max translational speed (50%), angular rate (10%), and acceleration limits while shooting. `DriveCommands` applies these limits via slew rate limiters.

Key file: `SpeedNanny.java`

---

## Motor & Device Inventory

### Drive

| CAN ID | Device Type | Role | Motor Model |
|--------|-------------|------|-------------|
| 1 | TalonFX | Front Right — Drive | Kraken X60 |
| 2 | TalonFX | Front Left — Steer | Kraken X60 |
| 3 | TalonFX | Front Left — Drive | Kraken X60 |
| 4 | CANcoder | Front Right — Encoder | — |
| 5 | CANcoder | Front Left — Encoder | — |
| 6 | CANcoder | Back Left — Encoder | — |
| 7 | CANcoder | Back Right — Encoder | — |
| 12 | TalonFX | Back Left — Steer | Kraken X60 |
| 13 | TalonFX | Back Left — Drive | Kraken X60 |
| 14 | TalonFX | Back Right — Steer | Kraken X60 |
| 15 | TalonFX | Back Right — Drive | Kraken X60 |
| 16 | TalonFX | Front Right — Steer | Kraken X60 |
| 19 | Pigeon 2 | Gyro / IMU | — |

### Shooter & Spindexer

| CAN ID | Device Type | Subsystem | Role | Motor Model |
|--------|-------------|-----------|------|-------------|
| 24 | TalonFX | Hood | Hood pitch (19:1 reduction) | Kraken X44 |
| 35 | TalonFX | BottomFlyWheel | Left flywheel — leader | Kraken X60 |
| 36 | TalonFX | BottomFlyWheel | Right flywheel — follower (opposed) | Kraken X60 |
| 38 | TalonFX | Kicker | Kicker feed roller | Falcon 500 |
| 39 | TalonFX | Spindexer | Intake/indexer roller | Falcon 500 |
| 40 | TalonFX | Turret | Turret yaw (12:1 reduction) ⚠️ force-stubbed | Kraken X60 |
| 44 | CANrange | Kicker | Ball proximity sensor (looks through polycarbonate) | — |

### Intake

| CAN ID | Device Type | Subsystem | Role | Motor Model |
|--------|-------------|-----------|------|-------------|
| 41 | TalonFX | IntakeDeployArm | Arm deploy/retract (60:1 reduction, COAST) | Kraken X60 |
| 42 | TalonFX | IntakeRoller | Game piece roller (COAST) | Kraken X60 |

### Climb Elevator

| CAN ID | Device Type | Subsystem | Role | Motor Model |
|--------|-------------|-----------|------|-------------|
| 43 | TalonFX | ClimbElevator | Elevator lift — reserved | Kraken X60 |

### LEDs

| CAN ID | Device Type | Subsystem | Role |
|--------|-------------|-----------|------|
| 50 | CANdle | LEDSubsystem | 300-LED GRB strip controller |

### Network Cameras & Coprocessors

These devices communicate over Ethernet (not CAN) and use static IPs in the `10.32.01.X` range.

| Device | Hostname | Static IP | Subsystem | Purpose |
|--------|----------|-----------|-----------|---------|
| Limelight (front left) | `limelight-frontl` | `10.32.01.11` | Vision | AprilTag pose estimation |
| Limelight (front right) | `limelight-frontr` | `10.32.01.12` | Vision | AprilTag pose estimation |
| PhotonVision coprocessor | `photonvision.local` | `10.32.01.13` | GamePieceVision | Game piece object detection (`intake-cam`) |

---

## All Devices by CAN ID

| CAN ID | Device | Subsystem |
|--------|--------|-----------|
| 1 | TalonFX — Front Right Drive | Drive |
| 2 | TalonFX — Front Left Steer | Drive |
| 3 | TalonFX — Front Left Drive | Drive |
| 4 | CANcoder — Front Right | Drive |
| 5 | CANcoder — Front Left | Drive |
| 6 | CANcoder — Back Left | Drive |
| 7 | CANcoder — Back Right | Drive |
| 12 | TalonFX — Back Left Steer | Drive |
| 13 | TalonFX — Back Left Drive | Drive |
| 14 | TalonFX — Back Right Steer | Drive |
| 15 | TalonFX — Back Right Drive | Drive |
| 16 | TalonFX — Front Right Steer | Drive |
| 19 | Pigeon 2 IMU | Drive |
| 24 | TalonFX — Hood Pitch | Shooter / Hood |
| 35 | TalonFX — Flywheel Left (leader) | Shooter / BottomFlyWheel |
| 36 | TalonFX — Flywheel Right (follower) | Shooter / BottomFlyWheel |
| 38 | TalonFX — Kicker | Shooter / Kicker |
| 39 | TalonFX — Spindexer | Spindexer |
| 40 | TalonFX — Turret Yaw ⚠️ | Shooter / Turret |
| 41 | TalonFX — Intake Deploy Arm | Intake / IntakeDeployArm |
| 42 | TalonFX — Intake Roller | Intake / IntakeRoller |
| 43 | TalonFX — Climb Elevator (reserved) | ClimbElevator |
| 44 | CANrange — Kicker ball sensor | Shooter / Kicker |
| 50 | CANdle — LED strip controller | LEDSubsystem |

**Total: 19 TalonFX motors, 4 CANcoders, 1 Pigeon 2, 1 CANdle, 1 CANrange**
**Network devices: 2 Limelights (`10.32.01.11`, `10.32.01.12`), 1 PhotonVision coprocessor (`10.32.01.13`)**
