# Vision pipeline (Limelight / MegaTag)

This document explains how the robot software ingests, labels, filters, and forwards vision pose observations (MegaTag1 / MegaTag2) to the pose estimator.

## High-level flow

- Each camera has an IO adapter implementing `VisionIO` (examples: `VisionIOLimelight`, `VisionIOPhotonVision`, `VisionIOPhotonVisionSim`).
- `RobotContainer` constructs a `Vision` subsystem and passes:
  - a `VisionConsumer` (drive::addVisionMeasurement),
  - a pose supplier (drive::getPose),
  - a pose resetter (drive::setPose), and
  - one `VisionIO` instance per camera.
- Every scheduler cycle `Vision.periodic()` calls `io[i].updateInputs(inputs[i])` for each camera IO. Each adapter reads raw network/SDK outputs and fills a `VisionIO.VisionIOInputs` structure containing:
  - `latestTargetObservation` (tx/ty),
  - `poseObservations[]` (zero or more `PoseObservation` entries parsed from the camera), and
  - `tagIds[]` (which tags were visible).
- `Vision.periodic()` filters each `PoseObservation` (Z bounds, ambiguity, field bounds), computes per-observation standard deviations based on distance and tag count (and per-camera factors), and forwards accepted observations to the `VisionConsumer`.
- The `Drive` subsystem (via `addVisionMeasurement`) integrates accepted vision poses into the robot pose estimator.

## MegaTag1 vs MegaTag2

- `VisionIOLimelight` subscribes to two separate NetworkTables topics:
  - `botpose_wpiblue` → treated as `MEGATAG_1`
  - `botpose_orb_wpiblue` → treated as `MEGATAG_2`
- Each topic is read separately; the adapter creates `PoseObservation` records and sets the `PoseObservationType` accordingly.
- `Vision.periodic()` applies different stddev multipliers for `MEGATAG_2` (`linearStdDevMegatag2Factor` and `angularStdDevMegatag2Factor`) so MEGATAG_2 observations are weighted differently during fusion.
- The code does not command the Limelight to run only one solver—both topics can be published by the Limelight and the robot subscribes to both. The software distinguishes observation types by topic, not by telling the camera which to publish.

## Where MegaTag2 orientation comes from

- `VisionIOLimelight` publishes a 6-element `robot_orientation_set` array (first element is yaw) to give the Limelight an orientation hint used by the orb/orbital solver. This helps disambiguate pose solutions for MegaTag2.

## Filtering, logging, and reseed

- Observations are rejected if they fail simple checks: tagCount == 0, single-tag ambiguity too high, unrealistic Z, or pose is outside the field boundaries.
- Accepted observations are forwarded to the `VisionConsumer` with a 3×1 stddev vector `[sigma_x, sigma_y, sigma_theta]` computed from distance and tag count and adjusted by camera-specific multipliers.
- `Vision` tracks recent MEGATAG_1 yaw readings per camera for gyro yaw validation and maintains a "best" MEGATAG_1 pose (prefer more tags → fresher) for reseeding the robot heading.
- `RobotContainer` adds dashboard buttons to toggle VisionGyro mode and to `reseedFromVision()` (reseeds heading from the best recent high-quality MEGATAG_1 observation).

## Where vision measurements end up

- The consumer (usually `Drive.addVisionMeasurement`) receives:
  - `Pose2d` (robot pose from camera),
  - `timestamp` (seconds), and
  - `Matrix<N3, N1>` of measurement stddevs (x, y, theta).
- The drive pose estimator fuses those measurements with odometry.

## How to change which solver is used

1. Configure the Limelight itself (recommended): use the Limelight web UI to enable/disable orbit/orb solver, configure pipelines, etc.
2. Command the Limelight from robot code (optional): write NetworkTables keys such as `pipeline` to the camera table:
   ```java
   NetworkTableInstance.getDefault().getTable(<limelightName>).getEntry("pipeline").setNumber(index);
   ```
   The project does not currently set pipelines from code.
3. Filter in robot code: prefer or ignore topics in `VisionIOLimelight` or weight them in `Vision.periodic()`.

## Debugging & telemetry

- Existing logs to check:
  - `Vision/CameraX/RobotPosesAccepted` and `RobotPosesRejected` (per-camera)
  - `Vision/Summary/RobotPoses` and `Vision/Summary/RobotPosesAccepted`
  - `Vision/CameraX/Megatag1YawDeg` and `Vision/GyroMode`
- Recommended additions:
  - Publish which topic supplied the last accepted observation (e.g., `Vision/CameraX/LastTopic`), and timestamps for quick debugging.

## Suggested next steps

- Add explicit telemetry that shows whether the Limelight is publishing MegaTag1, MegaTag2, or both (helpful during tuning).
- Optionally add a Shuffleboard toggle and code to set the Limelight `pipeline` from robot (useful for runtime control).
- Consider a software policy in `Vision.periodic()` to prefer MEGATAG_1 for reseed and MEGATAG_2 while moving, if one proves more reliable in practice.

## Code locations

- `src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java` — Limelight IO adapter (subscribes to `botpose_wpiblue` and `botpose_orb_wpiblue`, publishes `robot_orientation_set`).
- `src/main/java/frc/robot/subsystems/vision/VisionIO.java` — IO data structures and `PoseObservationType` enum.
- `src/main/java/frc/robot/subsystems/vision/Vision.java` — filtering, stddev handling, forwarding to consumer, MegaTag1 yaw tracking and reseed logic.
- `src/main/java/frc/robot/RobotContainer.java` — wiring of `Vision` in robot init and dashboard buttons.
- `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` — tuning constants for stddevs, multipliers, and reseed thresholds.
