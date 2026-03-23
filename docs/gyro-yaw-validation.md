# Gyro Yaw Validation & Vision Reseed

## Problem

The Pigeon2 gyro initializes to 0 degrees at boot, which is incorrect whenever the robot is placed at a non-zero heading. The gyro can also drift after jumping the ramp mid-match.

## Solution

Two operational modes with continuous yaw monitoring and a manual reseed command:

- **VisionGyro mode** — Active yaw monitoring via MegaTag 1. Use this when you need to validate or correct the gyro heading (pre-match placement, post-ramp recovery).
- **NormalGyro mode** — Standard operation. Gyro trusted for heading, MegaTag 2 for position. Yaw monitoring still runs for awareness but no correction is expected.

Three Elastic dashboard buttons control the system:
- **"Enable Vision Gyro"** — Switch to VisionGyro mode
- **"Enable Normal Gyro"** — Switch to NormalGyro mode
- **"Reseed Gyro From Vision"** — Manually reseed heading from best MegaTag 1 observation

All buttons work while the robot is disabled.

## How It Works

### Yaw Tracking (every cycle, both modes)

During each `Vision.periodic()` cycle, accepted MegaTag 1 observations have their yaw extracted and stored per camera. Only MegaTag 1 is used because it performs an independent 6DOF solve — MegaTag 2 is gyro-assisted and would create a circular dependency.

The Limelight produces both MegaTag 1 and MegaTag 2 data simultaneously — no pipeline switching is needed. Mode switching is purely robot-side logic controlling which data we act on.

### Yaw Comparison (every cycle, both modes)

- Gets the current gyro yaw from the pose estimator
- Checks each camera's latest MegaTag 1 yaw (must be fresh — less than 2 seconds old)
- Compares gyro vs each camera, and cameras vs each other, using angle wraparound-safe math
- Sets `IsGyroYawGood = false` if any pair disagrees by more than the tolerance
- If no camera data is fresh, trusts the gyro (defaults to `true`)

### Best Pose for Reseeding

A separate "best MegaTag 1 pose" is tracked with stricter quality thresholds:
- Minimum 2 tags visible (multi-tag solve is much more reliable for yaw)
- Average tag distance under 4 meters
- Prefers observations with more tags, then most recent

### Reseed Command

The "Reseed Gyro From Vision" button calls `vision.reseedFromVision()` which:
1. Checks for a valid, fresh (<5 seconds) high-quality MegaTag 1 observation
2. Logs the **before** yaw (`Vision/ReseedBeforeYawDeg`)
3. Resets the pose estimator heading to the MegaTag 1 yaw (keeps current XY position)
4. Logs the **after** yaw (`Vision/ReseedAfterYawDeg`)

## NetworkTables Keys

| Key | Type | Description |
|-----|------|-------------|
| `Vision/GyroMode` | string | Current mode: "VisionGyro" or "NormalGyro" |
| `Vision/GyroYawDeg` | double | Current pose estimator yaw in degrees |
| `Vision/Camera0/Megatag1YawDeg` | double | FR-limelight MegaTag 1 yaw in degrees |
| `Vision/Camera1/Megatag1YawDeg` | double | FL-limelight MegaTag 1 yaw in degrees |
| `Vision/IsGyroYawGood` | boolean | True when all fresh yaws agree within tolerance |
| `Vision/ReseedBeforeYawDeg` | double | Gyro yaw before last reseed (logged on reseed) |
| `Vision/ReseedAfterYawDeg` | double | Vision yaw applied by last reseed (logged on reseed) |

## Tunable Constants (VisionConstants.java)

| Constant | Default | Description |
|----------|---------|-------------|
| `gyroYawToleranceDeg` | 5.0 | Max allowed yaw disagreement in degrees |
| `reseedMinTagCount` | 2 | Minimum tags for reseed-quality pose |
| `reseedMaxAverageDistanceMeters` | 4.0 | Max average tag distance for reseed |

## Files Changed

- `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` — Yaw validation and reseed constants
- `src/main/java/frc/robot/subsystems/vision/Vision.java` — Mode state, yaw tracking, `reseedFromVision()`, `enableVisionGyro()`, `enableNormalGyro()`
- `src/main/java/frc/robot/RobotContainer.java` — Pass pose supplier/resetter to Vision, add 3 dashboard buttons

## Usage

### Pre-match
1. Place robot on field facing AprilTags
2. Press **"Enable Vision Gyro"** in Elastic
3. Wait for `Vision/IsGyroYawGood` — if `false`, the gyro disagrees with vision
4. Press **"Reseed Gyro From Vision"** to correct the heading
5. Verify `Vision/IsGyroYawGood` becomes `true`
6. Press **"Enable Normal Gyro"** before the match starts

### Mid-match (after ramp jump)
1. Monitor `Vision/IsGyroYawGood` on the dashboard
2. If it goes `false` after a ramp jump:
   - Press **"Enable Vision Gyro"**
   - When the robot can see 2+ tags, press **"Reseed Gyro From Vision"**
   - Press **"Enable Normal Gyro"** to return to standard operation
