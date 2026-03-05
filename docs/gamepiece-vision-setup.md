# Game Piece Vision — Setup, Configuration & Tuning Guide

This guide walks through everything needed to bring up the `intake-cam` object-detection subsystem from scratch and tune `AutoDriveToFuel`. It assumes:

- The PhotonVision coprocessor is already mounted on the robot and powered.
- The object-detection pipeline is already trained and working when you test it directly in the PhotonVision web UI (you can see bounding boxes around fuel game pieces).
- The Limelight cameras have been configured per the shakedown checklist (see [`shakedown.md`](shakedown.md), section 0a.5).
- The robot radio is configured and the robot network is up.

---

## 1 — Network Overview

All devices on the robot use static IPs in the `10.32.01.X` range. **Never use DHCP** on competition hardware — if the radio restarts, DHCP addresses can change and the robot will lose contact with cameras mid-match.

| Device | Hostname | Static IP | Notes |
|--------|----------|-----------|-------|
| Robot radio | — | `10.32.01.1` | Gateway for all devices |
| RoboRIO | `roborio-3201-frc.local` | `10.32.01.2` | Set by the FRC imaging tool |
| Driver Station laptop | — | `10.32.01.5` | Set in Windows network settings |
| limelight-frontl | `limelight-frontl` | `10.32.01.11` | See shakedown.md §0a.5 |
| limelight-frontr | `limelight-frontr` | `10.32.01.12` | See shakedown.md §0a.5 |
| PhotonVision coprocessor | `photonvision.local` | `10.32.01.13` | **This guide** |

> **Why static IPs?** The robot code connects to each camera by name (NetworkTables hostname). If the IP changes, the connection breaks silently — the robot just stops seeing game pieces without any obvious error.

---

## 2 — PhotonVision Coprocessor: Static IP & Camera Name

### 2.1 Connect to the coprocessor

1. Power the robot on.
2. On your laptop, open a browser and go to:
   ```
   http://photonvision.local:5800
   ```
   If `photonvision.local` doesn't resolve, the coprocessor may still be on DHCP. Connect your laptop directly to the coprocessor via USB-ethernet or check your router's DHCP table to find its current IP, then browse to `http://<that IP>:5800`.

### 2.2 Set the static IP

1. In the PhotonVision web UI, click the **Settings** gear icon (top right).
2. Go to **Networking**.
3. Set **IP Assignment** to **Static**.
4. Set **IP Address** to `10.32.01.13`.
5. Set **Subnet Mask** to `255.255.255.0`.
6. Set **Gateway** to `10.32.01.1`.
7. Click **Save** and wait for the coprocessor to reboot (~20 seconds).
8. Browse to `http://10.32.01.13:5800` to confirm the UI loads at the new address.

- [ ] PhotonVision web UI reachable at `http://10.32.01.13:5800`

### 2.3 Verify the camera name

The robot code connects to the camera by the exact name `"intake-cam"`. It is set in:

```
src/main/java/frc/robot/subsystems/gamepiece/GamePieceVisionConstants.java
→ CAMERA_NAME = "intake-cam"
```

In the PhotonVision web UI:
1. Click on your camera in the left sidebar.
2. At the top of the camera panel, check the **Camera Nickname** field.
3. If it says anything other than `intake-cam`, click the pencil icon and rename it to `intake-cam` (case-sensitive, no spaces).
4. Click **Save**.

- [ ] Camera nickname is exactly `intake-cam`

### 2.4 Select the object-detection pipeline

1. In the camera panel, click the **Pipeline** dropdown.
2. Select your trained object-detection pipeline (it should already exist and be detecting game pieces).
3. Confirm bounding boxes appear around fuel game pieces in the camera stream.

- [ ] Object detection pipeline selected and showing bounding boxes

---

## 3 — Verify the Class ID

PhotonVision assigns each object class a numeric ID starting at 0. The robot code filters detections by class ID:

```
src/main/java/frc/robot/subsystems/gamepiece/GamePieceVisionConstants.java
→ FUEL_CLASS_ID = 0
```

To confirm the correct value:

1. In the PhotonVision web UI, look at the **Targets** panel while pointing the camera at a game piece.
2. Note the **Class ID** shown on the bounding box or in the target list.
3. If the class ID is **not 0**, update the constant:
   ```java
   public static final int FUEL_CLASS_ID = <your value here>;
   ```
4. Rebuild and redeploy: `./gradlew deploy`

- [ ] `FUEL_CLASS_ID` confirmed or updated to match pipeline output

---

## 4 — Measure and Set the Camera Mount Transform

The robot code needs to know exactly where the camera is on the robot and which direction it faces. This is the `ROBOT_TO_INTAKE_CAM` transform in `GamePieceVisionConstants.java`.

### 4.1 Coordinate system

All measurements are relative to the **robot center** (center of the robot frame at floor level):

| Axis | Positive direction |
|------|--------------------|
| X | Toward the front of the robot. **Negative X = toward the rear/intake.** |
| Y | Toward the left of the robot |
| Z | Upward |
| Roll | Rotation around X — tilting left/right |
| Pitch | Rotation around Y — **negative = tilted down toward floor** |
| Yaw | Rotation around Z — 0° = facing forward, **180° = facing backward** |

> The intake camera is at the **rear** of the robot, facing backward and tilted downward to see game pieces on the floor behind the robot. X will be a **negative** number and yaw will be near **180°**.

### 4.2 Measure off the robot

Use a tape measure and a protractor or angle gauge. Measure with the robot on a flat surface.

| Measurement | Value | Unit |
|-------------|-------|------|
| X (forward from robot center — **negative** for rear camera) | `______` | meters |
| Y (left from robot center) | `______` | meters |
| Z (height above floor) | `______` | meters |
| Roll | `0.0` (unless camera is canted sideways) | degrees |
| Pitch (tilt down) | `______` (negative number) | degrees |
| Yaw (pan left/right) | `______` (**~180** for a rear-facing camera) | degrees |

### 4.3 Update the constant

Open `GamePieceVisionConstants.java` and update `ROBOT_TO_INTAKE_CAM`:

```java
public static final Transform3d ROBOT_TO_INTAKE_CAM =
    new Transform3d(
        /* x */ -0.30,  // ← negative = behind robot center; replace with your measurement
        /* y */  0.0,   // ← replace with your Y measurement
        /* z */  0.25,  // ← replace with your Z measurement
        new Rotation3d(
            /* roll  */ 0.0,
            /* pitch */ Degrees.of(-30).in(Radians),   // ← replace -30 with your pitch
            /* yaw   */ Degrees.of(180).in(Radians)));  // ← 180 = rear-facing; adjust if off-center
```

Rebuild and redeploy after making changes: `./gradlew deploy`

- [ ] Camera mount measured off the robot
- [ ] `ROBOT_TO_INTAKE_CAM` updated with real measurements
- [ ] Code rebuilt and deployed

> **Why does this matter?** The transform is not used for steering (which uses raw camera yaw directly), but it will be needed in the future if we add field-position estimation for game pieces. Getting it right now saves rework later.

---

## 5 — First Boot Verification

Before tuning, confirm the subsystem is working end-to-end.

### 5.1 Check connectivity alert

1. Enable the robot in Teleop mode.
2. Open **AdvantageScope** and connect to the robot.
3. In the log tree, expand `GamePieceVision/`.
4. Check `GamePieceVision/connected` — it should be `true`.
   - If it is `false`: the camera name doesn't match. Double-check the PhotonVision nickname is exactly `intake-cam` (step 2.3).

- [ ] `GamePieceVision/connected = true`

### 5.2 Check target detection

1. Point the camera at a fuel game piece on the floor.
2. In AdvantageScope, watch these values:

| Key | Expected when piece is visible |
|-----|-------------------------------|
| `GamePieceVision/TargetCount` | ≥ 1 |
| `GamePieceVision/TotalAreaPercent` | > 0.5 |
| `GamePieceVision/SmoothedYawDeg` | Non-zero (±angle to the piece) |
| `GamePieceVision/IsReadyToChase` | `true` |
| `GamePieceVision/IsStale` | `false` |

If `TargetCount` stays at 0 even when the piece is in front of the camera:
- Check `FUEL_CLASS_ID` (step 3)
- Check `MIN_CONFIDENCE` — try lowering it to `0.3f` temporarily
- Check `MIN_TARGET_AREA_PERCENT` — try lowering it to `0.2` temporarily

- [ ] `GamePieceVision/TargetCount ≥ 1` when piece is in view
- [ ] `GamePieceVision/IsReadyToChase = true` when piece is in view

### 5.3 Check yaw direction and drive direction (sign test)

This is the most important hardware-specific test. **The robot will steer away from pieces or drive forward (away from the intake) if the signs are wrong.**

The intake-cam faces **backward**. Keep that in mind for all tests below.

**Test 1 — yaw sign:**
1. Stand behind the robot and place a game piece slightly to the **robot's left** (which is to the camera's **right** since it faces backward).
2. Watch `GamePieceVision/SmoothedYawDeg` in AdvantageScope.
3. It should show a **positive** number (piece is to camera's right).
4. Enable the robot, hold **B button**, and observe `AutoDriveToFuel/OmegaRadPerSec`:
   - Positive piece yaw → omega should be **positive** (CCW = robot turns **left**, swinging rear intake toward the piece).
   - If the robot turns **away**, flip the sign in `GamePieceVisionConstants.java`:
     ```java
     public static final double YAW_KP = -0.10;  // flip sign if robot steers wrong way
     ```

**Test 2 — drive direction:**
1. Hold **B button** with a piece directly behind the robot (centered in the camera).
2. The robot should drive **backward** (rear/intake side moves toward the piece).
3. If the robot drives **forward** (away from the piece), the `CHASE_SPEED_MPS` sign is wrong — this would be a code bug; report it to your mentor.

- [ ] Positive yaw = piece is to camera's right (robot's left) ✓
- [ ] Robot turns left (CCW) when piece is to the left ✓
- [ ] Robot drives backward (intake side toward piece) when B is held ✓

---

## 6 — Tuning

All tuning constants are in one place:

```
src/main/java/frc/robot/subsystems/gamepiece/GamePieceVisionConstants.java
```

### 6.1 `MIN_CONFIDENCE` and `MIN_TARGET_AREA_PERCENT` — detection filtering

These filter out false positives and distant/noisy detections.

| Constant | Default | Effect of raising | Effect of lowering |
|----------|---------|-------------------|--------------------|
| `MIN_CONFIDENCE` | `0.5f` | Fewer false detections | May pick up shadows/markings as game pieces |
| `MIN_TARGET_AREA_PERCENT` | `0.5` | Only detects pieces when robot is close | May chase pieces that are too far to intake |

**How to tune:**
1. Watch `GamePieceVision/TargetCount` during a match replay or driving test.
2. If you see detections on objects that are not game pieces, raise `MIN_CONFIDENCE`.
3. If the robot stops detecting pieces at a distance where it should still be driving toward them, lower `MIN_TARGET_AREA_PERCENT`.

### 6.2 `YAW_KP` — steering aggressiveness

Controls how hard the robot turns to align with the detected centroid.

| Value | Behavior |
|-------|----------|
| Too low (e.g. 0.03) | Robot barely corrects — drives past pieces to the side |
| Too high (e.g. 0.25) | Robot oscillates left-right while chasing |
| Default: 0.10 | 10° error → 1.0 rad/s correction |

**How to tune:**
1. Hold B button and drive toward a piece that is 20° off-center.
2. Watch `AutoDriveToFuel/OmegaRadPerSec` — it should be ~2 rad/s at 20° error with default KP.
3. If the robot overshoots and wobbles, reduce `YAW_KP`. If it barely corrects, increase it.

### 6.3 `YAW_EMA_ALPHA` — smoothing (anti-flicker)

Controls how quickly the smoothed yaw tracks the raw centroid. Lower = smoother but slower to respond.

| Value | Behavior |
|-------|----------|
| 0.10 | Very smooth — slow response to real target movement |
| 0.35 | Default — good balance |
| 0.70 | Fast response — may flicker if multiple pieces are visible |

**How to tune:** If the robot visibly oscillates side-to-side even when aimed at a steady cluster of pieces, lower `YAW_EMA_ALPHA` to 0.2 or 0.15.

### 6.4 `CHASE_SPEED_MPS` — approach speed

How fast the robot drives forward while chasing. Default is `1.5` m/s (~5 ft/s).

- Raise to 2.0–2.5 for faster collection if the intake can keep up.
- Lower to 1.0 if the robot overshoots and misses pieces.

### 6.5 `CHASE_TIMEOUT_SEC` — safety timeout

The command ends automatically after this many seconds even if the driver is still holding B. Default is `5.0` seconds. Increase if drivers need longer chase runs; lower if you want a tighter safety margin.

---

## 7 — Tuning Checklist

Complete these in order on a real game piece laid on the floor:

- [ ] Hold B button with piece directly in front — robot drives straight at it
- [ ] Hold B with piece 30° to the left — robot turns left and drives toward it
- [ ] Hold B with piece 30° to the right — robot turns right and drives toward it
- [ ] Hold B with no pieces visible — robot stops (does not drive blind)
- [ ] Release B at any time — robot stops and driver regains joystick control immediately
- [ ] Place 3 pieces in a cluster 1m away and 1 piece alone 2m away — robot steers toward the cluster (it has more combined area)
- [ ] Hold B for longer than `CHASE_TIMEOUT_SEC` — command ends automatically

---

## 8 — Code Reference

| File | What to change |
|------|---------------|
| `GamePieceVisionConstants.java` | All tuning: camera name, class ID, mount transform, KP, speed, timeout |
| `GamePieceVisionIOPhoton.java` | Camera hardware interface — edit only if changing PhotonVision API usage |
| `GamePieceVisionSubsystem.java` | Centroid computation and EMA logic |
| `AutoDriveToFuel.java` | Drive + intake command logic |

### AdvantageScope telemetry keys

| Key | Meaning |
|-----|---------|
| `GamePieceVision/connected` | Camera coprocessor connected |
| `GamePieceVision/TargetCount` | Number of accepted detections this cycle |
| `GamePieceVision/TotalAreaPercent` | Sum of all target bounding box areas |
| `GamePieceVision/SmoothedYawDeg` | EMA-filtered centroid yaw sent to command |
| `GamePieceVision/IsReadyToChase` | True when subsystem has a fresh valid target |
| `GamePieceVision/IsStale` | True when no new camera frames in 0.5s |
| `AutoDriveToFuel/State` | Chasing / TargetLost / Initialized / Interrupted |
| `AutoDriveToFuel/YawErrorDeg` | Current yaw error being corrected |
| `AutoDriveToFuel/ForwardSpeedMps` | Forward speed being commanded |
| `AutoDriveToFuel/OmegaRadPerSec` | Rotational correction being commanded |
| `AutoDriveToFuel/TimeoutElapsedSec` | How long the current chase has been running |
