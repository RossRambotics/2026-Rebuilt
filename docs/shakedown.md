# Robot Shakedown & Configuration Plan

**Robot:** FRC 2026 Rebuilt
**Priority Order:** Shooter → Drive → Intake → Climb
**Date Started:** 2026-02-25

---

## Phase 0a — System Imaging & Firmware Upgrades (Do This Before Anything Else)

Complete all imaging and firmware upgrades **before** connecting to the robot in code or running any tests. The order below matters — image the RoboRIO first so it can host the firmware update process for downstream devices.

---

### 0a.1 — Image the RoboRIO

**Tool:** [FRC roboRIO Imaging Tool](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html) (installed with WPILib)

1. Connect RoboRIO to laptop via USB-B cable
2. Open **FRC roboRIO Imaging Tool** from the WPILib start menu
3. Wait for the tool to detect the RoboRIO
4. Select the latest FRC image (2026.x.x)
5. Click **Format Target** — this wipes and reimages the RoboRIO
6. Wait for the process to complete (~5–10 min)
7. Confirm the tool reports **Success**
8. Set team number to **`3201`** in the imaging tool

- [ ] RoboRIO imaged to latest 2026 image
- [ ] Team number set

---

### 0a.2 — Configure & Update the Radio

**Radio:** Vivid-Hosting **VH-109**
**Tool:** [FRC Radio Configuration Utility](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html) (v24+ required for VH-109 support)

1. Power the VH-109 via the robot (or bench supply — 12V, 2A min)
2. Connect laptop to the radio's **18V POE port** via ethernet (the port labeled for configuration)
3. Open **FRC Radio Configuration Utility**
4. Select radio model: **VH-109**
5. Enter team number: **3201**
6. Click **Configure** — utility flashes firmware and sets team SSID / WPA2 key
7. Wait for completion — all radio LEDs go solid green
8. Disconnect config cable; reconnect laptop wirelessly to verify team SSID appears

> The VH-109 hosts a web UI at `http://10.32.01.1` after configuration — use it to verify settings and check link status.

- [ ] Radio firmware updated via FRC Radio Configuration Utility
- [ ] Team number 3201 / SSID configured
- [ ] Radio accessible from driver station (green DS comms light)
- [ ] Web UI reachable at `http://10.32.01.1`

---

### 0a.3 — Upgrade Phoenix 6 Firmware on All CTRE Devices

**Tool:** [Phoenix Tuner X](https://apps.microsoft.com/detail/9NVV4PWDW27Z) — connect via USB-B to RoboRIO or over WiFi after radio config

All TalonFX, CANcoder, and Pigeon2 devices must be on the latest **Phoenix 6** firmware to match the Phoenix6 vendordep in this project.

**Connection:** USB-B to RoboRIO → Tuner X detects devices on both CAN buses automatically.

> **This robot has two CAN buses:**
> - **CANivore** — Drivetrain (drive motors, steer motors, CANcoders) and Pigeon2 gyro
> - **RoboRIO native** — All other mechanisms (shooter, intake, climb, spindexer)

First, update the **CANivore firmware** itself before updating downstream devices:

1. In Tuner X, go to **CANivore** tab
2. Select the CANivore device
3. Click **Update Firmware** — bring it to the latest version
4. Confirm it appears healthy with the correct name/ID

- [ ] CANivore firmware updated

Then update all CAN devices:

1. Open Phoenix Tuner X
2. Connect to robot (USB or WiFi)
3. Go to **Devices** tab — all CAN devices on both buses should be listed
4. Click **Update All** or update each device individually

**CANivore bus — Drivetrain:**

| CAN ID | Device | Target Firmware | Updated |
|--------|--------|----------------|---------|
| 1 | Front Right Drive (TalonFX) | Latest Phoenix 6 | [ ] |
| 2 | Front Right Steer (TalonFX) | Latest Phoenix 6 | [ ] |
| 3 | Front Left Drive (TalonFX) | Latest Phoenix 6 | [ ] |
| 4 | Front Right CANcoder | Latest Phoenix 6 | [ ] |
| 5 | Front Left CANcoder | Latest Phoenix 6 | [ ] |
| 6 | Back Left CANcoder | Latest Phoenix 6 | [ ] |
| 7 | Back Right CANcoder | Latest Phoenix 6 | [ ] |
| 12 | Back Left Steer (TalonFX) | Latest Phoenix 6 | [ ] |
| 13 | Back Left Drive (TalonFX) | Latest Phoenix 6 | [ ] |
| 14 | Back Right Steer (TalonFX) | Latest Phoenix 6 | [ ] |
| 15 | Back Right Drive (TalonFX) | Latest Phoenix 6 | [ ] |
| 16 | Front Left Steer (TalonFX) | Latest Phoenix 6 | [ ] |
| 19 | Pigeon2 Gyro | Latest Phoenix 6 | [ ] |

**RoboRIO native bus — Mechanisms:**

| CAN ID | Device | Target Firmware | Updated |
|--------|--------|----------------|---------|
| 24 | Hood (TalonFX) | Latest Phoenix 6 | [ ] |
| 35 | Flywheel Left (TalonFX) | Latest Phoenix 6 | [ ] |
| 36 | Flywheel Right (TalonFX) | Latest Phoenix 6 | [ ] |
| 38 | Kicker (TalonFX) | Latest Phoenix 6 | [ ] |
| 39 | Spindexer (TalonFX) | Latest Phoenix 6 | [ ] |
| 40 | Turret (TalonFX) | Latest Phoenix 6 | [ ] |
| 41 | Intake Arm (TalonFX) | Latest Phoenix 6 | [ ] |
| 42 | Intake Roller (TalonFX) | Latest Phoenix 6 | [ ] |
| 43 | Climb Elevator (TalonFX) | Latest Phoenix 6 | [ ] |

> Note: Falcon 500s (IDs 38, 39) use Phoenix 6 firmware when running with the Phoenix6 API — confirm they appear as `TalonFX` in Tuner X, not as legacy `Talon FX (Phoenix 5)`.

- [ ] CANivore firmware updated
- [ ] All 22 CTRE devices updated to latest Phoenix 6 firmware
- [ ] No devices showing firmware mismatch warnings in Tuner X

---

### 0a.4 — Activate Phoenix Pro License

Phoenix Pro unlocks FOC (Field Oriented Control) on Kraken X60 and Kraken X44 motors — required for the performance gains the code is tuned for. The project uses `TorqueCurrentFOC` and `VelocityVoltage` FOC control modes.

**License type needed:** Phoenix Pro (per-robot seat, annual)

**Activation steps:**

1. In Phoenix Tuner X, go to **Licensing** tab
2. Sign in with the CTR Electronics account holding the Phoenix Pro license
3. Confirm the license is assigned to this robot's serial number (RoboRIO serial)
4. Click **Activate License on Robot**
5. Verify: each TalonFX in the Devices list shows **Pro Licensed** status

**Devices that benefit from Phoenix Pro (FOC):**

| CAN ID | Device | FOC Mode Used |
|--------|--------|---------------|
| 1, 3, 13, 15 | Drive motors (Kraken X60) | VelocityTorqueCurrentFOC |
| 2, 12, 14, 16 | Steer motors | PositionVoltage / MotionMagicVoltage |
| 24 | Hood (Kraken X44) | MotionMagicVoltage |
| 35, 36 | Flywheel (Kraken X60) | VelocityVoltage FOC |
| 40 | Turret (Kraken X60) | MotionMagicVoltage |
| 41 | Intake Arm (Kraken X60) | MotionMagicVoltage |
| 43 | Climb Elevator (Kraken X60) | MotionMagicVoltage |

> Falcons (IDs 38, 39) do **not** support Phoenix Pro FOC — they operate in non-FOC voltage mode regardless.

- [ ] Phoenix Pro license activated on robot
- [ ] All Kraken motors show **Pro** badge in Tuner X devices list
- [ ] No `LicenseCheck` faults on any TalonFX

---

### 0a.5 — Limelight Firmware & Network Configuration

Each Limelight is configured via its own web UI. Connect a laptop directly to each camera (or via the robot network after radio config).

**Default Limelight IP:** `http://limelight.local:5801` (before custom hostname is set)

> The code connects to these exact hostnames — they **must** match or the robot will log "Vision camera disconnected" alerts:
> - Camera 0: `limelight-frontl`
> - Camera 1: `limelight-frontr`

#### Before You Start — Physical Labels

Apply a piece of tape or a printed label to each Limelight **before** mounting them on the robot, so they can be identified by hand without powering up:

| Camera | Label to apply |
|--------|---------------|
| limelight-frontl | `LL-FRONTL  10.32.01.11` |
| limelight-frontr | `LL-FRONTR  10.32.01.12` |

- [ ] limelight-frontl labeled on housing
- [ ] limelight-frontr labeled on housing

---

#### Limelight 0 — `limelight-frontl` (Front Left)

1. Connect to Limelight via USB or direct ethernet
2. Open web UI at `http://limelight.local:5801` (or current IP)
3. **Firmware:** Go to **Settings → Update** — update to latest Limelight firmware
4. **Networking:**
   - Set hostname to `limelight-frontl`
   - Set team number to `3201`
   - Set IP assignment to **Static**: `10.32.01.11`
   - Subnet: `255.255.255.0`, Gateway: `10.32.01.1`
5. **Camera Mounting Pose (robot-to-camera transform):**
   > The code does **not** supply the mounting transform to the Limelight — it must be set here in the web UI. Measure carefully off the robot frame.

   Go to **Settings → Robot Offset** (or **Camera Pose** tab depending on firmware version) and enter the physical mounting position and orientation of this camera relative to the robot center:

   | Field | Value | Notes |
   |-------|-------|-------|
   | Forward (X, m) | `______` | + = toward robot front |
   | Side (Y, m) | `______` | + = toward robot left |
   | Up (Z, m) | `______` | + = upward |
   | Roll (deg) | `______` | rotation about X axis |
   | Pitch (deg) | `______` | rotation about Y axis (tilt up/down) |
   | Yaw (deg) | `______` | rotation about Z axis (pan left/right) |

   > Tip: Use a tape measure and protractor/angle gauge on the robot. Positive yaw = camera faces left of robot center.

6. **Pipeline:** Create/select an **AprilTag** pipeline
   - Set to correct AprilTag family (2026 game tag family — confirm from game manual)
   - Tune camera exposure and gain for field lighting conditions
   - Confirm tag detections appear in the crosshair view
7. **3D Mode:** Enable MegaTag2 pose estimation (requires Limelight 3 or newer)
   - Upload the 2026 field AprilTag map JSON to the Limelight

- [ ] Firmware updated
- [ ] Hostname set to `limelight-frontl`
- [ ] Team number set to `3201`
- [ ] Static IP set to `10.32.01.11`
- [ ] Camera position entered in web UI (X, Y, Z)
- [ ] Camera orientation entered in web UI (roll, pitch, yaw)
- [ ] AprilTag pipeline configured and active
- [ ] Field map uploaded
- [ ] Tags visible and tracked in web UI crosshair view

**Record:**
- Firmware version: `______`
- Pipeline index used: `______`
- Static IP: `10.32.01.11`

#### Limelight 1 — `limelight-frontr` (Front Right)

Repeat all steps above for the second camera. This camera has a **different mounting position and yaw** — measure independently.

1. Firmware update
2. Networking: hostname `limelight-frontr`, team number `3201`, static IP `10.32.01.12`, subnet `255.255.255.0`, gateway `10.32.01.1`
3. **Camera Mounting Pose:**

   | Field | Value | Notes |
   |-------|-------|-------|
   | Forward (X, m) | `______` | |
   | Side (Y, m) | `______` | |
   | Up (Z, m) | `______` | |
   | Roll (deg) | `______` | |
   | Pitch (deg) | `______` | |
   | Yaw (deg) | `______` | |

4. Same AprilTag pipeline (same pipeline index as limelight-frontl)
5. Upload same 2026 field map

- [ ] Firmware updated
- [ ] Hostname set to `limelight-frontr`
- [ ] Team number set to `3201`
- [ ] Static IP set to `10.32.01.12`
- [ ] Camera position entered in web UI (X, Y, Z)
- [ ] Camera orientation entered in web UI (roll, pitch, yaw)
- [ ] AprilTag pipeline configured and active (same pipeline index)
- [ ] Field map uploaded
- [ ] Tags visible and tracked in web UI crosshair view

**Record:**
- Firmware version: `______`
- Pipeline index used: `______`
- Static IP: `10.32.01.12`

#### Network Reachability Check

After configuring both cameras and connecting to the robot radio:

```
ping 10.32.01.11   # limelight-frontl
ping 10.32.01.12   # limelight-frontr
```

- [ ] `10.32.01.11` (limelight-frontl) pingable from driver station laptop
- [ ] `10.32.01.12` (limelight-frontr) pingable from driver station laptop
- [ ] Both web UIs reachable: `http://10.32.01.11:5801` and `http://10.32.01.12:5801`

---

### 0a.6 — Post-Upgrade Sanity Check

After all imaging, firmware upgrades, and camera configuration:

- [ ] Deploy robot code: `./gradlew deploy`
- [ ] Driver station shows robot enabled (no comms errors)
- [ ] AdvantageKit log starts recording (check AdvantageScope connection)
- [ ] Phoenix Tuner X shows all 22 devices healthy, no sticky faults
- [ ] No firmware version mismatch errors in Rio console
- [ ] No "Vision camera disconnected" alerts in Driver Station or AdvantageScope
- [ ] Both Limelights streaming live in their web UIs

---

## Phase 0 — CAN Bus Verification (Do This Before Testing)

Open **Phoenix Tuner X** and connect to the robot. Every device in the table below must appear, respond to blink/identify, and show no faults before proceeding to any subsystem testing.

> **All devices are on the RoboRIO native CAN bus. No CANivore is configured.**

### Complete CAN Device Inventory

| CAN ID | Device | Type | Subsystem | Verified |
|--------|--------|------|-----------|----------|
| 1 | Front Right — Drive | TalonFX (Kraken X60) | Drive | [ ] |
| 2 | Front Right — Steer | TalonFX | Drive | [ ] |
| 3 | Front Left — Drive | TalonFX (Kraken X60) | Drive | [ ] |
| 4 | Front Right — CANcoder | CANcoder | Drive | [ ] |
| 5 | Front Left — CANcoder | CANcoder | Drive | [ ] |
| 6 | Back Left — CANcoder | CANcoder | Drive | [ ] |
| 7 | Back Right — CANcoder | CANcoder | Drive | [ ] |
| 12 | Back Left — Steer | TalonFX | Drive | [ ] |
| 13 | Back Left — Drive | TalonFX (Kraken X60) | Drive | [ ] |
| 14 | Back Right — Steer | TalonFX | Drive | [ ] |
| 15 | Back Right — Drive | TalonFX (Kraken X60) | Drive | [ ] |
| 16 | Front Left — Steer | TalonFX | Drive | [ ] |
| 19 | Pigeon2 Gyro | Pigeon2 | Drive | [ ] |
| 24 | Hood | TalonFX (Kraken X44) | Shooter | [ ] |
| 35 | Flywheel Left (leader) | TalonFX (Kraken X60) | Shooter | [ ] |
| 36 | Flywheel Right (follower, opposed) | TalonFX (Kraken X60) | Shooter | [ ] |
| 38 | Kicker | TalonFX (Falcon 500) | Shooter | [ ] |
| 39 | Spindexer | TalonFX (Falcon 500) | Feed | [ ] |
| 40 | Turret | TalonFX (Kraken X60) | Shooter | [ ] |
| 41 | Intake Deploy Arm | TalonFX (Kraken X60) | Intake | [ ] |
| 42 | Intake Roller | TalonFX (Kraken X60) | Intake | [ ] |
| 43 | Climb Elevator | TalonFX (Kraken X60) | Climb | [ ] |

**Total: 22 devices** (12 swerve drive/steer, 4 CANcoders, 1 Pigeon2, 5 shooter, 1 feed, 2 intake, 1 climb)

### CAN Bus Health Checks

- [ ] No duplicate CAN IDs (Tuner X shows no conflicts)
- [ ] All devices at firmware version ≥ latest recommended (update if needed)
- [ ] CAN bus utilization < 70% at idle (check Tuner X diagnostics)
- [ ] No devices showing sticky faults

### CANcoder Offset Verification

With robot on stands and all modules physically aligned to forward (wheels pointing straight ahead):

| Module | CANcoder ID | Raw Absolute (deg) | Offset in TunerConstants (deg) | Verified |
|--------|-------------|---------------------|-------------------------------|----------|
| Front Right | 4 | | | [ ] |
| Front Left | 5 | | | [ ] |
| Back Left | 6 | | | [ ] |
| Back Right | 7 | | | [ ] |

If any raw absolute reading doesn't match the TunerConstants offset → realign module mechanically or update offset in `TunerConstants.java` via Tuner X.

---

## Pre-Power-On Checklist

- [ ] All 22 CAN IDs verified in table above
- [ ] No CAN conflicts or missing devices
- [ ] All CANcoder offsets recorded
- [ ] Battery fully charged (> 12.4V)
- [ ] Robot on stands, wheels clear of ground
- [ ] Driver station connected, FMS light green
- [ ] AdvantageScope / Elastic open and logging
- [ ] SmartDashboard / Shuffleboard open with calibration tab visible

---

## Phase 1 — Shooter Subsystem

### 1.1 Electrical / Mechanical Verification

- [ ] Confirm all shooter motor CAN IDs respond in Phoenix Tuner X
- [ ] Confirm both flywheel motors (35, 36) spin freely by hand (no binding)
- [ ] Confirm turret turns freely through full ±90° range by hand — **no obstructions**
- [ ] Confirm hood moves freely through 15°–45° range by hand
- [ ] Confirm kicker spins freely by hand
- [ ] Confirm spindexer feeds freely by hand
- [ ] Check all limit switch / hard-stop positions match mechanical reality

### 1.2 Turret Calibration

> **Goal:** Zero the turret encoder at the mechanical center (robot forward).

1. Enable robot in **Teleop** mode
2. Open SmartDashboard → Shooter calibration tab
3. Manually rotate turret to the mechanical hard stop (one end)
4. Press **Turret — Home & Calibrate** button
   - Robot drives toward hard stop at −0.1 duty cycle
   - Watch telemetry: `Shooter/Turret/VelocityRPS` drops below 0.5 RPS and current spikes
   - 100ms debounce fires → encoder zeros → YAMS syncs to MIN position
5. Command turret to 0° via **Turret Center** button — confirm it holds center
6. Command turret to +45° and −45° — confirm smooth, symmetric motion
7. If encoder zeroed in wrong place: use **Turret — Calibrate In Place** to zero at current position, then adjust `HoodConstants` / turret offset constants accordingly

**Record:**
- Turret home offset needed (rotations): `______`
- Turret at center reads (deg): `______`

### 1.3 Hood Calibration

> **Goal:** Zero the hood encoder at the MIN angle (15° mechanical).

1. Enable robot in **Teleop**
2. Manually move hood to lowest position (resting on hard stop)
3. Press **Hood — Home & Calibrate**
   - Motor drives toward hard stop
   - Stall detect fires → encoder zeros → YAMS syncs to MIN
4. Press **Hood — Zero** to command to MIN angle — confirm it holds
5. Command hood to 45° — confirm smooth travel to MAX
6. Command hood back to 15° — confirm no oscillation

**Record:**
- Hood at 15° reads (deg): `______`
- Hood at 45° reads (deg): `______`
- Any oscillation / tune kP needed: `______`

### 1.4 Flywheel Smoke Test

1. From SmartDashboard manual flywheel tab, ramp flywheel to **1000 RPM**
2. Confirm both motors spin (watch `Shooter/Flywheel/ActualRPM`)
3. Ramp to **3000 RPM** — confirm stable, no vibration
4. Ramp to **5000 RPM** — confirm stable
5. Cut to 0 — confirm both motors coast to stop together (follower in sync)

**Record:**
- RPM steady-state error at 3000: `______`
- Any mechanical vibration noted: `______`

### 1.5 Kicker & Spindexer Smoke Test

1. Run **Spindexer → Run** from SmartDashboard — confirm forward feed at 500 RPM
2. Run **Spindexer → Reverse** — confirm reverse
3. Run **Kicker** from manual tab at low duty — confirm rotation direction matches feed path
4. Feed a game piece through manually and verify path: intake → spindexer → kicker → flywheel

### 1.6 Shooter SysID — Hood

> Run YAMS SysId on the hood to determine kS, kV, kG (gravity), kA and PID gains.

1. Go to **Shuffleboard → YAMS_SysID** tab
2. Select **Hood** mechanism
3. Clear the field, robot on stands
4. Run **Quasistatic Forward** — slowly ramps voltage up while recording velocity
5. Run **Quasistatic Reverse**
6. Run **Dynamic Forward** — step voltage input
7. Run **Dynamic Reverse**
8. Export log to AdvantageScope / SysID analyzer tool
9. Extract `kS`, `kV`, `kG`, `kA`
10. Update `HoodConstants` with new feedforward values
11. Tune PID (kP) for position response — command 15° → 45° → 15°, minimize overshoot

**Record Hood FF/PID:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kG | |
| kA | |
| kP | |
| kD | |

### 1.7 Shooter SysID — Flywheel

1. In SmartDashboard, run **Flywheel FF Characterization** auto (if registered) or use manual SysId routine from YAMS tab
2. Run quasistatic and dynamic routines in both directions
3. Extract `kS`, `kV`, `kA` for flywheel
4. Update `BottomFlyWheel` constants
5. Verify RPM steady-state error < 50 RPM at target speeds

**Record Flywheel FF:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kA | |

### 1.8 Shooter SysID — Turret

1. Select **Turret** in YAMS_SysID tab
2. Run Quasistatic Forward / Reverse (limit to safe angular range)
3. Run Dynamic Forward / Reverse
4. Extract and update turret FF/PID constants
5. Command turret +45°, −45°, 0° in sequence — confirm tracking < 1° steady-state error

**Record Turret FF/PID:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kA | |
| kP | |
| kD | |

### 1.9 Fixed Shot Tuning

> Tune the three preset fixed-distance shooting positions. Run these after characterization.

**Procedure for each distance:**

1. Place robot at measured distance (1m / 2m / 3m from target)
2. Enable **Fixed Shot Xm** command from SmartDashboard
3. Confirm hood angles to correct position and flywheel spins up
4. Enable **Auto Shoot** — fire several game pieces
5. Observe trajectory — adjust hood angle and/or RPM until shots are consistent
6. Update constants in `Shooter.java`:
   ```java
   // Fixed shot constants to tune:
   // 1m:  hood deg, RPM
   // 2m:  hood deg, RPM
   // 3m:  hood deg, RPM
   ```

**Record Fixed Shots:**

| Distance | Hood Angle (deg) | RPM | Notes |
|----------|------------------|-----|-------|
| 1m | | | |
| 2m | | | |
| 3m | | | |

### 1.10 Auto-Targeting Verification

1. Place robot at various distances (1m–5m) from AprilTag target
2. Enable **Normal Targeting Mode**
3. Confirm turret tracks AprilTag heading (watch `Shooter/Turret/TargetAngleDeg`)
4. Confirm hood angle adjusts based on distance LUT
5. Confirm flywheel RPM adjusts based on distance LUT
6. Verify all three readiness flags go true simultaneously before shoot
7. Test **Predictive Targeting Mode** — robot moving while targeting

### 1.11 Shooter Readiness Tuning

Adjust readiness tolerances in `Shooter.java` if needed:

| Parameter | Default | Tuned Value |
|-----------|---------|-------------|
| Turret tolerance (deg) | 2.0 | |
| Hood tolerance (deg) | 2.0 | |
| Flywheel tolerance (RPM) | 150 | |
| Debounce time (ms) | 100 | |

---

## Phase 2 — Drive Subsystem

### 2.1 Swerve Module Verification

- [ ] All 4 drive motors respond to input in Phoenix Tuner X
- [ ] All 4 steer motors respond to input
- [ ] All 4 CANcoders reading absolute position
- [ ] Physically align all modules to 0° (wheels pointing forward)
- [ ] Verify CANcoder offsets in `TunerConstants.java` match physical alignment

### 2.2 Gyro Verification

1. Connect Pigeon2 — confirm CAN presence in Phoenix Tuner X
2. Enable robot — check `Drive/Gyro/YawRad` telemetry at 0°
3. Rotate robot 90° CW — confirm telemetry reads ~−90° (or +90° depending on convention)
4. Press **Reset Gyro** (Back button) — confirm resets to 0

### 2.3 Basic Drive Test

1. Enable in Teleop on stands — push left stick forward, confirm all 4 wheels spin forward
2. Push left stick sideways — confirm all 4 wheels rotate correctly for strafe
3. Push right stick — confirm rotation
4. Observe: no module fighting, no oscillation
5. Place robot on ground — drive slow figure-8
6. Confirm odometry tracks visually (check Elastic field visualization)

### 2.4 Drive — Wheel Radius Characterization

> Measures actual wheel radius to fix odometry scale error.

1. Clear open floor space (~3m radius)
2. From auto chooser, select **Drive Wheel Radius Characterization**
3. Enable auto — robot spins in place slowly for several rotations
4. When done, record logged wheel radius from telemetry / AdvantageScope
5. Update `TunerConstants` (wheel radius) if different from current value

**Record:**
- Measured wheel radius (m): `______`
- Current constant (m): `______`

### 2.5 Drive — Simple FF Characterization

1. Clear straight path (~6m)
2. Select **Drive Simple FF Characterization** from auto chooser
3. Run — robot ramps voltage slowly in a straight line
4. Record `kS` and `kV` from log output
5. Update drive FF constants in `TunerConstants` / drive configuration

**Record Drive FF:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |

### 2.6 Drive — Full SysId

1. Select **Drive SysId (Quasistatic Forward)** — run
2. Select **Drive SysId (Quasistatic Reverse)** — run
3. Select **Drive SysId (Dynamic Forward)** — run
4. Select **Drive SysId (Dynamic Reverse)** — run
5. Open logs in SysId analyzer
6. Extract `kS`, `kV`, `kA` and update module drive PID/FF
7. Re-run Basic Drive Test to verify improved tracking

**Record Drive SysId:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kA | |
| kP (drive) | |
| kP (steer) | |

### 2.7 Odometry & Vision Fusion

1. Place robot at a known field position
2. Verify pose estimate matches known position (< 5cm error)
3. Enable camera — verify vision measurements appear in AdvantageScope (`Drive/VisionMeasurements`)
4. Drive robot around field — confirm pose stays consistent
5. Test **Reseed from Vision** button — pose snaps to vision estimate
6. Test **Enable Vision Gyro** — heading follows vision instead of IMU

### 2.8 SpeedNanny Verification

1. Enable **Auto Shoot** or shooter armed state
2. Drive — confirm linear speed and angular rate are reduced while shooter reports `isShooting()`
3. Verify slew rate limits applied correctly (no jerky motion)

### 2.9 PathPlanner Verification

1. Create a simple 2-point test path in PathPlanner
2. Load as auto routine
3. Enable auto — robot should follow path smoothly
4. Verify final position within 5cm of target
5. Test **Drive to Spot** (Y button) for on-the-fly pathfinding

---

## Phase 3 — Intake Subsystem

### 3.1 Electrical / Mechanical Verification

- [ ] Arm motor (TalonFX) responds in Phoenix Tuner X
- [ ] Roller motor (TalonFX) responds in Phoenix Tuner X
- [ ] Arm moves freely through full range by hand
- [ ] Roller spins freely by hand
- [ ] No binding through deploy → retract range

### 3.2 Arm Calibration

1. Manually move arm to the retracted hard stop
2. Press **Intake Arm — Home & Calibrate** from SmartDashboard
   - Motor drives toward hard stop
   - Stall detect → encoder zeros → YAMS syncs to MIN
3. Command **Deploy** — arm should travel to 0° (deployed)
4. Command **Retract** — arm should travel to 90° (retracted)
5. Confirm tolerance ±5° per `IntakeConstants.AT_ANGLE_TOLERANCE_DEG`

**Record:**
- Deploy angle actual (deg): `______`
- Retract angle actual (deg): `______`
- Any overshoot / tune needed: `______`

### 3.3 Intake SysID — Deploy Arm

1. Go to YAMS_SysID tab, select **IntakeDeployArm**
2. Run Quasistatic Forward / Reverse
3. Run Dynamic Forward / Reverse
4. Extract kS, kV, kG, kA
5. Update `IntakeConstants` with characterization results
6. Tune kP for smooth deploy / retract without overshoot

**Record Arm FF/PID:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kG | |
| kA | |
| kP | |

### 3.4 Roller Tuning

1. Run **Roller On** — confirm motor spins at `ROLLER_INTAKE_RPM` (2000 RPM)
2. Hold a game piece against roller — confirm consistent intake force
3. Run **Eject / Reverse** — confirm reversal at `ROLLER_REVERSE_RPM` (−1000 RPM)
4. Tune intake/reverse RPM until reliable pick-up and eject

**Record:**
- Intake RPM (tuned): `______`
- Reverse RPM (tuned): `______`

### 3.5 Full Intake Cycle Test

1. Place game piece on floor in front of robot
2. Press **Deploy** command
3. Drive forward — confirm intake picks up game piece
4. Press **Retract** — arm retracts, roller stops
5. Confirm game piece feeds into spindexer path
6. Run spindexer — confirm game piece reaches kicker

---

## Phase 4 — Climb Subsystem

### 4.1 Electrical / Mechanical Verification

- [ ] Climb motor (TalonFX, CAN ID 43) responds in Phoenix Tuner X
- [ ] Elevator moves freely by hand through full range
- [ ] Drum radius and carriage mass measurements taken (update `ClimbElevatorConstants`)
- [ ] Max height physically measured on robot — update `MAX_HEIGHT_M`

**Record Physical Measurements:**
- Drum radius (in): `______`
- Carriage mass (lbs): `______`
- Max height (m): `______`
- Gear reduction: `______`

### 4.2 Climb Elevator Calibration

1. Lower elevator manually to bottom hard stop
2. Press **Climb — Home & Calibrate** from SmartDashboard
   - Motor drives down at duty cycle
   - Stall detect fires → encoder zeros → YAMS syncs to MIN (0.0 m)
3. Command **Extend** — confirm elevator travels to MAX height
4. Command **Retract** — confirm returns to 0

### 4.3 Climb SysID

1. Go to YAMS_SysID tab, select **ClimbElevator**
2. Run Quasistatic Forward (extend direction)
3. Run Quasistatic Reverse (retract direction)
4. Run Dynamic Forward / Reverse
5. Extract kS, kV, kG (carriage weight), kA
6. Update `ClimbElevatorConstants`
7. Tune kP for smooth extend/retract

**Record Climb FF/PID:**

| Gain | Value |
|------|-------|
| kS | |
| kV | |
| kG | |
| kA | |
| kP | |

### 4.4 Manual Climb Test

1. Use **Manual Climb** joystick control (SmartDashboard or operator control)
2. Verify elevator moves smoothly up and down
3. Verify soft limits activate at MIN and MAX height (no crashing)
4. Verify BRAKE mode holds elevator in place when released

### 4.5 Tilt Detection Verification

1. With robot on flat ground, confirm `Shooter/Gyro` roll+pitch reads ~0°
2. Tilt robot manually — confirm `ClimbElevator/TiltDetected` goes true at ~10° threshold
3. Adjust `TILT_HOOK_THRESHOLD_RAD` in `ClimbElevatorConstants` if needed

**Record:**
- Tilt threshold tuned (deg): `______`

### 4.6 SuperFastClimb Sequence Test

> Run after all above pass. Do initial test at low speed in open space.

1. Place robot at approach position (~3m from climb tower)
2. Press **SuperFastClimb (Center)** from SmartDashboard
3. Observe sequence:
   - [ ] Pathfind to approach spot
   - [ ] Elevator extends to MAX (< 3s timeout)
   - [ ] Robot drives forward 1m at pass speed (1.5 m/s)
   - [ ] Tilt detection triggers (or drive completes)
   - [ ] Drive stops
   - [ ] Winch if tilted, hold if not
4. Adjust `CLIMB_APPROACH_MAX_SPEED_MPS`, `CLIMB_PASS_DISTANCE_M` as needed
5. Test Left and Right climb spots similarly

---

## Phase 5 — Integration & End-to-End Tests

### 5.1 Full Teleop Cycle

- [ ] Drive to game piece → intake → feed to shooter → shoot
- [ ] Drive using AprilTag targeting — confirm turret auto-aims during drive
- [ ] Fixed-distance shot from 1m, 2m, 3m
- [ ] RPM trim works (+/− 100 per press)
- [ ] Turret nudge works (±5°)
- [ ] SpeedNanny reduces speed while shooter is armed

### 5.2 Button Box Verification

Review all SmartDashboard / operator control bindings — confirm each button triggers correct command:

- [ ] A — Auto Shoot
- [ ] B — Auto Shoot (backup binding)
- [ ] X — X-pattern wheel lock
- [ ] Y — Drive to spot / Hood retract
- [ ] Back — Reset gyro
- [ ] RB — Bump traversal (45° lock)
- [ ] All calibration buttons (turret, hood, arm, climb)
- [ ] Vision gyro enable/disable

### 5.3 Autonomous Verification

- [ ] PathPlanner path loads correctly in auto chooser
- [ ] Simple 2m forward path executes accurately
- [ ] Named commands registered (SuperFastClimb Left/Center/Right)
- [ ] Multi-step auto: drive to shoot position → aim → fire
- [ ] SysId options appear in auto chooser and run cleanly

### 5.4 Vision End-to-End

- [ ] Both Limelights connected and streaming
- [ ] AprilTag IDs detected correctly (correct tag map loaded)
- [ ] Robot pose within 10cm of actual after vision fusion
- [ ] Turret uses vision-derived robot pose for firing solution

---

## Tuning Log

Use this section to record tuned values as you go. After each session, update constants in code and record here.

### Shooter

| Parameter | Default | Tuned | File |
|-----------|---------|-------|------|
| Hood MIN angle (deg) | 15.0 | | HoodConstants.java |
| Hood MAX angle (deg) | 45.0 | | HoodConstants.java |
| Fixed 1m hood (deg) | 40.0 | | Shooter.java |
| Fixed 1m RPM | 2000 | | Shooter.java |
| Fixed 2m hood (deg) | 30.0 | | Shooter.java |
| Fixed 2m RPM | 3000 | | Shooter.java |
| Fixed 3m hood (deg) | 25.0 | | Shooter.java |
| Fixed 3m RPM | 4000 | | Shooter.java |
| Turret ready tol (deg) | 2.0 | | Shooter.java |
| Hood ready tol (deg) | 2.0 | | Shooter.java |
| Flywheel ready tol (RPM) | 150 | | Shooter.java |
| Spindexer run RPM | 500 | | Spindexer.java |

### Intake

| Parameter | Default | Tuned | File |
|-----------|---------|-------|------|
| Deploy angle (deg) | 0.0 | | IntakeConstants.java |
| Retract angle (deg) | 90.0 | | IntakeConstants.java |
| Roller intake RPM | 2000 | | IntakeConstants.java |
| Roller reverse RPM | −1000 | | IntakeConstants.java |

### Climb

| Parameter | Default | Tuned | File |
|-----------|---------|-------|------|
| Max height (m) | 0.5 | | ClimbElevatorConstants.java |
| Drum radius (in) | 1.0 | | ClimbElevatorConstants.java |
| Carriage mass (lbs) | 20.0 | | ClimbElevatorConstants.java |
| Gear reduction | 20:1 | | ClimbElevatorConstants.java |
| Tilt threshold (deg) | 10.0 | | ClimbElevatorConstants.java |
| Approach speed (m/s) | 3.0 | | ClimbElevatorConstants.java |
| Pass speed (m/s) | 1.5 | | ClimbElevatorConstants.java |
| Pass distance (m) | 1.0 | | ClimbElevatorConstants.java |

### Drive

| Parameter | Default | Tuned | File |
|-----------|---------|-------|------|
| Wheel radius (m) | per TunerConstants | | TunerConstants.java |
| Drive kS | per TunerConstants | | TunerConstants.java |
| Drive kV | per TunerConstants | | TunerConstants.java |
| Drive kA | per TunerConstants | | TunerConstants.java |

---

## Issues & Notes

Record any issues found during shakedown:

| Date | Subsystem | Issue | Status |
|------|-----------|-------|--------|
| | | | |
| | | | |
| | | | |
