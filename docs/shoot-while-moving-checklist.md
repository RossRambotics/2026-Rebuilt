# Shoot While Moving Checklist

## Pose Estimation & Vision

- [ ] Measure and update camera transforms in `VisionConstants` — current values `(0.2, 0.2, 0.2)` look like placeholders
- [ ] Verify vision pose estimation accuracy while driving (log pose error vs ground truth)
- [ ] Confirm vision update latency is accounted for in pose estimator (stale poses degrade prediction)
- [ ] Test with both cameras active — verify multi-camera fusion doesn't oscillate under motion

## Firing Solution & Prediction

- [ ] Validate `predictPoseLinear` accuracy — it assumes constant ChassisSpeeds (no acceleration), which is less accurate during speed changes and turning
- [ ] Consider using `predictRobotPoseCurve` (currently unused) for better accuracy during rotation-heavy driving
- [ ] Verify LUT iteration count (currently 2) converges well enough for typical engagement distances
- [ ] The firing solution does not account for the ball inheriting the turret's angular velocity at launch — assess whether this matters at typical engagement ranges

## Turret

- [ ] Verify turret real robot PID gain (kP=4) is sufficient for tracking a moving target — sim uses kP=25, which is 6x higher
- [x] Test turret tracking latency — the firing solution is computed in `periodic()` but consumed by the turret's default command one cycle later (20ms delay)
- [ ] Measure and update turret offsets (`TURRET_X_OFFSET_M`, `TURRET_Y_OFFSET_M`, `TURRET_Z_OFFSET_M`) from CAD or physical robot — currently marked TODO
- [ ] Verify turret can physically track fast enough when robot is rotating (360 deg/s turret max vs ~180 deg/s robot rotation)

## Hood

- [x] Fix feedforward kV/kA unit conversion (V/RPS to V/(rad/s)) — divide by 2pi
- [x] Set real robot PID gains (kP=50)
- [x] Align motion profile constraints with HoodOld (180 deg/s, 720 deg/s^2)
- [x] Set current limit to 20A to match HoodOld
- [x] Zero command sets initial goal angle
- [ ] Verify hood tracks dynamic angle changes smoothly on real robot (angle changes continuously while moving)
- [ ] Confirm hood fallback angle of 0 deg when no firing solution exists doesn't fight the soft limit (HOOD_DEGREES_MIN = 15 deg)

## Flywheel (BottomFlyWheel)

- [ ] Set real robot PID gains — currently `withClosedLoopController(0, 0, 0, ...)` (same issue Hood had)
- [ ] Verify feedforward units — `SimpleMotorFeedforward(0, .12, 0)` may need conversion depending on how YAMS passes FF to TalonFX for velocity control (check if YAMS converts RPS to rad/s for `SimpleMotorFeedforward` like it does for `ArmFeedforward`)
- [ ] The `upperSoftLimit` is set to `RPM.of(1000)` but the LUT commands up to 3200 RPM — this may cap flywheel speed
- [ ] Verify flywheel spin-up time from idle to target RPM at various distances — if too slow, consider keeping flywheel pre-spun at a baseline RPM
- [ ] Tune RPM LUT with real ballistics data (current values are samples)

## Feed Mechanism (Kicker & Spindexer)

- [ ] `getAutoShootCmd()` only calls `launchFuelSim()` — need to also actuate kicker and spindexer to feed real game pieces on the actual robot
- [ ] Determine kicker/spindexer RPM required for reliable feeding while moving
- [ ] Test feed timing — `AUTO_SHOOT_INTERVAL_SEC = 0.5` (2 balls/sec) may be too fast if spindexer can't index that quickly

## Drive & SpeedNanny

- [ ] Verify SpeedNanny shooting-mode limits are appropriate: 50% max speed, 10% max angular rate, 1.5 m/s^2 accel, 1.5 rad/s^2 angular accel
- [x] Potential double-scaling bug in `DriveCommands.joystickDrive()` — `getRotationalVelocityFromJoysticks` returns rad/s (already scaled by SpeedNanny angular rate limit), then result is multiplied again by `drive.getMaxAngularSpeedRadPerSec()` (line 158). Same issue in `joystickDriveRobotCentric` (line 382). Verify this is intentional or fix.
- [ ] Test driver feel with SpeedNanny limits active — transitions between shooting/not-shooting should feel smooth, not jarring

## Calibration & Physical Measurements

- [ ] Populate RPM and hood angle LUTs with real data at multiple distances (current tables cover 1-6 meters in 1m increments — may need finer resolution)
- [ ] Validate `rpmToExitVelocity()` constants (roller radii, efficiency factor 0.95, RPM ratio 1.0) against measured exit velocities
- [ ] Replace ToF LUT placeholder values and remove the `tune = 0.2` fudge factor — use the empirical measurement procedure below
- [ ] Verify motor type in Hood (`DCMotor.getKrakenX44(1)`) — confirmed X44

---

## Empirical TOF Measurement

The robot measures ball time-of-flight (TOF) empirically using a CANrange proximity sensor on the
kicker as the launch detector and a button-box confirmation button as the arrival marker. Results
feed directly into the `m_tofTable` in `BottomFlyWheel.java`, which the firing solution uses to
compute the predicted future robot pose.

### How it works

1. **Launch detection** — `Shooter.updateTofTracking()` runs every loop while autoshoot is active.
   A CANrange sensor (CAN 44) mounted on the kicker detects each ball as it transits from the
   spindexer into the flywheel. The **falling edge** of detection (ball leaving the sensor's view)
   records `Timer.getFPGATimestamp()` as the launch time and increments the shot counter. Up to 5
   launches are tracked per session.

2. **Arrival confirmation** — The operator (or a second person watching the target) presses
   **"Shooter / TOF Confirm"** on the Elastic button box the moment each ball visibly arrives at
   the target. The first press records the arrival time for ball 1, the second for ball 2, etc.
   TOF = arrival timestamp − launch timestamp.

3. **Unconfirmed launches** — If the right trigger is released before all launches are confirmed,
   any unconfirmed balls are recorded as 0 s and excluded from the average and std dev. Only
   button-press confirmations contribute to the statistics.

4. **Session reset** — Every new right-trigger press clears all counters and timestamps
   automatically, so each shooting run starts fresh.

5. **Statistics** — After 5 confirmations (or on trigger release), `logTofResults()` logs per-ball
   TOF, average, and standard deviation.

### Step 1 — Tune the CANrange proximity threshold

The CANrange looks through polycarbonate. Before collecting TOF data, verify it is detecting balls
reliably and not generating false positives.

1. Stand stationary at a known distance. Hold the right trigger to spin up and begin feeding.
2. Open AdvantageScope and watch **`Kicker/HasFuel`**, **`Kicker/FuelDistanceM`**, and
   **`Shooter/TOF/LaunchCount`** simultaneously.
3. Confirm that `HasFuel` pulses once per ball and `LaunchCount` increments exactly once per pulse
   with no spurious extra increments.
4. If `HasFuel` flickers without a ball present (false positives through the polycarbonate),
   **decrease** `FUEL_DETECTION_THRESHOLD_M` in `Kicker.java`, or **increase**
   `FUEL_DETECTION_HYSTERESIS_M` to dampen chatter.
5. If a ball passes through but `HasFuel` never goes true, **increase** `FUEL_DETECTION_THRESHOLD_M`
   (try 0.13–0.15 m).
6. If `MinSignalStrengthForValidMeasurement` filtering is needed, set it via `canRangeConfig.ProximityParams`
   in `Kicker.java`.

> Threshold constants are in `Kicker.java`:
> ```java
> private static final double FUEL_DETECTION_THRESHOLD_M = 0.1016; // 4 inches — tune for polycarbonate
> private static final double FUEL_DETECTION_HYSTERESIS_M = 0.010; // 1 cm
> ```

### Step 2 — Collect data at each LUT distance

The existing LUT covers 1 m through 6 m in 1 m increments. Collect at least 5 shots per distance
point and use the average.

**Procedure per distance:**

1. Position the robot at the target distance (measure with a tape — use the robot's center, not
   the bumper).
2. Enable the robot in teleop. Hold the right trigger to start autoshoot. Wait for the flywheel to
   reach target RPM (watch `Shooter/IsBottomFlyWheelReady`).
3. Fire one ball. The second you see (or hear) it hit the target, press **"Shooter / TOF Confirm"**
   on the button box. Reaction time is the main error source — be consistent.
4. Fire a second ball and press again. Repeat for all 5 balls in the session.
5. Release the right trigger. AdvantageScope will log:
   - `Shooter/TOF/Ball1Sec` … `Ball5Sec` — individual TOFs
   - `Shooter/TOF/AverageSec` — mean for this session
   - `Shooter/TOF/StdDevSec` — standard deviation
6. Record `AverageSec` for this distance. Repeat 2–3 sessions and average across sessions if std
   dev is large (> 0.05 s).

**Recommended distance points:** 1 m, 2 m, 3 m, 4 m, 5 m, 6 m. Add intermediate points (e.g.,
1.5 m, 2.5 m) if the interpolated values feel inaccurate at those ranges during integration
testing.

### Step 3 — Update the LUT

Edit the `m_tofTable` in `BottomFlyWheel.java` (constructor, around line 196):

```java
// Before (placeholders + fudge factor):
double tune = 0.2;
m_tofTable.put(1.0, 0.885 + tune);
m_tofTable.put(2.0, 1.028 + tune);
...

// After (replace with measured averages, remove tune):
m_tofTable.put(1.0, <measured average at 1 m>);
m_tofTable.put(2.0, <measured average at 2 m>);
m_tofTable.put(3.0, <measured average at 3 m>);
m_tofTable.put(4.0, <measured average at 4 m>);
m_tofTable.put(5.0, <measured average at 5 m>);
m_tofTable.put(6.0, <measured average at 6 m>);
```

Remove the `double tune = 0.2;` line and the `+ tune` additions once real data is in place.

### AdvantageScope log keys

| Key | Description |
|-----|-------------|
| `Kicker/HasFuel` | True while a ball is at the CANrange sensor |
| `Kicker/FuelDistanceM` | Raw CANrange distance in meters (use to tune threshold) |
| `Shooter/KickerHasFuel` | Same as `Kicker/HasFuel`, mirrored onto the Shooter table |
| `Shooter/TOF/LaunchCount` | Balls detected via CANrange falling edge this session |
| `Shooter/TOF/ConfirmCount` | Button-press confirmations this session |
| `Shooter/TOF/Ball1Sec` … `Ball5Sec` | Per-ball measured TOF |
| `Shooter/TOF/AverageSec` | Session mean TOF |
| `Shooter/TOF/StdDevSec` | Session std dev (logged after 5 confirms) |
| `Shooter/TOF/ShotCount` | Valid (positive) TOF readings in session |
| `Shooter/TOF/ConfirmStatus` | Text status of last confirm button press |
| `Shooter/IsHopperEmpty` | True if no ball detected for > 1 s during a shoot session |
| `Shooter/TimeOfFlightSec` | **Predicted** TOF from the LUT (compare to measured) |

### Acceptance criteria

- Std dev < 0.05 s at each distance point (consistent operator timing).
- After LUT update: `Shooter/TimeOfFlightSec` (predicted) matches measured average within ~0.05 s
  at all tested distances while stationary.
- During shoot-while-moving: `FutureRobotPose2d` should align with the robot's actual position at
  the time of impact (validate in AdvantageScope replay).

---

## Integration Testing

- [ ] Test stationary shooting at known distances — verify accuracy before adding motion
- [ ] Test shooting while driving straight (no rotation) — isolates prediction from turret tracking
- [ ] Test shooting while rotating in place — isolates turret tracking from prediction
- [ ] Test shooting while driving and rotating — full integration
- [ ] Log and compare `FutureRobotPose2d` vs actual pose at time of impact to validate prediction accuracy
- [ ] Test at various distances (near/mid/far) — prediction errors scale with distance and ToF
- [ ] Verify readiness debounce timing (100ms) isn't too aggressive — may reject valid shots during motion, or too lenient — may allow shots when not settled
