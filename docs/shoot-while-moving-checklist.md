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

- [ ] Populate RPM, hood angle, and ToF LUTs with real data at multiple distances (current tables cover 1-6 meters in 1m increments — may need finer resolution)
- [ ] Validate `rpmToExitVelocity()` constants (roller radii, efficiency factor 0.95, RPM ratio 1.0) against measured exit velocities
- [ ] ToF table has a `tune = 0.2` fudge factor — calibrate with real data
- [ ] Verify motor type in Hood (`DCMotor.getKrakenX44(1)`) — confirmed X44

## Integration Testing

- [ ] Test stationary shooting at known distances — verify accuracy before adding motion
- [ ] Test shooting while driving straight (no rotation) — isolates prediction from turret tracking
- [ ] Test shooting while rotating in place — isolates turret tracking from prediction
- [ ] Test shooting while driving and rotating — full integration
- [ ] Log and compare `FutureRobotPose2d` vs actual pose at time of impact to validate prediction accuracy
- [ ] Test at various distances (near/mid/far) — prediction errors scale with distance and ToF
- [ ] Verify readiness debounce timing (100ms) isn't too aggressive — may reject valid shots during motion, or too lenient — may allow shots when not settled
