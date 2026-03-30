// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootingTarget;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TimedSubsystem;
import frc.robot.util.FuelSim;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends TimedSubsystem {
  // location of the turret relative to the robot
  // Turret offsets from robot center (meters)
  private final double TURRET_X_OFFSET_M = 0.14; // meters
  private final double TURRET_Y_OFFSET_M = -0.158; // meters
  private final double TURRET_Z_OFFSET_M = 0.46; // meters

  // Turret pose in field-relative coordinates (Pose3d)
  private Pose3d m_turretPoseField3d = null;

  // Turret subsystem (controls yaw of the shooter)
  private final Turret m_turret;

  // Hood and flywheel subsystems
  private final Hood m_hood;
  private final BottomFlyWheel m_bottomFlyWheel;

  // Kicker and spindexer (intake/feeds)
  private final Kicker m_kicker;
  private final Spindexer m_spindexer;

  // Whether turret auto-targeting is active
  private boolean m_isTurretActive = true;

  // Human-readable name of current target
  private String m_targetName = ShootingTarget.HUB.displayName;

  // Current target pose (field-relative)
  // Target pose in field-relative coordinates (Pose3d)
  private Pose3d m_targetPoseField3d = ShootingTarget.HUB.getFieldPose3d();

  // Last calculated turret values / telemetry
  private Angle m_turretTargetRobotAngle = Degrees.of(0);
  private Angle m_lastTurretErrorRobotAngle = Degrees.of(0);
  // Shuffleboard entry: target hood angle in degrees
  private DoubleEntry m_targetHoodAngleDegEntry = null; // Shuffleboard entry (Deg)
  // Last hood angle and flywheel RPM are tracked in telemetry via Logger; remove
  // redundant fields
  private FiringSolution m_lastFiringSolution = null;
  private boolean m_isShooting = false;
  private boolean m_isTurretServoMode = false;

  // ── Turret jam detection ───────────────────────────────────────────────────
  // Reuses TURRET_ANGLE_TOLERANCE_DEG as the error threshold.
  // Velocity threshold matches Turret homing stall detection.
  private static final double TURRET_STUCK_VELOCITY_RPS = 0.5;
  private static final double TURRET_STUCK_DURATION_SEC = 0.25;
  private boolean m_isTurretJammed = false;
  private final Timer m_turretStuckTimer = new Timer();

  // Persistent in-match RPM offset accumulated by the trim buttons.
  // Added inside every flywheel velocity calculation (auto and fixed).
  private double m_rpmTrimOffsetRpm = -50.0;

  // Persistent in-match hood angle offset (degrees) accumulated by the trim
  // buttons.
  // Applied in every dynamic hood setpoint and the readiness check.
  private double m_hoodTrimOffsetDeg = 0.0;

  // Persistent in-match turret yaw offset (degrees) accumulated by the trim
  // buttons.
  // Applied in every dynamic turret targeting calculation and the readiness
  // check.
  private static final double TURRET_TRIM_MAX_DEG = 15.0;
  private double m_turretTrimOffsetDeg = 0.0;

  // EMA of actual flywheel RPM — smooths the noisy raw reading for dashboard
  // display.
  private static final double RPM_AVERAGE_ALPHA = 0.05; // 5% weight → ~20-sample window at 50 Hz
  private double m_rpmAverageRpm = 0.0;

  // ── TOF empirical measurement state ───────────────────────────────────────
  // Launch timestamps recorded on CANrange falling edge (ball leaving kicker).
  private final double[] m_tofLaunchTimestampsSec = new double[TOF_MAX_SHOTS];
  // Measured TOF per shot; -1.0 = not yet confirmed.
  private final double[] m_tofMeasuredSec = new double[TOF_MAX_SHOTS];
  // How many CANrange falling-edge events detected (balls launched) this session.
  private int m_tofLaunchCount = 0;
  // How many button-press confirmations (ball arrivals) recorded this session.
  private int m_tofConfirmCount = 0;
  // CANrange edge detection: track last-loop state for falling-edge detection.
  private boolean m_kickerHadFuelLastLoop = false;
  // Cumulative count of all balls launched since robot startup (never reset).
  private int m_totalBallsLaunched = 0;

  // ── Hopper-empty detection ─────────────────────────────────────────────────
  // Timestamp of the most recent loop where kicker reported hasFuel().
  private double m_hopperLastSeenTimestampSec = 0.0;
  // Guards against a false-empty at the very start of a shoot sequence.
  private boolean m_hopperSeenSinceShootStart = false;
  private boolean m_isHopperEmpty = false;
  private static final double HOPPER_EMPTY_TIMEOUT_SEC = 1.0;

  // Readiness thresholds (allowable error)
  private static final double TURRET_ANGLE_TOLERANCE_DEG = 3.0; // degrees
  private static final double HOOD_ANGLE_TOLERANCE_DEG = 2.0; // degrees
  private static final double BOTTOM_FLYWHEEL_RPM_TOLERANCE = 200.0; // RPM
  // Debounce durations for readiness signals (seconds)
  private static final double TURRET_READY_DEBOUNCE_SEC = 0.05; // 100 ms
  private static final double HOOD_READY_DEBOUNCE_SEC = 0.05; // 100 ms
  private static final double BOTTOM_FLYWHEEL_READY_DEBOUNCE_SEC = 0.05; // 100 ms
  private static final double AUTO_SHOOT_INTERVAL_SEC = 0.5; // 2 balls per second
  // Extra time added to TOF when predicting future pose, to compensate for system
  // latency
  // (sensor delay, loop delay, actuator delay). Tune on hardware.
  private static final double POSE_PREDICT_TIMESHIFT_SEC = 0.00;
  // Cap the velocity magnitude used for pose prediction to prevent over-leading
  // at high robot speeds where the constant-velocity assumption breaks down.
  private static final double MAX_PREDICTION_SPEED_MPS = 1.5;

  // ── TOF empirical measurement ──────────────────────────────────────────────
  private static final int TOF_MAX_SHOTS = 5;

  // ── Fixed-distance shot constants ────────────────────────────────────────
  // TODO: tune all six values by shooting from measured distances on the field.
  private static final double FIXED_SHOT_1M_HOOD_DEG = 15.0; // TODO: tune
  private static final double FIXED_SHOT_1M_RPM = 2150.0; // TODO: tune
  private static final double FIXED_SHOT_2M_HOOD_DEG = 30.0; // TODO: tune
  private static final double FIXED_SHOT_2M_RPM = 2000.0; // TODO: tune
  private static final double FIXED_SHOT_3M_HOOD_DEG = 35.0; // TODO: tune
  private static final double FIXED_SHOT_3M_RPM = 2200.0; // TODO: tune
  private static final double FIXED_SHOT_4M_HOOD_DEG = 35.0; // TODO: tune
  private static final double FIXED_SHOT_4M_RPM = 2600.0; // TODO: tune
  private static final double FIXED_SHOT_5M_HOOD_DEG = 35.0; // TODO: tune
  private static final double FIXED_SHOT_5M_RPM = 2800.0; // TODO: tune
  private static final double FIXED_SHOT_6M_HOOD_DEG = 35.0; // TODO: tune
  private static final double FIXED_SHOT_6M_RPM = 3000.0; // TODO: tune
  private static final double FIXED_SHOT_7M_HOOD_DEG = 35.0; // TODO: tune
  private static final double FIXED_SHOT_7M_RPM = 3200.0; // TODO: tune
  private static final double FIXED_SHOT_8M_HOOD_DEG = 40.0; // TODO: tune
  private static final double FIXED_SHOT_8M_RPM = 3450.0; // TODO: tune

  // Manual control rates (tunable — degrees or RPM per second)
  private static final double MANUAL_TURRET_RATE_DEG_PER_SEC = 180.0;
  private static final double MANUAL_HOOD_RATE_DEG_PER_SEC = 90.0;
  private static final double MANUAL_FLYWHEEL_RATE_RPM_PER_SEC = 1000.0;
  private static final double MANUAL_FLYWHEEL_MAX_RPM = 6000.0;

  // Idle flywheel speed — pre-spun for fast response when shooting starts
  public static final double IDLE_FLYWHEEL_RPM = 2000.0;
  // Kicker speed during shooting. TODO: tune on hardware.
  private static final double KICKER_SHOOT_RPM = 6000.0;

  // Readiness flags
  private boolean m_isShooterReady = false;
  private boolean m_isTurretReady = false;
  private boolean m_isSpeedReady = false;
  private boolean m_isBottomFlyWheelReady = false;
  private final Debouncer m_bottomFlywheelReadyDebouncer =
      new Debouncer(BOTTOM_FLYWHEEL_READY_DEBOUNCE_SEC); // 100 ms
  // continuous
  // stability
  // Debouncers for turret and hood readiness
  private final Debouncer m_turretReadyDebouncer = new Debouncer(TURRET_READY_DEBOUNCE_SEC);
  private final Debouncer m_hoodReadyDebouncer = new Debouncer(HOOD_READY_DEBOUNCE_SEC);
  private boolean m_isHoodReady = false;

  /** Creates a new Shooter. */
  public Shooter(boolean isReal) {
    m_turret = new Turret(isReal);
    m_hood = new Hood(isReal);
    m_bottomFlyWheel = new BottomFlyWheel(isReal);
    m_kicker = new Kicker(isReal);
    m_spindexer = new Spindexer(isReal);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_targetHoodAngleDegEntry = table.getDoubleTopic("TargetHoodAngleDeg").getEntry(0.0);
    m_targetHoodAngleDegEntry.set(HoodConstants.HOOD_DEGREES_MIN);

    // Set default commands:
    // Turret — always tracks predictively (same in IDLE and PREDICTIVE TARGETING
    // modes)
    // Hood — idle: retracted to HOOD_DEGREES_MIN (safe for driving)
    // Flywheel — idle: pre-spun at IDLE_FLYWHEEL_RPM (fast response on trigger)
    // Kicker — stop: ensures zero output when not commanded
    // Spindexer — stop: ensures zero output when not commanded
    m_turret.setDefaultCommand(getPredictiveTargetingCommand());
    m_hood.setDefaultCommand(getIdleHoodCmd());
    m_bottomFlyWheel.setDefaultCommand(getIdleFlywheelCmd());
    m_kicker.setDefaultCommand(m_kicker.stop());
    m_spindexer.setDefaultCommand(m_spindexer.stop());

    Shuffleboard.getTab("YAMS_SysID").add(m_hood.sysId());

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 0", Degrees.of(0.0)));

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 20", Degrees.of(20.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 30", Degrees.of(30.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 35", Degrees.of(35.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 40", Degrees.of(40.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 45", Degrees.of(45.0)));
    Shuffleboard.getTab("Hood").add(m_hood.getZeroHoodAngleCmd());

    Shuffleboard.getTab("BottomFlyWheel").add(this.getFlyWheelRPMCmd("Dynamic"));
    Shuffleboard.getTab("Kicker").add(this.getKickerRPMCmd("Kicker Dynamic"));
    Shuffleboard.getTab("Spindexer").add(this.getSpindexerRPMCmd("Spindexer Dynamic"));
  }

  /**
   * Sets the target pose for the shooter.
   *
   * @param targetPose The target pose in field-relative coordinates.
   */
  public void setTargetPose(String name, Pose3d targetPose) {
    m_targetPoseField3d = targetPose;
    setTargetName(name);
  }

  private Rotation2d getAngleToTarget(Pose2d robotPose, Pose2d targetPose) {

    // Vector from robot to target
    double dx = targetPose.getX() - robotPose.getX();
    double dy = targetPose.getY() - robotPose.getY();

    // Angle (in radians) from robot -> target in field coordinates
    Rotation2d angleToTarget = new Rotation2d(Math.atan2(dy, dx));

    // Robot's heading
    Rotation2d robotHeading = robotPose.getRotation();

    // Angle from robot's heading to target
    return angleToTarget.minus(robotHeading);
  }

  private Angle getTurretTargetAnglePredicted() {
    if (m_lastFiringSolution == null) {
      return getTurretTargetAngle();
    }

    Angle a = Radians.of(m_lastFiringSolution.turretAngleRad);
    a =
        Degrees.of(
            MathUtil.clamp(
                a.in(Degrees) + m_turretTrimOffsetDeg,
                Turret.TURRET_DEGREES_MIN,
                Turret.TURRET_DEGREES_MAX));

    m_turretTargetRobotAngle = a;
    double turretHeadingDeg = m_turret.getAngle().in(Degrees);
    m_lastTurretErrorRobotAngle = Degrees.of(a.in(Degrees) - turretHeadingDeg);

    return a;
  }

  private Angle getTurretTargetAngle() {
    Pose2d robotPose = RobotContainer.getInstance().getRobotPose2d();

    // use the turrets location, but the robots heading
    // this is because we need to adjust the turret based on its angle being robot
    // relative.
    Pose2d temp = getTurretPose3dFieldRelative().toPose2d();
    Pose2d turretPose = new Pose2d(temp.getX(), temp.getY(), robotPose.getRotation());

    Rotation2d angleToTarget = getAngleToTarget(turretPose, m_targetPoseField3d.toPose2d());

    // clamp the turret target to the rotation available for the turret
    double turretTargetDeg = angleToTarget.getDegrees();
    turretTargetDeg =
        MathUtil.clamp(
            turretTargetDeg + m_turretTrimOffsetDeg,
            Turret.TURRET_DEGREES_MIN,
            Turret.TURRET_DEGREES_MAX);

    m_turretTargetRobotAngle = Degrees.of(turretTargetDeg);

    double turretHeadingDeg = m_turret.getAngle().in(Degrees);
    m_lastTurretErrorRobotAngle = Degrees.of(turretTargetDeg - turretHeadingDeg);

    return m_turretTargetRobotAngle;
  }

  /**
   * Command factory: returns a command that aims the turret using the current location (no
   * prediction).
   */
  public Command getNormalTargetingCommand() {
    return m_turret.setAngle(
        () -> {
          if (m_isTurretActive) {
            return getTurretTargetAngle();
          }
          return Degrees.of(0);
        });
  }

  /**
   * Command factory: returns a command that aims the turret using the predictive firing solution.
   */
  public Command getPredictiveTargetingCommand() {
    return m_turret.setAngle(
        () -> {
          if (m_isTurretActive) {
            return getTurretTargetAnglePredicted();
          }
          return Degrees.of(0);
        });
  }

  /**
   * Computes the robot heading required to aim at the target given the turret's current angle.
   *
   * <p>Because {@code robotHeading + turretAngle = fieldAngleToTarget}, the required robot heading
   * is {@code fieldAngleToTarget − turretCurrentAngle + turretTrimOffset}. This correctly accounts
   * for wherever the turret is physically stuck: if it is jammed at 35° the robot only needs to
   * rotate by the remaining error, not the full field angle. Returns {@link Rotation2d#kZero} if
   * the turret pose is not yet available.
   */
  public Rotation2d getRobotHeadingForTurretServo() {
    if (m_turretPoseField3d == null) {
      return Rotation2d.kZero;
    }
    Pose2d turretPos = m_turretPoseField3d.toPose2d();
    Translation2d delta =
        m_targetPoseField3d.toPose2d().getTranslation().minus(turretPos.getTranslation());
    Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    Rotation2d turretCurrentAngle = Rotation2d.fromDegrees(m_turret.getAngle().in(Degrees));
    return fieldAngleToTarget
        .minus(turretCurrentAngle)
        .plus(Rotation2d.fromDegrees(m_turretTrimOffsetDeg));
  }

  private Command getDynamicHoodCommand() {
    Command c =
        m_hood.setAngle(
            () -> {
              if (m_lastFiringSolution != null && m_isTurretActive) {
                double trimmedDeg =
                    MathUtil.clamp(
                        Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees)
                            + m_hoodTrimOffsetDeg,
                        HoodConstants.HOOD_DEGREES_MIN,
                        HoodConstants.HOOD_DEGREES_MAX);
                return Degrees.of(trimmedDeg);
              }
              return Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
            });
    c.addRequirements(m_hood);
    return c;
  }

  private Command getDynamicFlywheelCommand() {
    return m_bottomFlyWheel.setVelocity(
        () -> {
          if (m_lastFiringSolution != null && m_isTurretActive) {
            return RPM.of(m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm);
          }
          return RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm);
        });
  }

  /** Default/idle hood command: retracted to minimum angle, safe for driving. */
  private Command getIdleHoodCmd() {
    return m_hood.setAngle(Degrees.of(HoodConstants.HOOD_DEGREES_MIN)).withName("Hood/Idle");
  }

  /** Default/idle flywheel command: pre-spun at idle RPM for fast spinup. */
  private Command getIdleFlywheelCmd() {
    // RPM trim is a shot-accuracy adjustment — do not apply it to the idle pre-spin
    // speed.
    return m_bottomFlyWheel.setVelocity(() -> RPM.of(IDLE_FLYWHEEL_RPM)).withName("Flywheel/Idle");
  }

  /** Returns a Shuffleboard/instant command that sets the turret default to normal targeting. */
  public Command getEnableNormalTargetingModeCmd() {
    Command c =
        Commands.runOnce(() -> m_turret.setDefaultCommand(getNormalTargetingCommand()), m_turret)
            .ignoringDisable(true);
    c.setName("NormalTargeting");
    return c;
  }

  /**
   * Returns a Shuffleboard/instant command that sets the turret default to predictive targeting.
   */
  public Command getEnablePredictiveTargetingModeCmd() {
    Command c =
        Commands.runOnce(
                () -> m_turret.setDefaultCommand(getPredictiveTargetingCommand()), m_turret)
            .ignoringDisable(true);
    c.setName("PredictiveTargeting");
    return c;
  }

  /**
   * returns the pose of the turret relative to the field
   *
   * @return Pose3d of the turret in field-relative coordinates
   */
  public Pose3d getTurretPose3dFieldRelative() {
    return m_turretPoseField3d;
  }

  @Override
  protected void periodicImpl() {
    // This method will be called once per scheduler run

    updateTurretPose();
    // Update firing solution and telemetry (compute early so commands/targeting can
    // use it)
    updateFiringSolution();
    // Detect turret jams; must run after updateFiringSolution() and before
    // updateReadiness()
    // updateJamDetection();
    // Updates target data (kept lightweight) and then publish telemetry last
    updateTargetData();
    // Update readiness flags (moved to its own method for clarity)
    updateReadiness();
    // Detect ball launches via CANrange falling edge for empirical TOF measurement
    updateTofTracking();

    updateTelemetry();
  }

  /**
   * Compute the firing solution (time-of-flight, turret & hood angles, rpm) and publish telemetry.
   * Kept outside of periodic to make the periodic loop cleaner.
   */
  private void updateFiringSolution() {
    // Shooter offset relative to robot center
    Translation2d shooterOffsetRobot = new Translation2d(TURRET_X_OFFSET_M, TURRET_Y_OFFSET_M);
    FiringSolution solution =
        calculateFiringSolutionLUT(
            RobotContainer.getInstance().getRobotPose2d(),
            RobotContainer.getInstance().getChassisSpeeds(),
            m_targetPoseField3d.toPose2d().getTranslation(),
            shooterOffsetRobot);

    // store computed solution for consumers; telemetry will be published in
    // updateTelemetry()
    m_lastFiringSolution = solution;
  }

  /**
   * Detects a turret that cannot reach its target during an active shooting sequence and latches
   * {@link #m_isTurretJammed}. Two conditions trigger the latch:
   *
   * <ul>
   *   <li><b>Mechanical jam:</b> position error exceeds {@link #TURRET_ANGLE_TOLERANCE_DEG} while
   *       turret velocity stays below {@link #TURRET_STUCK_VELOCITY_RPS}.
   *   <li><b>Geometric infeasibility:</b> the required turret angle is outside ±90° ({@code
   *       !isTurretFeasible()}). The turret reaches its clamped limit with near-zero error, so the
   *       velocity check alone would not catch this case.
   * </ul>
   *
   * <p>Either condition must persist for {@link #TURRET_STUCK_DURATION_SEC} before the flag is
   * latched. Once latched, {@link #m_isTurretJammed} stays {@code true} until the next shooting
   * sequence resets it.
   *
   * <p>Must be called after {@link #updateFiringSolution()} so {@link #m_lastFiringSolution} is
   * current.
   */
  private void updateJamDetection() {
    // Only detect while actively shooting; skip if already latched or in manual
    // servo mode.
    if (!m_isShooting || m_isTurretJammed || m_isTurretServoMode) return;
    if (m_lastFiringSolution == null) return;

    double commandedDeg = Math.toDegrees(m_lastFiringSolution.turretAngleRad);
    double actualDeg = m_turret.getAngle().in(Degrees);
    double errorDeg = Math.abs(commandedDeg - actualDeg);
    double velocityRps =
        Math.abs(m_turret.getVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond));

    boolean isStuck =
        !m_lastFiringSolution.isTurretFeasible()
            || (errorDeg > TURRET_ANGLE_TOLERANCE_DEG && velocityRps < TURRET_STUCK_VELOCITY_RPS);

    if (isStuck) {
      m_turretStuckTimer.start();
    } else {
      m_turretStuckTimer.reset();
    }

    if (m_turretStuckTimer.hasElapsed(TURRET_STUCK_DURATION_SEC)) {
      m_isTurretJammed = true;
    }
  }

  /** Returns {@code true} if the turret has been detected as jammed this shooting sequence. */
  public boolean isTurretJammed() {
    return m_isTurretJammed;
  }

  public FiringSolution calculateFiringSolutionLUT(
      Pose2d currentPose,
      ChassisSpeeds chassisSpeeds,
      Translation2d targetFieldPos,
      Translation2d shooterOffsetRobot) {

    // --- Cap prediction velocity ---
    // At high speeds the constant-velocity model over-predicts where the robot
    // will be, causing the turret to over-lead. Clamp the speed magnitude so the
    // prediction stays trustworthy.
    double speedMag = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    if (speedMag > MAX_PREDICTION_SPEED_MPS && speedMag > 0) {
      double scale = MAX_PREDICTION_SPEED_MPS / speedMag;
      chassisSpeeds =
          new ChassisSpeeds(
              chassisSpeeds.vxMetersPerSecond * scale,
              chassisSpeeds.vyMetersPerSecond * scale,
              chassisSpeeds.omegaRadiansPerSecond * scale);
    }

    // --- Initial guess for TOF ---
    // Warm-start from last solution when available (already near-converged);
    // otherwise seed from
    // LUT using the current (non-predicted) shooter-to-target distance.
    double tofSec;
    if (m_lastFiringSolution != null) {
      tofSec = m_lastFiringSolution.tofSec;
    } else {
      Translation2d initialShooterFieldPos =
          currentPose.getTranslation().plus(shooterOffsetRobot.rotateBy(currentPose.getRotation()));
      double initialDistanceMeters = initialShooterFieldPos.getDistance(targetFieldPos);
      tofSec = m_bottomFlyWheel.calculateToF(initialDistanceMeters);
    }

    Pose2d futurePose;
    Translation2d shooterFieldPos;
    double distanceMeters;

    // --- Iterate to converge TOF estimate ---
    for (int i = 0; i < 3; i++) {
      futurePose = predictPoseLinear(currentPose, chassisSpeeds, tofSec);
      shooterFieldPos =
          futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

      distanceMeters = shooterFieldPos.getDistance(targetFieldPos);

      // LUT-based TOF instead of physics
      tofSec = m_bottomFlyWheel.calculateToF(distanceMeters);
    }

    // --- Final future pose ---
    futurePose = predictPoseLinear(currentPose, chassisSpeeds, tofSec);
    shooterFieldPos =
        futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

    // --- LUT lookup ---
    distanceMeters = shooterFieldPos.getDistance(targetFieldPos);
    Logger.recordOutput("Shooter/DistanceMeters", distanceMeters);

    double hoodMechAngleRad =
        Degrees.of(m_bottomFlyWheel.calculateAngle(distanceMeters)).in(Radians);
    double flywheelRPM = m_bottomFlyWheel.calculateRPM(distanceMeters);

    // --- Turret angle (robot-relative) ---
    Translation2d delta = targetFieldPos.minus(shooterFieldPos);

    // MathUtil.angleModulus normalizes to (-π, π], preventing wrap errors near
    // ±180°
    double rawTurretAngleRad =
        MathUtil.angleModulus(
            Math.atan2(delta.getY(), delta.getX()) - futurePose.getRotation().getRadians());

    double rawTurretAngleDeg = Math.toDegrees(rawTurretAngleRad);

    boolean turretFeasible =
        rawTurretAngleDeg >= Turret.TURRET_DEGREES_MIN
            && rawTurretAngleDeg <= Turret.TURRET_DEGREES_MAX;

    double turretAngleRad =
        Math.toRadians(
            MathUtil.clamp(
                rawTurretAngleDeg, Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX));

    return new FiringSolution(
        turretAngleRad,
        hoodMechAngleRad,
        flywheelRPM,
        futurePose,
        tofSec,
        distanceMeters,
        turretFeasible);
  }

  public FiringSolution calculateFiringSolution(
      Pose2d currentPose,
      ChassisSpeeds chassisSpeeds,
      Translation2d targetFieldPos,
      Translation2d shooterOffsetRobot) {
    // --- Initial guess for TOF ---
    double tofSec = 1.0; // seconds (good initial guess)

    Pose2d futurePose;
    Translation2d shooterFieldPos;
    double distanceMeters;

    // --- Iterate once or twice ---
    for (int i = 0; i < 2; i++) {
      futurePose = predictPoseLinear(currentPose, chassisSpeeds, tofSec);

      shooterFieldPos =
          futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

      distanceMeters = shooterFieldPos.getDistance(targetFieldPos);

      double hoodMechRad = Degrees.of(m_bottomFlyWheel.calculateAngle(distanceMeters)).in(Radians);
      double flywheelRPM = m_bottomFlyWheel.calculateRPM(distanceMeters);

      double exitVelocityMps = rpmToExitVelocity(flywheelRPM);

      double heightDiffMeters = m_targetPoseField3d.getZ() - TURRET_Z_OFFSET_M;

      // Ball launches perpendicular to hood surface, so convert mech angle to launch
      // angle
      double launchAngleRad = hoodMechToLaunchAngle(hoodMechRad);
      double tofVacSec =
          (exitVelocityMps * Math.sin(launchAngleRad)
                  + Math.sqrt(
                      Math.pow(exitVelocityMps * Math.sin(launchAngleRad), 2)
                          + 2 * 9.81 * heightDiffMeters))
              / 9.81;

      tofSec = tofVacSec * 1.12 * 0.6; // drag correction
    }

    // --- Final future pose ---
    futurePose = predictPoseLinear(currentPose, chassisSpeeds, tofSec);

    shooterFieldPos =
        futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

    // --- LUT lookup ---
    distanceMeters = shooterFieldPos.getDistance(targetFieldPos);
    double hoodMechAngleRad =
        Degrees.of(m_bottomFlyWheel.calculateAngle(distanceMeters)).in(Radians);
    double flywheelRPM = m_bottomFlyWheel.calculateRPM(distanceMeters);

    // --- Velocity compensation for turret ---
    Translation2d delta = targetFieldPos.minus(shooterFieldPos);

    // MathUtil.angleModulus normalizes to (-π, π], preventing wrap errors near
    // ±180°
    double rawTurretAngleRad =
        MathUtil.angleModulus(
            Math.atan2(delta.getY(), delta.getX()) - futurePose.getRotation().getRadians());

    double rawTurretAngleDeg = Math.toDegrees(rawTurretAngleRad);

    boolean turretFeasible =
        rawTurretAngleDeg >= Turret.TURRET_DEGREES_MIN
            && rawTurretAngleDeg <= Turret.TURRET_DEGREES_MAX;

    double turretAngleRad =
        Math.toRadians(
            MathUtil.clamp(
                rawTurretAngleDeg, Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX));

    return new FiringSolution(
        turretAngleRad,
        hoodMechAngleRad,
        flywheelRPM,
        futurePose,
        tofSec,
        distanceMeters,
        turretFeasible);
  }

  /**
   * Stores the calculated parameters needed to make a shot.
   *
   * <p>Hood angle convention: {@code hoodMechAngleRad} is the <em>mechanical</em> angle of the hood
   * from horizontal. At 0° the hood surface is horizontal and the ball launches straight up (90°
   * above horizontal); at 90° the hood is vertical and the ball launches horizontally. The
   * operating range is [{@link HoodConstants#HOOD_DEGREES_MIN}, {@link
   * HoodConstants#HOOD_DEGREES_MAX}]. Use {@link #hoodMechToLaunchAngle} to convert to a
   * physics-compatible launch angle when needed.
   */
  public record FiringSolution(
      double turretAngleRad,
      double hoodMechAngleRad,
      double flywheelRPM,
      Pose2d futurePose,
      double tofSec,
      double distanceToTargetMeters,
      boolean isTurretFeasible) {}

  private static final double EFFICIENCY_K = 0.95; // efficiency factor

  private static final double R_TOP_METERS = Inches.of(1.0).in(Meters); // 2" diameter
  private static final double R_BOTTOM_METERS = Inches.of(1.5).in(Meters); // 3" diameter
  private static final double RPM_RATIO = 1.0; // same speed

  public static double rpmToExitVelocity(double rpmBottom) {
    double rpmTop = rpmBottom * RPM_RATIO;
    double omegaTop = 2.0 * Math.PI * rpmTop / 60.0;
    double omegaBottom = 2.0 * Math.PI * rpmBottom / 60.0;

    double vTop = omegaTop * R_TOP_METERS;
    double vBottom = omegaBottom * R_BOTTOM_METERS;

    return EFFICIENCY_K * (vTop + vBottom) / 2.0;
  }

  /**
   * Converts a mechanical hood angle to a launch angle above horizontal.
   *
   * <p>The hood launches the ball perpendicular to its surface, so a horizontal hood (0°) fires
   * straight up (90° above horizontal). The relationship is: launch angle = 90° − hood mech angle.
   *
   * @param hoodMechRad Mechanical hood angle in radians (0 = horizontal, ball fires straight up)
   * @return Launch angle above horizontal in radians (0 = horizontal shot, π/2 = vertical shot)
   */
  public static double hoodMechToLaunchAngle(double hoodMechRad) {
    return Math.PI / 2 - hoodMechRad;
  }

  public static Pose2d predictPoseLinear(
      Pose2d start,
      ChassisSpeeds speeds, // robot-relative
      double dtSeconds) {
    double theta = start.getRotation().getRadians();
    dtSeconds += POSE_PREDICT_TIMESHIFT_SEC;

    double fieldVx =
        speeds.vxMetersPerSecond * Math.cos(theta) - speeds.vyMetersPerSecond * Math.sin(theta);

    double fieldVy =
        speeds.vxMetersPerSecond * Math.sin(theta) + speeds.vyMetersPerSecond * Math.cos(theta);

    Translation2d newTranslation =
        start.getTranslation().plus(new Translation2d(fieldVx * dtSeconds, fieldVy * dtSeconds));

    Rotation2d newRotation =
        start.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * dtSeconds));

    return new Pose2d(newTranslation, newRotation);
  }

  public Pose2d predictRobotPoseCurve(Pose2d start, ChassisSpeeds speeds, double totalTime) {
    totalTime += POSE_PREDICT_TIMESHIFT_SEC;
    final double dt = 0.005;
    int steps = (int) (totalTime / dt);
    double remainder = totalTime - steps * dt;

    double x = start.getX();
    double y = start.getY();
    double theta = start.getRotation().getRadians();

    for (int i = 0; i < steps; i++) {
      double cos = Math.cos(theta);
      double sin = Math.sin(theta);

      double fieldVx = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
      double fieldVy = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

      x += fieldVx * dt;
      y += fieldVy * dt;
      theta += speeds.omegaRadiansPerSecond * dt;
    }

    // Partial last step to exactly hit totalTime
    if (remainder > 0.0) {
      double cos = Math.cos(theta);
      double sin = Math.sin(theta);
      x += (speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin) * remainder;
      y += (speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos) * remainder;
      theta += speeds.omegaRadiansPerSecond * remainder;
    }

    return new Pose2d(x, y, new Rotation2d(theta));
  }

  public static Translation2d[] buildAimRay(
      Pose2d futureRobotPose, // field-relative pose (meters, radians)
      double turretAngleRad, // robot-relative turret angle (radians)
      Translation2d shooterOffsetRobot, // robot-relative turret position (meters)
      double distanceMeters // ray length (meters)
      ) {
    // Turret position in field frame
    Translation2d turretFieldPos =
        futureRobotPose
            .getTranslation()
            .plus(shooterOffsetRobot.rotateBy(futureRobotPose.getRotation()));

    // Turret pointing direction in field frame
    double turretFieldAngleRad = futureRobotPose.getRotation().getRadians() + turretAngleRad;

    // Ray endpoint
    Translation2d rayEnd =
        turretFieldPos.plus(
            new Translation2d(
                distanceMeters * Math.cos(turretFieldAngleRad),
                distanceMeters * Math.sin(turretFieldAngleRad)));

    return new Translation2d[] {turretFieldPos, rayEnd};
  }

  private void updateTargetData() {
    // Kept lightweight: currently no per-loop side effects needed here. Telemetry
    // is published in
    // updateTelemetry() which runs last in the loop.
  }

  /**
   * Update readiness flags (turret, hood, bottom flywheel, overall shooter) based on the latest
   * firing solution and current subsystem sensor readings.
   */
  private void updateReadiness() {
    if (m_lastFiringSolution != null) {
      if (m_isTurretServoMode || m_isTurretJammed) {
        // Servo mode (manual or auto jam fallback): check drive heading alignment.
        // getRobotHeadingForTurretServo() accounts for the turret's actual current
        // angle.
        Rotation2d requiredHeading = getRobotHeadingForTurretServo();
        Rotation2d currentHeading = RobotContainer.getInstance().getRobotPose2d().getRotation();
        double headingErrorDeg = Math.abs(requiredHeading.minus(currentHeading).getDegrees());
        boolean rawServoReady = headingErrorDeg < TURRET_ANGLE_TOLERANCE_DEG;
        m_isTurretReady = m_turretReadyDebouncer.calculate(rawServoReady);
      } else {
        // Normal mode: check turret angle error
        boolean rawTurretReady =
            m_lastFiringSolution.isTurretFeasible()
                && Math.abs(m_lastTurretErrorRobotAngle.in(Degrees)) < TURRET_ANGLE_TOLERANCE_DEG;
        m_isTurretReady = m_turretReadyDebouncer.calculate(rawTurretReady);
      }

      // Bottom flywheel readiness: compare current RPM to the trimmed target (the
      // actual
      // commanded speed includes the trim offset, so the target must too).
      double currentBottomRpm = m_bottomFlyWheel.getVelocity().in(RPM);
      double targetBottomRpm = m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm;
      double errorBottomRpm = Math.abs(currentBottomRpm - targetBottomRpm);
      Logger.recordOutput("Shooter/ErrorBottomRPM", errorBottomRpm);
      Logger.recordOutput("Shooter/targetBottomRPM", targetBottomRpm);
      Logger.recordOutput(
          "Shooter/TargetHoodAngleDegrees",
          Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees));

      boolean rawBottomReady = errorBottomRpm < BOTTOM_FLYWHEEL_RPM_TOLERANCE;
      m_isBottomFlyWheelReady = m_bottomFlywheelReadyDebouncer.calculate(rawBottomReady);

      // Hood readiness: compare current hood angle to trimmed target (degrees,
      // debounced)
      double currentHoodDeg = m_hood.getAngle().in(Degrees);
      double targetHoodDeg =
          MathUtil.clamp(
              Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees) + m_hoodTrimOffsetDeg,
              HoodConstants.HOOD_DEGREES_MIN,
              HoodConstants.HOOD_DEGREES_MAX);
      boolean rawHoodReady = Math.abs(currentHoodDeg - targetHoodDeg) < HOOD_ANGLE_TOLERANCE_DEG;
      m_isHoodReady = m_hoodReadyDebouncer.calculate(rawHoodReady);

      // Speed readiness: robot translational speed must be under SpeedNanny's current
      // limit
      m_isSpeedReady = RobotContainer.getInstance().m_speedNanny.isUnderSpeedLimit();

      // Shooter as-a-whole is ready when turret, hood, bottom flywheel, and speed are
      // ready
      m_isShooterReady =
          m_isTurretReady && m_isHoodReady && m_isBottomFlyWheelReady && m_isSpeedReady;
    } else {
      m_isShooterReady = false;
    }
  }

  private void updateTurretPose() {
    // Updates the turret pose in field relative coordinates
    Pose2d robotPose2d = RobotContainer.getInstance().getRobotPose2d();
    Pose2d turretPose2d =
        new Pose2d(
            robotPose2d.getTranslation().getX() + TURRET_X_OFFSET_M,
            robotPose2d.getTranslation().getY() + TURRET_Y_OFFSET_M,
            Rotation2d.fromDegrees(m_turret.getAngle().in(Degrees)));
    turretPose2d =
        turretPose2d.rotateAround(
            new Translation2d(
                robotPose2d.getTranslation().getX(), robotPose2d.getTranslation().getY()),
            robotPose2d.getRotation()); // turret offset
    Pose3d turretPosed3d =
        new Pose3d(turretPose2d)
            .plus(new Transform3d(0.0, 0.0, TURRET_Z_OFFSET_M, new Rotation3d()));

    m_turretPoseField3d = turretPosed3d;
  }

  private void setTargetName(String name) {
    m_targetName = name;
    Logger.recordOutput("Shooter/TargetName", m_targetName);
  }

  /** Centralized telemetry publish. Called once at the end of periodic(). */
  private void updateTelemetry() {
    // Readiness indicators (always published)
    Logger.recordOutput("Shooter/IsTurretReady", m_isTurretReady);
    Logger.recordOutput("Shooter/IsBottomFlyWheelReady", m_isBottomFlyWheelReady);
    Logger.recordOutput("Shooter/IsShooterReady", m_isShooterReady);
    Logger.recordOutput("Shooter/IsShooting", m_isShooting);
    Logger.recordOutput("Shooter/IsHoodReady", m_isHoodReady);
    Logger.recordOutput("Shooter/IsSpeedReady", m_isSpeedReady);
    Logger.recordOutput("Shooter/RPMTrimOffsetRpm", m_rpmTrimOffsetRpm);
    Logger.recordOutput("Shooter/HoodTrimOffsetDeg", m_hoodTrimOffsetDeg);
    Logger.recordOutput("Shooter/TurretTrimOffsetDeg", m_turretTrimOffsetDeg);
    Logger.recordOutput("Shooter/IsHopperEmpty", m_isHopperEmpty);
    Logger.recordOutput("Shooter/KickerHasFuel", m_kicker.hasFuel());
    Logger.recordOutput("Shooter/IsTurretServoMode", m_isTurretServoMode);
    Logger.recordOutput("Shooter/IsTurretJammed", m_isTurretJammed);
    Logger.recordOutput("Shooter/TurretStuckTimerSec", m_turretStuckTimer.get());
    Logger.recordOutput("Shooter/TurretActualDegRobot", m_turret.getAngle().in(Degrees));
    Logger.recordOutput("Shooter/TurretErrorDeg", m_lastTurretErrorRobotAngle.in(Degrees));
    Logger.recordOutput("Shooter/TargetPose2d", m_targetPoseField3d.toPose2d());

    Logger.recordOutput(
        "Shooter/TurretServoRobotHeadingDeg",
        m_turretPoseField3d != null ? getRobotHeadingForTurretServo().getDegrees() : 0.0);

    if (Constants.VERBOSE_TELEMETRY) {
      // Basic turret and robot heading
      if (m_turretPoseField3d != null) {
        Logger.recordOutput("Shooter/TurretPoseField3d", m_turretPoseField3d);
      }

      double robotHeadingDeg =
          RobotContainer.getInstance().getRobotPose2d().getRotation().getDegrees();
      Logger.recordOutput("Shooter/robotHeadingFieldDeg", robotHeadingDeg);

      Logger.recordOutput("Shooter/TurretTargetDegRobot", m_turretTargetRobotAngle.in(Degrees));

      // Distance from robot center to target (2D, Z component ignored)
      Pose2d targetPose2d = m_targetPoseField3d.toPose2d();
      Logger.recordOutput(
          "Shooter/DistanceRobotCenterToTargetDistanceMeters",
          RobotContainer.getInstance()
              .getRobotPose2d()
              .getTranslation()
              .getDistance(targetPose2d.getTranslation()));
      // Distance from turret center to target (2D, Z component ignored)
      if (m_turretPoseField3d != null) {
        Logger.recordOutput(
            "Shooter/DistanceTurretToTargetDistanceMeters",
            m_turretPoseField3d
                .toPose2d()
                .getTranslation()
                .getDistance(targetPose2d.getTranslation()));
      }

      // Firing solution telemetry
      if (m_lastFiringSolution != null) {
        FiringSolution solution = m_lastFiringSolution;
        Logger.recordOutput("Shooter/FutureRobotPose2d", solution.futurePose);
        double predictedDistanceMeters =
            RobotContainer.getInstance()
                .getRobotPose2d()
                .getTranslation()
                .getDistance(solution.futurePose.getTranslation());
        Logger.recordOutput("Shooter/FutureRobotPoseDistanceMeters", predictedDistanceMeters);
        Logger.recordOutput("Shooter/TimeOfFlightSec", solution.tofSec);
        Logger.recordOutput(
            "Shooter/DistanceFiringSolutionToTargetMeters", solution.distanceToTargetMeters);
        double hoodTargetAngleDeg =
            MathUtil.clamp(
                Radians.of(solution.hoodMechAngleRad).in(Degrees) + m_hoodTrimOffsetDeg,
                HoodConstants.HOOD_DEGREES_MIN,
                HoodConstants.HOOD_DEGREES_MAX);
        Logger.recordOutput("Shooter/HoodTargetAngleDeg", hoodTargetAngleDeg);
        m_targetHoodAngleDegEntry.set(hoodTargetAngleDeg);
        Logger.recordOutput("Shooter/HoodActualAngleDeg", m_hood.getAngle().in(Degrees));
        Logger.recordOutput("Shooter/FlyWheelTargetRPM", solution.flywheelRPM + m_rpmTrimOffsetRpm);
        Logger.recordOutput(
            "Shooter/CalculatedTurretAngleDeg", Radians.of(solution.turretAngleRad).in(Degrees));

        Translation2d[] aimRay =
            buildAimRay(
                solution.futurePose,
                solution.turretAngleRad,
                new Translation2d(TURRET_X_OFFSET_M, TURRET_Y_OFFSET_M),
                solution.distanceToTargetMeters);
        Logger.recordOutput("Shooter/CalculatedAimRay", aimRay);
      }

      // Chassis speeds and sensors
      ChassisSpeeds chassisSpeeds = RobotContainer.getInstance().getChassisSpeeds();
      Logger.recordOutput("Shooter/ChassisSpeeds.vxMps", chassisSpeeds.vxMetersPerSecond);
      Logger.recordOutput("Shooter/ChassisSpeeds.vyMps", chassisSpeeds.vyMetersPerSecond);
      Logger.recordOutput(
          "Shooter/ChassisSpeeds.omegaRadPerSec", chassisSpeeds.omegaRadiansPerSecond);

      // On-target and motor readings
      double actualRpm = m_bottomFlyWheel.getVelocity().in(RPM);
      m_rpmAverageRpm = RPM_AVERAGE_ALPHA * actualRpm + (1.0 - RPM_AVERAGE_ALPHA) * m_rpmAverageRpm;
      Logger.recordOutput("Shooter/FlyWheelActualRPM", actualRpm);
      Logger.recordOutput("Shooter/FlyWheelAverageRPM", m_rpmAverageRpm);
      Logger.recordOutput("Shooter/KickerActualRPM", m_kicker.getVelocity().in(RPM));
      Logger.recordOutput("Shooter/KickerTargetRPM", m_kicker.getTargetVelocity().in(RPM));
      Logger.recordOutput("Shooter/SpindexerActualRPM", m_spindexer.getVelocity().in(RPM));
      Logger.recordOutput("Shooter/SpindexerTargetRPM", m_spindexer.getTargetVelocity().in(RPM));
      Logger.recordOutput("Shooter/TOF/LaunchCount", m_tofLaunchCount);
      Logger.recordOutput("Shooter/TOF/ConfirmCount", m_tofConfirmCount);
      Logger.recordOutput("Shooter/TotalBallsLaunched", m_totalBallsLaunched);
    }
  }

  /**
   * Detects ball launches via CANrange falling edge (ball leaving the kicker sensor view).
   * Edge-triggered — one event per ball departure. Records the launch timestamp in {@code
   * m_tofLaunchTimestampsSec} and increments {@code m_tofLaunchCount}. Also manages hopper-empty
   * detection: if no ball has been seen for {@link #HOPPER_EMPTY_TIMEOUT_SEC} after at least one
   * was detected this shoot session, {@code m_isHopperEmpty} is set true.
   */
  private void updateTofTracking() {
    boolean hasFuelNow = m_kicker.hasFuel();

    // Track last-seen timestamp for hopper-empty detection
    if (hasFuelNow) {
      m_hopperLastSeenTimestampSec = Timer.getFPGATimestamp();
      m_hopperSeenSinceShootStart = true;
    }

    // Hopper empty: shooting active, seen at least one ball, nothing for 1+ second
    if (m_isShooting && m_hopperSeenSinceShootStart) {
      m_isHopperEmpty =
          (Timer.getFPGATimestamp() - m_hopperLastSeenTimestampSec) > HOPPER_EMPTY_TIMEOUT_SEC;
    } else if (!m_isShooting) {
      m_isHopperEmpty = false;
      m_hopperSeenSinceShootStart = false;
    }

    // TOF tracking: only when shooting and under max shot count
    if (!m_isShooting || m_tofLaunchCount >= TOF_MAX_SHOTS) {
      m_kickerHadFuelLastLoop = hasFuelNow;
      return;
    }

    // Falling edge of detection: ball just left the kicker (entered flywheel) —
    // record launch time
    if (!hasFuelNow && m_kickerHadFuelLastLoop) {
      m_tofLaunchTimestampsSec[m_tofLaunchCount] = Timer.getFPGATimestamp();
      m_tofLaunchCount++;
      m_totalBallsLaunched++;
      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput("Shooter/TOF/LaunchCount", m_tofLaunchCount);
        Logger.recordOutput("Shooter/TotalBallsLaunched", m_totalBallsLaunched);
      }
    }
    m_kickerHadFuelLastLoop = hasFuelNow;
  }

  /** True if no fuel has been detected at the kicker for {@link #HOPPER_EMPTY_TIMEOUT_SEC}. */
  public boolean isHopperEmpty() {
    return m_isHopperEmpty;
  }

  /**
   * Records the arrival time for the next unconfirmed launch, computing the empirical TOF.
   *
   * <p>Map this command to a button-box button. Press once per ball, in order, as each ball visibly
   * arrives at the target. After {@link #TOF_MAX_SHOTS} confirmations, logs average and std dev.
   * Autoshoot resets the session automatically on each trigger press.
   *
   * <p>Unconfirmed launches at trigger release are zeroed out and excluded from session stats.
   */
  public Command getTofConfirmCmd() {
    return Commands.runOnce(
            () -> {
              if (m_tofLaunchCount == 0) {
                if (Constants.VERBOSE_TELEMETRY) {
                  Logger.recordOutput("Shooter/TOF/ConfirmStatus", "No launches recorded");
                }
                return;
              }
              if (m_tofConfirmCount >= m_tofLaunchCount) {
                if (Constants.VERBOSE_TELEMETRY) {
                  Logger.recordOutput(
                      "Shooter/TOF/ConfirmStatus", "All launches already confirmed");
                }
                return;
              }
              double arrivalTimeSec = Timer.getFPGATimestamp();
              m_tofMeasuredSec[m_tofConfirmCount] =
                  arrivalTimeSec - m_tofLaunchTimestampsSec[m_tofConfirmCount];
              if (Constants.VERBOSE_TELEMETRY) {
                Logger.recordOutput(
                    "Shooter/TOF/Ball" + (m_tofConfirmCount + 1) + "Sec",
                    m_tofMeasuredSec[m_tofConfirmCount]);
                Logger.recordOutput(
                    "Shooter/TOF/ConfirmStatus", "Ball " + (m_tofConfirmCount + 1) + " confirmed");
              }
              m_tofConfirmCount++;
              if (m_tofConfirmCount >= TOF_MAX_SHOTS) {
                logTofResults();
              }
            })
        .withName("TOF/Confirm");
  }

  /** Logs all per-ball TOFs, average, and std dev. Called on session end or after 5 confirms. */
  private void logTofResults() {
    if (!Constants.VERBOSE_TELEMETRY) return;
    int validCount = 0;
    double sum = 0.0;
    for (int i = 0; i < m_tofConfirmCount; i++) {
      double tof = m_tofMeasuredSec[i];
      if (tof > 0) {
        Logger.recordOutput("Shooter/TOF/Ball" + (i + 1) + "Sec", tof);
        sum += tof;
        validCount++;
      }
    }
    if (validCount == 0) return;
    double avgSec = sum / validCount;
    Logger.recordOutput("Shooter/TOF/ShotCount", validCount);
    Logger.recordOutput("Shooter/TOF/AverageSec", avgSec);
    if (validCount > 1) {
      double variance = 0.0;
      for (int i = 0; i < m_tofConfirmCount; i++) {
        if (m_tofMeasuredSec[i] > 0) {
          double diff = m_tofMeasuredSec[i] - avgSec;
          variance += diff * diff;
        }
      }
      Logger.recordOutput("Shooter/TOF/StdDevSec", Math.sqrt(variance / validCount));
    }
  }

  public Command getSetTargetCmd(ShootingTarget target) {
    Command c =
        Commands.runOnce(
                () -> {
                  this.setTargetPose(target.displayName, target.getFieldPose3d());
                })
            .ignoringDisable(true);
    c.setName("Target/" + target.displayName);

    return c;
  }

  public Command getHoodAngleTestCmd(String name, Angle a) {
    Command c = m_hood.setAngle(a);
    c.setName("Hood/" + name);

    return c;
  }

  public Command getFlyWheelRPMCmd(String name, double rpm) {
    if (rpm == 0.0) {
      Command c = m_bottomFlyWheel.set(0);
      c.setName("BottomFlyWheel/" + name);
      return c;
    }

    Command c = m_bottomFlyWheel.setVelocity(RPM.of(rpm));
    c.setName("BottomFlyWheel/" + name);

    return c;
  }

  public Command getFlyWheelRPMCmd(String name) {

    Command c = m_bottomFlyWheel.setVelocity();
    c.setName("BottomFlyWheel/" + name);

    return c;
  }

  public Command getKickerRPMCmd(String name) {

    Command c = m_kicker.setVelocity();
    c.setName("Kicker/" + name);

    return c;
  }

  public Command getSpindexerRPMCmd(String name) {

    Command c = m_spindexer.setVelocity();
    c.setName("Spindexer/" + name);

    return c;
  }

  // ── Turret calibration (button box) ──────────────────────────────────────

  /** Drives turret to hard stop and zeroes encoder. Requires enable. */
  public Command getTurretHomeAndCalibrateCmd() {
    return m_turret.homeAndCalibrate();
  }

  /** Zeroes turret encoder at current position. Runs while disabled. */
  public Command getTurretCalibrateInPlaceCmd() {
    return m_turret.calibrateInPlace().ignoringDisable(true);
  }

  // ── Hood calibration (button box) ────────────────────────────────────────

  /** Drives hood to hard stop and zeroes encoder. Requires enable. */
  public Command getHoodHomeAndCalibrateCmd() {
    return m_hood.homeAndCalibrate();
  }

  /** Zeroes hood encoder at current position. Runs while disabled. */
  public Command getHoodCalibrateInPlaceCmd() {
    return m_hood.calibrateInPlace().ignoringDisable(true);
  }

  /** Zeroes hood encoder to HOOD_DEGREES_MIN. Runs while disabled. */
  public Command getHoodZeroCmd() {
    return m_hood.getZeroHoodAngleCmd();
  }

  // ── Spindexer (button box) ────────────────────────────────────────────────

  /** Run spindexer forward to feed toward shooter. */
  public Command getSpindexerRunCmd() {
    return m_spindexer.getRunCmd();
  }

  /** Stop spindexer. */
  public Command getSpindexerStopCmd() {
    return m_spindexer.getStopCmd();
  }

  /** Run spindexer in reverse to clear jams. */
  public Command getSpindexerReverseCmd() {
    return m_spindexer.getReverseCmd();
  }

  public void launchFuelSim() {
    Pose3d turretPoseFR = m_turretPoseField3d; // turret pose field relative
    // Ball launches perpendicular to hood surface; convert mechanical angle to
    // launch angle
    double hoodAngleRad = hoodMechToLaunchAngle(m_lastFiringSolution.hoodMechAngleRad);
    double vel = rpmToExitVelocity(m_lastFiringSolution.flywheelRPM);
    double yawRad = turretPoseFR.getRotation().getZ();
    // double yawRad = m_lastFiringSolution.turretAngleRad;

    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("Shooter/LaunchFuelYawFieldDeg", Radians.of(yawRad).in(Degrees));
    }

    Translation3d initialPosition = turretPoseFR.getTranslation();
    FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, hoodAngleRad, yawRad));
    // launchVel(vel, 0, Degrees.of(-45).in(Radians)));
  }

  // NOTE: lightweight helper removed; keep 'launchVel' which includes robot
  // velocity.

  /**
   * Computes the initial field-relative velocity of a launched fuel game piece.
   *
   * <p>The resulting velocity includes:
   *
   * <ul>
   *   <li>The velocity imparted by the shooter, based on exit speed, hood angle, and turret yaw
   *       (all field-relative).
   *   <li>The current translational velocity of the robot, converted to field-relative coordinates
   *       and added to the shot.
   * </ul>
   *
   * <p>Coordinate conventions:
   *
   * <ul>
   *   <li>X/Y are field-relative (WPILib standard: +X forward, +Y left).
   *   <li>Z is vertical and is affected only by the hood angle.
   *   <li>{@code turretYawRad} is a field-relative yaw angle.
   * </ul>
   *
   * <p>Robot angular velocity (omega) is intentionally ignored; only linear chassis velocity is
   * applied to the fuel at launch.
   *
   * @param vel_ms Shooter exit velocity in meters per second
   * @param launchAngleAboveHorizontalRad Launch angle in radians above horizontal (0 = horizontal
   *     shot, π/2 = straight up). Use {@link #hoodMechToLaunchAngle} to convert from a mechanical
   *     hood angle.
   * @param turretYawRad Turret yaw angle in radians, field-relative
   * @return Field-relative velocity vector of the fuel at launch
   */
  private Translation3d launchVel(
      double vel_ms, double launchAngleAboveHorizontalRad, double turretYawRad) {
    // Shooter imparted, field relative velocity
    double horizontal = vel_ms * Math.cos(launchAngleAboveHorizontalRad);
    double vx = horizontal * Math.cos(turretYawRad);
    double vy = horizontal * Math.sin(turretYawRad);
    double vz = vel_ms * Math.sin(launchAngleAboveHorizontalRad);

    // Robot field velocity
    ChassisSpeeds speeds = RobotContainer.getInstance().getChassisSpeeds();
    double theta = RobotContainer.getInstance().getRobotPose2d().getRotation().getRadians();
    double robotVxField =
        speeds.vxMetersPerSecond * Math.cos(theta) - speeds.vyMetersPerSecond * Math.sin(theta);
    double robotVyField =
        speeds.vxMetersPerSecond * Math.sin(theta) + speeds.vyMetersPerSecond * Math.cos(theta);

    // Add robot motion
    vx += robotVxField;
    vy += robotVyField;

    return new Translation3d(vx, vy, vz);
  }

  /**
   * Returns a command that manually controls the turret rotation at a constant rate.
   *
   * <p>The {@code inputSupplier} should return a value from -1.0 to +1.0 (like a joystick axis).
   * Positive values rotate the turret to the right (positive degrees), negative to the left. The
   * turret angle is clamped to [{@link Turret#TURRET_DEGREES_MIN}, {@link
   * Turret#TURRET_DEGREES_MAX}].
   *
   * <p>While this command is running, auto-targeting is paused. When it ends, auto-targeting
   * resumes automatically.
   *
   * @param inputSupplier supplier returning -1.0 to +1.0 (e.g., joystick axis)
   * @return a command that manually controls the turret
   */
  public Command getManualTurretCmd(Supplier<Double> inputSupplier) {
    double[] angleDeg = {0.0};
    double[] lastTimeSec = {0.0};

    return m_turret
        .setAngle(
            () -> {
              double nowSec = Timer.getFPGATimestamp();
              double dtSec = nowSec - lastTimeSec[0];
              lastTimeSec[0] = nowSec;
              // Skip first tick to avoid a large dt jump
              if (dtSec > 0.1) dtSec = 0.0;

              angleDeg[0] += inputSupplier.get() * MANUAL_TURRET_RATE_DEG_PER_SEC * dtSec;
              angleDeg[0] =
                  MathUtil.clamp(angleDeg[0], Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX);
              return Degrees.of(angleDeg[0]);
            })
        .beforeStarting(
            () -> {
              angleDeg[0] = m_turret.getAngle().in(Degrees);
              lastTimeSec[0] = Timer.getFPGATimestamp();
            });
  }

  /**
   * Returns a command that manually controls the hood pitch at a constant rate.
   *
   * <p>The {@code inputSupplier} should return a value from -1.0 to +1.0 (like a joystick axis).
   * Positive values raise the hood (increase angle), negative values lower it. The hood angle is
   * clamped to [{@link HoodConstants#HOOD_DEGREES_MIN}, {@link HoodConstants#HOOD_DEGREES_MAX}].
   *
   * <p>While this command is running, auto-aiming is paused. When it ends, auto-aiming resumes
   * automatically.
   *
   * @param inputSupplier supplier returning -1.0 to +1.0 (e.g., joystick axis)
   * @return a command that manually controls the hood
   */
  public Command getManualHoodCmd(Supplier<Double> inputSupplier) {
    double[] angleDeg = {0.0};
    double[] lastTimeSec = {0.0};

    return m_hood
        .setAngle(
            () -> {
              double nowSec = Timer.getFPGATimestamp();
              double dtSec = nowSec - lastTimeSec[0];
              lastTimeSec[0] = nowSec;
              if (dtSec > 0.1) dtSec = 0.0;

              angleDeg[0] += inputSupplier.get() * MANUAL_HOOD_RATE_DEG_PER_SEC * dtSec;
              angleDeg[0] =
                  MathUtil.clamp(
                      angleDeg[0], HoodConstants.HOOD_DEGREES_MIN, HoodConstants.HOOD_DEGREES_MAX);
              return Degrees.of(angleDeg[0]);
            })
        .beforeStarting(
            () -> {
              angleDeg[0] = m_hood.getAngle().in(Degrees);
              lastTimeSec[0] = Timer.getFPGATimestamp();
            });
  }

  /**
   * Returns a command that manually controls the flywheel speed at a constant rate.
   *
   * <p>The {@code inputSupplier} should return a value from -1.0 to +1.0 (like a joystick axis).
   * Positive values increase RPM, negative values decrease it. The RPM is clamped to [0, {@link
   * #MANUAL_FLYWHEEL_MAX_RPM}].
   *
   * <p>While this command is running, the automatic flywheel speed control is paused. When it ends,
   * auto control resumes automatically.
   *
   * @param inputSupplier supplier returning -1.0 to +1.0 (e.g., joystick axis)
   * @return a command that manually controls the flywheel
   */
  public Command getManualFlywheelCmd(Supplier<Double> inputSupplier) {
    double[] rpmValue = {0.0};
    double[] lastTimeSec = {0.0};

    return m_bottomFlyWheel
        .setVelocity(
            () -> {
              double nowSec = Timer.getFPGATimestamp();
              double dtSec = nowSec - lastTimeSec[0];
              lastTimeSec[0] = nowSec;
              if (dtSec > 0.1) dtSec = 0.0;

              rpmValue[0] += inputSupplier.get() * MANUAL_FLYWHEEL_RATE_RPM_PER_SEC * dtSec;
              rpmValue[0] = MathUtil.clamp(rpmValue[0], 0, MANUAL_FLYWHEEL_MAX_RPM);
              return RPM.of(rpmValue[0]);
            })
        .beforeStarting(
            () -> {
              rpmValue[0] = m_bottomFlyWheel.getVelocity().in(RPM);
              lastTimeSec[0] = Timer.getFPGATimestamp();
            });
  }

  /**
   * PREDICTIVE TARGETING mode — triggered by right trigger.
   *
   * <p>Overrides the hood and flywheel idle defaults with targeting-derived setpoints from the
   * firing solution, then gates the kicker and spindexer on {@link #isReady()}. When the trigger is
   * released, hood and flywheel default commands (idle) resume and kicker/spindexer default
   * commands (stop) resume automatically.
   *
   * <p>In SIM mode, also spawns FuelSim projectiles on the {@link #AUTO_SHOOT_INTERVAL_SEC} timer
   * once the shooter is ready (same behaviour as the original sim-only implementation).
   */
  public Command getAutoShootCmd() {
    // Build the feed command: in SIM also launch fuel projectiles on a timer.
    final double[] lastLaunchTimeSec = {0.0};
    Command feedCmd =
        m_kicker.setVelocity(RPM.of(KICKER_SHOOT_RPM)).alongWith(m_spindexer.getRunCmd());
    if (Constants.currentMode == Constants.Mode.SIM) {
      feedCmd =
          feedCmd.alongWith(
              Commands.run(
                  () -> {
                    double nowSec = Timer.getFPGATimestamp();
                    if (nowSec - lastLaunchTimeSec[0] >= AUTO_SHOOT_INTERVAL_SEC) {
                      launchFuelSim();
                      lastLaunchTimeSec[0] = nowSec;
                    }
                  }));
    }

    return Commands.parallel(
            // Override turret default: preempts any locked turret command (e.g.
            // TurretCenter)
            // so the turret resumes predictive targeting as soon as AutoShoot is triggered.
            getPredictiveTargetingCommand(),
            // Override hood idle default: command targeting angle from firing solution
            m_hood.setAngle(
                () -> {
                  if (m_lastFiringSolution != null) {
                    double trimmedDeg =
                        MathUtil.clamp(
                            Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees)
                                + m_hoodTrimOffsetDeg,
                            HoodConstants.HOOD_DEGREES_MIN,
                            HoodConstants.HOOD_DEGREES_MAX);
                    return Degrees.of(trimmedDeg);
                  }
                  return Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
                }),
            // Override flywheel idle default: command targeting RPM from firing solution
            m_bottomFlyWheel.setVelocity(
                () ->
                    m_lastFiringSolution != null
                        ? RPM.of(m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm)
                        : RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm)),
            // Kicker + spindexer (+ sim fuel launch): wait until ready, feed until turret
            // infeasible, then loop back to wait for ready again.
            Commands.sequence(
                    Commands.waitUntil(() -> m_isShooterReady),
                    feedCmd.until(
                        () ->
                            m_lastFiringSolution == null
                                || !m_lastFiringSolution.isTurretFeasible()))
                .repeatedly())
        .beforeStarting(
            () -> {
              m_isShooting = true;
              // Reset jam detection state for new shooting session
              m_isTurretJammed = false;
              m_turretStuckTimer.reset();
              // Reset TOF state for new shooting session
              m_tofLaunchCount = 0;
              m_tofConfirmCount = 0;
              m_kickerHadFuelLastLoop = false;
              m_hopperSeenSinceShootStart = false;
              m_isHopperEmpty = false;
              Arrays.fill(m_tofLaunchTimestampsSec, 0.0);
              Arrays.fill(m_tofMeasuredSec, -1.0);
            })
        .finallyDo(
            () -> {
              m_isShooting = false;
              // Zero out any launched-but-unconfirmed shots so they are excluded from stats
              while (m_tofConfirmCount < m_tofLaunchCount) {
                m_tofMeasuredSec[m_tofConfirmCount] = 0.0;
                m_tofConfirmCount++;
              }
              logTofResults();
            })
        .withName("AutoShoot");
  }

  /**
   * TURRET-SERVO FALLBACK mode — activated by holding the A button.
   *
   * <p>Locks the turret to 0° robot-relative and relies on the drive chassis rotating to aim. Hood
   * and flywheel continue to use the firing-solution LUT for distance-based setpoints. {@link
   * #m_isTurretServoMode} is set so that {@link #updateReadiness()} checks drive heading alignment
   * (within 2°) instead of turret angle error. Once aligned, kicker and spindexer fire.
   *
   * <p>Use when the turret is mechanically or electrically inoperable. Pair with {@link
   * DriveCommands#joystickDriveAtAngle} passing {@link #getRobotHeadingForTurretServo()} as the
   * heading supplier so the drive actively steers to the required heading.
   */
  public Command getTurretServoShootCmd() {
    final double[] lastLaunchTimeSec = {0.0};
    Command feedCmd =
        m_kicker.setVelocity(RPM.of(KICKER_SHOOT_RPM)).alongWith(m_spindexer.getRunCmd());
    if (Constants.currentMode == Constants.Mode.SIM) {
      feedCmd =
          feedCmd.alongWith(
              Commands.run(
                  () -> {
                    double nowSec = Timer.getFPGATimestamp();
                    if (nowSec - lastLaunchTimeSec[0] >= AUTO_SHOOT_INTERVAL_SEC) {
                      launchFuelSim();
                      lastLaunchTimeSec[0] = nowSec;
                    }
                  }));
    }

    return Commands.parallel(
            // Turret locked to 0° — robot rotation provides the aim
            m_turret.setAngle(Degrees.of(0)),
            // Hood from firing solution (same as AutoShoot)
            m_hood.setAngle(
                () -> {
                  if (m_lastFiringSolution != null) {
                    double trimmedDeg =
                        MathUtil.clamp(
                            Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees)
                                + m_hoodTrimOffsetDeg,
                            HoodConstants.HOOD_DEGREES_MIN,
                            HoodConstants.HOOD_DEGREES_MAX);
                    return Degrees.of(trimmedDeg);
                  }
                  return Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
                }),
            // Flywheel from firing solution (same as AutoShoot)
            m_bottomFlyWheel.setVelocity(
                () ->
                    m_lastFiringSolution != null
                        ? RPM.of(m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm)
                        : RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm)),
            // Feed when heading-aligned; stop if firing solution becomes infeasible
            Commands.sequence(
                    Commands.waitUntil(() -> m_isShooterReady),
                    feedCmd.until(
                        () ->
                            m_lastFiringSolution == null
                                || !m_lastFiringSolution.isTurretFeasible()))
                .repeatedly())
        .beforeStarting(
            () -> {
              m_isTurretServoMode = true;
              m_isShooting = true;
              // Reset jam detection state for new shooting session
              m_isTurretJammed = false;
              m_turretStuckTimer.reset();
              m_tofLaunchCount = 0;
              m_tofConfirmCount = 0;
              m_kickerHadFuelLastLoop = false;
              m_hopperSeenSinceShootStart = false;
              m_isHopperEmpty = false;
              Arrays.fill(m_tofLaunchTimestampsSec, 0.0);
              Arrays.fill(m_tofMeasuredSec, -1.0);
            })
        .finallyDo(
            () -> {
              m_isTurretServoMode = false;
              m_isShooting = false;
              while (m_tofConfirmCount < m_tofLaunchCount) {
                m_tofMeasuredSec[m_tofConfirmCount] = 0.0;
                m_tofConfirmCount++;
              }
              logTofResults();
            })
        .withName("TurretServoShoot");
  }

  public boolean isShooting() {
    return m_isShooting;
  }

  /**
   * Returns true if the shooter is fully ready to fire (turret, hood, flywheel, and speed are all
   * within tolerance and debounced).
   */
  public boolean isReady() {
    return m_isShooterReady;
  }

  /**
   * MANUAL mode — triggered by right bumper.
   *
   * <p>Runs the kicker and spindexer at shooting speed. Assumes the turret, hood, and flywheel are
   * already at the correct setpoints (positioned by the operator or a prior auto-aim sequence).
   * Does not require Hood or BottomFlyWheel — whatever command is running them continues
   * uninterrupted. When the bumper is released, the kicker and spindexer default commands (stop)
   * resume.
   */
  public Command getManualShootCmd() {
    return m_kicker
        .setVelocity(RPM.of(KICKER_SHOOT_RPM))
        .alongWith(m_spindexer.getRunCmd())
        .withName("ManualShoot");
  }

  /**
   * FIRE-NOW mode — fires the kicker and spindexer immediately without waiting for {@link
   * #isReady()}, while holding the hood and flywheel at their current firing-solution setpoints.
   *
   * <p>Use this instead of {@link #getManualShootCmd()} when {@link #getAutoShootCmd()} is active,
   * so that triggering the kicker does not cancel the auto-shoot parallel group and drop the hood
   * and flywheel back to their idle defaults.
   *
   * <p>When this command ends, all four subsystems release and their default commands resume.
   */
  public Command getAutoFireNowCmd() {
    return Commands.parallel(
            // Hold hood at firing-solution angle (same logic as AutoShoot)
            m_hood.setAngle(
                () -> {
                  if (m_lastFiringSolution != null) {
                    double trimmedDeg =
                        MathUtil.clamp(
                            Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees)
                                + m_hoodTrimOffsetDeg,
                            HoodConstants.HOOD_DEGREES_MIN,
                            HoodConstants.HOOD_DEGREES_MAX);
                    return Degrees.of(trimmedDeg);
                  }
                  return Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
                }),
            // Hold flywheel at firing-solution RPM (same logic as AutoShoot)
            m_bottomFlyWheel.setVelocity(
                () ->
                    m_lastFiringSolution != null
                        ? RPM.of(m_lastFiringSolution.flywheelRPM + m_rpmTrimOffsetRpm)
                        : RPM.of(IDLE_FLYWHEEL_RPM + m_rpmTrimOffsetRpm)),
            m_kicker.setVelocity(RPM.of(KICKER_SHOOT_RPM)),
            m_spindexer.getRunCmd())
        .withName("AutoFireNow");
  }

  /**
   * OFF mode — stops all shooter mechanisms. Not driver-bound; triggered from Elastic/Shuffleboard.
   * Uses {@code setDutyCycle(0.0)} for positional mechanisms (Hood, Turret) since they have no raw
   * {@code set()} equivalent.
   */
  public Command getOffCmd() {
    return Commands.parallel(
            m_turret.setDutyCycle(0.0),
            m_hood.setDutyCycle(0.0),
            m_bottomFlyWheel.set(0),
            m_kicker.stop(),
            m_spindexer.stop())
        .withName("Shooter/Off");
  }

  /**
   * Returns a command that retracts the hood to its minimum angle. While this command is running,
   * the hood's default (auto-aiming) command is paused. When this command ends, the default command
   * resumes automatically.
   *
   * @param reason Human-readable description of why the hood is being retracted (logged to
   *     "Shooter/Hood/RetractReason").
   */
  public Command getRetractHoodCmd(String reason) {
    return m_hood
        .setAngle(Degrees.of(HoodConstants.HOOD_DEGREES_MIN))
        .beforeStarting(
            () -> {
              Logger.recordOutput("Shooter/Hood/IsRetracted", true);
              Logger.recordOutput("Shooter/Hood/RetractReason", reason);
            })
        .finallyDo(
            interrupted -> {
              Logger.recordOutput("Shooter/Hood/IsRetracted", false);
            });
  }

  // ── Fixed-distance shots ──────────────────────────────────────────────────

  /**
   * Locks the turret to 0° robot-relative, commands the hood to a fixed angle, and spins the
   * flywheel to a fixed RPM. Holds indefinitely — cancel via Elastic to return to normal auto mode.
   *
   * <p>Use when pose estimation is unavailable. Pair with {@link #getAutoShootCmd()} to fire.
   *
   * @param hoodAngle fixed hood angle
   * @param rpm fixed flywheel speed
   */
  public Command getFixedShotCmd(Angle hoodAngle, double rpm) {
    return Commands.parallel(
            m_turret.setAngle(Degrees.of(0)),
            m_hood.setAngle(hoodAngle),
            m_bottomFlyWheel.setVelocity(() -> RPM.of(rpm + m_rpmTrimOffsetRpm)))
        .withName("FixedShot");
  }

  /** Fixed shot from 1 m. Tune {@code FIXED_SHOT_1M_HOOD_DEG} and {@code FIXED_SHOT_1M_RPM}. */
  public Command getFixedShot1mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_1M_HOOD_DEG), FIXED_SHOT_1M_RPM)
        .withName("FixedShot/1m");
  }

  /** Fixed shot from 2 m. Tune {@code FIXED_SHOT_2M_HOOD_DEG} and {@code FIXED_SHOT_2M_RPM}. */
  public Command getFixedShot2mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_2M_HOOD_DEG), FIXED_SHOT_2M_RPM)
        .withName("FixedShot/2m");
  }

  /** Fixed shot from 3 m. Tune {@code FIXED_SHOT_3M_HOOD_DEG} and {@code FIXED_SHOT_3M_RPM}. */
  public Command getFixedShot3mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_3M_HOOD_DEG), FIXED_SHOT_3M_RPM)
        .withName("FixedShot/3m");
  }

  /** Fixed shot from 4 m. Tune {@code FIXED_SHOT_4M_HOOD_DEG} and {@code FIXED_SHOT_4M_RPM}. */
  public Command getFixedShot4mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_4M_HOOD_DEG), FIXED_SHOT_4M_RPM)
        .withName("FixedShot/4m");
  }

  /** Fixed shot from 5 m. Tune {@code FIXED_SHOT_5M_HOOD_DEG} and {@code FIXED_SHOT_5M_RPM}. */
  public Command getFixedShot5mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_5M_HOOD_DEG), FIXED_SHOT_5M_RPM)
        .withName("FixedShot/5m");
  }

  /** Fixed shot from 6 m. Tune {@code FIXED_SHOT_6M_HOOD_DEG} and {@code FIXED_SHOT_6M_RPM}. */
  public Command getFixedShot6mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_6M_HOOD_DEG), FIXED_SHOT_6M_RPM)
        .withName("FixedShot/6m");
  }

  /** Fixed shot from 7 m. Tune {@code FIXED_SHOT_7M_HOOD_DEG} and {@code FIXED_SHOT_7M_RPM}. */
  public Command getFixedShot7mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_7M_HOOD_DEG), FIXED_SHOT_7M_RPM)
        .withName("FixedShot/7m");
  }

  /** Fixed shot from 8 m. Tune {@code FIXED_SHOT_8M_HOOD_DEG} and {@code FIXED_SHOT_8M_RPM}. */
  public Command getFixedShot8mCmd() {
    return getFixedShotCmd(Degrees.of(FIXED_SHOT_8M_HOOD_DEG), FIXED_SHOT_8M_RPM)
        .withName("FixedShot/8m");
  }

  // ── In-match RPM trim ────────────────────────────────────────────────────

  /**
   * Adds {@code deltaRpm} to the persistent RPM trim offset. Applied to every flywheel velocity
   * calculation (auto and fixed). Runs as a one-shot — each Elastic click accumulates.
   *
   * @param deltaRpm RPM to add (positive or negative)
   */
  public Command getRPMTrimCmd(double deltaRpm) {
    return Commands.runOnce(() -> m_rpmTrimOffsetRpm += deltaRpm)
        .withName("RPMTrim/" + (deltaRpm > 0 ? "+" : "") + (int) deltaRpm)
        .ignoringDisable(true);
  }

  /** Resets the RPM trim offset to zero. */
  public Command getRPMTrimResetCmd() {
    return Commands.runOnce(() -> m_rpmTrimOffsetRpm = 0.0)
        .withName("RPMTrim/Reset")
        .ignoringDisable(m_hopperSeenSinceShootStart);
  }

  /**
   * Adds {@code deltaDeg} to the persistent hood trim offset. Applied to every dynamic hood
   * setpoint (auto and auto-shoot). Clamped so the commanded angle stays within the hood's valid
   * range regardless of how many times it is pressed.
   *
   * @param deltaDeg degrees to add (positive = more open, negative = more retracted)
   */
  public Command getHoodTrimCmd(double deltaDeg) {
    double range = HoodConstants.HOOD_DEGREES_MAX - HoodConstants.HOOD_DEGREES_MIN;
    return Commands.runOnce(
            () ->
                m_hoodTrimOffsetDeg = MathUtil.clamp(m_hoodTrimOffsetDeg + deltaDeg, -range, range))
        .withName("HoodTrim/" + (deltaDeg > 0 ? "+" : "") + (int) deltaDeg);
  }

  /** Resets the hood trim offset to zero. */
  public Command getHoodTrimResetCmd() {
    return Commands.runOnce(() -> m_hoodTrimOffsetDeg = 0.0).withName("HoodTrim/Reset");
  }

  // ── In-match turret trim ──────────────────────────────────────────────────

  /**
   * Adds {@code deltaDeg} to the persistent turret trim offset. Applied to every dynamic turret
   * targeting calculation. Clamped to ±TURRET_TRIM_MAX_DEG.
   */
  public Command getTurretTrimCmd(double deltaDeg) {
    return Commands.runOnce(
            () ->
                m_turretTrimOffsetDeg =
                    MathUtil.clamp(
                        m_turretTrimOffsetDeg + deltaDeg,
                        -TURRET_TRIM_MAX_DEG,
                        TURRET_TRIM_MAX_DEG))
        .withName("TurretTrim/" + (deltaDeg > 0 ? "+" : "") + (int) deltaDeg);
  }

  /** Resets the turret trim offset to zero. */
  public Command getTurretTrimResetCmd() {
    return Commands.runOnce(() -> m_turretTrimOffsetDeg = 0.0).withName("TurretTrim/Reset");
  }

  // ── Turret manual controls ────────────────────────────────────────────────

  /**
   * Captures the current turret angle, adds {@code offset}, clamps to soft limits, then holds there
   * indefinitely. Cancel via Elastic to return to auto-targeting.
   *
   * <p>Pressing again while active captures the new physical angle, so presses accumulate naturally
   * (e.g. two +5° presses hold at +10°).
   *
   * @param offset angle to add to the current turret position (positive = clockwise)
   */
  public Command getTurretNudgeCmd(Angle offset) {
    Angle[] targetAngle = {Degrees.of(0)};
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    targetAngle[0] =
                        Degrees.of(
                            MathUtil.clamp(
                                m_turret.getAngle().in(Degrees) + offset.in(Degrees),
                                Turret.TURRET_DEGREES_MIN,
                                Turret.TURRET_DEGREES_MAX))),
            m_turret.setAngle(() -> targetAngle[0]))
        .withName("Turret/Nudge");
  }

  /**
   * Snaps the turret to 0° robot-relative and holds. Cancel via Elastic to return to *
   * auto-targeting.
   */
  public Command getTurretCenterCmd() {
    return m_turret.setAngle(Degrees.of(0)).withName("Turret/Center");
  }
}
