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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // TODO update the location of the turret relative to the robot
  // Turret offsets from robot center (meters)
  private final double TURRET_X_OFFSET_M = 0.12; // meters
  private final double TURRET_Y_OFFSET_M = -0.12; // meters
  private final double TURRET_Z_OFFSET_M = 0.35; // meters

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
  private String m_targetName = "Blue Hub";

  // Current target pose (field-relative)
  // Target pose in field-relative coordinates (Pose3d)
  private Pose3d m_targetPoseField3d = new Pose3d(4.5, 4.1, 1.83, new Rotation3d());

  // Last calculated turret values / telemetry
  private Angle m_turretTargetRobotAngle = Degrees.of(0);
  private Angle m_lastTurretErrorRobotAngle = Degrees.of(0);
  // Shuffleboard entry: target hood angle in degrees
  private DoubleEntry m_targetHoodAngleDegEntry = null; // Shuffleboard entry (Deg)
  // Last hood angle and flywheel RPM are tracked in telemetry via Logger; remove
  // redundant fields
  private FiringSolution m_lastFiringSolution = null;
  private boolean m_isShooting = false;

  // Readiness thresholds (allowable error)
  private static final double TURRET_ANGLE_TOLERANCE_DEG = 2.0; // degrees
  private static final double HOOD_ANGLE_TOLERANCE_DEG = 2.0; // degrees
  private static final double BOTTOM_FLYWHEEL_RPM_TOLERANCE = 150.0; // RPM
  // Debounce durations for readiness signals (seconds)
  private static final double TURRET_READY_DEBOUNCE_SEC = 0.10; // 100 ms
  private static final double HOOD_READY_DEBOUNCE_SEC = 0.10; // 100 ms
  private static final double BOTTOM_FLYWHEEL_READY_DEBOUNCE_SEC = 0.10; // 100 ms
  private static final double AUTO_SHOOT_INTERVAL_SEC = 0.5; // 2 balls per second

  // Manual control rates (tunable — degrees or RPM per second)
  private static final double MANUAL_TURRET_RATE_DEG_PER_SEC = 180.0;
  private static final double MANUAL_HOOD_RATE_DEG_PER_SEC = 90.0;
  private static final double MANUAL_FLYWHEEL_RATE_RPM_PER_SEC = 1000.0;
  private static final double MANUAL_FLYWHEEL_MAX_RPM = 6000.0;

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
    m_turret = new Turret(false);
    m_hood = new Hood(isReal);
    m_bottomFlyWheel = new BottomFlyWheel(isReal);
    m_kicker = new Kicker(isReal);
    m_spindexer = new Spindexer(isReal);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_targetHoodAngleDegEntry = table.getDoubleTopic("TargetHoodAngleDeg").getEntry(0.0);
    m_targetHoodAngleDegEntry.set(HoodConstants.HOOD_DEGREES_MIN);

    // Set default command to aim turret at target (normal targeting)
    m_turret.setDefaultCommand(getPredictiveTargetingCommand());
    m_bottomFlyWheel.setDefaultCommand(getDynamicFlywheelCommand());
    m_hood.setDefaultCommand(getDynamicHoodCommand());

    // Add Shuffleboard commands to switch between targeting modes
    Shuffleboard.getTab("Shooter").add(getEnableNormalTargetingModeCmd());
    Shuffleboard.getTab("Shooter").add(getEnablePredictiveTargetingModeCmd());

    // Shuffleboard commands for targeting (BLUE ALLIANCE)
    Shuffleboard.getTab("Shooter")
        .add(this.getSetTargetCmd("Blue Hub", new Pose3d(4.5, 4.1, 1.83, new Rotation3d())));

    Shuffleboard.getTab("Shooter")
        .add(
            this.getSetTargetCmd(
                "Blue Neutral Target Left", new Pose3d(0.85, 1.5, 0.0, new Rotation3d())));
    Shuffleboard.getTab("Shooter")
        .add(
            this.getSetTargetCmd(
                "Blue Neutral Target Right", new Pose3d(0.85, 6.5, 0.0, new Rotation3d())));

    Shuffleboard.getTab("Shooter")
        .add(
            this.getSetTargetCmd(
                "Blue Opposition Target Left", new Pose3d(2.0, 2.0, 0.0, new Rotation3d())));
    Shuffleboard.getTab("Shooter")
        .add(
            this.getSetTargetCmd(
                "Blue Opposition Target Right", new Pose3d(2.0, 6.0, 0.0, new Rotation3d())));

    // Shuffleboard commands for targeting (RED ALLIANCE)
    Shuffleboard.getTab("Shooter")
        .add(this.getSetTargetCmd("Red Hub", new Pose3d(12.0, 4.1, 1.83, new Rotation3d())));

    // Shuffleboard.getTab("Shooter")
    // .add(
    // this.getSetTargetCmd(
    // "Blue Neutral Target Left", new Pose3d(0.85, 1.5, 0.0, new Rotation3d())));
    // Shuffleboard.getTab("Shooter")
    // .add(
    // this.getSetTargetCmd(
    // "Blue Neutral Target Right", new Pose3d(0.85, 6.5, 0.0, new Rotation3d())));

    // Shuffleboard.getTab("Shooter")
    // .add(
    // this.getSetTargetCmd(
    // "Blue Opposition Target Left", new Pose3d(2.0, 2.0, 0.0, new Rotation3d())));
    // Shuffleboard.getTab("Shooter")
    // .add(
    // this.getSetTargetCmd(
    // "Blue Opposition Target Right", new Pose3d(2.0, 6.0, 0.0, new
    // Rotation3d())));

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 0", Degrees.of(0.0)));

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 20", Degrees.of(20.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 30", Degrees.of(30.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 35", Degrees.of(35.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 40", Degrees.of(40.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 45", Degrees.of(45.0)));
    Shuffleboard.getTab("Hood").add(m_hood.getZeroHoodAngleCmd());

    Shuffleboard.getTab("BottomFlyWheel").add(this.getFlyWheelRPMCmd("Bottom Flywheel 0 RPM", 0.0));
    Shuffleboard.getTab("BottomFlyWheel")
        .add(this.getFlyWheelRPMCmd("Bottom Flywheel 1000 RPM", 1000.0));
    Shuffleboard.getTab("BottomFlyWheel")
        .add(this.getFlyWheelRPMCmd("Bottom Flywheel 2000 RPM", 2000.0));
    Shuffleboard.getTab("BottomFlyWheel").add(this.getFlyWheelRPMCmd("Dynamic"));
    Shuffleboard.getTab("BottomFlyWheel")
        .add(this.getFlyWheelRPMCmd("Bottom Flywheel 3000 RPM", 3000.0));
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
            MathUtil.clamp(a.in(Degrees), Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX));

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
        MathUtil.clamp(turretTargetDeg, Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX);

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

  private Command getDynamicHoodCommand() {
    Command c =
        m_hood.setAngle(
            () -> {
              if (m_lastFiringSolution != null && m_isTurretActive) {
                return Radians.of(m_lastFiringSolution.hoodMechAngleRad);
              }
              return Degrees.of(0);
            });
    c.addRequirements(m_hood);
    return c;
  }

  private Command getDynamicFlywheelCommand() {
    return m_bottomFlyWheel.setVelocity(
        () -> {
          if (m_lastFiringSolution != null && m_isTurretActive) {
            return RPM.of(m_lastFiringSolution.flywheelRPM);
          }
          return RPM.of(2000);
        });
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
  public void periodic() {
    // This method will be called once per scheduler run

    updateTurretPose();
    // Update firing solution and telemetry (compute early so commands/targeting can
    // use it)
    updateFiringSolution();
    // Updates target data (kept lightweight) and then publish telemetry last
    updateTargetData();
    // Update readiness flags (moved to its own method for clarity)
    updateReadiness();

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

  public FiringSolution calculateFiringSolutionLUT(
      Pose2d currentPose,
      ChassisSpeeds chassisSpeeds,
      Translation2d targetFieldPos,
      Translation2d shooterOffsetRobot) {

    // --- Initial guess for TOF ---
    double tofSec = 1.0;

    Pose2d futurePose;
    Translation2d shooterFieldPos;
    double distanceMeters;

    // --- Iterate once or twice ---
    for (int i = 0; i < 2; i++) {
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
    double hoodMechAngleRad =
        Degrees.of(m_bottomFlyWheel.calculateAngle(distanceMeters)).in(Radians);
    double flywheelRPM = m_bottomFlyWheel.calculateRPM(distanceMeters);

    // --- Turret angle (robot-relative) ---
    Translation2d delta = targetFieldPos.minus(shooterFieldPos);

    double rawTurretAngleRad =
        Math.atan2(delta.getY(), delta.getX()) - currentPose.getRotation().getRadians();

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

      // Ball launches perpendicular to hood surface, so convert mech angle to launch angle
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

    double rawTurretAngleRad =
        Math.atan2(delta.getY(), delta.getX()) - currentPose.getRotation().getRadians();

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
    final double dt = 0.005;
    int steps = (int) Math.ceil(totalTime / dt);

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
      // Turret ready when the turret error is within tolerance (debounced)
      boolean rawTurretReady =
          m_lastFiringSolution.isTurretFeasible()
              && Math.abs(m_lastTurretErrorRobotAngle.in(Degrees)) < TURRET_ANGLE_TOLERANCE_DEG;
      m_isTurretReady = m_turretReadyDebouncer.calculate(rawTurretReady);

      // Bottom flywheel readiness: compare current RPM to target RPM
      double currentBottomRpm = m_bottomFlyWheel.getVelocity().in(RPM);
      double targetBottomRpm = m_lastFiringSolution.flywheelRPM;
      boolean rawBottomReady =
          Math.abs(currentBottomRpm - targetBottomRpm) < BOTTOM_FLYWHEEL_RPM_TOLERANCE;
      m_isBottomFlyWheelReady = m_bottomFlywheelReadyDebouncer.calculate(rawBottomReady);

      // Hood readiness: compare current hood angle to target hood angle (degrees,
      // debounced)
      double currentHoodDeg = m_hood.getAngle().in(Degrees);
      double targetHoodDeg = Radians.of(m_lastFiringSolution.hoodMechAngleRad).in(Degrees);
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
    // Basic turret and robot heading
    if (m_turretPoseField3d != null) {
      Logger.recordOutput("Shooter/TurretPoseField3d", m_turretPoseField3d);
    }

    double robotHeadingDeg =
        RobotContainer.getInstance().getRobotPose2d().getRotation().getDegrees();
    Logger.recordOutput("Shooter/robotHeadingFieldDeg", robotHeadingDeg);

    Logger.recordOutput("Shooter/TurretActualDegRobot", m_turret.getAngle().in(Degrees));
    Logger.recordOutput("Shooter/TurretTargetDegRobot", m_turretTargetRobotAngle.in(Degrees));
    Logger.recordOutput("Shooter/TurretErrorDeg", m_lastTurretErrorRobotAngle.in(Degrees));

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
          "Shooter/HoodTargetAngleDeg", Radians.of(solution.hoodMechAngleRad).in(Degrees));
      Logger.recordOutput("Shooter/HoodActualAngleDeg", m_hood.getAngle().in(Degrees));
      Logger.recordOutput("Shooter/FlyWheelTargetRPM", solution.flywheelRPM);
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
    Logger.recordOutput("Shooter/FlyWheelActualRPM", m_bottomFlyWheel.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/KickerActualRPM", m_kicker.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/KickerTargetRPM", m_kicker.getTargetVelocity().in(RPM));
    Logger.recordOutput("Shooter/SpindexerActualRPM", m_spindexer.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/SpindexerTargetRPM", m_spindexer.getTargetVelocity().in(RPM));
    // Readiness indicators
    Logger.recordOutput("Shooter/IsTurretReady", m_isTurretReady);
    Logger.recordOutput("Shooter/IsBottomFlyWheelReady", m_isBottomFlyWheelReady);
    Logger.recordOutput("Shooter/IsShooterReady", m_isShooterReady);
    Logger.recordOutput("Shooter/IsHoodReady", m_isHoodReady);
    Logger.recordOutput("Shooter/IsSpeedReady", m_isSpeedReady);
  }

  public Command getSetTargetCmd(String name, Pose3d target) {
    Command c =
        Commands.runOnce(
                () -> {
                  this.setTargetPose(name, target);
                })
            .ignoringDisable(true);
    c.setName("Target/" + name);

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

  public void launchFuelSim() {
    Pose3d turretPoseFR = m_turretPoseField3d; // turret pose field relative
    // Ball launches perpendicular to hood surface; convert mechanical angle to launch angle
    double hoodAngleRad = hoodMechToLaunchAngle(m_lastFiringSolution.hoodMechAngleRad);
    double vel = rpmToExitVelocity(m_lastFiringSolution.flywheelRPM);
    double yawRad = turretPoseFR.getRotation().getZ();
    // double yawRad = m_lastFiringSolution.turretAngleRad;

    Logger.recordOutput("Shooter/LaunchFuelYawFieldDeg", Radians.of(yawRad).in(Degrees));

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

  public Command getAutoShootCmd() {
    final double[] lastLaunchTimeSec = {0.0};
    Command c =
        Commands.run(
                () -> {
                  if (m_isShooterReady) {
                    double nowSec = Timer.getFPGATimestamp();
                    if (nowSec - lastLaunchTimeSec[0] >= AUTO_SHOOT_INTERVAL_SEC) {
                      launchFuelSim();
                      lastLaunchTimeSec[0] = nowSec;
                    }
                  }
                })
            .beforeStarting(() -> m_isShooting = true)
            .finallyDo(() -> m_isShooting = false);
    c.setName("AutoShoot");
    return c;
  }

  public boolean isShooting() {
    return m_isShooting;
  }

  /**
   * Returns a command that retracts the hood to its minimum angle. While this command is running,
   * the hood's default (auto-aiming) command is paused. When this command ends, the default command
   * resumes automatically.
   */
  public Command getRetractHoodCmd() {
    return m_hood.setAngle(Degrees.of(HoodConstants.HOOD_DEGREES_MIN));
  }
}
