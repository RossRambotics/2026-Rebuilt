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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // TODO update the location of the turret relative to the robot
  private final double TURRET_X_OFFSET = 0.25; // meters
  private final double TURRET_Y_OFFSET = -0.25; // meters
  private final double TURRET_Z_OFFSET = 0.55; // meters

  private Pose3d m_turrPose3dFR = null; // Field Relative turret pose

  // Turret subsystem (controls yaw of the shooter)
  private final Turret m_turret = new Turret();

  // Hood and flywheel subsystems
  private final HoodOld m_hood = new HoodOld();
  private final BottomFlyWheel m_bottomFlyWheel = new BottomFlyWheel();

  // Kicker and spindexer (intake/feeds)
  private final Kicker m_kicker = new Kicker();
  private final Spindexer m_spindexer = new Spindexer();

  // Whether turret auto-targeting is active
  private boolean m_isTurretActive = true;

  // Human-readable name of current target
  private String m_targetName = "Blue Hub";

  // Current target pose (field-relative)
  private Pose3d m_targetPose = new Pose3d(4.5, 4.1, 1.83, new Rotation3d());

  // Last calculated turret values / telemetry
  private Angle m_turretTarget = Degrees.of(0);
  private Angle m_lastTurretError = Degrees.of(0);
  private DoubleEntry m_TargetHoodAngleDegrees = null; // Shuffleboard entry
  // Last hood angle and flywheel RPM are tracked in telemetry via Logger; remove redundant fields
  private FiringSolution m_lastFiringSolution = null;

  /** Creates a new Shooter. */
  public Shooter() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_TargetHoodAngleDegrees = table.getDoubleTopic("TargetHoodAngleDegrees").getEntry(0.0);
    m_TargetHoodAngleDegrees.set(Hood.HOOD_DEGREES_MIN);

    // Set default command to aim turret at target (normal targeting)
    m_turret.setDefaultCommand(getNormalTargetingCommand());

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
    //     .add(
    //         this.getSetTargetCmd(
    //             "Blue Neutral Target Left", new Pose3d(0.85, 1.5, 0.0, new Rotation3d())));
    // Shuffleboard.getTab("Shooter")
    //     .add(
    //         this.getSetTargetCmd(
    //             "Blue Neutral Target Right", new Pose3d(0.85, 6.5, 0.0, new Rotation3d())));

    // Shuffleboard.getTab("Shooter")
    //     .add(
    //         this.getSetTargetCmd(
    //             "Blue Opposition Target Left", new Pose3d(2.0, 2.0, 0.0, new Rotation3d())));
    // Shuffleboard.getTab("Shooter")
    //     .add(
    //         this.getSetTargetCmd(
    //             "Blue Opposition Target Right", new Pose3d(2.0, 6.0, 0.0, new Rotation3d())));

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 0", Degrees.of(0.0)));

    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 20", Degrees.of(20.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 30", Degrees.of(30.0)));
    Shuffleboard.getTab("Hood").add(this.getHoodAngleTestCmd("Hood Angle 40", Degrees.of(40.0)));
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
    m_targetPose = targetPose;
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

  private Angle getTurrentTargetAnglePredicted() {
    if (m_lastFiringSolution == null) {
      return getTurretTargetAngle();
    }

    Angle a = Radians.of(m_lastFiringSolution.turretAngleRad);
    a =
        Degrees.of(
            MathUtil.clamp(a.in(Degrees), Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX));

    // TODO need to add a flag to if clamped, so that we known there is no turret based solution

    return a;
  }

  private Angle getTurretTargetAngle() {
    Pose2d robotPose = RobotContainer.getInstance().getRobotPose2d();

    // use the turrets location, but the robots heading
    // this is because we need to adjust the turret based on its angle being robot relative.
    Pose2d temp = getTurretPose3dFieldRelative().toPose2d();
    Pose2d turretPose = new Pose2d(temp.getX(), temp.getY(), robotPose.getRotation());

    Rotation2d angleToTarget = getAngleToTarget(turretPose, m_targetPose.toPose2d());

    // clamp the turret target to the rotation available for the turret
    double turretTarget = angleToTarget.getDegrees();
    turretTarget =
        MathUtil.clamp(turretTarget, Turret.TURRET_DEGREES_MIN, Turret.TURRET_DEGREES_MAX);

    m_turretTarget = Degrees.of(turretTarget);

    double turretHeading = m_turret.getAngle().in(Degrees);
    m_lastTurretError = Degrees.of(turretTarget - turretHeading);

    return m_turretTarget;
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
            return getTurrentTargetAnglePredicted();
          }
          return Degrees.of(0);
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
    return m_turrPose3dFR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateTurretPose();
    // Update firing solution and telemetry (compute early so commands/targeting can use it)
    updateFiringSolution();
    // Updates target data (kept lightweight) and then publish telemetry last
    updateTargetData();
    updateTelemetry();
  }

  /**
   * Compute the firing solution (time-of-flight, turret & hood angles, rpm) and publish telemetry.
   * Kept outside of periodic to make the periodic loop cleaner.
   */
  private void updateFiringSolution() {
    // Shooter offset relative to robot center
    Translation2d shooterOffsetRobot = new Translation2d(TURRET_X_OFFSET, TURRET_Y_OFFSET);
    FiringSolution solution =
        calculateFiringSolution(
            RobotContainer.getInstance().getRobotPose2d(),
            RobotContainer.getInstance().getChassisSpeeds(),
            m_targetPose.toPose2d().getTranslation(),
            shooterOffsetRobot);

    // store computed solution for consumers; telemetry will be published in updateTelemetry()
    m_lastFiringSolution = solution;
  }

  public FiringSolution calculateFiringSolution(
      Pose2d currentPose,
      ChassisSpeeds chassisSpeeds,
      Translation2d targetFieldPos,
      Translation2d shooterOffsetRobot) {
    // --- Initial guess for TOF ---
    double tof = 1.0; // seconds (good initial guess)

    Pose2d futurePose;
    Translation2d shooterFieldPos;
    double distance;

    // --- Iterate once or twice ---
    for (int i = 0; i < 2; i++) {
      futurePose = predictPoseLinear(currentPose, chassisSpeeds, tof);

      shooterFieldPos =
          futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

      distance = shooterFieldPos.getDistance(targetFieldPos);

      double hoodRad = Degrees.of(m_bottomFlyWheel.calculateAngle(distance)).in(Radians);
      double rpm = m_bottomFlyWheel.calculateRPM(distance);

      double exitVelocity = rpmToExitVelocity(rpm);

      double heightDiff = m_targetPose.getZ() - TURRET_Z_OFFSET;

      // convert hoodRad to angle above horizontal
      hoodRad = Math.PI / 2 - hoodRad;
      double tofVac =
          (exitVelocity * Math.sin(hoodRad)
                  + Math.sqrt(
                      Math.pow(exitVelocity * Math.sin(hoodRad), 2) + 2 * 9.81 * heightDiff))
              / 9.81;

      tof = tofVac * 1.12 * 0.6; // drag correction
    }

    // --- Final future pose ---
    futurePose = predictPoseLinear(currentPose, chassisSpeeds, tof);

    shooterFieldPos =
        futurePose.getTranslation().plus(shooterOffsetRobot.rotateBy(futurePose.getRotation()));

    // --- LUT lookup ---
    distance = shooterFieldPos.getDistance(targetFieldPos);
    double hoodAngle = Degrees.of(m_bottomFlyWheel.calculateAngle(distance)).in(Radians);
    double rpm = m_bottomFlyWheel.calculateRPM(distance);

    // --- Velocity compensation for turret ---
    Translation2d delta = targetFieldPos.minus(shooterFieldPos);

    double turretAngle =
        Math.atan2(delta.getY(), delta.getX()) - futurePose.getRotation().getRadians();

    return new FiringSolution(turretAngle, hoodAngle, rpm, futurePose, tof, distance);
  }

  public record FiringSolution(
      double turretAngleRad,
      double hoodAngleRad,
      double flywheelRPM,
      Pose2d futurePose,
      double tof,
      double distanceToTarget) {}

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
    // Kept lightweight: currently no per-loop side effects needed here. Telemetry is published in
    // updateTelemetry() which runs last in the loop.
  }

  private void updateTurretPose() {
    // Updates the turret pose in field relative coordinates
    Pose2d robotPose2d = RobotContainer.getInstance().getRobotPose2d();
    Pose2d turretPose2d =
        new Pose2d(
            robotPose2d.getTranslation().getX() + TURRET_X_OFFSET,
            robotPose2d.getTranslation().getY() + TURRET_Y_OFFSET,
            Rotation2d.fromDegrees(m_turret.getAngle().in(Degrees)));
    turretPose2d =
        turretPose2d.rotateAround(
            new Translation2d(
                robotPose2d.getTranslation().getX(), robotPose2d.getTranslation().getY()),
            robotPose2d.getRotation()); // turret offset
    Pose3d turretPosed3d =
        new Pose3d(turretPose2d).plus(new Transform3d(0.0, 0.0, TURRET_Z_OFFSET, new Rotation3d()));

    m_turrPose3dFR = turretPosed3d;
  }

  private void setTargetName(String name) {
    m_targetName = name;
    Logger.recordOutput("Shooter/TargetName", m_targetName);
  }

  /** Centralized telemetry publish. Called once at the end of periodic(). */
  private void updateTelemetry() {
    // Basic turret and robot heading
    if (m_turrPose3dFR != null) {
      Logger.recordOutput("Shooter/TurretPose", m_turrPose3dFR);
    }

    double robotHeading = RobotContainer.getInstance().getRobotPose2d().getRotation().getDegrees();
    Logger.recordOutput("Shooter/robotHeading", robotHeading);

    Logger.recordOutput("Shooter/TurretCurrentDegrees", m_turret.getAngle().in(Degrees));
    Logger.recordOutput("Shooter/TurretTargetDegrees", m_turretTarget);
    Logger.recordOutput("Shooter/TurretErrorDegrees", m_lastTurretError.in(Degrees));

    // Firing solution telemetry
    if (m_lastFiringSolution != null) {
      FiringSolution solution = m_lastFiringSolution;
      Logger.recordOutput("Shooter/FutureRobotPose2d", solution.futurePose);
      double predictedDistance =
          RobotContainer.getInstance()
              .getRobotPose2d()
              .getTranslation()
              .getDistance(solution.futurePose.getTranslation());
      Logger.recordOutput("Shooter/FutureRobotPoseDistance", predictedDistance);
      Logger.recordOutput("Shooter/TimeOfFlight", solution.tof);
      Logger.recordOutput(
          "Shooter/CalculatedHoodAngleDegrees", Radians.of(solution.hoodAngleRad).in(Degrees));
      Logger.recordOutput("Shooter/CalculatedFlyWheelRPM", solution.flywheelRPM);
      Logger.recordOutput(
          "Shooter/CalculatedTurretAngleDegrees", Radians.of(solution.turretAngleRad).in(Degrees));

      Translation2d[] aimRay =
          buildAimRay(
              solution.futurePose,
              solution.turretAngleRad,
              new Translation2d(TURRET_X_OFFSET, TURRET_Y_OFFSET),
              solution.distanceToTarget);
      Logger.recordOutput("Shooter/CalculatedAimRay", aimRay);
    }

    // Chassis speeds and sensors
    ChassisSpeeds chassisSpeeds = RobotContainer.getInstance().getChassisSpeeds();
    Logger.recordOutput("Shooter/ChassisSpeeds.vx", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Shooter/ChassisSpeeds.vy", chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Shooter/ChassisSpeeds.omega", chassisSpeeds.omegaRadiansPerSecond);

    // On-target and motor readings
    boolean onTarget = Math.abs(m_lastTurretError.in(Degrees)) < 1.0;
    Logger.recordOutput("Shooter/TurretOnTarget", onTarget);
    Logger.recordOutput("Shooter/FlyWheelRPM", m_bottomFlyWheel.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/KickerRPM", m_kicker.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/SpindexerRPM", m_spindexer.getVelocity().in(RPM));
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
    Command c = m_hood.getSetGoalCommand(a);
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
    Pose3d turretPoseFR = m_turrPose3dFR; // turret pose field relative
    double hoodAngleRad =
        Math.PI / 2 - m_lastFiringSolution.hoodAngleRad; // switch the angle to be above horizontal
    double vel = rpmToExitVelocity(m_lastFiringSolution.flywheelRPM);
    double yawRad = turretPoseFR.getRotation().getZ();
    // double yawRad = m_lastFiringSolution.turretAngleRad;

    Logger.recordOutput("Shooter/LaunchFuelYawDeg", Radians.of(yawRad).in(Degrees));

    Translation3d initialPosition = turretPoseFR.getTranslation();
    FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, hoodAngleRad, yawRad));
    // launchVel(vel, 0, Degrees.of(-45).in(Radians)));
  }

  // NOTE: lightweight helper removed; keep 'launchVel' which includes robot velocity.

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
   * @param hoodAngleRad Hood angle in radians, measured above horizontal
   * @param turretYawRad Turret yaw angle in radians, field-relative
   * @return Field-relative velocity vector of the fuel at launch
   */
  private Translation3d launchVel(double vel_ms, double hoodAngleRad, double turretYawRad) {
    // Shooter imparted, field relative velocity
    double horizontal = vel_ms * Math.cos(hoodAngleRad);
    double vx = horizontal * Math.cos(turretYawRad);
    double vy = horizontal * Math.sin(turretYawRad);
    double vz = vel_ms * Math.sin(hoodAngleRad);

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
}
