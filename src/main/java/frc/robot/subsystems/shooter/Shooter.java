// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // TODO update the location of the turret relative to the robot
  private final double TURRET_X_OFFSET = 0.25; // meters
  private final double TURRET_Y_OFFSET = -0.25; // meters
  private final double TURRET_Z_OFFSET = 0.25; // meters

  private Pose3d m_turrPose3dFR = null; // Field Relative turret pose

  private final Turret m_turret = new Turret();
  private PIDController m_turretPIDController = new PIDController(0.0035, 0, 0);

  private final HoodOld m_hood = new HoodOld();
  private final BottomFlyWheel m_bottomFlyWheel = new BottomFlyWheel();

  private final Kicker m_kicker = new Kicker();

  private final Spindexer m_spindexer = new Spindexer();

  private boolean m_isTurretActive = true;

  private String m_targetName = "Blue Hub";

  private Pose3d m_targetPose = new Pose3d(4.5, 4.1, 1.83, new Rotation3d());
  private Angle m_turretTarget = Degrees.of(0);
  private Angle m_lastTurretError = Degrees.of(0);
  private DoubleEntry m_TargetHoodAngleDegrees = null;

  /** Creates a new Shooter. */
  public Shooter() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_TargetHoodAngleDegrees = table.getDoubleTopic("TargetHoodAngleDegrees").getEntry(0.0);
    m_TargetHoodAngleDegrees.set(Hood.HOOD_DEGREES_MIN);

    // turretPIDController.enableContinuousInput(-180, 180);
    // turret.setAngle(() -> getTurretTargetAngle());

    // m_turret.setDefaultCommand(
    //     m_turret.setDutyCycle(
    //         () -> {
    //           if (m_isTurretActive) {
    //             return m_turretPIDController.calculate(
    //                 -getTurretTargetAngleError2().in(Degrees), 0);
    //           }
    //           return 0.0;
    //         }));

    m_turret.setDefaultCommand(
        m_turret.setAngle(
            () -> {
              if (m_isTurretActive) {
                return getTurretTargetAngle();
              }
              return Degrees.of(0);
            }));

    // Add Shuffleboard commands for targeting
    Shuffleboard.getTab("Shooter")
        .add(this.getSetTargetCmd("Blue Hub", new Pose3d(4.5, 4.1, 1.83, new Rotation3d())));
    Shuffleboard.getTab("Shooter")
        .add(this.getSetTargetCmd("Red Hub", new Pose3d(12.0, 4.1, 1.83, new Rotation3d())));

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

  private Angle getTurretTargetAngleError2() {
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
    double error = turretTarget - turretHeading;

    // clamp error to prevent extreme values
    error = MathUtil.clamp(error, -20, 20);

    Angle a = Degrees.of(error);

    if (turretHeading > Turret.TURRET_DEGREES_MAX && error > 0) {
      a = Degrees.of(0);
    } else if (turretHeading < Turret.TURRET_DEGREES_MIN && error < 0) {
      a = Degrees.of(0);
    }

    m_lastTurretError = a;

    return a;
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
    updateTargetData();

    double robotHeading = RobotContainer.getInstance().getRobotPose2d().getRotation().getDegrees();
    Logger.recordOutput("Shooter/TurretCurrentDegrees", m_turret.getAngle().in(Degrees));
    Logger.recordOutput("Shooter/robotHeading", robotHeading);
    Logger.recordOutput("Shooter/TurretTargetDegrees", m_turretTarget);
    Logger.recordOutput("Shooter/TurretErrorDegrees", m_lastTurretError.in(Degrees));
    Logger.recordOutput(
        "Shooter/AimLine",
        new Translation2d[] {
          m_turrPose3dFR.toPose2d().getTranslation(),
          m_targetPose.getTranslation().toTranslation2d()
        });

    Pose2d futureRobotPose2d =
        predictRobotPose(
            RobotContainer.getInstance().getRobotPose2d(),
            RobotContainer.getInstance().getChassisSpeeds(),
            0.25);
    Logger.recordOutput("Shooter/FutureRobotPose2d", futureRobotPose2d);

    boolean onTarget = Math.abs(m_lastTurretError.in(Degrees)) < 1.0;
    Logger.recordOutput("Shooter/TurretOnTarget", onTarget);
    Logger.recordOutput("Shooter/FlyWheelRPM", m_bottomFlyWheel.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/KickerRPM", m_kicker.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/SpindexerRPM", m_spindexer.getVelocity().in(RPM));
  }

  public Pose2d predictRobotPose(Pose2d start, ChassisSpeeds speeds, double totalTime) {
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

  private void updateTargetData() {
    // Updates target data for logging
    Pose2d robotPose2d = getTurretPose3dFieldRelative().toPose2d();
    Translation2d toTarget =
        m_targetPose.toPose2d().getTranslation().minus(robotPose2d.getTranslation());
    double distanceToTarget = toTarget.getNorm();
    Logger.recordOutput("Shooter/DistanceToTarget", distanceToTarget);
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
    Logger.recordOutput("Shooter/TurretPose", turretPosed3d);

    m_turrPose3dFR = turretPosed3d;
  }

  private void setTargetName(String name) {
    m_targetName = name;
    Logger.recordOutput("Shooter/TargetName", m_targetName);
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
}
