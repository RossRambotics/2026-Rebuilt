// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final Turret turret = new Turret();
  private PIDController turretPIDController = new PIDController(0.001, 0, 0);
  private boolean isTurretActive = true;
  private DoubleEntry m_currentTurretDegrees = null;
  private DoubleEntry m_currentRobotDegrees = null;
  private DoubleEntry m_currentTurretTarget = null;
  private DoubleEntry m_currentTurretError = null;

  /** Creates a new Shooter. */
  public Shooter() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");

    m_currentTurretDegrees = table.getDoubleTopic("Current Turret Degrees").getEntry(0);
    m_currentTurretDegrees.set(0);
    m_currentRobotDegrees = table.getDoubleTopic("Current Robot Degrees").getEntry(0);
    m_currentRobotDegrees.set(0);
    m_currentTurretTarget = table.getDoubleTopic("Current Turret Target Degrees").getEntry(0);
    m_currentTurretTarget.set(0);
    m_currentTurretError = table.getDoubleTopic("Current Turret Error Degrees").getEntry(0);
    m_currentTurretError.set(0);

    // turretPIDController.enableContinuousInput(-180, 180);
    // turret.setAngle(() -> getTurretTargetAngle());

    turret.setDefaultCommand(
        turret.setDutyCycle(
            () -> {
              if (isTurretActive) {
                return turretPIDController.calculate(-getTurretTargetAngleError2().in(Degrees), 0);
              }
              return 0.0;
            }));
  }

  public Rotation2d getAngleToTarget(Pose2d robotPose, Pose2d targetPose) {

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

  private Angle getTurretTargetAngleError2() {
    Pose2d robotPose = RobotContainer.getInstance().getRobotPose2d();
    Pose2d targetPose =
        new Pose2d(4.6, 4.05, new Rotation2d()); // Example blue goal target at (4.6,4.05)
    Rotation2d angleToTarget = getAngleToTarget(robotPose, targetPose);

    double robotHeading = robotPose.getRotation().getDegrees();
    double turretTarget = angleToTarget.getDegrees();
    double turretHeading = turret.getAngle().in(Degrees);
    double error = turretTarget - turretHeading;
    Angle a = Degrees.of(error);

    if (turretHeading > 170 && error > 0) a = Degrees.of(0);
    else if (turretHeading < -170 && error < 0) a = Degrees.of(0);

    m_currentTurretDegrees.set(turret.getAngle().in(Degrees));
    m_currentRobotDegrees.set(robotHeading);
    m_currentTurretTarget.set(turretTarget);
    m_currentTurretError.set(a.in(Degrees));

    return a;
  }

  private Angle getTurretTargetAngleError() {
    double robotHeading = RobotContainer.getInstance().getRobotPose2d().getRotation().getDegrees();
    double turretTarget = -robotHeading;
    double turretHeading = turret.getAngle().in(Degrees);
    double error = turretTarget - turretHeading;
    Angle a = Degrees.of(error);

    if (turretHeading > 170 && error > 0) a = Degrees.of(0);
    else if (turretHeading < -170 && error < 0) a = Degrees.of(0);

    m_currentTurretDegrees.set(turret.getAngle().in(Degrees));
    m_currentRobotDegrees.set(robotHeading);
    m_currentTurretTarget.set(turretTarget);
    m_currentTurretError.set(a.in(Degrees));

    return a;
  }

  // TODO update the location of the turret relative to the robot
  private final double TURRET_X_OFFSET = 0.25; // meters
  private final double TURRET_Y_OFFSET = -0.25; // meters
  private final double TURRET_Z_OFFSET = 0.25; // meters
  private Pose3d m_turrPose3dFR = null; // Field Relative turret pose

  public Pose3d getTurretPose3dFieldRelative() {
    return m_turrPose3dFR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose2d = RobotContainer.getInstance().getRobotPose2d();
    Pose2d turretPose2d =
        new Pose2d(
            robotPose2d.getTranslation().getX() + TURRET_X_OFFSET,
            robotPose2d.getTranslation().getY() + TURRET_Y_OFFSET,
            Rotation2d.fromDegrees(turret.getAngle().in(Degrees)));
    turretPose2d =
        turretPose2d.rotateAround(
            new Translation2d(
                robotPose2d.getTranslation().getX(), robotPose2d.getTranslation().getY()),
            robotPose2d.getRotation()); // turret offset
    Pose3d turretPosed3d =
        new Pose3d(turretPose2d).plus(new Transform3d(0.0, 0.0, TURRET_Z_OFFSET, new Rotation3d()));
    Logger.recordOutput("TurretPose", turretPosed3d);
    m_turrPose3dFR = turretPosed3d;
  }
}
