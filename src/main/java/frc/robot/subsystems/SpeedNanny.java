// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class SpeedNanny extends SubsystemBase {
  private static final double LOOP_PERIOD_SEC =
      0.02; // 20 ms loop period, used to calculate epsilon for speed checks based on max
  // acceleration

  private final double m_kMaxAcceleration = 15.0;
  private final double m_kMaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
  // top speed
  private double m_kMaxAngularRate =
      RotationsPerSecond.of(0.50).in(RadiansPerSecond); // 3/4 of a rotation per second
  // max angular velocity
  private double m_kMaxAngularAcceleration =
      RotationsPerSecondPerSecond.of(1.5).in(RadiansPerSecondPerSecond); // rad/s^2

  private double m_speedLimit = m_kMaxSpeed;
  private double m_accelerationLimit = m_kMaxAcceleration;
  private double m_angularRateLimit = m_kMaxAngularRate;
  private double m_angularAccelLimit = m_kMaxAngularAcceleration;

  /** Creates a new SpeedNanny. */
  public SpeedNanny() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Calculate new speed limit based on arm angle & arm extension
    m_speedLimit = m_kMaxSpeed;
    m_angularRateLimit = m_kMaxAngularRate;
    m_angularAccelLimit = m_kMaxAngularAcceleration;
    m_accelerationLimit = m_kMaxAcceleration;

    if (RobotContainer.getInstance().m_shooter.isShooting()) {
      m_speedLimit = m_kMaxSpeed * 0.5; // Reduce max speed to make it easier to aim while shooting
      m_angularRateLimit =
          m_kMaxAngularRate
              * 0.1; // Reduce max angular rate to make it easier to aim while shooting
      m_accelerationLimit = 1.5; // Reduce acceleration to make it easier to aim while shooting
      m_angularAccelLimit =
          1.5; // Reduce angular acceleration to make it easier to aim while shooting
    }
  }

  public double getSpeedLimit() {
    return m_speedLimit;
  }

  public double getAccelerationLimit() {
    return m_accelerationLimit;
  }

  public double getAngularRateLimit() {
    return m_angularRateLimit;
  }

  public double getAngularAccelerationLimit() {
    return m_angularAccelLimit;
  }

  /**
   * Returns true if the robot's translational speed and angular rate are at or below their current
   * limits.
   */
  public boolean isUnderSpeedLimit() {
    ChassisSpeeds speeds = RobotContainer.getInstance().getChassisSpeeds();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double robotAngularRateRadPerSec = Math.abs(speeds.omegaRadiansPerSecond);

    double speedEpsilonMps = m_kMaxAcceleration * LOOP_PERIOD_SEC;
    double angularRateEpsilonRadPerSec = m_kMaxAngularAcceleration * LOOP_PERIOD_SEC;

    boolean speedCheck = robotSpeedMps <= m_speedLimit + speedEpsilonMps;
    boolean angularRateCheck =
        robotAngularRateRadPerSec <= m_angularRateLimit + angularRateEpsilonRadPerSec;
    boolean isUnderLimit = speedCheck && angularRateCheck;

    Logger.recordOutput("SpeedNanny/CurrentSpeed_mps", robotSpeedMps);
    Logger.recordOutput("SpeedNanny/SpeedLimit_mps", m_speedLimit);
    Logger.recordOutput("SpeedNanny/AccelerationLimit_mps2", m_accelerationLimit);
    Logger.recordOutput("SpeedNanny/SpeedWithEpsilon_mps", m_speedLimit + speedEpsilonMps);
    Logger.recordOutput("SpeedNanny/CurrentAngularRate_radPerSec", robotAngularRateRadPerSec);
    Logger.recordOutput("SpeedNanny/AngularRateLimit_radPerSec", m_angularRateLimit);
    Logger.recordOutput(
        "SpeedNanny/AngularRateWithEpsilon_radPerSec",
        m_angularRateLimit + angularRateEpsilonRadPerSec);
    Logger.recordOutput("SpeedNanny/SpeedCheck", speedCheck);
    Logger.recordOutput("SpeedNanny/AngularRateCheck", angularRateCheck);
    Logger.recordOutput("SpeedNanny/IsUnderSpeedLimit", isUnderLimit);

    return isUnderLimit;
  }
}
