// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class SpeedNanny extends TimedSubsystem {
  private static final double LOOP_PERIOD_SEC =
      0.02; // 20 ms loop period, used to calculate epsilon for speed checks
  // based on max
  // acceleration

  private final double m_kMaxAcceleration = 50.0;
  private final double m_kMaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
  // top speed
  // Increase rotational capability: target is faster turning. Values expressed in
  // rotations/sec and rotations/sec^2 then converted to rad/s and rad/s^2.
  // Previously: 1.0 rps (≈6.28 rad/s) and 1.5 rps/s (≈9.42 rad/s^2).
  // New defaults provide more responsive rotation on the field.
  private double m_kMaxAngularRate =
      RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 2.0 rotations per second
  // (~12.57 rad/s)
  // max angular velocity

  private double m_kMaxAngularAcceleration =
      RotationsPerSecondPerSecond.of(2.0).in(RadiansPerSecondPerSecond); // 3.0
  // rps/s
  // (~18.85
  // rad/s^2)

  private double m_speedLimit = m_kMaxSpeed;
  private double m_accelerationLimit = m_kMaxAcceleration;
  private double m_angularRateLimit = m_kMaxAngularRate;
  private double m_angularAccelLimit = m_kMaxAngularAcceleration;

  /** Creates a new SpeedNanny. */
  public SpeedNanny() {}

  @Override
  protected void periodicImpl() {
    // This method will be called once per scheduler run
    // Calculate new speed limit based on arm angle & arm extension
    m_speedLimit = m_kMaxSpeed;
    m_angularRateLimit = m_kMaxAngularRate;
    m_angularAccelLimit = m_kMaxAngularAcceleration;
    m_accelerationLimit = m_kMaxAcceleration;

    if (RobotContainer.getInstance().m_shooter.isShooting()) {
      m_speedLimit = m_kMaxSpeed * 0.3; // Reduce max speed to make it easier to aim while shooting
      m_angularRateLimit =
          m_kMaxAngularRate
              * 0.25; // Reduce max angular rate to make it easier to aim while shooting
      // m_accelerationLimit = 1.5; // Reduce acceleration to make it easier to aim
      // while shooting
      // m_angularAccelLimit =
      // 1.5; // Reduce angular acceleration to make it easier to aim while shooting
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

    if (Constants.VERBOSE_TELEMETRY) {
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
    }

    return isUnderLimit;
  }
}
