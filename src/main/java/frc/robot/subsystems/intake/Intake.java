// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Intake coordinator subsystem. Owns {@link IntakeRoller} and {@link IntakeDeployArm} and exposes
 * compound and standalone commands for operator use.
 */
public class Intake extends SubsystemBase {

  private final boolean m_isReal;
  private final IntakeRoller m_roller;
  private final IntakeDeployArm m_deployArm;

  /** Creates a new Intake coordinator. */
  public Intake(boolean isReal) {
    m_isReal = isReal;
    m_roller = new IntakeRoller(isReal);
    m_deployArm = new IntakeDeployArm(isReal);
  }

  // ---------------------------------------------------------------------------
  // Compound commands
  // ---------------------------------------------------------------------------

  /**
   * Deploy the intake arm and run the roller.
   *
   * <p>Phase 1: position-control the arm to the deploy angle while starting the roller. Ends when
   * the arm reaches the deploy angle (within tolerance) or after a safety timeout.
   *
   * <p>Phase 2: release arm to coast (duty-cycle 0) while the roller continues running. This allows
   * the arm to fold back freely if struck by another robot.
   */
  public Command getDeployCmd() {
    return Commands.sequence(
            // Phase 1 — move arm to deploy while starting roller
            Commands.parallel(
                    m_deployArm.setAngle(Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG)),
                    m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM)))
                .until(
                    () ->
                        m_deployArm.isNearAngle(
                            Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG),
                            Degrees.of(IntakeConstants.AT_ANGLE_TOLERANCE_DEG)))
                .withTimeout(0.5),
            // Phase 2 — coast arm, keep roller running
            Commands.parallel(
                m_deployArm.setDutyCycle(0.0),
                m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM))))
        .withName("Intake/Deploy");
  }

  /**
   * Retract the intake arm and stop the roller.
   *
   * <p>Stops the roller first, then position-controls the arm back to the retracted angle and holds
   * it there.
   */
  public Command getRetractCmd() {
    return Commands.sequence(
            Commands.runOnce(() -> {}, m_roller), // interrupt any running roller command
            m_roller.stop().withTimeout(0.1), // brief stop to settle roller
            m_deployArm.setAngle(Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG)))
        .withName("Intake/Retract");
  }

  // ---------------------------------------------------------------------------
  // Standalone roller commands
  // ---------------------------------------------------------------------------

  /** Run the roller at the intake RPM without moving the arm. */
  public Command getRollerOnCmd() {
    return m_roller
        .setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM))
        .withName("Intake/RollerOn");
  }

  /** Stop the roller without moving the arm. */
  public Command getRollerOffCmd() {
    return m_roller.stop().withName("Intake/RollerOff");
  }

  /** Run the roller in reverse (eject) without moving the arm. */
  public Command getReverseCmd() {
    return m_roller
        .setVelocity(RPM.of(IntakeConstants.ROLLER_REVERSE_RPM))
        .withName("Intake/Reverse");
  }

  // ---------------------------------------------------------------------------
  // Deploy arm calibration wrappers
  // ---------------------------------------------------------------------------

  /** Drives deploy arm to hard stop, detects stall, and zeroes encoder. Requires enable. */
  public Command getDeployArmHomeAndCalibrateCmd() {
    return m_deployArm.homeAndCalibrate();
  }

  /** Zeroes deploy arm encoder at current position without motion. Runs while disabled. */
  public Command getDeployArmCalibrateInPlaceCmd() {
    return m_deployArm.calibrateInPlace().ignoringDisable(true);
  }

  // ---------------------------------------------------------------------------
  // Accessors for sub-subsystems (for direct control if needed)
  // ---------------------------------------------------------------------------

  public IntakeRoller getRoller() {
    return m_roller;
  }

  public IntakeDeployArm getDeployArm() {
    return m_deployArm;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/IsStubbed", !m_isReal);
  }
}
