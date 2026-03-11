// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.TimedSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Intake coordinator subsystem. Owns {@link IntakeRoller} and {@link IntakeDeployArm} and exposes
 * compound and standalone commands for operator use.
 */
public class Intake extends TimedSubsystem {

  private final boolean m_isReal;
  private final IntakeRoller m_roller;
  private final IntakeDeployArm m_deployArm;

  /** Creates a new Intake coordinator. */
  public Intake(boolean isReal) {
    m_isReal = isReal;
    m_roller = new IntakeRoller(isReal);
    m_deployArm = new IntakeDeployArm(isReal);
    m_roller.setDefaultCommand(getRollerOffCmd());
    m_deployArm.setDefaultCommand(m_deployArm.setDutyCycle(0.0).withName("IntakeDeployArm/Coast"));
  }

  // ---------------------------------------------------------------------------
  // Compound commands
  // ---------------------------------------------------------------------------

  /**
   * Deploy the intake arm and run the roller.
   *
   * <p>Phase 1: burst duty cycle toward the deploy angle while starting the roller, for
   * DEPLOY_BURST_SEC.
   *
   * <p>Phase 2: release arm to coast (duty-cycle 0) while the roller continues running. This allows
   * the arm to fold back freely if struck by another robot.
   */
  public Command getDeployCmd() {
    return Commands.sequence(
            // Phase 1 — burst arm toward deploy while starting roller
            Commands.parallel(
                    m_deployArm.setDutyCycle(IntakeConstants.DEPLOY_BURST_DUTY_CYCLE),
                    m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM)))
                .withTimeout(IntakeConstants.DEPLOY_BURST_SEC),
            // Phase 2 — coast arm, keep roller running
            Commands.parallel(
                m_deployArm.setDutyCycle(0.0),
                m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM))))
        .withName("Intake/Deploy");
  }

  /**
   * Kick off arm deploy and roller spin-up, then finish immediately.
   *
   * <p>Schedules the arm burst sequence and roller as independent background commands so the caller
   * (e.g. a PathPlanner NamedCommand) finishes in one cycle without holding subsystem requirements.
   * Safe to trigger multiple times across auto runs.
   */
  public Command getDeployInstantCmd() {
    return Commands.runOnce(
            () -> {
              Commands.sequence(
                      m_deployArm
                          .setDutyCycle(IntakeConstants.DEPLOY_BURST_DUTY_CYCLE)
                          .withTimeout(IntakeConstants.DEPLOY_BURST_SEC),
                      m_deployArm.setDutyCycle(0.0))
                  .schedule();
              m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM)).schedule();
            })
        .withName("Intake/StartIntake");
  }

  /**
   * Deploy the intake arm only (no roller).
   *
   * <p>Phase 1: burst duty cycle toward the deploy angle for DEPLOY_BURST_SEC.
   *
   * <p>Phase 2: release arm to coast (duty-cycle 0). This allows the arm to fold back freely if
   * struck by another robot.
   */
  public Command getDeployArmCmd() {
    return Commands.sequence(
            m_deployArm
                .setDutyCycle(IntakeConstants.DEPLOY_BURST_DUTY_CYCLE)
                .withTimeout(IntakeConstants.DEPLOY_BURST_SEC),
            m_deployArm.setDutyCycle(0.0))
        .withName("Intake/DeployArm");
  }

  /**
   * Retract the intake arm and stop the roller.
   *
   * <p>Stops the roller first, then bursts duty cycle toward the retracted angle for
   * RETRACT_BURST_SEC, then coasts.
   */
  public Command getRetractCmd() {
    return Commands.sequence(
            Commands.runOnce(() -> {}, m_roller), // interrupt any running roller command
            m_roller
                .stop()
                .withTimeout(
                    IntakeConstants.RETRACT_ROLLER_STOP_TIMEOUT_SEC), // brief stop to settle roller
            m_deployArm
                .setDutyCycle(IntakeConstants.RETRACT_BURST_DUTY_CYCLE)
                .withTimeout(IntakeConstants.RETRACT_BURST_SEC),
            m_deployArm.setDutyCycle(0.0))
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
  protected void periodicImpl() {
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("Intake/IsStubbed", !m_isReal);
    }
  }
}
