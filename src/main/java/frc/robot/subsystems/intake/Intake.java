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
  private boolean m_isDeployed = false;

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
   * Deploy the intake arm and run the roller. Runs indefinitely until cancelled (e.g. by
   * getRetractCmd()).
   *
   * <p>If the arm has not been deployed yet this match: burst toward the deploy angle, then seat
   * firmly against the deployed hard stop (stall detection), then periodically re-nudge every
   * ARM_DEPLOY_NUDGE_INTERVAL_SEC.
   *
   * <p>If the arm is already deployed (m_isDeployed): skip the burst and go straight to re-seating,
   * then periodic nudge. This prevents jarring the mechanism when the driver toggles intake off and
   * back on.
   */
  public Command getDeployCmd() {
    Command armSequence =
        Commands.either(
                // Already deployed — skip burst, just re-seat
                m_deployArm.driveToDeployedHardStop(),
                // First deploy — burst first, then seat, then mark deployed
                Commands.sequence(
                    m_deployArm
                        .setDutyCycle(IntakeConstants.DEPLOY_BURST_DUTY_CYCLE)
                        .withTimeout(IntakeConstants.DEPLOY_BURST_SEC),
                    m_deployArm.driveToDeployedHardStop(),
                    Commands.runOnce(() -> m_isDeployed = true)),
                () -> m_isDeployed)
            .andThen(
                Commands.sequence(
                        Commands.waitSeconds(IntakeConstants.ARM_DEPLOY_NUDGE_INTERVAL_SEC),
                        m_deployArm.driveToDeployedHardStop())
                    .repeatedly());

    return Commands.parallel(
            armSequence, m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM)))
        .withName("Intake/Deploy");
  }

  /**
   * Kick off full deploy (arm + roller) as a background command, then finish immediately.
   *
   * <p>Allows PathPlanner NamedCommands to trigger intake deployment without blocking the path.
   */
  public Command getDeployInstantCmd() {
    return Commands.runOnce(() -> getDeployCmd().schedule()).withName("Intake/DeployInstant");
  }

  /**
   * Retract the intake arm and stop the roller.
   *
   * <p>Stops the roller first, then bursts duty cycle toward the retracted angle for
   * RETRACT_BURST_SEC, then coasts.
   */
  public Command getRetractCmd() {
    return Commands.sequence(
            Commands.runOnce(() -> m_isDeployed = false),
            m_roller.setVelocity(RPM.of(IntakeConstants.ROLLER_INTAKE_RPM)).withTimeout(0.1),
            m_deployArm
                .setDutyCycle(IntakeConstants.RETRACT_BURST_DUTY_CYCLE)
                .withTimeout(IntakeConstants.RETRACT_BURST_SEC),
            m_deployArm.setDutyCycle(0.0),
            m_roller.stop())
        .withName("Intake/Retract");
    // .withTimeout(
    // IntakeConstants.RETRACT_BURST_DUTY_CYCLE + IntakeConstants.RETRACT_BURST_SEC
    // + 0.1);
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
    return m_roller.setDutyCycle(-12).withName("Intake/Reverse");
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
