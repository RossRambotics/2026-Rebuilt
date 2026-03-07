// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TimedSubsystem;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeRoller extends TimedSubsystem {

  private final boolean m_isReal;
  private FlyWheel roller;
  private AngularVelocity m_stubbedVelocity = RPM.of(0);

  /** Creates a new IntakeRoller. */
  public IntakeRoller(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      SmartMotorControllerConfig smcConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  IntakeConstants.ROLLER_KP,
                  IntakeConstants.ROLLER_KI,
                  IntakeConstants.ROLLER_KD,
                  RotationsPerSecond.of(IntakeConstants.ROLLER_MAX_VEL_RPS),
                  RotationsPerSecondPerSecond.of(IntakeConstants.ROLLER_MAX_ACCEL_RPS_SQ))
              .withSimClosedLoopController(
                  IntakeConstants.ROLLER_SIM_KP,
                  IntakeConstants.ROLLER_KI,
                  IntakeConstants.ROLLER_KD,
                  RotationsPerSecond.of(IntakeConstants.ROLLER_MAX_VEL_RPS),
                  RotationsPerSecondPerSecond.of(IntakeConstants.ROLLER_MAX_ACCEL_RPS_SQ))
              .withFeedforward(
                  new SimpleMotorFeedforward(
                      IntakeConstants.ROLLER_FF_KS,
                      IntakeConstants.ROLLER_FF_KV,
                      IntakeConstants.ROLLER_FF_KA))
              .withSimFeedforward(
                  new SimpleMotorFeedforward(
                      IntakeConstants.ROLLER_SIM_FF_KS,
                      IntakeConstants.ROLLER_SIM_FF_KV,
                      IntakeConstants.ROLLER_SIM_FF_KA))
              .withTelemetry("IntakeRoller", TelemetryVerbosity.LOW)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
              .withMotorInverted(true)
              .withIdleMode(MotorMode.COAST)
              .withStatorCurrentLimit(Amps.of(IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS));

      TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID);
      var rollerController = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60(1), smcConfig);

      FlyWheelConfig rollerConfig =
          new FlyWheelConfig(rollerController)
              .withDiameter(Inches.of(IntakeConstants.ROLLER_DIAMETER_IN))
              .withMass(Pounds.of(IntakeConstants.ROLLER_MASS_LBS))
              .withTelemetry("IntakeRollerMech", TelemetryVerbosity.LOW);

      roller = new FlyWheel(rollerConfig);
    }
  }

  /** Run the roller at a fixed velocity. Positive spins game pieces into the robot. */
  public Command setVelocity(AngularVelocity speed) {
    if (roller != null) return roller.setSpeed(speed);
    return Commands.run(() -> m_stubbedVelocity = speed, this);
  }

  /** Run the roller at a fixed duty cycle output. */
  public Command setDutyCycle(double dutyCycle) {
    if (roller != null) return roller.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /** Stop the roller immediately. */
  public Command stop() {
    if (roller != null) return roller.set(0.0);
    return Commands.run(() -> m_stubbedVelocity = RPM.of(0), this);
  }

  /** Returns the current roller velocity. */
  public AngularVelocity getVelocity() {
    if (roller != null) return roller.getSpeed();
    return m_stubbedVelocity;
  }

  @Override
  protected void periodicImpl() {
    Logger.recordOutput("IntakeRoller/IsStubbed", !m_isReal);

    if (roller != null) {
      roller.updateTelemetry();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (roller != null) {
      roller.simIterate();
    }
  }
}
