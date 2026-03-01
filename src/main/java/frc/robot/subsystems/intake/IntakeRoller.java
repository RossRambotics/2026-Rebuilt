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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class IntakeRoller extends SubsystemBase {

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
                  0, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
              .withSimClosedLoopController(
                  50, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
              .withFeedforward(new SimpleMotorFeedforward(0, .12, 0))
              .withSimFeedforward(new SimpleMotorFeedforward(0, .52, .34))
              .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
              .withMotorInverted(true)
              .withIdleMode(MotorMode.COAST)
              .withStatorCurrentLimit(Amps.of(40));

      TalonFX rollerMotor = new TalonFX(42);
      var rollerController = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60(1), smcConfig);

      FlyWheelConfig rollerConfig =
          new FlyWheelConfig(rollerController)
              .withDiameter(Inches.of(2))
              .withMass(Pounds.of(0.5))
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
  public void periodic() {
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
