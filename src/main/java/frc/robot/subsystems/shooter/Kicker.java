// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

public class Kicker extends SubsystemBase {

  private final boolean m_isReal;
  private FlyWheel kicker;
  private AngularVelocity m_stubbedVelocity = RPM.of(0);
  private DoubleEntry m_kickerTargetRPM = null;

  /** Creates a new Kicker. */
  public Kicker(boolean isReal) {
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
              .withTelemetry("Kicker", TelemetryVerbosity.HIGH)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
              .withMotorInverted(true)
              .withIdleMode(MotorMode.COAST)
              .withStatorCurrentLimit(Amps.of(40));

      TalonFX kickerMotor = new TalonFX(38);
      var kickerController = new TalonFXWrapper(kickerMotor, DCMotor.getFalcon500(1), smcConfig);

      FlyWheelConfig kickerConfig =
          new FlyWheelConfig(kickerController)
              .withDiameter(Inches.of(4))
              .withMass(Pounds.of(1))
              .withUpperSoftLimit(RPM.of(1000))
              .withTelemetry("KickerMech", TelemetryVerbosity.LOW);

      kicker = new FlyWheel(kickerConfig);
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_kickerTargetRPM = table.getDoubleTopic("KickerTargetRPM").getEntry(0.0);
    m_kickerTargetRPM.set(0.0);
  }

  /** Run the kicker at a set speed (0.0 to 1.0). Positive usually feeds INTO the shooter. */
  public Command runKicker(double percentOutput) {
    if (kicker != null) return kicker.set(percentOutput);
    return Commands.run(() -> {}, this);
  }

  /** Stop the kicker immediately. */
  public Command stop() {
    if (kicker != null) return kicker.set(0.0);
    return Commands.run(() -> m_stubbedVelocity = RPM.of(0), this);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Kicker/IsStubbed", !m_isReal);

    if (kicker != null) {
      kicker.updateTelemetry();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (kicker != null) {
      kicker.simIterate();
    }
  }

  public Command setVelocity(AngularVelocity speed) {
    if (kicker != null) return kicker.setSpeed(speed);
    return Commands.run(() -> m_stubbedVelocity = speed, this);
  }

  public Command setVelocity() {
    if (kicker != null) return kicker.setSpeed(() -> RPM.of(m_kickerTargetRPM.getAsDouble()));
    return Commands.run(() -> m_stubbedVelocity = RPM.of(m_kickerTargetRPM.getAsDouble()), this);
  }

  public AngularVelocity getVelocity() {
    if (kicker != null) return kicker.getSpeed();
    return m_stubbedVelocity;
  }

  public AngularVelocity getTargetVelocity() {
    if (kicker != null) {
      return kicker.getMotorController().getMechanismSetpointVelocity().orElse(RPM.of(0));
    }
    return m_stubbedVelocity;
  }
}
