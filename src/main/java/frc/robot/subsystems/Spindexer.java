// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Spindexer extends SubsystemBase {

  private final boolean m_isReal;
  private FlyWheel spindexer;
  private AngularVelocity m_stubbedVelocity = RPM.of(0);
  private DoubleEntry m_spindexerTargetRPM = null;

  /** Creates a new Spindexer. */
  public Spindexer(boolean isReal) {
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
              .withTelemetry("Spindexer", TelemetryVerbosity.HIGH)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
              .withMotorInverted(true)
              .withIdleMode(MotorMode.COAST)
              .withStatorCurrentLimit(Amps.of(40));

      TalonFX spindexerMotor = new TalonFX(39);
      var spindexerController =
          new TalonFXWrapper(spindexerMotor, DCMotor.getFalcon500(1), smcConfig);

      FlyWheelConfig spindexerConfig =
          new FlyWheelConfig(spindexerController)
              .withDiameter(Inches.of(4))
              .withMass(Pounds.of(1))
              .withUpperSoftLimit(RPM.of(1000))
              .withTelemetry("SpindexerMech", TelemetryVerbosity.LOW);

      spindexer = new FlyWheel(spindexerConfig);
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");
    m_spindexerTargetRPM = table.getDoubleTopic("SpindexerTargetRPM").getEntry(0.0);
    m_spindexerTargetRPM.set(0.0);
  }

  /** Forward feed speed toward the shooter. TODO: tune on hardware. */
  private static final double RUN_RPM = 2000.0;

  /** Reverse speed for clearing jams. TODO: tune on hardware. */
  private static final double REVERSE_RPM = -500.0;

  /** Command factory: run spindexer forward to feed toward shooter. */
  public Command getRunCmd() {
    return setVelocity(RPM.of(RUN_RPM)).withName("Spindexer/Run");
  }

  /** Command factory: stop spindexer. */
  public Command getStopCmd() {
    return stop().withName("Spindexer/Stop");
  }

  /** Command factory: run spindexer in reverse to clear jams. */
  public Command getReverseCmd() {
    return setVelocity(RPM.of(REVERSE_RPM)).withName("Spindexer/Reverse");
  }

  /** Run the spindexer at a set speed (0.0 to 1.0). Positive usually feeds INTO the shooter. */
  public Command runSpindexer(double percentOutput) {
    if (spindexer != null) return spindexer.set(percentOutput);
    return Commands.run(() -> {}, this);
  }

  /** Stop the spindexer immediately. */
  public Command stop() {
    if (spindexer != null) return spindexer.set(0.0);
    return Commands.run(() -> m_stubbedVelocity = RPM.of(0), this);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Spindexer/IsStubbed", !m_isReal);

    if (spindexer != null) {
      spindexer.updateTelemetry();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (spindexer != null) {
      spindexer.simIterate();
    }
  }

  public Command setVelocity(AngularVelocity speed) {
    if (spindexer != null) return spindexer.setSpeed(speed);
    return Commands.run(() -> m_stubbedVelocity = speed, this);
  }

  public Command setVelocity() {
    if (spindexer != null)
      return spindexer.setSpeed(() -> RPM.of(m_spindexerTargetRPM.getAsDouble()));
    return Commands.run(() -> m_stubbedVelocity = RPM.of(m_spindexerTargetRPM.getAsDouble()), this);
  }

  public AngularVelocity getVelocity() {
    if (spindexer != null) return spindexer.getSpeed();
    return m_stubbedVelocity;
  }

  public AngularVelocity getTargetVelocity() {
    if (spindexer != null) {
      return spindexer.getMotorController().getMechanismSetpointVelocity().orElse(RPM.of(0));
    }
    return m_stubbedVelocity;
  }
}
