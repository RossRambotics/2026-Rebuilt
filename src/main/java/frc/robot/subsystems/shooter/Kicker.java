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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Kicker extends SubsystemBase {

  // --- CONFIGURATION ---
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
          .withSimClosedLoopController(
              50, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, .12, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, .52, .34))
          // Telemetry name and verbosity level
          .withTelemetry("Kicker", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private TalonFX kickerMotor = new TalonFX(38);

  // WRAPPER: Use TalonFXWrapper and Falcon500 simulation physics
  private SmartMotorController kickerController =
      new TalonFXWrapper(kickerMotor, DCMotor.getFalcon500(1), smcConfig);

  private final FlyWheelConfig kickerConfig =
      new FlyWheelConfig(kickerController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          .withTelemetry("KickerMech", TelemetryVerbosity.LOW);

  private FlyWheel kicker = new FlyWheel(kickerConfig);

  private DoubleEntry m_kickerTargetRPM = null;
  /** Creates a new Kicker. */
  public Kicker() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");

    // Target RPM setup
    m_kickerTargetRPM = table.getDoubleTopic("KickerTargetRPM").getEntry(0.0);
    m_kickerTargetRPM.set(0.0);
  }

  /** Run the kicker at a set speed (0.0 to 1.0). Positive usually feeds INTO the shooter. */
  public Command runKicker(double percentOutput) {
    return kicker.set(percentOutput);
  }

  /** Stop the kicker immediately. */
  public Command stop() {
    return kicker.set(0.0);
  }

  @Override
  public void periodic() {
    // kicker.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    kicker.simIterate();
  }

  public Command setVelocity(AngularVelocity speed) {
    return kicker.setSpeed(speed);
  }

  public Command setVelocity() {
    return kicker.setSpeed(() -> RPM.of(m_kickerTargetRPM.getAsDouble()));
  }

  public AngularVelocity getVelocity() {
    return kicker.getSpeed();
  }
}
