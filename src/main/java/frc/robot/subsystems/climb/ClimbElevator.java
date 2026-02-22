// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbElevator extends SubsystemBase {

  private final boolean m_isReal;
  private Elevator elevator;
  private Distance m_stubbedHeight = Meters.of(ClimbElevatorConstants.MIN_HEIGHT_M);

  /** Creates a new ClimbElevator. */
  public ClimbElevator(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      TalonFX elevatorMotor = new TalonFX(43);

      SmartMotorControllerConfig smcConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  0, 0, 0, RotationsPerSecond.of(5), RotationsPerSecondPerSecond.of(10))
              .withSimClosedLoopController(
                  50, 0, 0, RotationsPerSecond.of(5), RotationsPerSecondPerSecond.of(10))
              // TODO: Tune ElevatorFeedforward (ks, kg, kv, ka) on hardware
              .withFeedforward(new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0))
              .withSimFeedforward(new ElevatorFeedforward(0.0, 0.5, 0.1, 0.0))
              .withTelemetry("ClimbElevatorMotor", TelemetryVerbosity.HIGH)
              .withGearing(
                  new MechanismGearing(
                      GearBox.fromReductionStages(ClimbElevatorConstants.GEAR_REDUCTION)))
              .withMotorInverted(false)
              // BRAKE holds the robot's weight when hanging
              .withIdleMode(MotorMode.BRAKE)
              .withStatorCurrentLimit(Amps.of(40));

      var elevatorSMC = new TalonFXWrapper(elevatorMotor, DCMotor.getKrakenX60(1), smcConfig);

      ElevatorConfig elevatorConfig =
          new ElevatorConfig(elevatorSMC)
              .withStartingHeight(Meters.of(ClimbElevatorConstants.MIN_HEIGHT_M))
              .withHardLimits(
                  Meters.of(ClimbElevatorConstants.MIN_HEIGHT_M),
                  Meters.of(ClimbElevatorConstants.MAX_HEIGHT_M))
              .withSoftLimits(
                  Meters.of(ClimbElevatorConstants.MIN_HEIGHT_M),
                  Meters.of(ClimbElevatorConstants.MAX_HEIGHT_M))
              .withDrumRadius(Inches.of(ClimbElevatorConstants.DRUM_RADIUS_INCHES))
              .withMass(Pounds.of(ClimbElevatorConstants.CARRIAGE_MASS_LBS))
              .withTelemetry("ClimbElevatorMech", TelemetryVerbosity.HIGH);

      elevator = new Elevator(elevatorConfig);
    }
  }

  // ---------------------------------------------------------------------------
  // Position control
  // ---------------------------------------------------------------------------

  /** Command the elevator to a specific height (holds position). */
  public Command setHeight(Distance height) {
    if (elevator != null) return elevator.setHeight(height);
    return Commands.run(() -> m_stubbedHeight = height, this);
  }

  /** Command the elevator to a height from a supplier (holds position). */
  public Command setHeight(Supplier<Distance> heightSupplier) {
    if (elevator != null) return elevator.setHeight(heightSupplier);
    return Commands.run(() -> m_stubbedHeight = heightSupplier.get(), this);
  }

  /** Returns the current elevator height. */
  public Distance getHeight() {
    if (elevator != null) return elevator.getHeight();
    return m_stubbedHeight;
  }

  // ---------------------------------------------------------------------------
  // Open-loop control (manual joystick)
  // ---------------------------------------------------------------------------

  /** Open-loop duty-cycle control. Positive moves the elevator up. */
  public Command setDutyCycle(double dutyCycle) {
    if (elevator != null) return elevator.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /** Open-loop duty-cycle control from a supplier. Used for joystick manual control. */
  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    if (elevator != null) return elevator.set(dutyCycleSupplier);
    return Commands.run(() -> {}, this);
  }

  // ---------------------------------------------------------------------------
  // Command factories
  // ---------------------------------------------------------------------------

  /** Extend elevator to the maximum (climb) height. */
  public Command getExtendCmd() {
    return setHeight(Meters.of(ClimbElevatorConstants.MAX_HEIGHT_M)).withName("Climb/Extend");
  }

  /** Retract elevator to the minimum (home) height. */
  public Command getRetractCmd() {
    return setHeight(Meters.of(ClimbElevatorConstants.MIN_HEIGHT_M)).withName("Climb/Retract");
  }

  /**
   * Manual joystick control. Positive input extends, negative retracts.
   *
   * @param joystick Supplier returning a duty-cycle value in [-1, 1].
   */
  public Command getManualCmd(Supplier<Double> joystick) {
    return setDutyCycle(joystick).withName("Climb/Manual");
  }

  // ---------------------------------------------------------------------------
  // SysId
  // ---------------------------------------------------------------------------

  /** SysId characterization routine. */
  public Command sysId() {
    if (elevator != null) {
      return elevator.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
    }
    return Commands.none();
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    Logger.recordOutput("ClimbElevator/IsStubbed", !m_isReal);

    if (elevator != null) {
      elevator.updateTelemetry();

      Logger.recordOutput("Climb/ElevatorCurrentHeightM", elevator.getHeight().in(Meters));
      Logger.recordOutput(
          "Climb/ElevatorTargetHeightM",
          elevator
              .getMotor()
              .getMechanismPositionSetpoint()
              .map(sp -> elevator.getMotor().getConfig().convertFromMechanism(sp).in(Meters))
              .orElse(Double.NaN));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (elevator != null) {
      elevator.simIterate();
    }
  }
}
