// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeDeployArm extends SubsystemBase {

  private final boolean m_isReal;
  private Arm arm;
  private Angle m_stubbedAngle = Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG);

  /** Creates a new IntakeDeployArm. */
  public IntakeDeployArm(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      TalonFX armMotor = new TalonFX(41);

      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  50, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
              .withSimClosedLoopController(
                  50, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
              // TODO: Tune feedforward (ks, kg, kv, ka) on hardware
              .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
              .withGearing(
                  new MechanismGearing(
                      GearBox.fromReductionStages(IntakeConstants.ARM_GEAR_REDUCTION)))
              // COAST is critical — arm must fold freely if struck by another robot
              .withIdleMode(MotorMode.COAST)
              .withMotorInverted(false)
              .withTelemetry("IntakeDeployArmMotor", TelemetryVerbosity.HIGH)
              .withStatorCurrentLimit(Amps.of(40));

      var armSMC = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);

      ArmConfig armConfig =
          new ArmConfig(armSMC)
              .withStartingPosition(Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG))
              .withHardLimit(
                  Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG),
                  Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG))
              .withSoftLimits(
                  Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG),
                  Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG))
              .withTelemetry("IntakeDeployArmMech", TelemetryVerbosity.HIGH)
              // TODO: Update arm length and mass once known
              .withLength(Meters.of(0.4))
              .withMass(Pounds.of(5.0));

      arm = new Arm(armConfig);
    }
  }

  /** Command the arm to a specific angle (holds position). */
  public Command setAngle(Angle angle) {
    if (arm != null) return arm.setAngle(angle);
    return Commands.run(() -> m_stubbedAngle = angle, this);
  }

  /** Command the arm to an angle from a supplier (holds position). */
  public Command setAngle(Supplier<Angle> angleSupplier) {
    if (arm != null) return arm.setAngle(angleSupplier);
    return Commands.run(() -> m_stubbedAngle = angleSupplier.get(), this);
  }

  /** Open-loop duty cycle control. Use 0.0 to release the arm to coast. */
  public Command setDutyCycle(double dutyCycle) {
    if (arm != null) return arm.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /** Open-loop duty cycle control from a supplier. */
  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    if (arm != null) return arm.set(dutyCycleSupplier);
    return Commands.run(() -> {}, this);
  }

  /** Returns the current arm angle. */
  public Angle getAngle() {
    if (arm != null) return arm.getAngle();
    return m_stubbedAngle;
  }

  /**
   * Returns true if the arm is within tolerance of the target angle. Used by Intake to detect when
   * the deploy motion is complete so position control can be released.
   */
  public boolean isNearAngle(Angle target, Angle tolerance) {
    return getAngle().isNear(target, tolerance);
  }

  /** SysId characterization routine. */
  public Command sysId() {
    if (arm != null) {
      return arm.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
    }
    return Commands.none();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakeDeployArm/IsStubbed", !m_isReal);

    if (arm != null) {
      arm.updateTelemetry();

      Logger.recordOutput(
          "Intake/ArmTargetAngleDeg",
          arm.getMechanismSetpoint().map(a -> a.in(Degrees)).orElse(Double.NaN));
      Logger.recordOutput("Intake/ArmCurrentAngleDeg", arm.getAngle().in(Degrees));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (arm != null) {
      arm.simIterate();
    }
  }
}
