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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.TimedSubsystem;
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

public class IntakeDeployArm extends TimedSubsystem {

  private final boolean m_isReal;
  private TalonFX armMotor;
  private Arm arm;
  private Angle m_stubbedAngle = Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG);
  private boolean m_isCalibrated = true;

  // Calibration request bypasses both hardware and software limits so the arm
  // can drive all the way to its mechanical hard stop.
  private final DutyCycleOut armCalibrationRequest =
      new DutyCycleOut(IntakeConstants.ARM_HOME_DUTY_CYCLE)
          .withIgnoreHardwareLimits(true)
          .withIgnoreSoftwareLimits(true);

  // StatusSignals used for hard-stop detection. Initialized to null and
  // assigned in the real-hardware block of the constructor.
  private StatusSignal<AngularVelocity> armMotorVelocity = null;
  private StatusSignal<Current> armMotorTorqueCurrent = null;

  /**
   * Fires when the intake arm has stalled against its retracted hard stop: velocity drops below
   * ARM_HOME_STALL_VELOCITY_RPS and stator current exceeds ARM_HOME_STALL_CURRENT_AMPS for
   * ARM_HOME_STALL_DEBOUNCE_SEC.
   */
  public final Trigger isHardStop =
      new Trigger(
              () -> {
                if (armMotorVelocity == null) return false;
                return Math.abs(armMotorVelocity.refresh().getValue().in(RotationsPerSecond))
                        < IntakeConstants.ARM_HOME_STALL_VELOCITY_RPS
                    && Math.abs(armMotorTorqueCurrent.refresh().getValue().in(Amps))
                        > IntakeConstants.ARM_HOME_STALL_CURRENT_AMPS;
              })
          .debounce(IntakeConstants.ARM_HOME_STALL_DEBOUNCE_SEC);

  /** Creates a new IntakeDeployArm. */
  public IntakeDeployArm(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      armMotor = new TalonFX(IntakeConstants.ARM_CAN_ID);
      armMotorVelocity = armMotor.getVelocity();
      armMotorTorqueCurrent = armMotor.getTorqueCurrent();

      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  IntakeConstants.ARM_KP,
                  IntakeConstants.ARM_KI,
                  IntakeConstants.ARM_KD,
                  DegreesPerSecond.of(IntakeConstants.ARM_MAX_VEL_DEG_PER_SEC),
                  DegreesPerSecondPerSecond.of(IntakeConstants.ARM_MAX_ACCEL_DEG_PER_SEC_SQ))
              .withSimClosedLoopController(
                  IntakeConstants.ARM_SIM_KP,
                  IntakeConstants.ARM_KI,
                  IntakeConstants.ARM_KD,
                  DegreesPerSecond.of(IntakeConstants.ARM_MAX_VEL_DEG_PER_SEC),
                  DegreesPerSecondPerSecond.of(IntakeConstants.ARM_MAX_ACCEL_DEG_PER_SEC_SQ))
              // TODO: Tune feedforward (ks, kg, kv, ka) on hardware
              .withFeedforward(
                  new ArmFeedforward(
                      IntakeConstants.ARM_FF_KS,
                      IntakeConstants.ARM_FF_KG,
                      IntakeConstants.ARM_FF_KV,
                      IntakeConstants.ARM_FF_KA))
              .withGearing(
                  new MechanismGearing(
                      GearBox.fromReductionStages(IntakeConstants.ARM_GEAR_REDUCTION)))
              // COAST is critical — arm must fold freely if struck by another robot
              .withIdleMode(MotorMode.COAST)
              .withMotorInverted(false)
              .withTelemetry("IntakeDeployArmMotor", TelemetryVerbosity.LOW)
              .withStatorCurrentLimit(Amps.of(IntakeConstants.ARM_STATOR_CURRENT_LIMIT_AMPS));

      var armSMC = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);

      ArmConfig armConfig =
          new ArmConfig(armSMC)
              .withStartingPosition(Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG))
              .withHardLimit(
                  Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG),
                  Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG))
              // .withSoftLimits(
              // Degrees.of(IntakeConstants.DEPLOY_ANGLE_DEG),
              // Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG))
              .withTelemetry("IntakeDeployArmMech", TelemetryVerbosity.LOW)
              // TODO: Update arm length and mass once known
              .withLength(Meters.of(IntakeConstants.ARM_LENGTH_M))
              .withMass(Pounds.of(IntakeConstants.ARM_MASS_LBS));

      arm = new Arm(armConfig);
    }

    // Auto-calibrate on the first enable if the encoder has not been zeroed yet.
    new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::ensureCalibrated));
  }

  /** Schedules homeAndCalibrate() on enable if the encoder has not yet been zeroed. */
  private void ensureCalibrated() {
    if (!m_isCalibrated) {
      CommandScheduler.getInstance().schedule(homeAndCalibrate());
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

  /**
   * Slowly drives the arm toward its retracted hard stop (RETRACT_ANGLE_DEG). When a velocity drop
   * and current spike are detected (isHardStop), the motor is commanded to zero for 250 ms to let
   * it settle, then the TalonFX encoder is zeroed to RETRACT_ANGLE_DEG and YAMS is resynced.
   *
   * <p>Returns immediately as a no-op when the subsystem is stubbed.
   *
   * @return Command to run
   */
  public Command homeAndCalibrate() {
    if (!m_isReal) {
      return this.runOnce(
              () -> {
                m_stubbedAngle = Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG);
                m_isCalibrated = true;
              })
          .withName("IntakeDeployArm/HomeAndCalibrate");
    }
    return run(() -> armMotor.setControl(armCalibrationRequest))
        .until(isHardStop)
        .andThen(
            setDutyCycle(0.0)
                .withTimeout(IntakeConstants.ARM_HOME_SETTLE_SEC)
                .finallyDo(
                    () -> {
                      armMotor.setPosition(Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG));
                      m_isCalibrated = true;
                    }))
        .withName("IntakeDeployArm/HomeAndCalibrate");
  }

  /**
   * Zeroes the arm encoder in-place, assuming the mechanism is already resting at its retracted
   * hard stop (RETRACT_ANGLE_DEG). No motion is commanded. Runs while disabled so it can be used on
   * the field before a match.
   *
   * @return Command to run
   */
  public Command calibrateInPlace() {
    Command c =
        Commands.runOnce(
            () -> {
              if (armMotor != null) {
                armMotor.setPosition(Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG));
              } else {
                m_stubbedAngle = Degrees.of(IntakeConstants.RETRACT_ANGLE_DEG);
              }
              m_isCalibrated = true;
            });
    c.setName("IntakeDeployArm/CalibrateInPlace");
    return c.ignoringDisable(true);
  }

  /** SysId characterization routine. */
  public Command sysId() {
    if (arm != null) {
      return arm.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
    }
    return Commands.none();
  }

  @Override
  protected void periodicImpl() {
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("IntakeDeployArm/IsStubbed", !m_isReal);
      Logger.recordOutput("IntakeDeployArm/IsCalibrated", m_isCalibrated);
    }

    if (arm != null) {
      arm.updateTelemetry();

      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput(
            "Intake/ArmTargetAngleDeg",
            arm.getMechanismSetpoint().map(a -> a.in(Degrees)).orElse(Double.NaN));
        Logger.recordOutput("Intake/ArmCurrentAngleDeg", arm.getAngle().in(Degrees));
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    if (arm != null) {
      arm.simIterate();
    }
  }
}
