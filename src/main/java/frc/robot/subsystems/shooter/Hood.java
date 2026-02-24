package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

public class Hood extends SubsystemBase {

  // TODO: Tune homing speed and hard-stop detection thresholds on hardware.
  // Negative duty cycle drives the hood toward its minimum angle (hard stop).
  private static final double HOME_DUTY_CYCLE = -0.1;
  private static final double HOME_STALL_VELOCITY_RPS = 0.5;
  private static final double HOME_STALL_CURRENT_AMPS = 15.0;

  private final boolean m_isReal;
  private TalonFX hoodMotor;
  private Arm hood;
  private Angle m_stubbedAngle = Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
  private boolean m_isCalibrated = false;

  // Calibration request bypasses both hardware and software limits so the hood
  // can drive all the way to its mechanical hard stop.
  private final DutyCycleOut hoodCalibrationRequest =
      new DutyCycleOut(HOME_DUTY_CYCLE)
          .withIgnoreHardwareLimits(true)
          .withIgnoreSoftwareLimits(true);

  // StatusSignals used for hard-stop detection. Initialized to null and
  // assigned in the real-hardware block of the constructor.
  private StatusSignal<AngularVelocity> hoodMotorVelocity = null;
  private StatusSignal<Current> hoodMotorTorqueCurrent = null;

  /**
   * Fires when the hood has stalled against its mechanical hard stop: velocity drops below
   * HOME_STALL_VELOCITY_RPS and stator current exceeds HOME_STALL_CURRENT_AMPS for 100 ms.
   */
  public final Trigger isHardStop =
      new Trigger(
              () -> {
                if (hoodMotorVelocity == null) return false;
                return Math.abs(hoodMotorVelocity.refresh().getValue().in(RotationsPerSecond))
                        < HOME_STALL_VELOCITY_RPS
                    && Math.abs(hoodMotorTorqueCurrent.refresh().getValue().in(Amps))
                        > HOME_STALL_CURRENT_AMPS;
              })
          .debounce(0.1);

  public Hood(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      hoodMotor = new TalonFX(24);
      hoodMotorVelocity = hoodMotor.getVelocity();
      hoodMotorTorqueCurrent = hoodMotor.getTorqueCurrent();

      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  50, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(720))
              .withSimClosedLoopController(
                  25, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(720))
              .withFeedforward(
                  new ArmFeedforward(0.25, 0.45, 0.1 / (2 * Math.PI), 0.01 / (2 * Math.PI)))
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(19)))
              .withIdleMode(MotorMode.BRAKE)
              .withMotorInverted(false)
              .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
              .withStatorCurrentLimit(Amps.of(20));

      var hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), motorConfig);

      ArmConfig hoodConfig =
          new ArmConfig(hoodSMC)
              .withStartingPosition(Degrees.of(HoodConstants.HOOD_DEGREES_MIN))
              .withHardLimit(
                  Degrees.of(HoodConstants.HOOD_DEGREES_MIN),
                  Degrees.of(HoodConstants.HOOD_DEGREES_MAX))
              .withSoftLimits(
                  Degrees.of(HoodConstants.HOOD_DEGREES_MIN),
                  Degrees.of(HoodConstants.HOOD_DEGREES_MAX))
              .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
              .withLength(Meters.of(0.25))
              .withMass(Pounds.of(5.0));

      hood = new Arm(hoodConfig);
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

  public Command setAngle(Angle angle) {
    if (hood != null) return hood.setAngle(angle);
    return Commands.run(() -> m_stubbedAngle = angle, this);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    if (hood != null) return hood.setAngle(angleSupplier);
    return Commands.run(() -> m_stubbedAngle = angleSupplier.get(), this);
  }

  public Angle getAngle() {
    if (hood != null) return hood.getAngle();
    return m_stubbedAngle;
  }

  public Command sysId() {
    if (hood != null) {
      return hood.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
    }
    return Commands.none();
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    if (hood != null) return hood.set(dutyCycleSupplier);
    return Commands.run(() -> {}, this);
  }

  public Command setDutyCycle(double dutyCycle) {
    if (hood != null) return hood.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /**
   * YAMS-aware zero command: aligns the underlying TalonFX encoder to the configured mechanical
   * minimum and provides an ignorable command that can be used at init to mimic the behavior of the
   * Talon-based `getZeroHoodAngleCmd()`.
   */
  public Command getZeroHoodAngleCmd() {
    Command c =
        Commands.runOnce(
                () -> {
                  if (hoodMotor != null) {
                    hoodMotor.setPosition(Degrees.of(HoodConstants.HOOD_DEGREES_MIN));
                  } else {
                    m_stubbedAngle = Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
                  }
                })
            .ignoringDisable(true);
    c.setName("Hood/ResetZero");
    return c;
  }

  /**
   * Slowly drives the hood toward its minimum-angle hard stop. When a velocity drop and current
   * spike are detected (isHardStop), the motor is commanded to zero for 250 ms to let it settle,
   * then the TalonFX encoder is zeroed to HOOD_DEGREES_MIN and YAMS is resynced.
   *
   * <p>Returns immediately as a no-op when the subsystem is stubbed.
   *
   * @return Command to run
   */
  public Command homeAndCalibrate() {
    if (!m_isReal) {
      return this.runOnce(
              () -> {
                m_stubbedAngle = Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
                m_isCalibrated = true;
              })
          .withName("Hood/HomeAndCalibrate");
    }
    return run(() -> hoodMotor.setControl(hoodCalibrationRequest))
        .until(isHardStop)
        .andThen(
            setDutyCycle(0.0)
                .withTimeout(0.25)
                .finallyDo(
                    () -> {
                      hoodMotor.setPosition(Degrees.of(HoodConstants.HOOD_DEGREES_MIN));
                      m_isCalibrated = true;
                    }))
        .withName("Hood/HomeAndCalibrate");
  }

  /**
   * Zeroes the hood encoder in-place, assuming the mechanism is already resting at its minimum
   * angle hard stop. No motion is commanded. Runs while disabled so it can be used on the field
   * before a match.
   *
   * @return Command to run
   */
  public Command calibrateInPlace() {
    Command c =
        Commands.runOnce(
            () -> {
              if (hoodMotor != null) {
                hoodMotor.setPosition(Degrees.of(HoodConstants.HOOD_DEGREES_MIN));
              } else {
                m_stubbedAngle = Degrees.of(HoodConstants.HOOD_DEGREES_MIN);
              }
              m_isCalibrated = true;
            });
    c.setName("Hood/CalibrateInPlace");
    return c.ignoringDisable(true);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hood/IsStubbed", !m_isReal);
    Logger.recordOutput("Hood/IsCalibrated", m_isCalibrated);

    if (hood != null) {
      hood.updateTelemetry();

      Logger.recordOutput(
          "Hood/HoodTargetAngleDegrees",
          this.hood.getMechanismSetpoint().map(a -> a.in(Degrees)).orElse(Double.NaN));
      Logger.recordOutput("Hood/HoodCurrentDegrees", this.hood.getAngle().in(Degrees));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (hood != null) {
      hood.simIterate();
    }
  }
}
