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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TimedSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends TimedSubsystem {
  public static final int TURRET_DEGREES_MIN = -100;
  public static final int TURRET_DEGREES_MAX = 100;

  // TODO: Tune homing speed and hard-stop detection thresholds on hardware.
  // Negative duty cycle drives the turret toward TURRET_DEGREES_MIN (left hard
  // stop).
  private static final double HOME_DUTY_CYCLE = -0.1;
  private static final double HOME_STALL_VELOCITY_RPS = 0.5;
  private static final double HOME_STALL_CURRENT_AMPS = 20.0;

  private final boolean m_isReal;
  private TalonFX turretMotor;
  private Pivot turret;
  private Angle m_stubbedAngle = Degrees.of(0);
  private boolean m_isCalibrated = false;

  // Calibration request bypasses both hardware and software limits so the turret
  // can drive all the way to its mechanical hard stop.
  private final DutyCycleOut turretCalibrationRequest =
      new DutyCycleOut(HOME_DUTY_CYCLE)
          .withIgnoreHardwareLimits(true)
          .withIgnoreSoftwareLimits(true);

  // StatusSignals used for hard-stop detection. Initialized to null and
  // assigned in the real-hardware block of the constructor.
  private StatusSignal<AngularVelocity> turretMotorVelocity = null;
  private StatusSignal<Current> turretMotorTorqueCurrent = null;

  /**
   * Fires when the turret has stalled against its mechanical hard stop: velocity drops below
   * HOME_STALL_VELOCITY_RPS and stator current exceeds HOME_STALL_CURRENT_AMPS for 100 ms.
   */
  public final Trigger isHardStop =
      new Trigger(
              () -> {
                if (turretMotorVelocity == null) return false;
                return Math.abs(turretMotorVelocity.refresh().getValue().in(RotationsPerSecond))
                        < HOME_STALL_VELOCITY_RPS
                    && Math.abs(turretMotorTorqueCurrent.refresh().getValue().in(Amps))
                        > HOME_STALL_CURRENT_AMPS;
              })
          .debounce(0.1);

  public Turret(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      turretMotor = new TalonFX(40);
      turretMotorVelocity = turretMotor.getVelocity();
      turretMotorTorqueCurrent = turretMotor.getTorqueCurrent();

      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  115, 0, 0.15, DegreesPerSecond.of(300), DegreesPerSecondPerSecond.of(300))
              .withSimClosedLoopController(
                  25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(900))
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(16.0)))
              .withIdleMode(MotorMode.BRAKE)
              .withMotorInverted(true)
              .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
              .withStatorCurrentLimit(Amps.of(40))
              .withClosedLoopRampRate(Seconds.of(0.25))
              .withOpenLoopRampRate(Seconds.of(0.25));

      var turretSMC = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

      MechanismPositionConfig mechConfig =
          new MechanismPositionConfig().withMovementPlane(Plane.XY);

      PivotConfig turretConfig =
          new PivotConfig(turretSMC)
              .withStartingPosition(Degrees.of(0))
              .withHardLimit(
                  Degrees.of(TURRET_DEGREES_MIN - 10), Degrees.of(TURRET_DEGREES_MAX + 10))
              .withSoftLimits(Degrees.of(TURRET_DEGREES_MIN), Degrees.of(TURRET_DEGREES_MAX))
              .withTelemetry("TurretMech", TelemetryVerbosity.LOW)
              .withMechanismPositionConfig(mechConfig)
              .withMOI(Meters.of(0.25), Pounds.of(10));

      turret = new Pivot(turretConfig);
    }

    // Auto-calibrate on the first enable if the encoder has not been zeroed yet.
    new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::ensureCalibrated));
  }

  /** Schedules homeAndCalibrate() on enable if the encoder has not yet been zeroed. */
  private void ensureCalibrated() {
    if (!m_isCalibrated) {
      // TODO: Once a physical hard stop is installed, uncomment this and remove the
      // temporary block
      // below.
      // CommandScheduler.getInstance().schedule(homeAndCalibrate());

      // No hard stop yet — assume turret starts at center (0°) and mark calibrated in
      // place.
      if (turretMotor != null) {
        turretMotor.setPosition(Degrees.of(0));
      }
      m_isCalibrated = true;
    }
  }

  public Command setAngle(Angle angle) {
    if (turret != null) return turret.setAngle(angle);
    return Commands.run(() -> m_stubbedAngle = angle, this);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    if (turret != null) return turret.setAngle(angleSupplier);
    return Commands.run(() -> m_stubbedAngle = angleSupplier.get(), this);
  }

  public Angle getAngle() {
    if (turret != null) return turret.getAngle();
    return m_stubbedAngle;
  }

  /** Returns the turret motor's reported velocity. Returns zero when stubbed. */
  public AngularVelocity getVelocity() {
    if (turretMotorVelocity != null) return turretMotorVelocity.refresh().getValue();
    return DegreesPerSecond.of(0);
  }

  public Command sysId() {
    if (turret != null) {
      return turret.sysId(Volts.of(4.0), Volts.per(Second).of(0.5), Seconds.of(8.0));
    }
    return Commands.none();
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    if (turret != null) return turret.set(dutyCycleSupplier);
    return Commands.run(() -> {}, this);
  }

  public Command setDutyCycle(double dutyCycle) {
    if (turret != null) return turret.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /**
   * Slowly drives the turret toward its minimum-angle hard stop (TURRET_DEGREES_MIN). When a
   * velocity drop and current spike are detected (isHardStop), the motor is commanded to zero for
   * 250 ms to let it settle, then the TalonFX encoder is zeroed to TURRET_DEGREES_MIN and YAMS is
   * resynced.
   *
   * <p>Returns immediately as a no-op when the subsystem is stubbed.
   *
   * @return Command to run
   */
  public Command homeAndCalibrate() {
    if (!m_isReal) {
      return this.runOnce(
              () -> {
                m_stubbedAngle = Degrees.of(TURRET_DEGREES_MIN);
                m_isCalibrated = true;
              })
          .withName("Turret/HomeAndCalibrate");
    }
    return run(() -> turretMotor.setControl(turretCalibrationRequest))
        .until(isHardStop)
        .andThen(
            setDutyCycle(0.0)
                .withTimeout(0.25)
                .finallyDo(
                    () -> {
                      turretMotor.setPosition(Degrees.of(TURRET_DEGREES_MIN));
                      m_isCalibrated = true;
                    }))
        .withName("Turret/HomeAndCalibrate");
  }

  /**
   * Zeroes the turret encoder in-place, assuming the mechanism is already resting at its minimum
   * angle hard stop (TURRET_DEGREES_MIN). No motion is commanded. Runs while disabled so it can be
   * used on the field before a match.
   *
   * @return Command to run
   */
  public Command calibrateInPlace() {
    Command c =
        Commands.runOnce(
            () -> {
              if (turretMotor != null) {
                turretMotor.setPosition(Degrees.of(0));
              } else {
                m_stubbedAngle = Degrees.of(0);
              }
              m_isCalibrated = true;
            });
    c.setName("Turret/CalibrateInPlace");
    return c.ignoringDisable(true);
  }

  @Override
  protected void periodicImpl() {
    // Always publish basic boolean status so external dashboards (Elastic,
    // AdvantageKit views, etc.) can read real-time calibration state even when
    // verbose telemetry is disabled.
    Logger.recordOutput("Turret/IsStubbed", !m_isReal);
    Logger.recordOutput("Turret/IsCalibrated", m_isCalibrated);

    if (turret != null) {
      turret.updateTelemetry();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (turret != null) {
      turret.simIterate();
    }
  }
}
