package frc.robot.subsystems.shooter;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class Turret extends SubsystemBase {
  public static final int TURRET_DEGREES_MIN = -90;
  public static final int TURRET_DEGREES_MAX = 90;

  private final boolean m_isReal;
  private Pivot turret;
  private Angle m_stubbedAngle = Degrees.of(0);

  public Turret(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      TalonFX turretMotor = new TalonFX(40);

      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withClosedLoopController(
                  4, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(900))
              .withSimClosedLoopController(
                  25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(900))
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
              .withIdleMode(MotorMode.BRAKE)
              .withMotorInverted(false)
              .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
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
              .withTelemetry("TurretMech", TelemetryVerbosity.HIGH)
              .withMechanismPositionConfig(mechConfig)
              .withMOI(Meters.of(0.25), Pounds.of(10));

      turret = new Pivot(turretConfig);
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

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/IsStubbed", !m_isReal);

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
