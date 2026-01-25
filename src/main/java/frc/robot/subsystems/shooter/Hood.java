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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
  public static final int HOOD_DEGREES_MIN = 15;
  public static final int HOOD_DEGREES_MAX = 45;

  private final TalonFX hoodMotor = new TalonFX(24);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              0.0001, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 3)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40));
  private final SmartMotorController hoodSMC =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), motorConfig);

  private final MechanismPositionConfig mechConfig =
      new MechanismPositionConfig().withMovementPlane(Plane.XY);
  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withStartingPosition(Degrees.of(Hood.HOOD_DEGREES_MIN)) // Starting position of the Pivot
          // .withWrapping(Rotations.of(-0.5), Rotations.of(0.5))
          // .withWrapping(Degrees.of(-180), Degrees.of(180)) // Wrapping enabled bc the pivot can
          // spin
          // infinitely
          .withHardLimit(
              Degrees.of(HOOD_DEGREES_MIN),
              Degrees.of(HOOD_DEGREES_MAX)) // Hard limit bc wiring prevents infinite
          .withSoftLimits(
              Degrees.of(HOOD_DEGREES_MIN),
              Degrees.of(HOOD_DEGREES_MAX)) // Soft limits to prevent mechanical interference when
          // spinning
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH) // Telemetry
          .withLength(Meters.of(0.25))
          .withMass(Pounds.of(1.0)); // MOI Calculation

  private final Arm hood = new Arm(hoodConfig);

  public Hood() {}

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return hood.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command sysId() {
    return hood.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return hood.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
