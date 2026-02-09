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
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor = new TalonFX(24);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // .withClosedLoopController(
          //     80, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))
          // .withSimClosedLoopController(
          //     80, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))
          // .withFeedforward(new ArmFeedforward(0.25, 0.45, 0.1))
          .withClosedLoopController(
              0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))
          .withFeedforward(new ArmFeedforward(0.25, 0.45, 0.1, 0.01))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(19)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40));
  private final SmartMotorController hoodSMC =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), motorConfig);
  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withStartingPosition(
              Degrees.of(HoodConstants.HOOD_DEGREES_MIN)) // Starting position of the Pivot
          .withHardLimit(
              Degrees.of(HoodConstants.HOOD_DEGREES_MIN),
              Degrees.of(HoodConstants.HOOD_DEGREES_MAX)) // Hard limit bc wiring prevents infinite
          .withSoftLimits(
              Degrees.of(HoodConstants.HOOD_DEGREES_MIN),
              Degrees.of(
                  HoodConstants
                      .HOOD_DEGREES_MAX)) // Soft limits to prevent mechanical interference when
          // spinning
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH) // Telemetry
          .withLength(Meters.of(0.25))
          .withMass(Pounds.of(5.0)); // MOI Calculation

  private final Arm hood = new Arm(hoodConfig);

  public Hood() {
    // Starting position is provided via ArmConfig.withStartingPosition(...)
    // If hardware requires an explicit sensor reset, do that from higher-level
    // initialization (HoodOld explicitly sets the Talon position). Keeping
    // the constructor lightweight avoids scheduling commands at construction.
  }

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

  /**
   * YAMS-aware zero command: aligns the underlying TalonFX encoder to the configured mechanical
   * minimum and provides an ignorable command that can be used at init to mimic the behavior of the
   * Talon-based `getZeroHoodAngleCmd()`.
   */
  public Command getZeroHoodAngleCmd() {
    Command c =
        Commands.runOnce(
                () -> {
                  // Set the vendor motor controller's sensor position like HoodOld
                  // so the YAMS Arm reads the correct encoder value on startup.
                  hoodMotor.setPosition(Degrees.of(HoodConstants.HOOD_DEGREES_MIN));
                })
            .ignoringDisable(true);
    c.setName("Hood/ResetZero");
    return c;
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();

    Logger.recordOutput(
        "Shooter/HoodTargetAngleDegrees",
        this.hood.getMechanismSetpoint().map(a -> a.in(Degrees)).orElse(Double.NaN));
    Logger.recordOutput("Shooter/HoodCurrentDegrees", this.hood.getAngle().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
