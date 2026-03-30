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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.TimedSubsystem;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.gearing.*;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class BottomFlyWheel extends TimedSubsystem {

  private final boolean m_isReal;

  // RPM Interpolation Table (Distance meters -> RPM)
  private final TreeMap<Double, Double> m_rpmTable = new TreeMap<>();

  // Hood Angle Interpolation Table (Distance meters -> Degrees)
  private final TreeMap<Double, Double> m_angleTable = new TreeMap<>();

  // Time-of-Flight Table (Distance meters -> seconds)
  private final TreeMap<Double, Double> m_tofTable = new TreeMap<>();

  private DoubleEntry m_targetRPM = null;

  // Shooter Mechanism (null when stubbed)
  private FlyWheel shooter;
  private AngularVelocity m_stubbedVelocity = RPM.of(0);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    if (shooter != null) return shooter.getSpeed();
    return m_stubbedVelocity;
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    if (shooter != null) return shooter.setSpeed(speed);
    return Commands.run(() -> m_stubbedVelocity = speed, this);
  }

  public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
    if (shooter != null) return shooter.setSpeed(speedSupplier);
    return Commands.run(() -> m_stubbedVelocity = speedSupplier.get(), this);
  }

  public Command setVelocity() {
    if (shooter != null) return shooter.setSpeed(() -> RPM.of(m_targetRPM.getAsDouble()));
    return Commands.run(() -> m_stubbedVelocity = RPM.of(m_targetRPM.getAsDouble()), this);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    if (shooter != null) return shooter.set(dutyCycle);
    return Commands.run(() -> {}, this);
  }

  /** Creates a new BottomSubsystem. */
  public BottomFlyWheel(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
      // Vendor motor controller objects
      TalonFX leftMotor = new TalonFX(35); // Use your actual CAN ID
      TalonFX rightMotor = new TalonFX(36); // Use your actual CAN ID

      SmartMotorControllerConfig smcConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              // Feedback Constants (PID Constants)
              .withClosedLoopController(
                  0, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
              .withSimClosedLoopController(
                  25, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
              // Feedforward Constants
              .withFeedforward(new SimpleMotorFeedforward(.1, .12, 0))
              .withSimFeedforward(new SimpleMotorFeedforward(0, .52, .34))
              // Telemetry name and verbosity level
              .withTelemetry("BottomFlyWheelMotor", TelemetryVerbosity.LOW)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
              // Motor properties to prevent over currenting.
              .withMotorInverted(false)
              .withIdleMode(MotorMode.COAST)
              .withFollowers(new Pair<Object, Boolean>(rightMotor, true))
              .withStatorCurrentLimit(Amps.of(40));

      var falconSmartMotorController =
          new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60(2), smcConfig);

      FlyWheelConfig shooterConfig =
          new FlyWheelConfig(falconSmartMotorController)
              // Diameter of the flywheel.
              .withDiameter(Inches.of(4))
              // Mass of the flywheel.
              .withMass(Pounds.of(6))
              // Maximum speed of the shooter.
              .withUpperSoftLimit(RPM.of(6000))
              // Telemetry name and verbosity for the arm.
              .withTelemetry("BottomFlyWheel", TelemetryVerbosity.LOW);

      shooter = new FlyWheel(shooterConfig);

      rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");

    // Target RPM setup
    m_targetRPM = table.getDoubleTopic("BottomTargetRPM").getEntry(0.0);
    m_targetRPM.set(0.0);

    double dist_tune = 0.0;

    // Populate Angle Table (Distance -> Degrees)
    m_angleTable.put(1.0 + dist_tune, 15.0);
    m_angleTable.put(1.5 + dist_tune, 22.6);
    m_angleTable.put(2.0 + dist_tune, 28.0);
    m_angleTable.put(3.0 + dist_tune, 35.0);
    m_angleTable.put(4.0 + dist_tune, 35.0);
    m_angleTable.put(5.0 + dist_tune, 38.0);
    m_angleTable.put(6.0 + dist_tune, 38.0);
    m_angleTable.put(7.0 + dist_tune, 38.0);
    m_angleTable.put(8.0 + dist_tune, 38.0);

    // Populate RPM Table (Distance -> RPM)
    m_rpmTable.put(1.0 + dist_tune, 2000.0);
    m_rpmTable.put(1.5 + dist_tune, 2075.0);
    m_rpmTable.put(2.0 + dist_tune, 2100.0);
    m_rpmTable.put(3.0 + dist_tune, 2450.0);
    m_rpmTable.put(4.0 + dist_tune, 2670.0);
    m_rpmTable.put(5.0 + dist_tune, 2950.0);
    m_rpmTable.put(6.0 + dist_tune, 3200.0);

    m_rpmTable.put(7.0 + dist_tune, 3450.0);
    m_rpmTable.put(8.0 + dist_tune, 3700.0);
    // --- Interpolation Code END ---
    // Populate ToF Table (Distance -> Time-of-Flight in seconds)
    // These are sample values; tune with real ballistics data.
    double tune = 0.6;
    m_tofTable.put(1.0 + dist_tune, 1.6 * tune);
    m_tofTable.put(2.0 + dist_tune, 1.5 * tune);
    m_tofTable.put(3.0 + dist_tune, 1.7 * tune);
    m_tofTable.put(4.2 + dist_tune, 1.75 * tune);
    m_tofTable.put(5.2 + dist_tune, 1.8 * tune);
    m_tofTable.put(6.0 + dist_tune, 1.85 * tune);
    m_tofTable.put(7.0 + dist_tune, 1.95 * tune);
    m_tofTable.put(8.0 + dist_tune, 2.0 * tune);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /** Helper function to calculate RPM from Distance using Linear Interpolation. */
  public double calculateRPM(double distance) {
    // 1. Check for exact match
    if (m_rpmTable.containsKey(distance)) return m_rpmTable.get(distance);

    // 2. Find neighbors
    Map.Entry<Double, Double> floor = m_rpmTable.floorEntry(distance);
    Map.Entry<Double, Double> ceiling = m_rpmTable.ceilingEntry(distance);

    // 3. Handle out of bounds (Clamping to min/max)
    if (floor == null) return ceiling.getValue(); // Lower than min distance
    if (ceiling == null) return floor.getValue(); // Higher than max distance

    // 4. Interpolate
    double x1 = floor.getKey();
    double y1 = floor.getValue();
    double x2 = ceiling.getKey();
    double y2 = ceiling.getValue();

    return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
  }

  /** Helper function to calculate Hood Angle from Distance. */
  public double calculateAngle(double distance) {
    // 1. Check for exact match
    if (m_angleTable.containsKey(distance)) return m_angleTable.get(distance);

    // 2. Find neighbors
    Map.Entry<Double, Double> floor = m_angleTable.floorEntry(distance);
    Map.Entry<Double, Double> ceiling = m_angleTable.ceilingEntry(distance);

    // 3. Handle out of bounds (Clamping)
    if (floor == null) return ceiling.getValue();
    if (ceiling == null) return floor.getValue();

    // 4. Interpolate
    double x1 = floor.getKey();
    double y1 = floor.getValue();
    double x2 = ceiling.getKey();
    double y2 = ceiling.getValue();

    return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
  }

  @Override
  protected void periodicImpl() {
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("BottomFlyWheel/IsStubbed", !m_isReal);
    }

    // 1. Update Telemetry
    if (shooter != null) {
      shooter.updateTelemetry();
    }
  }

  /** Helper function to calculate Time-of-Flight from Distance using Linear Interpolation. */
  public double calculateToF(double distance) {
    // 1. Check for exact match
    if (m_tofTable.containsKey(distance)) return m_tofTable.get(distance);

    // 2. Find neighbors
    Map.Entry<Double, Double> floor = m_tofTable.floorEntry(distance);
    Map.Entry<Double, Double> ceiling = m_tofTable.ceilingEntry(distance);

    // 3. Handle out of bounds (Clamping to min/max)
    if (floor == null) return ceiling.getValue(); // Lower than min distance
    if (ceiling == null) return floor.getValue(); // Higher than max distance

    // 4. Interpolate
    double x1 = floor.getKey();
    double y1 = floor.getValue();
    double x2 = ceiling.getKey();
    double y2 = ceiling.getValue();

    return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (shooter != null) {
      shooter.simIterate();
    }
  }
}
