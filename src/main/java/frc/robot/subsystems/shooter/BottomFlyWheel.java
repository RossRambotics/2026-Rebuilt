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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.TreeMap;
import yams.gearing.*;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class BottomFlyWheel extends SubsystemBase {

  // --- Interpolation Code START ---
  // 1. The Interpolation Table
  private final TreeMap<Double, Double> m_rpmTable = new TreeMap<>();

  // 2. NetworkTable Entries for debugging/dashboard
  private DoubleEntry m_inputDistance; // We read this from the dashboard (or camera)
  private DoubleEntry m_interpolatedRPM; // We write the result here
  // --- Interpolation Code END ---
  // --- Hood Angle Code START ---
  // 1. The Angle Interpolation Table
  private final TreeMap<Double, Double> m_angleTable = new TreeMap<>();

  // 2. NetworkTable Entry for the calculated angle
  private DoubleEntry m_interpolatedAngle;
  // --- Hood Angle Code END ---

  private DoubleEntry m_targetRPM = null;

  // Vendor motor controller object
  private TalonFX m_leftMotor = new TalonFX(35); // Use your actual CAN ID
  private TalonFX m_rightMotor = new TalonFX(36); // Use your actual CAN ID

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
          .withSimClosedLoopController(
              50, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(1000))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("BottomFlyWheelMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withFollowers(new Pair<Object, Boolean>(m_rightMotor, true))
          .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController falconSmartMotorController =
      new TalonFXWrapper(m_leftMotor, DCMotor.getKrakenX60(2), smcConfig);

  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig(falconSmartMotorController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("BottomFlyWheel", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setVelocity() {
    return shooter.setSpeed(() -> RPM.of(m_targetRPM.getAsDouble()));
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  /** Creates a new BottomSubsystem. */
  public BottomFlyWheel() {
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shooter");

    // Target RPM setup
    m_targetRPM = table.getDoubleTopic("BottomTargetRPM").getEntry(0.0);
    m_targetRPM.set(0.0);

    // --- Interpolation Code START ---
    m_inputDistance = table.getDoubleTopic("InputDistance").getEntry(0.0);
    m_inputDistance.set(0.0); // Forces it to appear on Dashboard

    m_interpolatedRPM = table.getDoubleTopic("CalculatedRPM").getEntry(0.0);
    m_interpolatedAngle = table.getDoubleTopic("CalculatedAngle").getEntry(0.0);

    // Populate Angle Table (Distance -> Degrees)
    m_angleTable.put(1.0, 45.0);
    m_angleTable.put(3.0, 35.0);
    m_angleTable.put(10.0, 25.0);

    // Populate RPM Table (Distance -> RPM)
    m_rpmTable.put(1.0, 500.0);
    m_rpmTable.put(3.0, 1000.0);
    m_rpmTable.put(10.0, 1600.0);
    // --- Interpolation Code END ---
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
  private double calculateRPM(double distance) {
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
  private double calculateAngle(double distance) {
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
  public void periodic() {
    // 1. Update Telemetry
    shooter.updateTelemetry();

    // --- Interpolation Code START ---
    // 2. Get the current distance
    double currentDist = m_inputDistance.get();

    // 3. Calculate RPM and Angle
    double calculatedRpm = calculateRPM(currentDist);
    double calculatedAngle = calculateAngle(currentDist);

    // 4. Push to Dashboard so you can see it
    m_interpolatedRPM.set(calculatedRpm);
    m_interpolatedAngle.set(calculatedAngle);
    // --- Interpolation Code END ---
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
