// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;

public class HoodOld extends SubsystemBase {
  final TalonFX m_LeftMotor = new TalonFX(24);
  // final CANcoder m_HoodCANcoder = new CANcoder(8, "rio");

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final double m_kGoalTolerance = 2.0; // 2 degree tolerance

  private GenericEntry m_GE_PID_kS = null;
  private GenericEntry m_GE_PID_kV = null;
  private GenericEntry m_GE_PID_kA = null;
  private GenericEntry m_GE_PID_kP = null;
  private GenericEntry m_GE_PID_kI = null;
  private GenericEntry m_GE_PID_kG = null;
  private GenericEntry m_GE_PID_kD = null;
  private GenericEntry m_GE_bUpdatePID = null;
  private GenericEntry m_GE_Position = null;
  private GenericEntry m_GE_Velocity = null;
  private GenericEntry m_GE_Goal = null;
  private GenericEntry m_GE_Timer = null;

  private Angle m_goal = Degrees.of(0);

  /** Creates a new Hood. */
  public HoodOld() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 19; // 1 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(2)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(20));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.3; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 4.1; // No output for integrated error
    slot0.kD = 0.0; // A velocity error of 1 rps results in 0.5 V output
    slot0.kG = 0.0; // No gravity feedforward

    m_GE_PID_kS = Shuffleboard.getTab("Hood").add("Hood_kS", slot0.kS).getEntry();
    m_GE_PID_kV = Shuffleboard.getTab("Hood").add("Hood_kV", slot0.kV).getEntry();
    m_GE_PID_kA = Shuffleboard.getTab("Hood").add("Hood_kA", slot0.kA).getEntry();
    m_GE_PID_kP = Shuffleboard.getTab("Hood").add("Hood_kP", slot0.kP).getEntry();
    m_GE_PID_kI = Shuffleboard.getTab("Hood").add("Hood_kI", slot0.kI).getEntry();
    m_GE_PID_kG = Shuffleboard.getTab("Hood").add("Hood_kG", slot0.kG).getEntry();
    m_GE_PID_kD = Shuffleboard.getTab("Hood").add("Hood_kD", slot0.kD).getEntry();
    m_GE_bUpdatePID = Shuffleboard.getTab("Hood").add("Hood_UpdatePID", false).getEntry();
    m_GE_Position = Shuffleboard.getTab("Hood").add("Hood_Position", 0).getEntry();
    m_GE_Velocity = Shuffleboard.getTab("Hood").add("Hood_Velocity", 0).getEntry();
    m_GE_Goal = Shuffleboard.getTab("Hood").add("Hood_Goal", 0).getEntry();
    m_GE_Timer = Shuffleboard.getTab("Hood").add("Hood_Timer", 0).getEntry();

    // setup software limits
    // SoftwareLimitSwitchConfigs swLimits = new SoftwareLimitSwitchConfigs();
    // swLimits.ForwardSoftLimitEnable = true;
    // swLimits.ForwardSoftLimitThreshold = Degrees.of(45).in(Degrees);
    // swLimits.ReverseSoftLimitEnable = true;
    // swLimits.ReverseSoftLimitThreshold = Degrees.of(15).in(Degrees);
    // fx_cfg.SoftwareLimitSwitch = swLimits;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_LeftMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    m_LeftMotor.setPosition(Degrees.of(Hood.HOOD_DEGREES_MIN));
    setGoal(Degrees.of(Hood.HOOD_DEGREES_MIN));
  }

  private void setGoal(Angle angle) {
    m_LeftMotor.setControl(m_mmReq.withPosition(angle.in(Rotations)).withSlot(0));
    m_GE_Goal.setDouble(angle.in(Degrees));
    m_goal = angle;

    var cfgReadback = new TalonFXConfiguration();
    m_LeftMotor.getConfigurator().refresh(cfgReadback);
    System.out.println("Ratio = " + cfgReadback.Feedback.SensorToMechanismRatio);
  }
  // private void setGoal(Angle angle) {
  //   double a = angle.in(Degrees);
  //   a = a / 360.0; // convert to rotations
  //   m_LeftMotor.setControl(m_mmReq.withPosition(a).withSlot(0));
  //   m_GE_Goal.setDouble(angle.in(Degrees));
  //   m_goal = angle;
  // }

  private Angle getError() {
    Angle goal = m_goal;
    Angle current = m_LeftMotor.getPosition().getValue();
    Angle error = goal.minus(current);
    error = Degrees.of(error.abs(Degrees));
    return error;
  }

  public Command getWaitUntilErrorLessThanCmd(Angle angle) {
    return new WaitUntilCommand(
        () -> {
          Angle error = getError();
          if (error.baseUnitMagnitude() <= angle.baseUnitMagnitude()) return true;
          return false;
        });
  }

  public Command getSetGoalCommand(Angle angle) {
    return Commands.runOnce(() -> setGoal(angle));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/HoodErrorDegrees", this.getError().in(Degrees));
    Logger.recordOutput("Shooter/HoodTargetAngleDegrees", this.m_goal.in(Degrees));
    Logger.recordOutput(
        "Shooter/HoodCurrentDegrees", this.m_LeftMotor.getPosition().getValue().in(Degrees));

    // Update PID?
    if (m_GE_bUpdatePID.getBoolean(false)) {
      Slot0Configs slot0 = new Slot0Configs();
      m_LeftMotor.getConfigurator().refresh(slot0);
      slot0.kS = m_GE_PID_kS.getDouble(slot0.kS);
      slot0.kV = m_GE_PID_kV.getDouble(slot0.kV);
      slot0.kA = m_GE_PID_kA.getDouble(slot0.kA);
      slot0.kP = m_GE_PID_kP.getDouble(slot0.kP);
      slot0.kI = m_GE_PID_kI.getDouble(slot0.kI);
      slot0.kG = m_GE_PID_kG.getDouble(slot0.kG);
      slot0.kD = m_GE_PID_kD.getDouble(slot0.kD);
      m_LeftMotor.getConfigurator().apply(slot0);
      m_GE_bUpdatePID.setBoolean(false);
      this.setGoal(Degrees.of(m_GE_Goal.getDouble(0.0)));
    }
    // This method will be called once per scheduler run
    m_GE_Velocity.setDouble(m_LeftMotor.getVelocity().getValueAsDouble());
    m_GE_Position.setDouble(m_LeftMotor.getPosition().getValue().in(Degree));
  }

  public void simulationInit() {
    // PhysicsSim.getInstance().addTalonFX(m_LeftMotor, .000000001);

  }

  public Command getZeroHoodAngleCmd() {

    Command c =
        Commands.runOnce(
                () -> {
                  m_LeftMotor.setPosition(Degrees.of(Hood.HOOD_DEGREES_MIN));
                  setGoal(Degrees.of(Hood.HOOD_DEGREES_MIN));
                })
            .ignoringDisable(true);
    c.setName("Hood/ResetZero");

    return c;
  }

  public Command getOpenCommand() {
    return Commands.runOnce(() -> setGoal(Degrees.of(0))).withName("Hood.OpenCommand");
  }

  public Command getCloseCommand() {
    return Commands.runOnce(() -> setGoal(Degrees.of(40))).withName("Hood.CloseCommand");
  }

  @Override
  public void simulationPeriodic() {

    // RobotContainer.m_mechanisms.updateHood(m_LeftMotor.getPosition(), m_LeftMotor.getVelocity());
  }

  public void doManualMove(double theta) {
    double kManualMaxSpeed = 1.0;
    double change = theta * kManualMaxSpeed;
    Angle newGoal = m_goal.plus(Degrees.of(change));

    if (newGoal.in(Degrees) > 60) {
      newGoal = Degrees.of(60);
    } else if (newGoal.in(Degrees) < -220) {
      newGoal = Degrees.of(-220);
    }

    setGoal(newGoal);
  }

  public boolean isStationary() {
    return Math.abs(m_LeftMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.002;
  }
}
