// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

public final class IntakeConstants {
  private IntakeConstants() {}

  // ---------------------------------------------------------------------------
  // CAN IDs
  // ---------------------------------------------------------------------------
  public static final int ARM_CAN_ID = 41;
  public static final int ROLLER_CAN_ID = 42;

  // ---------------------------------------------------------------------------
  // Deploy arm — target angles
  // ---------------------------------------------------------------------------
  // TODO: Tune deploy/retract angles on hardware
  public static final double DEPLOY_ANGLE_DEG = -26.0;
  public static final double RETRACT_ANGLE_DEG = 110.0;

  // Tolerance used when detecting that the arm has reached the deploy position
  public static final double AT_ANGLE_TOLERANCE_DEG = 5.0;

  // ---------------------------------------------------------------------------
  // Deploy arm — gearing
  // ---------------------------------------------------------------------------
  // TODO: Tune gearing reduction on hardware
  public static final double ARM_GEAR_REDUCTION = 12.0;

  // ---------------------------------------------------------------------------
  // Deploy arm — homing
  // ---------------------------------------------------------------------------
  // TODO: Verify homing direction and tune thresholds on hardware.
  // Positive duty cycle must drive the arm toward RETRACT_ANGLE_DEG (the hard
  // stop).
  public static final double ARM_HOME_DUTY_CYCLE = 0.25;
  public static final double ARM_HOME_STALL_VELOCITY_RPS = 0.5;
  public static final double ARM_HOME_STALL_CURRENT_AMPS = 20.0;
  public static final double ARM_HOME_STALL_DEBOUNCE_SEC = 0.1;
  public static final double ARM_HOME_SETTLE_SEC = 0.25;

  // ---------------------------------------------------------------------------
  // Deploy arm — PID / motion magic
  // ---------------------------------------------------------------------------
  // TODO: Tune PID gains on hardware
  public static final double ARM_KP = 150.0;
  public static final double ARM_KI = 0.0;
  public static final double ARM_KD = 0.0;
  public static final double ARM_MAX_VEL_DEG_PER_SEC = 180.0;
  public static final double ARM_MAX_ACCEL_DEG_PER_SEC_SQ = 360.0;
  public static final double ARM_SIM_KP = 50.0;

  // ---------------------------------------------------------------------------
  // Deploy arm — feedforward
  // ---------------------------------------------------------------------------
  // TODO: Tune feedforward (ks, kg, kv, ka) on hardware
  public static final double ARM_FF_KS = 0.1;
  public static final double ARM_FF_KG = 0.1;
  public static final double ARM_FF_KV = 0.2;
  public static final double ARM_FF_KA = 0.0;

  // ---------------------------------------------------------------------------
  // Deploy arm — hardware limits
  // ---------------------------------------------------------------------------
  public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  // TODO: Update arm length and mass once measured
  public static final double ARM_LENGTH_M = 0.4;
  public static final double ARM_MASS_LBS = 5.0;

  // ---------------------------------------------------------------------------
  // Intake roller — RPM setpoints
  // ---------------------------------------------------------------------------
  // TODO: Tune RPM values on hardware
  public static final double ROLLER_INTAKE_RPM = 4000.0;
  public static final double ROLLER_REVERSE_RPM = -1000.0;

  // ---------------------------------------------------------------------------
  // Intake roller — PID / motion
  // ---------------------------------------------------------------------------
  // TODO: Tune PID gains on hardware
  public static final double ROLLER_KP = 0.5;
  public static final double ROLLER_KI = 0.0;
  public static final double ROLLER_KD = 0.0;
  public static final double ROLLER_MAX_VEL_RPS = 100.0;
  public static final double ROLLER_MAX_ACCEL_RPS_SQ = 1000.0;
  public static final double ROLLER_SIM_KP = 50.0;

  // ---------------------------------------------------------------------------
  // Intake roller — feedforward
  // ---------------------------------------------------------------------------
  // TODO: Tune feedforward on hardware
  public static final double ROLLER_FF_KS = 0.0;
  public static final double ROLLER_FF_KV = 0.12;
  public static final double ROLLER_FF_KA = 0.0;
  public static final double ROLLER_SIM_FF_KS = 0.0;
  public static final double ROLLER_SIM_FF_KV = 0.52;
  public static final double ROLLER_SIM_FF_KA = 0.34;

  // ---------------------------------------------------------------------------
  // Intake roller — hardware
  // ---------------------------------------------------------------------------
  public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  // TODO: Update roller diameter and mass once measured
  public static final double ROLLER_DIAMETER_IN = 2.0;
  public static final double ROLLER_MASS_LBS = 0.5;

  // ---------------------------------------------------------------------------
  // Deploy arm — burst duty cycle
  // ---------------------------------------------------------------------------
  // TODO: Tune burst duty cycles and durations on hardware
  // Negative drives toward DEPLOY_ANGLE_DEG, positive toward RETRACT_ANGLE_DEG
  public static final double DEPLOY_BURST_DUTY_CYCLE = -5.0;
  public static final double RETRACT_BURST_DUTY_CYCLE = 5.0;
  public static final double DEPLOY_BURST_SEC = 0.3;
  public static final double RETRACT_BURST_SEC = 0.3;

  // ---------------------------------------------------------------------------
  // Timing
  // ---------------------------------------------------------------------------
  // Brief pause to let the roller settle before retracting the arm
  public static final double RETRACT_ROLLER_STOP_TIMEOUT_SEC = 0.1;
}
