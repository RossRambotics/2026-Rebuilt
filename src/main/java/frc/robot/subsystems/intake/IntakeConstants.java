// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

public final class IntakeConstants {
  private IntakeConstants() {}

  // TODO: Tune deploy/retract angles on hardware
  public static final double DEPLOY_ANGLE_DEG = 0.0;
  public static final double RETRACT_ANGLE_DEG = 90.0;

  // TODO: Tune gearing reduction on hardware
  public static final double ARM_GEAR_REDUCTION = 60.0;

  // TODO: Tune RPM values on hardware
  public static final double ROLLER_INTAKE_RPM = 2000.0;
  public static final double ROLLER_REVERSE_RPM = -1000.0;

  // Tolerance used when detecting that the arm has reached the deploy position
  public static final double AT_ANGLE_TOLERANCE_DEG = 5.0;
}
