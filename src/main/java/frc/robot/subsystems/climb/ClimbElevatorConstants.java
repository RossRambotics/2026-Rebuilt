// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

public final class ClimbElevatorConstants {
  private ClimbElevatorConstants() {}

  // TODO: Tune min/max heights on hardware (meters)
  public static final double MIN_HEIGHT_M = 0.0;
  public static final double MAX_HEIGHT_M = 0.5;

  // TODO: Tune gear reduction on hardware
  public static final double GEAR_REDUCTION = 20.0;

  // TODO: Measure drum radius on hardware (inches)
  public static final double DRUM_RADIUS_INCHES = 1.0;

  // TODO: Measure carriage mass on hardware (lbs)
  public static final double CARRIAGE_MASS_LBS = 20.0;

  // Tolerance for at-height detection
  public static final double AT_HEIGHT_TOLERANCE_M = 0.01;
}
