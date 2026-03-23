// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

public final class ClimbElevatorConstants {
  private ClimbElevatorConstants() {}

  // TODO: Tune min/max heights on hardware (meters)
  public static final double MIN_HEIGHT_M = 0.0;
  public static final double MAX_HEIGHT_M = 0.5;

  // TODO: Tune robot-climb height on hardware (meters) — high enough to lift robot off the ground
  public static final double ROBOT_CLIMB_HEIGHT_M = 0.3;

  // TODO: Tune gear reduction on hardware
  public static final double GEAR_REDUCTION = 20.0;

  // TODO: Measure drum radius on hardware (inches)
  public static final double DRUM_RADIUS_INCHES = 1.0;

  // TODO: Measure carriage mass on hardware (lbs)
  public static final double CARRIAGE_MASS_LBS = 20.0;

  // Tolerance for at-height detection
  public static final double AT_HEIGHT_TOLERANCE_M = 0.01;

  // ── SuperFastClimb command constants ─────────────────────────────────────

  /**
   * Tilt magnitude (Math.hypot(rollRobotRad, pitchRobotRad)) in radians that confirms the climb arm
   * has hooked the tower bar. When exceeded, the drive stops and the arm retracts (winch). TODO:
   * Tune on hardware — start large and tighten until reliable.
   */
  public static final double TILT_HOOK_THRESHOLD_RAD = Math.toRadians(10.0);

  /** Meters the robot drives past the approach pose in the approach heading direction. */
  public static final double CLIMB_PASS_DISTANCE_M = 1.0;

  /** Max seconds to wait for the elevator to reach MAX_HEIGHT before proceeding to the pass. */
  public static final double CLIMB_EXTEND_TIMEOUT_SEC = 3.0;

  /** Approach phase PathPlanner linear speed limit (m/s). */
  public static final double CLIMB_APPROACH_MAX_SPEED_MPS = 3.0;

  /** Approach phase PathPlanner linear acceleration limit (m/s²). */
  public static final double CLIMB_APPROACH_MAX_ACCEL_MPS2 = 2.5;

  /** Tower-pass phase PathPlanner linear speed limit (m/s). Slower to allow tilt monitoring. */
  public static final double CLIMB_PASS_MAX_SPEED_MPS = 1.5;

  /** Tower-pass phase PathPlanner linear acceleration limit (m/s²). */
  public static final double CLIMB_PASS_MAX_ACCEL_MPS2 = 1.5;
}
