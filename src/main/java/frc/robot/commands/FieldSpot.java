// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Predefined field spots the robot can autonomously drive to via {@link DriveToSpot}.
 *
 * <p>All poses are defined for the <strong>Blue alliance</strong>. PathPlanner's alliance-flip
 * lambda (configured in {@code Drive}) automatically mirrors them to Red when needed.
 *
 * <p>TODO: Replace placeholder Pose2d values with actual 2026 field coordinates once available.
 */
public enum FieldSpot {
  // ── Alliance side ──────────────────────────────────────────────────────────
  ALLIANCE_TRENCH_LEFT("Alliance Trench Left", new Pose2d(4.5, 7.5, Rotation2d.fromDegrees(0))),

  ALLIANCE_TRENCH_RIGHT("Alliance Trench Right", new Pose2d(4.5, 0.5, Rotation2d.fromDegrees(0))),

  // ── Opponent side ──────────────────────────────────────────────────────────
  OPPONENT_TRENCH_LEFT(
      "Opponent Trench Left",
      new Pose2d(
          /* TODO: fill in 2026 opponent trench left x */ 14.0,
          /* TODO: fill in 2026 opponent trench left y */ 7.0,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  OPPONENT_TRENCH_RIGHT(
      "Opponent Trench Right",
      new Pose2d(
          /* TODO: fill in 2026 opponent trench right x */ 14.0,
          /* TODO: fill in 2026 opponent trench right y */ 1.0,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  // ── Hub positions ──────────────────────────────────────────────────────────
  HUB_ALLIANCE("Hub Alliance", new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0))),

  HUB_NEUTRAL(
      "Hub Neutral",
      new Pose2d(
          /* TODO: fill in 2026 hub neutral x */ 8.27,
          /* TODO: fill in 2026 hub neutral y */ 4.1,
          Rotation2d.fromDegrees(/* TODO */ 0))),

  // ── Climb positions ────────────────────────────────────────────────────────
  CLIMB_LEFT(
      "Climb Left",
      new Pose2d(
          /* TODO: fill in 2026 climb left x */ 6.0,
          /* TODO: fill in 2026 climb left y */ 5.5,
          Rotation2d.fromDegrees(/* TODO */ 90))),

  CLIMB_CENTER(
      "Climb Center",
      new Pose2d(
          /* TODO: fill in 2026 climb center x */ 6.0,
          /* TODO: fill in 2026 climb center y */ 4.1,
          Rotation2d.fromDegrees(/* TODO */ 90))),

  CLIMB_RIGHT(
      "Climb Right",
      new Pose2d(
          /* TODO: fill in 2026 climb right x */ 6.0,
          /* TODO: fill in 2026 climb right y */ 2.7,
          Rotation2d.fromDegrees(/* TODO */ 90)));

  /**
   * Full length of the 2026 field in meters (driver-station wall to driver-station wall). TODO:
   * verify against the official 2026 field drawings.
   */
  public static final double FIELD_LENGTH_M = 16.540;

  /**
   * Full width of the 2026 field in meters (side wall to side wall). TODO: verify against the
   * official 2026 field drawings.
   */
  public static final double FIELD_WIDTH_M = 8.070;

  /**
   * Mirrors a Blue-alliance pose to Red if the Driver Station reports Red.
   *
   * <p>The 2026 field is rotationally symmetric (180° rotation about field center):
   *
   * <ul>
   *   <li>x → fieldLength - x
   *   <li>y → fieldWidth - y
   *   <li>rotation → rotation + 180°
   * </ul>
   *
   * <p>All {@link FieldSpot} poses are defined in Blue-alliance coordinates. This is the single
   * place that converts them for Red.
   *
   * @param pose Blue-alliance pose
   * @return alliance-correct pose
   */
  public static Pose2d flipIfRed(Pose2d pose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red) {
      return pose;
    }
    return new Pose2d(
        FIELD_LENGTH_M - pose.getX(),
        FIELD_WIDTH_M - pose.getY(),
        pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  /** Human-readable name shown in telemetry and driver displays. */
  public final String displayName;

  /** Target pose on the Blue-alliance field (meters, radians). */
  public final Pose2d pose;

  FieldSpot(String displayName, Pose2d pose) {
    this.displayName = displayName;
    this.pose = pose;
  }
}
