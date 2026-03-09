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

  // Shooting positions in the alliance zone
  ALLIANCE_SHOOTING_LEFT(
      "Alliance Shooting Left", new Pose2d(2.6, 5.7, Rotation2d.fromDegrees(-45.0))),
  ALLIANCE_SHOOTING_RIGHT(
      "Alliance Shooting Right", new Pose2d(2.6, 2.3, Rotation2d.fromDegrees(45.0))),

  // ── Opponent side ──────────────────────────────────────────────────────────
  OPPONENT_TRENCH_LEFT(
      "Opponent Trench Left",
      new Pose2d(
          /* TODO: fill in 2026 opponent trench left x */ 12.0,
          /* TODO: fill in 2026 opponent trench left y */ 7.5,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  OPPONENT_TRENCH_RIGHT(
      "Opponent Trench Right",
      new Pose2d(
          /* TODO: fill in 2026 opponent trench right x */ 12.0,
          /* TODO: fill in 2026 opponent trench right y */ 0.5,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  // ── Hub positions ──────────────────────────────────────────────────────────
  HUB_ALLIANCE("Hub Alliance", new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0))),

  HUB_NEUTRAL(
      "Hub Neutral",
      new Pose2d(
          /* TODO: fill in 2026 hub neutral x */ 8.27,
          /* TODO: fill in 2026 hub neutral y */ 4.1,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  // ── Climb positions ────────────────────────────────────────────────────────
  CLIMB_LEFT(
      "Climb Left",
      new Pose2d(
          /* TODO: fill in 2026 climb left x */ 0.69,
          /* TODO: fill in 2026 climb left y */ 4.6,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  CLIMB_CENTER(
      "Climb Center",
      new Pose2d(
          /* TODO: fill in 2026 climb center x */ 1.5,
          /* TODO: fill in 2026 climb center y */ 3.7,
          Rotation2d.fromDegrees(/* TODO */ 0))),

  CLIMB_RIGHT(
      "Climb Right",
      new Pose2d(
          /* TODO: fill in 2026 climb right x */ 1.5,
          /* TODO: fill in 2026 climb right y */ 3.0,
          Rotation2d.fromDegrees(/* TODO */ 0)));

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

  // ── Zone routing constants ────────────────────────────────────────────────

  /**
   * X coordinate (meters) of the Alliance-side trench. Both ALLIANCE_TRENCH_LEFT and
   * ALLIANCE_TRENCH_RIGHT share this X. Paths crossing this boundary are routed through one of the
   * two trench corridors.
   */
  public static final double ALLIANCE_TRENCH_X_M = ALLIANCE_TRENCH_LEFT.pose.getX();

  /**
   * X coordinate (meters) of the Opponent-side trench. Both OPPONENT_TRENCH_LEFT and
   * OPPONENT_TRENCH_RIGHT share this X.
   */
  public static final double OPPONENT_TRENCH_X_M = OPPONENT_TRENCH_LEFT.pose.getX();

  /**
   * Y split (meters) used to choose the trench corridor. Robot starting Y below this value uses the
   * low-Y trench (ALLIANCE_TRENCH_RIGHT / OPPONENT_TRENCH_RIGHT); at or above uses the high-Y
   * trench (ALLIANCE_TRENCH_LEFT / OPPONENT_TRENCH_LEFT).
   */
  public static final double Y_SPLIT_M = 4.05;

  /** The three field zones separated by the two trench boundaries. */
  public enum Zone {
    ALLIANCE,
    NEUTRAL,
    OPPOSITION
  }

  /**
   * Returns the zone corresponding to the given Blue-alliance X coordinate.
   *
   * @param xMeters Blue-alliance X position in meters
   * @return the zone the position falls in
   */
  public static Zone getZone(double xMeters) {
    if (xMeters < ALLIANCE_TRENCH_X_M) return Zone.ALLIANCE;
    if (xMeters <= OPPONENT_TRENCH_X_M) return Zone.NEUTRAL;
    return Zone.OPPOSITION;
  }

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
