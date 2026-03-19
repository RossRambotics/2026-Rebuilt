// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Predefined field spots the robot can autonomously drive to via
 * {@link DriveToSpot}.
 *
 * <p>
 * All poses are defined for the <strong>Blue alliance</strong>. PathPlanner's
 * alliance-flip
 * lambda (configured in {@code Drive}) automatically mirrors them to Red when
 * needed.
 *
 * <p>
 * Trench and shooting positions are named using absolute field geometry:
 *
 * <ul>
 * <li><strong>TOP</strong> — high Y in blue-frame (≈ 7.5 m); Blue driver calls
 * this LEFT, Red
 * driver calls this RIGHT.
 * <li><strong>BOTTOM</strong> — low Y in blue-frame (≈ 0.5 m); Blue driver
 * calls this RIGHT, Red
 * driver calls this LEFT.
 * </ul>
 *
 * <p>
 * Field regions left-to-right in blue-frame X:
 *
 * <pre>
 * Blue wall — BLUE HOME — [BLUE TRENCH] — NEUTRAL — [RED TRENCH] — RED HOME — Red wall
 *              X &lt; 4.5       X ≈ 4.5       4.5–12      X ≈ 12       X &gt; 12
 * </pre>
 *
 * <p>
 * TODO: Replace placeholder Pose2d values with actual 2026 field coordinates
 * once available.
 */
public enum FieldSpot {
  // ── Blue alliance trench corridors ─────────────────────────────────────────
  /**
   * Blue-alliance trench, top corridor (high Y ≈ 7.5 m). Blue driver calls this
   * LEFT.
   */
  BLUE_TRENCH_TOP("Blue Trench Top", new Pose2d(4.5, 7.5, Rotation2d.fromDegrees(0))),

  /**
   * Blue-alliance trench, bottom corridor (low Y ≈ 0.5 m). Blue driver calls this
   * RIGHT.
   */
  BLUE_TRENCH_BOTTOM("Blue Trench Bottom", new Pose2d(4.5, 0.5, Rotation2d.fromDegrees(0))),

  // ── Shooting positions (Blue-alliance side) ─────────────────────────────────
  /**
   * Shooting position on the top half of the field (high Y). Blue driver calls
   * this LEFT.
   */
  SHOOTING_TOP("Shooting Top", new Pose2d(2.6, 5.7, Rotation2d.fromDegrees(-45.0))),

  /**
   * Shooting position on the bottom half of the field (low Y). Blue driver calls
   * this RIGHT.
   */
  SHOOTING_BOTTOM("Shooting Bottom", new Pose2d(2.6, 2.3, Rotation2d.fromDegrees(45.0))),

  // ── Red alliance trench corridors ──────────────────────────────────────────
  /**
   * Red-alliance trench, top corridor (high Y ≈ 7.5 m). Red driver calls this
   * RIGHT.
   */
  RED_TRENCH_TOP(
      "Red Trench Top",
      new Pose2d(
          /* TODO: fill in 2026 red trench top x */ 12.0,
          /* TODO: fill in 2026 red trench top y */ 7.5,
          Rotation2d.fromDegrees(/* TODO */ 180))),

  /**
   * Red-alliance trench, bottom corridor (low Y ≈ 0.5 m). Red driver calls this
   * LEFT.
   */
  RED_TRENCH_BOTTOM(
      "Red Trench Bottom",
      new Pose2d(
          /* TODO: fill in 2026 red trench bottom x */ 12.0,
          /* TODO: fill in 2026 red trench bottom y */ 0.5,
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
   * Full length of the 2026 field in meters (driver-station wall to
   * driver-station wall). TODO:
   * verify against the official 2026 field drawings.
   */
  public static final double FIELD_LENGTH_M = 16.540;

  /**
   * Full width of the 2026 field in meters (side wall to side wall). TODO: verify
   * against the
   * official 2026 field drawings.
   */
  public static final double FIELD_WIDTH_M = 8.070;

  // ── Zone routing constants ────────────────────────────────────────────────

  /**
   * X coordinate (meters) of the Blue-alliance trench boundary. Both
   * {@link #BLUE_TRENCH_TOP} and
   * {@link #BLUE_TRENCH_BOTTOM} share this X. Paths crossing this boundary are
   * routed through one
   * of the two trench corridors.
   */
  public static final double BLUE_TRENCH_X_M = BLUE_TRENCH_TOP.pose.getX();

  /**
   * X coordinate (meters) of the Red-alliance trench boundary. Both
   * {@link #RED_TRENCH_TOP} and
   * {@link #RED_TRENCH_BOTTOM} share this X.
   */
  public static final double RED_TRENCH_X_M = RED_TRENCH_TOP.pose.getX();

  /**
   * Y split (meters) used to choose the trench corridor. Robot starting Y below
   * this value uses the
   * bottom trench ({@link #BLUE_TRENCH_BOTTOM} / {@link #RED_TRENCH_BOTTOM}); at
   * or above uses the
   * top trench ({@link #BLUE_TRENCH_TOP} / {@link #RED_TRENCH_TOP}).
   */
  public static final double Y_SPLIT_M = 4.05;

  /**
   * The three field zones separated by the two trench boundaries.
   *
   * <ul>
   * <li>{@link Zone#BLUE_HOME} — X &lt; {@link #BLUE_TRENCH_X_M}: the Blue
   * alliance's starting
   * zone (also called the Red opposition zone).
   * <li>{@link Zone#NEUTRAL} — between the two trench boundaries.
   * <li>{@link Zone#RED_HOME} — X &gt; {@link #RED_TRENCH_X_M}: the Red
   * alliance's starting zone
   * (also called the Blue opposition zone).
   * </ul>
   */
  public enum Zone {
    /** Blue alliance's home zone (X < {@link FieldSpot#BLUE_TRENCH_X_M}). */
    BLUE_HOME,
    /** The neutral zone between the two trench boundaries. */
    NEUTRAL,
    /** Red alliance's home zone (X > {@link FieldSpot#RED_TRENCH_X_M}). */
    RED_HOME
  }

  /**
   * Returns the zone corresponding to the given Blue-alliance X coordinate.
   *
   * @param xMeters Blue-alliance X position in meters
   * @return the zone the position falls in
   */
  public static Zone getZone(double xMeters) {
    if (xMeters < BLUE_TRENCH_X_M)
      return Zone.BLUE_HOME;
    if (xMeters <= RED_TRENCH_X_M)
      return Zone.NEUTRAL;
    return Zone.RED_HOME;
  }

  /**
   * Mirrors a Blue-alliance pose to Red if the Driver Station reports Red.
   *
   * <p>
   * The 2026 field is rotationally symmetric (180° rotation about field center):
   *
   * <ul>
   * <li>x → fieldLength - x
   * <li>y → fieldWidth - y
   * <li>rotation → rotation + 180°
   * </ul>
   *
   * <p>
   * All {@link FieldSpot} poses are defined in Blue-alliance coordinates. This is
   * the single
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
