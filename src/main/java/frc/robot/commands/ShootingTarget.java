// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Predefined shooting targets the shooter can aim at.
 *
 * <p>All poses are defined for the <strong>Blue alliance</strong>. Call {@link #getFieldPose3d()}
 * to obtain the alliance-correct pose at runtime. Use {@link #flipIfRed3d(Pose3d)} for raw Pose3d
 * values.
 *
 * <p>TODO: Replace placeholder Pose3d values with actual 2026 field coordinates once available.
 */
public enum ShootingTarget {
  HUB("Hub", new Pose3d(4.5, 4.1, 1.83, new Rotation3d())),

  NEUTRAL_LEFT("Neutral Left", new Pose3d(0.85, 1.5, 0.0, new Rotation3d())),

  NEUTRAL_RIGHT("Neutral Right", new Pose3d(0.85, 6.5, 0.0, new Rotation3d())),

  OPPOSITION_LEFT("Opposition Left", new Pose3d(2.0, 2.0, 0.0, new Rotation3d())),

  OPPOSITION_RIGHT("Opposition Right", new Pose3d(2.0, 6.0, 0.0, new Rotation3d()));

  /** Human-readable name shown in telemetry and driver displays. */
  public final String displayName;

  /** Target pose on the Blue-alliance field (meters). */
  public final Pose3d bluePose3d;

  ShootingTarget(String displayName, Pose3d bluePose3d) {
    this.displayName = displayName;
    this.bluePose3d = bluePose3d;
  }

  /**
   * Returns the field-relative Pose3d for the current alliance.
   *
   * @return alliance-correct pose
   */
  public Pose3d getFieldPose3d() {
    return flipIfRed3d(bluePose3d);
  }

  /**
   * Mirrors a Blue-alliance Pose3d to Red if the Driver Station reports Red.
   *
   * <p>Same X/Y flip as {@link FieldSpot#flipIfRed}; Z height is unchanged; rotation is rotated
   * 180° around Z.
   *
   * @param pose Blue-alliance pose
   * @return alliance-correct pose
   */
  public static Pose3d flipIfRed3d(Pose3d pose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red) {
      return pose;
    }
    return new Pose3d(
        FieldSpot.FIELD_LENGTH_M - pose.getX(),
        FieldSpot.FIELD_WIDTH_M - pose.getY(),
        pose.getZ(),
        pose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));
  }
}
