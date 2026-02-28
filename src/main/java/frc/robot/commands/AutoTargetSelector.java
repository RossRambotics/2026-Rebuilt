// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTargetSelector;
import frc.robot.subsystems.drive.Drive;

/**
 * Continuously selects the appropriate {@link ShootingTarget} based on the robot's field zone.
 *
 * <p>Zone boundaries are defined in Blue-alliance coordinates and mirrored for Red using the same
 * flip logic as {@link FieldSpot#flipIfRed} and {@link ShootingTarget#flipIfRed3d}.
 *
 * <ul>
 *   <li><strong>Alliance zone</strong> (x &lt; {@value #ALLIANCE_NEUTRAL_X_M} m): always {@link
 *       ShootingTarget#HUB}
 *   <li><strong>Neutral zone</strong> ({@value #ALLIANCE_NEUTRAL_X_M} ≤ x &lt; {@value
 *       #NEUTRAL_OPPOSITION_X_M} m): {@link ShootingTarget#NEUTRAL_LEFT} when y &gt; field center,
 *       {@link ShootingTarget#NEUTRAL_RIGHT} otherwise
 *   <li><strong>Opposition zone</strong> (x ≥ {@value #NEUTRAL_OPPOSITION_X_M} m): {@link
 *       ShootingTarget#OPPOSITION_LEFT} when y &gt; field center, {@link
 *       ShootingTarget#OPPOSITION_RIGHT} otherwise
 * </ul>
 *
 * <p>This command is the default command of {@link ShooterTargetSelector}. When a manual selection
 * is active ({@link ShooterTargetSelector#isAutoEnabled()} returns {@code false}) the command runs
 * as a no-op, preserving the manual selection until {@link ShooterTargetSelector#cmdEnableAuto()}
 * is triggered.
 */
public class AutoTargetSelector extends Command {

  // ── Zone boundaries (Blue-alliance coordinates, meters) ───────────────────

  /** X threshold separating the alliance zone from the neutral zone (Blue-alliance coords). */
  private static final double ALLIANCE_NEUTRAL_X_M = 4.6;

  /** X threshold separating the neutral zone from the opposition zone (Blue-alliance coords). */
  private static final double NEUTRAL_OPPOSITION_X_M = 11.9;

  /** Y midpoint dividing left/right within the neutral and opposition zones. */
  private static final double LEFT_RIGHT_SPLIT_Y_M = FieldSpot.FIELD_WIDTH_M / 2.0; // 4.035 m

  // ── Dependencies ──────────────────────────────────────────────────────────

  private final ShooterTargetSelector m_targetSelector;
  private final Drive m_drive;

  /**
   * @param targetSelector subsystem that owns the active {@link ShootingTarget} state
   * @param drive drivetrain subsystem used to read the robot's current field pose
   */
  public AutoTargetSelector(ShooterTargetSelector targetSelector, Drive drive) {
    m_targetSelector = targetSelector;
    m_drive = drive;
    addRequirements(targetSelector);
  }

  @Override
  public void execute() {
    if (!m_targetSelector.isAutoEnabled()) return;

    Pose2d robotPose = m_drive.getPose();

    // Convert to Blue-alliance equivalent so zone boundaries apply to both alliances.
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    double xBlue = isRed ? FieldSpot.FIELD_LENGTH_M - robotPose.getX() : robotPose.getX();
    double yBlue = isRed ? FieldSpot.FIELD_WIDTH_M - robotPose.getY() : robotPose.getY();

    ShootingTarget target;
    if (xBlue < ALLIANCE_NEUTRAL_X_M) {
      target = ShootingTarget.HUB;
    } else if (xBlue < NEUTRAL_OPPOSITION_X_M) {
      target =
          yBlue > LEFT_RIGHT_SPLIT_Y_M ? ShootingTarget.NEUTRAL_LEFT : ShootingTarget.NEUTRAL_RIGHT;
    } else {
      target =
          yBlue > LEFT_RIGHT_SPLIT_Y_M
              ? ShootingTarget.OPPOSITION_LEFT
              : ShootingTarget.OPPOSITION_RIGHT;
    }

    m_targetSelector.selectAuto(target);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
