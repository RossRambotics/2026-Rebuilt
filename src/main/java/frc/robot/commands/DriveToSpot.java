// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SpotSelector;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

/**
 * Autonomously drives to the spot currently selected in {@link SpotSelector} using PathPlanner's
 * on-the-fly pathfinding ({@code AutoBuilder.pathfindToPose}).
 *
 * <p>Hold the Y button to pathfind; release to cancel. If no spot is selected the command exits
 * immediately. If the robot is already within {@value #AT_TARGET_THRESHOLD_M} meters of the target
 * the pathfinding step is skipped.
 *
 * <p>Alliance mirroring is handled automatically by the flip lambda registered in {@link
 * Drive#AutoBuilder.configure}.
 */
public class DriveToSpot extends Command {

  /** Distance (meters) below which the robot is considered already at the target. */
  private static final double AT_TARGET_THRESHOLD_M = 0.5;

  /**
   * Distance into the path (meters) at which rotation must be complete. Path 1 drives from the
   * robot to this intermediate point with the target rotation as its goal, so PathPlanner finishes
   * the heading change within the first meter. Path 2 continues to the final target, starting from
   * {@link #CARRY_VELOCITY_MPS} so translation is uninterrupted through the junction.
   */
  private static final double ROTATION_SPLIT_DIST_M = 1.0;

  /**
   * Translation speed (m/s) maintained at the path-1 / path-2 junction. Must be reachable given the
   * distance to the intermediate point and the current constraints.
   */
  private static final double CARRY_VELOCITY_MPS = 2.0;

  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(4.0, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private final Drive m_drive;
  private final SpotSelector m_spotSelector;

  private Command m_pathFindingCommand = null;
  private boolean m_noTarget = false;

  public DriveToSpot(Drive drive, SpotSelector spotSelector) {
    m_drive = drive;
    m_spotSelector = spotSelector;
    // DriveToSpot does not directly require the drive subsystem; the scheduled
    // pathfinding command does. This avoids conflicting with the default drive
    // command while still allowing cancellation on release.
    addRequirements(spotSelector);
  }

  @Override
  public void initialize() {
    Optional<Pose2d> targetOpt = m_spotSelector.getSelectedPose();

    if (targetOpt.isEmpty()) {
      // No spot selected — finish immediately.
      m_noTarget = true;
      m_pathFindingCommand = null;
      return;
    }

    m_noTarget = false;
    Pose2d targetPose2d = FieldSpot.flipIfRed(targetOpt.get());
    Pose2d currentPose = m_drive.getPose();

    double distanceM = currentPose.getTranslation().getDistance(targetPose2d.getTranslation());

    if (distanceM > ROTATION_SPLIT_DIST_M) {
      // Path 1: drive 1 m toward the target, ending with the target rotation already
      // applied.
      // A non-zero goal velocity keeps translation flowing into path 2 without a
      // stop.
      Translation2d delta = targetPose2d.getTranslation().minus(currentPose.getTranslation());
      Translation2d intermediateXY =
          currentPose.getTranslation().plus(delta.times(ROTATION_SPLIT_DIST_M / distanceM));
      Pose2d intermediatePose = new Pose2d(intermediateXY, targetPose2d.getRotation());

      m_pathFindingCommand =
          Commands.sequence(
              AutoBuilder.pathfindToPose(intermediatePose, CONSTRAINTS, CARRY_VELOCITY_MPS),
              AutoBuilder.pathfindToPose(targetPose2d, CONSTRAINTS));
    } else if (distanceM > AT_TARGET_THRESHOLD_M) {
      // Too close for a split — single path to target.
      m_pathFindingCommand = AutoBuilder.pathfindToPose(targetPose2d, CONSTRAINTS);
    } else {
      // Already at the target; nothing to do.
      m_pathFindingCommand = Commands.none();
    }

    m_spotSelector.setAutoDriving(true);
    CommandScheduler.getInstance().schedule(m_pathFindingCommand);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_spotSelector.setAutoDriving(false);
    if (m_pathFindingCommand != null) {
      m_pathFindingCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    // No target was set — exit right away.
    if (m_noTarget) {
      return true;
    }
    // Otherwise run until the driver releases the button (command is interrupted).
    return false;
  }
}
