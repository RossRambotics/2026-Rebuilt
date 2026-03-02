// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
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

  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

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

    double distanceM =
        m_drive.getPose().getTranslation().getDistance(targetPose2d.getTranslation());

    if (distanceM > AT_TARGET_THRESHOLD_M) {
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
