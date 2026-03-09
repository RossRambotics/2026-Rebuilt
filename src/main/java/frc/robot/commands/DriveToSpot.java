// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FieldSpot.Zone;
import frc.robot.subsystems.SpotSelector;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Autonomously drives to the spot currently selected in {@link SpotSelector} using a zone-aware
 * waypoint route.
 *
 * <p>The field has three zones (Alliance, Neutral, Opposition) separated by two trench boundaries.
 * When the target is in a different zone, trench approach and exit waypoints are inserted and a
 * {@link RotationTarget} is placed at each approach waypoint to achieve the intake-leading heading
 * before the robot enters the narrow trench, while keeping the route as a single smooth path.
 *
 * <p>Note: waypoint {@link Pose2d} rotations control the <em>path tangent direction</em> (direction
 * of travel) for holonomic drives. Robot heading is controlled separately via {@link
 * RotationTarget}, {@link IdealStartingState}, and {@link GoalEndState}.
 *
 * <p>Hold the Y button to drive; release to cancel. Exits immediately if no spot is selected.
 *
 * <p>Alliance mirroring is handled automatically by the flip lambda registered in {@link
 * Drive#AutoBuilder.configure}.
 */
public class DriveToSpot extends Command {

  /** Distance (meters) below which the robot is considered already at the target. */
  private static final double AT_TARGET_THRESHOLD_M = 0.5;

  /**
   * How far before the trench X boundary the approach waypoint is placed (meters). A {@link
   * RotationTarget} at this point tells PathPlanner to complete the intake-leading rotation before
   * the robot enters the narrow trench.
   */
  private static final double TRENCH_APPROACH_M = 1.5;

  /**
   * How far past the trench X boundary the exit waypoint is placed (meters). The robot clears the
   * narrow section before the path curves toward the next milestone.
   */
  private static final double TRENCH_EXIT_M = 1.5;

  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private final Drive m_drive;
  private final SpotSelector m_spotSelector;

  private Command m_pathCommand = null;
  private boolean m_noTarget = false;

  public DriveToSpot(Drive drive, SpotSelector spotSelector) {
    m_drive = drive;
    m_spotSelector = spotSelector;
    // DriveToSpot does not directly require the drive subsystem; the scheduled
    // path command does. This avoids conflicting with the default drive command
    // while still allowing cancellation on release.
    addRequirements(spotSelector);
  }

  @Override
  public void initialize() {
    Optional<Pose2d> targetOpt = m_spotSelector.getSelectedPose();

    if (targetOpt.isEmpty()) {
      // No spot selected — finish immediately.
      m_noTarget = true;
      m_pathCommand = null;
      return;
    }

    m_noTarget = false;
    // All FieldSpot poses are in Blue-alliance coordinates. AutoBuilder.followPath() applies the
    // alliance flip registered in Drive.configure(), so no manual flipIfRed is needed here.
    Pose2d targetPose = targetOpt.get();
    Pose2d currentPose = m_drive.getPose();

    double distanceM = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    if (distanceM <= AT_TARGET_THRESHOLD_M) {
      // Already at the target; nothing to do.
      m_pathCommand = null;
      m_spotSelector.setAutoDriving(true);
      return;
    }

    PathPlannerPath path = buildPath(currentPose, targetPose);
    m_pathCommand = AutoBuilder.followPath(path);

    m_spotSelector.setAutoDriving(true);
    CommandScheduler.getInstance().schedule(m_pathCommand);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_spotSelector.setAutoDriving(false);
    if (m_pathCommand != null) {
      m_pathCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    if (m_noTarget) {
      return true;
    }
    // Run until the driver releases the button (command is interrupted).
    return false;
  }

  // ── Route building ──────────────────────────────────────────────────────

  /**
   * Builds a single {@link PathPlannerPath} from {@code start} to {@code target}, inserting trench
   * approach and exit waypoints for each zone boundary crossed.
   *
   * <p>A {@link RotationTarget} is placed at each approach waypoint to achieve the intake-leading
   * heading before the trench entrance.
   *
   * <p>Waypoint {@link Pose2d} rotations are set to the travel direction (path tangent), not the
   * robot heading. Robot heading is managed entirely through {@link RotationTarget}s, {@link
   * IdealStartingState}, and {@link GoalEndState}.
   */
  private PathPlannerPath buildPath(Pose2d start, Pose2d target) {
    List<Pose2d> poses = new ArrayList<>();
    List<RotationTarget> rotationTargets = new ArrayList<>();

    poses.add(start);

    Zone startZone = FieldSpot.getZone(start.getX());
    Zone targetZone = FieldSpot.getZone(target.getX());

    if (startZone != targetZone) {
      boolean lowY = start.getY() < FieldSpot.Y_SPLIT_M;
      boolean forwardX = target.getX() > start.getX();

      // Path tangent direction for trench waypoints — direction of travel, not robot
      // heading.
      Rotation2d travelDir = forwardX ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
      // Robot heading through the trench — 180° from travel so intake leads.
      Rotation2d trenchRobotHeading =
          forwardX ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

      double allianceTrenchY =
          lowY
              ? FieldSpot.ALLIANCE_TRENCH_RIGHT.pose.getY()
              : FieldSpot.ALLIANCE_TRENCH_LEFT.pose.getY();
      double opponentTrenchY =
          lowY
              ? FieldSpot.OPPONENT_TRENCH_RIGHT.pose.getY()
              : FieldSpot.OPPONENT_TRENCH_LEFT.pose.getY();

      if (forwardX) {
        if (startZone == Zone.ALLIANCE) {
          addTrenchWaypoints(
              poses,
              rotationTargets,
              FieldSpot.ALLIANCE_TRENCH_X_M,
              allianceTrenchY,
              travelDir,
              trenchRobotHeading,
              true);
        }
        if (targetZone == Zone.OPPOSITION) {
          addTrenchWaypoints(
              poses,
              rotationTargets,
              FieldSpot.OPPONENT_TRENCH_X_M,
              opponentTrenchY,
              travelDir,
              trenchRobotHeading,
              true);
        }
      } else {
        if (startZone == Zone.OPPOSITION) {
          addTrenchWaypoints(
              poses,
              rotationTargets,
              FieldSpot.OPPONENT_TRENCH_X_M,
              opponentTrenchY,
              travelDir,
              trenchRobotHeading,
              false);
        }
        if (targetZone == Zone.ALLIANCE) {
          addTrenchWaypoints(
              poses,
              rotationTargets,
              FieldSpot.ALLIANCE_TRENCH_X_M,
              allianceTrenchY,
              travelDir,
              trenchRobotHeading,
              false);
        }
      }

      Logger.recordOutput("DriveToSpot/TrenchRobotHeadingDeg", trenchRobotHeading.getDegrees());
    }

    poses.add(target);
    Logger.recordOutput("DriveToSpot/RoutePoses", poses.toArray(new Pose2d[0]));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
    return new PathPlannerPath(
        waypoints,
        rotationTargets,
        List.of(), // point-towards zones
        List.of(), // constraint zones
        List.of(), // event markers
        CONSTRAINTS,
        new IdealStartingState(0.0, start.getRotation()),
        new GoalEndState(0.0, target.getRotation()),
        false);
  }

  /**
   * Appends approach and exit waypoints for one trench crossing, and registers a {@link
   * RotationTarget} at the approach position.
   *
   * <p>The approach waypoint index in the waypoint list equals {@code poses.size()} before the
   * waypoint is added (since {@code waypointsFromPoses} produces one entry per pose).
   *
   * @param poses accumulating pose list
   * @param rotationTargets accumulating rotation-target list
   * @param trenchX X boundary of the trench (meters)
   * @param trenchY Y corridor of the trench (meters)
   * @param travelDir path tangent direction (direction of travel)
   * @param robotHeading desired robot heading through the trench (intake-leading)
   * @param forwardX true if traveling in the +X direction
   */
  private void addTrenchWaypoints(
      List<Pose2d> poses,
      List<RotationTarget> rotationTargets,
      double trenchX,
      double trenchY,
      Rotation2d travelDir,
      Rotation2d robotHeading,
      boolean forwardX) {
    // The approach waypoint will be at this index in the waypoints list.
    double approachIdx = poses.size();

    double approachX = forwardX ? trenchX - TRENCH_APPROACH_M : trenchX + TRENCH_APPROACH_M;
    double exitX = forwardX ? trenchX + TRENCH_EXIT_M : trenchX - TRENCH_EXIT_M;

    poses.add(new Pose2d(approachX, trenchY, travelDir));
    poses.add(new Pose2d(exitX, trenchY, travelDir));

    rotationTargets.add(new RotationTarget(approachIdx, robotHeading));
  }
}
