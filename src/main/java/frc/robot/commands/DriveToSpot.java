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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
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
 * <p>The field has three zones (Blue Home, Neutral, Red Home) separated by two trench boundaries.
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
 * Drive#AutoBuilder.configure}. All waypoints built here are in Blue-alliance coordinates;
 * PathPlanner converts them to Red when needed.
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
    // All FieldSpot poses are in Blue-alliance coordinates.
    // AutoBuilder.followPath() applies the
    // alliance flip registered in Drive.configure(), so no manual flipIfRed is
    // needed here.
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

    // buildPathRed computes waypoints at the robot's actual blue-frame coordinates.
    // AutoBuilder.followPath applies the registered alliance flip, which would mirror
    // those waypoints to the wrong side of the field.  Pre-flipping here makes
    // PathPlanner's flip cancel out so the robot follows the intended trajectory.
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
      path = path.flipPath();
    }

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
   * Dispatches to the alliance-specific path builder. All waypoints are expressed in Blue-alliance
   * coordinates; PathPlanner's registered flip lambda converts the entire path for Red.
   */
  private PathPlannerPath buildPath(Pose2d start, Pose2d target) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
      // FieldSpot poses are defined in Blue-alliance coordinates.
      // Convert the target to the mirrored Red-side position so buildPathRed
      // routes to the correct location on Red's side of the field.
      // (start is drive.getPose() — already in blue-frame at the robot's actual position.)
      return buildPathRed(start, FieldSpot.flipIfRed(target));
    }
    return buildPathBlue(start, target);
  }

  /**
   * Builds a {@link PathPlannerPath} for a <strong>Blue-alliance</strong> robot.
   *
   * <p>Blue's home zone is {@link Zone#BLUE_HOME} (X &lt; {@link FieldSpot#BLUE_TRENCH_X_M}).
   * Normal travel is in the +X direction toward the opponent side. All coordinates are in
   * Blue-alliance frame; PathPlanner does <em>not</em> flip for Blue.
   *
   * <p>Trench corridor (TOP vs BOTTOM) is chosen by the robot's starting Y: at or above {@link
   * FieldSpot#Y_SPLIT_M} uses the TOP corridor (high Y); below uses the BOTTOM corridor (low Y).
   */
  private PathPlannerPath buildPathBlue(Pose2d start, Pose2d target) {
    List<Pose2d> poses = new ArrayList<>();
    List<RotationTarget> rotTargets = new ArrayList<>();
    poses.add(start);

    Zone startZone = FieldSpot.getZone(start.getX());
    Zone targetZone = FieldSpot.getZone(target.getX());

    if (startZone != targetZone) {
      boolean topHalf = start.getY() >= FieldSpot.Y_SPLIT_M;
      double blueTrenchY =
          topHalf
              ? FieldSpot.BLUE_TRENCH_TOP.pose.getY() // 7.5 m — top corridor
              : FieldSpot.BLUE_TRENCH_BOTTOM.pose.getY(); // 0.5 m — bottom corridor
      double redTrenchY =
          topHalf ? FieldSpot.RED_TRENCH_TOP.pose.getY() : FieldSpot.RED_TRENCH_BOTTOM.pose.getY();

      boolean forwardX = target.getX() > start.getX();
      // Path tangent direction: 0° for +X travel, 180° for -X travel.
      Rotation2d travelDir = forwardX ? Rotation2d.kZero : Rotation2d.k180deg;
      // Robot heading through the trench: intake leads (180° from travel direction).
      Rotation2d trenchHeading = forwardX ? Rotation2d.k180deg : Rotation2d.kZero;

      if (forwardX) {
        // Blue moving +X: leaving BLUE_HOME crosses the blue trench;
        // entering RED_HOME crosses the red trench.
        if (startZone == Zone.BLUE_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.BLUE_TRENCH_X_M,
              blueTrenchY,
              travelDir,
              trenchHeading,
              true);
        }
        if (targetZone == Zone.RED_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.RED_TRENCH_X_M,
              redTrenchY,
              travelDir,
              trenchHeading,
              true);
        }
      } else {
        // Blue moving -X (uncommon): leaving RED_HOME crosses the red trench;
        // entering BLUE_HOME crosses the blue trench.
        if (startZone == Zone.RED_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.RED_TRENCH_X_M,
              redTrenchY,
              travelDir,
              trenchHeading,
              false);
        }
        if (targetZone == Zone.BLUE_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.BLUE_TRENCH_X_M,
              blueTrenchY,
              travelDir,
              trenchHeading,
              false);
        }
      }

      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput("DriveToSpot/TrenchHeadingDeg", trenchHeading.getDegrees());
      }
    }

    poses.add(target);
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("DriveToSpot/RoutePoses", poses.toArray(new Pose2d[0]));
    }
    return assemblePath(poses, rotTargets, start, target);
  }

  /**
   * Builds a {@link PathPlannerPath} for a <strong>Red-alliance</strong> robot.
   *
   * <p>Red's home zone is {@link Zone#RED_HOME} (X &gt; {@link FieldSpot#RED_TRENCH_X_M} in
   * Blue-alliance frame). Normal travel toward the shooting positions or hub is in the -X direction
   * in blue-frame (which becomes +X after PathPlanner's alliance flip for Red).
   *
   * <p>All coordinates are expressed in Blue-alliance frame. PathPlanner flips the entire completed
   * path — including all trench waypoints — when executing for Red. The relative order (approach →
   * trench → exit) is preserved correctly after the flip.
   *
   * <p>Trench corridor selection (TOP vs BOTTOM) is based on the robot's Y in blue-frame. Because
   * the flip inverts Y, a robot at high Y in blue-frame (TOP) ends up at low Y in red-frame (also
   * TOP from the physical field perspective), and the TOP trench waypoint flips to the same
   * physical corridor. No inversion of the corridor selection is needed.
   */
  private PathPlannerPath buildPathRed(Pose2d start, Pose2d target) {
    List<Pose2d> poses = new ArrayList<>();
    List<RotationTarget> rotTargets = new ArrayList<>();
    poses.add(start);

    Zone startZone = FieldSpot.getZone(start.getX());
    Zone targetZone = FieldSpot.getZone(target.getX());

    if (startZone != targetZone) {
      boolean topHalf = start.getY() >= FieldSpot.Y_SPLIT_M;
      double blueTrenchY =
          topHalf
              ? FieldSpot.BLUE_TRENCH_TOP.pose.getY()
              : FieldSpot.BLUE_TRENCH_BOTTOM.pose.getY();
      double redTrenchY =
          topHalf ? FieldSpot.RED_TRENCH_TOP.pose.getY() : FieldSpot.RED_TRENCH_BOTTOM.pose.getY();

      boolean forwardX = target.getX() > start.getX();
      Rotation2d travelDir = forwardX ? Rotation2d.kZero : Rotation2d.k180deg;
      Rotation2d trenchHeading = forwardX ? Rotation2d.k180deg : Rotation2d.kZero;

      if (forwardX) {
        // Red going +X in blue-frame = heading toward its own home (RED_HOME) or
        // opponent side.
        if (startZone == Zone.BLUE_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.BLUE_TRENCH_X_M,
              blueTrenchY,
              travelDir,
              trenchHeading,
              true);
        }
        if (targetZone == Zone.RED_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.RED_TRENCH_X_M,
              redTrenchY,
              travelDir,
              trenchHeading,
              true);
        }
      } else {
        // Red going -X in blue-frame = leaving RED_HOME toward neutral/blue (most
        // common for Red).
        // After PathPlanner flip, -X blue-frame becomes +X red-frame (toward opponent).
        if (startZone == Zone.RED_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.RED_TRENCH_X_M,
              redTrenchY,
              travelDir,
              trenchHeading,
              false);
        }
        if (targetZone == Zone.BLUE_HOME) {
          addTrenchWaypoints(
              poses,
              rotTargets,
              FieldSpot.BLUE_TRENCH_X_M,
              blueTrenchY,
              travelDir,
              trenchHeading,
              false);
        }
      }

      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput("DriveToSpot/TrenchHeadingDeg", trenchHeading.getDegrees());
      }
    }

    poses.add(target);
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("DriveToSpot/RoutePoses", poses.toArray(new Pose2d[0]));
    }
    return assemblePath(poses, rotTargets, start, target);
  }

  /**
   * Constructs a {@link PathPlannerPath} from the accumulated pose list and rotation targets.
   *
   * @param poses waypoint poses (Blue-alliance frame); first = start, last = target
   * @param rotTargets rotation targets to lock heading at specific waypoints
   * @param start robot starting pose (for {@link IdealStartingState})
   * @param target final target pose (for {@link GoalEndState})
   */
  private PathPlannerPath assemblePath(
      List<Pose2d> poses, List<RotationTarget> rotTargets, Pose2d start, Pose2d target) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
    return new PathPlannerPath(
        waypoints,
        rotTargets,
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
   * @param rotTargets accumulating rotation-target list
   * @param trenchX X boundary of the trench (meters, blue-frame)
   * @param trenchY Y corridor of the trench (meters, blue-frame)
   * @param travelDir path tangent direction (direction of travel)
   * @param robotHeading desired robot heading through the trench (intake-leading)
   * @param forwardX true if traveling in the +X direction
   */
  private void addTrenchWaypoints(
      List<Pose2d> poses,
      List<RotationTarget> rotTargets,
      double trenchX,
      double trenchY,
      Rotation2d travelDir,
      Rotation2d robotHeading,
      boolean forwardX) {
    double approachX = forwardX ? trenchX - TRENCH_APPROACH_M : trenchX + TRENCH_APPROACH_M;
    double exitX = forwardX ? trenchX + TRENCH_EXIT_M : trenchX - TRENCH_EXIT_M;

    double approachIdx = poses.size();
    poses.add(new Pose2d(approachX, trenchY, travelDir));
    rotTargets.add(new RotationTarget(approachIdx, robotHeading));
    poses.add(new Pose2d(exitX, trenchY, travelDir));
  }
}
