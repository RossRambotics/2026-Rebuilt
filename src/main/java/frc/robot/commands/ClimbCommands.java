// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbElevator;
import frc.robot.subsystems.climb.ClimbElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Command factories for the climb mechanism.
 *
 * <p>All poses are defined for the Blue alliance; {@link #flipIfRed} converts them to Red when the
 * Driver Station reports Red alliance.
 */
public final class ClimbCommands {

  private ClimbCommands() {}

  /**
   * Builds the SuperFastClimb command sequence for one approach spot.
   *
   * <p>Sequence:
   *
   * <ol>
   *   <li>Pathfind to the chosen {@code approachSpot} using PathPlanner on-the-fly pathfinding.
   *   <li>Extend the elevator arm to MAX_HEIGHT (with a safety timeout).
   *   <li>Race: drive {@value ClimbElevatorConstants#CLIMB_PASS_DISTANCE_M}m past the tower vs. a
   *       tilt sentinel vs. elevator hold. The tilt sentinel ends the race when roll+pitch
   *       magnitude exceeds {@value ClimbElevatorConstants#TILT_HOOK_THRESHOLD_RAD} radians,
   *       indicating the arm has hooked the bar.
   *   <li>Stop the drive immediately.
   *   <li>If the tilt sentinel triggered: retract the arm (winch the robot up). Otherwise (drive
   *       finished first): hold arm at max height and let the driver decide.
   * </ol>
   *
   * <p>When bound with {@code whileTrue()}, releasing the button cancels the sequence and pressing
   * again restarts it from Phase 1.
   *
   * @param drive the Drive subsystem
   * @param climb the ClimbElevator subsystem
   * @param approachSpot the target approach {@link FieldSpot} (Blue-alliance coordinates)
   * @return the full climb command sequence
   */
  public static Command superFastClimb(Drive drive, ClimbElevator climb, FieldSpot approachSpot) {
    // Mutable flag captured by lambdas — set true only when the tilt sentinel wins the race.
    boolean[] tiltHooked = {false};

    return Commands.sequence(
            // Reset abort flag each time the command is freshly scheduled.
            Commands.runOnce(
                () -> {
                  tiltHooked[0] = false;
                  if (Constants.VERBOSE_TELEMETRY) {
                    Logger.recordOutput("Climb/SuperFast/TiltHooked", false);
                  }
                }),

            // Phase 1: Pathfind to the approach pose.
            buildApproachCmd(approachSpot),

            // Phase 2: Extend elevator to MAX_HEIGHT.
            buildExtendCmd(climb),

            // Phase 3: Race — drive past tower vs. tilt sentinel vs. arm hold.
            buildPassRaceCmd(drive, climb, approachSpot, tiltHooked),

            // Phase 4: Stop the drive unconditionally.
            Commands.runOnce(drive::stop, drive),

            // Phase 5: Robot climb (lift off ground) if tilt hooked; hold arm if drive finished
            // first.
            Commands.either(climb.getRobotClimbCmd(), Commands.none(), () -> tiltHooked[0]))
        .withName("SuperFastClimb/" + approachSpot.displayName);
  }

  // ---------------------------------------------------------------------------
  // Phase helpers
  // ---------------------------------------------------------------------------

  /** Phase 1: pathfind to the approach pose at full speed. */
  private static Command buildApproachCmd(FieldSpot approachSpot) {
    PathConstraints approachConstraints =
        new PathConstraints(
            ClimbElevatorConstants.CLIMB_APPROACH_MAX_SPEED_MPS,
            ClimbElevatorConstants.CLIMB_APPROACH_MAX_ACCEL_MPS2,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    Pose2d targetPose = flipIfRed(approachSpot.pose);
    return AutoBuilder.pathfindToPose(targetPose, approachConstraints)
        .beforeStarting(
            () -> {
              if (Constants.VERBOSE_TELEMETRY) {
                Logger.recordOutput("Climb/SuperFast/ApproachTarget", targetPose);
              }
            });
  }

  /**
   * Phase 2: extend elevator until at max height or timeout.
   *
   * <p>{@link ClimbElevator#getExtendCmd()} is an infinite hold-position command; the {@code
   * until()} converts it to a one-shot that completes once the elevator arrives.
   */
  private static Command buildExtendCmd(ClimbElevator climb) {
    return climb
        .getExtendCmd()
        .until(
            () ->
                Math.abs(climb.getHeight().in(Meters) - ClimbElevatorConstants.MAX_HEIGHT_M)
                    < ClimbElevatorConstants.AT_HEIGHT_TOLERANCE_M)
        .withTimeout(ClimbElevatorConstants.CLIMB_EXTEND_TIMEOUT_SEC);
  }

  /**
   * Phase 3: race between three commands.
   *
   * <ul>
   *   <li>Pass drive — PathPlanner drives 1m past the approach pose.
   *   <li>Tilt sentinel — ends naturally when tilt threshold exceeded, setting {@code
   *       tiltHooked[0]=true}.
   *   <li>Arm hold — {@link ClimbElevator#getExtendCmd()} (infinite, keeps arm commanded to
   *       MAX_HEIGHT throughout the pass).
   * </ul>
   *
   * <p>{@link Commands#race} ends when any participant finishes; the others are interrupted. If the
   * pass drive finishes first, the tilt sentinel is interrupted (flag stays false). If the tilt
   * sentinel finishes first, the drive is interrupted (flag is true).
   */
  private static Command buildPassRaceCmd(
      Drive drive, ClimbElevator climb, FieldSpot approachSpot, boolean[] tiltHooked) {
    PathConstraints passConstraints =
        new PathConstraints(
            ClimbElevatorConstants.CLIMB_PASS_MAX_SPEED_MPS,
            ClimbElevatorConstants.CLIMB_PASS_MAX_ACCEL_MPS2,
            Units.degreesToRadians(360),
            Units.degreesToRadians(540));

    Pose2d approachPose = flipIfRed(approachSpot.pose);
    Pose2d passPose = computePassPose(approachPose);

    Command passCmd =
        AutoBuilder.pathfindToPose(passPose, passConstraints)
            .beforeStarting(
                () -> {
                  if (Constants.VERBOSE_TELEMETRY) {
                    Logger.recordOutput("Climb/SuperFast/PassTarget", passPose);
                  }
                });

    Command tiltSentinel =
        Commands.waitUntil(
                () -> drive.getTiltMagnitudeRad() > ClimbElevatorConstants.TILT_HOOK_THRESHOLD_RAD)
            .finallyDo(
                (interrupted) -> {
                  if (!interrupted) {
                    // Sentinel ended naturally — tilt threshold crossed.
                    tiltHooked[0] = true;
                    if (Constants.VERBOSE_TELEMETRY) {
                      Logger.recordOutput("Climb/SuperFast/TiltHooked", true);
                      Logger.recordOutput(
                          "Climb/SuperFast/TiltMagAtHookRad", drive.getTiltMagnitudeRad());
                    }
                  }
                });

    return Commands.race(passCmd, tiltSentinel, climb.getExtendCmd());
  }

  // ---------------------------------------------------------------------------
  // Geometry helpers
  // ---------------------------------------------------------------------------

  /**
   * Computes the pose {@link ClimbElevatorConstants#CLIMB_PASS_DISTANCE_M} meters forward from the
   * approach pose in its heading direction.
   *
   * <p>{@link Transform2d} applies in the pose's own frame, so a 90°-heading pose moves along +Y.
   *
   * @param approachPose the alliance-correct approach pose
   * @return pose 1m forward in the approach heading direction
   */
  private static Pose2d computePassPose(Pose2d approachPose) {
    return approachPose.transformBy(
        new Transform2d(ClimbElevatorConstants.CLIMB_PASS_DISTANCE_M, 0.0, Rotation2d.kZero));
  }

  private static Pose2d flipIfRed(Pose2d pose) {
    return FieldSpot.flipIfRed(pose);
  }
}
