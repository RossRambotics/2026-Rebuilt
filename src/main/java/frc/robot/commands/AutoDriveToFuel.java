package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.GamePieceVisionConstants;
import frc.robot.subsystems.gamepiece.GamePieceVisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/**
 * Autonomously drives toward detected game pieces using yaw-servo proportional steering.
 *
 * <p>The target yaw is the EMA-smoothed area-weighted centroid of all detected game pieces,
 * computed by {@link GamePieceVisionSubsystem}. This naturally steers toward the densest nearby
 * grouping while close/large pieces dominate via their larger bounding-box area.
 *
 * <h3>Behavior</h3>
 *
 * <ul>
 *   <li><b>initialize:</b> deploys the intake.
 *   <li><b>execute:</b> drives forward at {@code CHASE_SPEED_MPS} while applying a proportional
 *       omega correction to steer toward the detected centroid. Stops if no target is visible.
 *   <li><b>end:</b> stops the drive. Intake stays deployed — driver retracts manually.
 *   <li><b>isFinished:</b> only the safety timeout — the driver terminates by releasing the button.
 * </ul>
 *
 * <h3>Yaw sign convention (rear-facing camera)</h3>
 *
 * <p>The intake-cam faces backward (yaw = 180°). PhotonVision reports yaw in the camera frame, so
 * positive camera yaw means the piece is to the camera's right = the robot's <em>left</em>. To
 * steer the rear intake toward the piece the robot must turn left (CCW = positive omega). Therefore
 * the omega correction is <em>positive</em>: {@code +yawErrorDeg * YAW_KP}. The robot also drives
 * backward ({@code vx = -CHASE_SPEED_MPS}) so the rear intake approaches the piece.
 *
 * <h3>Recommended binding</h3>
 *
 * <pre>
 *   controller.b().whileTrue(new AutoDriveToFuel(drive, intake, gamePieceVision));
 * </pre>
 */
public class AutoDriveToFuel extends Command {

  private final Drive m_drive;
  private final Intake m_intake;
  private final GamePieceVisionSubsystem m_gamePieceVision;

  private final Timer m_timeoutTimer = new Timer();

  /** The deploy sub-command scheduled in initialize(), cancelled in end(). */
  private Command m_deployCommand = null;

  /**
   * Creates a new AutoDriveToFuel command.
   *
   * @param drive The Drive subsystem (required — prevents conflict with default drive command).
   * @param intake The Intake subsystem (required — prevents conflict with manual intake commands).
   * @param gamePieceVision The game piece vision subsystem (sensor only, not required).
   */
  public AutoDriveToFuel(Drive drive, Intake intake, GamePieceVisionSubsystem gamePieceVision) {
    m_drive = drive;
    m_intake = intake;
    m_gamePieceVision = gamePieceVision;
    // Require drive and intake. GamePieceVisionSubsystem is sensor-only.
    addRequirements(drive, intake);
  }

  @Override
  public void initialize() {
    m_timeoutTimer.restart();

    // Deploy intake immediately. Schedule as a sub-command so it runs in parallel
    // with the drive loop without blocking execute().
    m_deployCommand = m_intake.getDeployCmd();
    CommandScheduler.getInstance().schedule(m_deployCommand);

    Logger.recordOutput("AutoDriveToFuel/State", "Initialized");
    Logger.recordOutput("AutoDriveToFuel/EndReason", "None");
  }

  @Override
  public void execute() {
    boolean hasTarget = m_gamePieceVision.isReadyToChase();

    double forwardSpeedMps;
    double omegaRadPerSec;

    if (hasTarget) {
      double yawErrorDeg = m_gamePieceVision.getYawRobotDeg();
      // Rear-facing camera: positive camera yaw = piece is to camera's right = robot's LEFT.
      // Turn left (CCW = positive omega) to align the rear intake with the piece.
      omegaRadPerSec =
          MathUtil.clamp(
              yawErrorDeg * GamePieceVisionConstants.YAW_KP,
              -GamePieceVisionConstants.MAX_OMEGA_RAD_PER_SEC,
              GamePieceVisionConstants.MAX_OMEGA_RAD_PER_SEC);
      // Drive backward so the rear intake approaches the piece.
      forwardSpeedMps = -GamePieceVisionConstants.CHASE_SPEED_MPS;

      Logger.recordOutput("AutoDriveToFuel/State", "Chasing");
      Logger.recordOutput("AutoDriveToFuel/YawErrorDeg", yawErrorDeg);
    } else {
      // No target visible — stop to avoid driving blind.
      omegaRadPerSec = 0.0;
      forwardSpeedMps = GamePieceVisionConstants.TARGET_LOST_COAST_MPS;

      Logger.recordOutput("AutoDriveToFuel/State", "TargetLost");
      Logger.recordOutput("AutoDriveToFuel/YawErrorDeg", 0.0);
    }

    // Robot-centric speeds: vx is negative (backward toward intake), vomega steers rear to target.
    m_drive.runVelocity(new ChassisSpeeds(forwardSpeedMps, 0.0, omegaRadPerSec));

    Logger.recordOutput("AutoDriveToFuel/ForwardSpeedMps", forwardSpeedMps);
    Logger.recordOutput("AutoDriveToFuel/OmegaRadPerSec", omegaRadPerSec);
    Logger.recordOutput("AutoDriveToFuel/HasTarget", hasTarget);
    Logger.recordOutput("AutoDriveToFuel/TimeoutElapsedSec", m_timeoutTimer.get());
  }

  @Override
  public boolean isFinished() {
    // The driver terminates by releasing the button (whileTrue binding).
    // Safety timeout prevents indefinite running if the button is held too long.
    if (m_timeoutTimer.hasElapsed(GamePieceVisionConstants.CHASE_TIMEOUT_SEC)) {
      Logger.recordOutput("AutoDriveToFuel/EndReason", "Timeout");
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drive.
    m_drive.runVelocity(new ChassisSpeeds());

    // Cancel the deploy sub-command if it is still running.
    if (m_deployCommand != null) {
      m_deployCommand.cancel();
      m_deployCommand = null;
    }

    Logger.recordOutput("AutoDriveToFuel/State", interrupted ? "Interrupted" : "Completed");
  }
}
