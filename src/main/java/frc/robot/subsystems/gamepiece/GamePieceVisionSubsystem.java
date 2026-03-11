package frc.robot.subsystems.gamepiece;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.TimedSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that wraps the intake-cam object-detection camera for game piece tracking.
 *
 * <p>Each cycle, all accepted detections are combined into an area-weighted centroid yaw and
 * smoothed with an exponential moving average (EMA) to prevent frame-to-frame flipping when the
 * bounding-box distribution shifts between detections.
 *
 * <p>Area-weighted centroid naturally handles the "closest vs. most pieces" tradeoff:
 *
 * <ul>
 *   <li>Bounding-box area scales roughly as 1/distance², so a close piece dominates over a distant
 *       cluster purely by area.
 *   <li>A cluster of N pieces contributes N× the combined area of one isolated piece at the same
 *       distance, steering toward the densest grouping.
 * </ul>
 *
 * <p>All inputs are logged via AdvantageKit for full replay support.
 */
public class GamePieceVisionSubsystem extends TimedSubsystem {

  private final GamePieceVisionIO m_io;
  private final GamePieceVisionIOInputsAutoLogged m_inputs =
      new GamePieceVisionIOInputsAutoLogged();

  /** EMA state for the area-weighted centroid yaw (degrees). */
  private double m_smoothedYawDeg = 0.0;

  private final Alert m_disconnectedAlert =
      new Alert("GamePieceVision (intake-cam) is disconnected.", AlertType.kWarning);

  public GamePieceVisionSubsystem(GamePieceVisionIO io) {
    m_io = io;
  }

  @Override
  protected void periodicImpl() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("GamePieceVision", m_inputs);

    m_disconnectedAlert.set(!m_inputs.connected);

    // Compute area-weighted centroid yaw across all accepted detections this cycle.
    double totalAreaPercent = 0.0;
    double weightedYawDeg = 0.0;
    for (int i = 0; i < m_inputs.targetYawsDeg.length; i++) {
      double area = m_inputs.targetAreaPercents[i];
      weightedYawDeg += m_inputs.targetYawsDeg[i] * area;
      totalAreaPercent += area;
    }

    if (totalAreaPercent > 0.0) {
      double rawCentroidYawDeg = weightedYawDeg / totalAreaPercent;
      // Apply EMA: smooths rapid distribution shifts without slugging real movement.
      m_smoothedYawDeg =
          GamePieceVisionConstants.YAW_EMA_ALPHA * rawCentroidYawDeg
              + (1.0 - GamePieceVisionConstants.YAW_EMA_ALPHA) * m_smoothedYawDeg;
    } else {
      // No targets visible — reset to 0 so stale EMA state doesn't bias the first
      // correction when a target is reacquired.
      m_smoothedYawDeg = 0.0;
    }

    Logger.recordOutput("GamePieceVision/IsReadyToChase", isReadyToChase());
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("GamePieceVision/TargetCount", m_inputs.targetYawsDeg.length);
      Logger.recordOutput("GamePieceVision/TotalAreaPercent", totalAreaPercent);
      Logger.recordOutput("GamePieceVision/SmoothedYawDeg", m_smoothedYawDeg);
      Logger.recordOutput("GamePieceVision/IsStubbed", !(m_io instanceof GamePieceVisionIOPhoton));
      Logger.recordOutput("GamePieceVision/IsStale", isStale());
    }
  }

  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

  /**
   * Returns true if at least one game piece was detected in the most recent camera frame.
   *
   * <p>Does not account for staleness — use {@link #isReadyToChase()} for command decisions.
   */
  public boolean hasTarget() {
    return m_inputs.targetYawsDeg.length > 0;
  }

  /**
   * Returns the EMA-smoothed area-weighted centroid yaw to the detected game piece group (degrees).
   *
   * <p>Positive = centroid is to the right of camera center. Returns 0.0 when no targets are
   * detected (check {@link #isReadyToChase()} before using this value).
   */
  public double getYawRobotDeg() {
    return m_smoothedYawDeg;
  }

  /**
   * Returns the sum of all accepted target areas as a percentage of image area.
   *
   * <p>Higher values indicate either a closer target or a denser cluster. Can be used as a proxy
   * for grouping size or proximity.
   */
  public double getTotalAreaPercent() {
    double total = 0.0;
    for (double area : m_inputs.targetAreaPercents) {
      total += area;
    }
    return total;
  }

  /**
   * Returns true if the latest result timestamp is older than the staleness threshold.
   *
   * <p>A stale reading means no new camera frames have arrived recently. The command stops forward
   * motion when stale to avoid driving blind.
   */
  public boolean isStale() {
    return (Timer.getFPGATimestamp() - m_inputs.latestResultTimestampSec)
        > GamePieceVisionConstants.DETECTION_STALENESS_SEC;
  }

  /**
   * Returns true when the subsystem has a fresh, valid target that the command can act on.
   *
   * <p>This is the primary predicate used by {@link frc.robot.commands.AutoDriveToFuel}.
   */
  public boolean isReadyToChase() {
    return !isStale() && hasTarget();
  }

  /** Returns true if the camera is connected to the coprocessor. */
  public boolean isConnected() {
    return m_inputs.connected;
  }
}
