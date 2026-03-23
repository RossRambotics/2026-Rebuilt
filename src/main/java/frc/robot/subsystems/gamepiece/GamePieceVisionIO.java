package frc.robot.subsystems.gamepiece;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the intake-cam object detection camera.
 *
 * <p>All detected targets are reported as parallel arrays so AdvantageKit logs every raw detection
 * for replay analysis. The {@link GamePieceVisionSubsystem} computes the area-weighted centroid
 * from these arrays.
 */
public interface GamePieceVisionIO {

  @AutoLog
  public static class GamePieceVisionIOInputs {
    /** True when the camera coprocessor is connected and responding. */
    public boolean connected = false;

    /**
     * Horizontal angle to each accepted target in the camera frame (degrees).
     *
     * <p>Positive = target is to the right of center in the image.
     */
    public double[] targetYawsDeg = new double[0];

    /**
     * Vertical angle to each accepted target in the camera frame (degrees).
     *
     * <p>Positive = target is above center in the image.
     */
    public double[] targetPitchesDeg = new double[0];

    /**
     * Bounding-box area of each accepted target as a percentage of total image area (0–100).
     *
     * <p>Larger area indicates a closer or larger target. Used for area-weighted centroid
     * computation.
     */
    public double[] targetAreaPercents = new double[0];

    /** Detection confidence for each accepted target (0.0–1.0). */
    public float[] targetConfidences = new float[0];

    /** Object class ID for each accepted target, matching the pipeline's trained class list. */
    public int[] targetClassIds = new int[0];

    /**
     * FPGA timestamp of the most recent camera result (seconds).
     *
     * <p>Used by the subsystem to detect stale readings when no new frames arrive.
     */
    public double latestResultTimestampSec = 0.0;
  }

  public default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
