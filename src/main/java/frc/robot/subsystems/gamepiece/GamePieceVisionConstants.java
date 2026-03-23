package frc.robot.subsystems.gamepiece;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Configuration constants for the intake-cam object detection subsystem and AutoDriveToFuel. */
public final class GamePieceVisionConstants {
  private GamePieceVisionConstants() {}

  /** Camera name — must match the name configured on the coprocessor. */
  public static final String CAMERA_NAME = "intake-cam";

  /**
   * Robot-to-camera transform for the intake-cam.
   *
   * <p>Assumes a forward-facing camera mounted at the front of the robot, tilted down toward the
   * ground to see game pieces on the floor.
   *
   * <p>TODO: Measure and update these values on hardware before competition.
   *
   * <ul>
   *   <li>x = -30 cm (behind robot center, toward the intake)
   *   <li>y = 0 cm (centered left-right)
   *   <li>z = 25 cm above ground
   *   <li>pitch = -30 deg (tilted down toward the floor)
   *   <li>yaw = 180 deg (camera faces backward)
   * </ul>
   */
  public static final Transform3d ROBOT_TO_INTAKE_CAM =
      new Transform3d(
          -0.30,
          0.0,
          0.25,
          new Rotation3d(0.0, Degrees.of(-30).in(Radians), Degrees.of(180).in(Radians)));

  /**
   * Object detection class ID for the fuel game piece.
   *
   * <p>TODO: Verify this index against the trained pipeline's class list on the coprocessor.
   */
  public static final int FUEL_CLASS_ID = 0;

  /** Minimum detection confidence to accept a target (0.0–1.0). */
  public static final float MIN_CONFIDENCE = 0.5f;

  /**
   * Minimum bounding-box area (% of image) to accept a target.
   *
   * <p>Filters out distant or noisy detections that are too small to act on.
   */
  public static final double MIN_TARGET_AREA_PERCENT = 0.5;

  /**
   * Age threshold for staleness check (seconds).
   *
   * <p>If no new camera result has arrived within this window, the subsystem reports stale and the
   * command stops forward motion.
   */
  public static final double DETECTION_STALENESS_SEC = 0.5;

  // ---------------------------------------------------------------------------
  // Multi-target centroid strategy
  //
  // All accepted detections contribute to an area-weighted average yaw.  A
  // cluster of N pieces produces N× the combined area of one isolated piece, so
  // the centroid naturally steers toward the densest grouping.  Proximity is
  // also captured because bounding-box area scales roughly as 1/distance², so a
  // very close piece dominates over a distant cluster.
  // ---------------------------------------------------------------------------

  /**
   * Exponential moving average (EMA) smoothing factor for the weighted-centroid yaw.
   *
   * <p>Range [0, 1]: 0 = frozen (never updates), 1 = no smoothing (raw value each cycle). A value
   * of ~0.35 responds meaningfully within a few cycles while filtering frame-to-frame distribution
   * shifts that would otherwise cause the robot to oscillate between targets.
   */
  public static final double YAW_EMA_ALPHA = 0.35;

  // ---------------------------------------------------------------------------
  // AutoDriveToFuel drive tuning
  // ---------------------------------------------------------------------------

  /**
   * Proportional gain for yaw correction (rad/s per degree of yaw error).
   *
   * <p>At 10 deg error this produces 1.0 rad/s. Sign note: positive camera yaw means the target is
   * to the right of center; for a forward-facing camera with WPILib CCW-positive convention, the
   * omega correction is negated so the robot turns clockwise (right) toward the piece.
   */
  public static final double YAW_KP = 0.10;

  /** Maximum rotational rate commanded during a fuel chase (rad/s). */
  public static final double MAX_OMEGA_RAD_PER_SEC = 2.0;

  /** Forward drive speed while chasing a detected game piece (m/s). */
  public static final double CHASE_SPEED_MPS = 1.5;

  /**
   * Forward drive speed when the target is lost (m/s).
   *
   * <p>Set to 0.0 to stop the robot immediately when no target is visible, preventing blind
   * driving. The command resumes chasing if the target is reacquired the next cycle.
   */
  public static final double TARGET_LOST_COAST_MPS = 0.0;

  /**
   * Safety timeout for AutoDriveToFuel (seconds).
   *
   * <p>The command ends automatically after this duration even if the driver is still holding the
   * button, preventing the robot from chasing indefinitely.
   */
  public static final double CHASE_TIMEOUT_SEC = 5.0;
}
