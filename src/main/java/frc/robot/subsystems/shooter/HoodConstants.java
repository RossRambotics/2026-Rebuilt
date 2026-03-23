package frc.robot.subsystems.shooter;

/** Constants for the Hood subsystem shared across implementations. */
public final class HoodConstants {
  private HoodConstants() {}

  // Hood mechanical angle limits in degrees.
  // Convention: 0° = hood surface horizontal = ball launches straight up (90° above horizontal).
  // The ball always exits perpendicular to the hood surface, so launch angle = 90° − hood angle.
  public static final int HOOD_DEGREES_MIN = 15;
  public static final int HOOD_DEGREES_MAX = 45;
}
