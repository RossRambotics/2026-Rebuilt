package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem backed by a CTRE CANdle. Runs in all robot modes including disabled.
 *
 * <p>LED layout (308 total):
 *
 * <ul>
 *   <li>Onboard 0–7: always solid green
 *   <li>Group 1 (8–57): left front limelight status; 48–57 spacer (off)
 *   <li>Group 2 (58–107): shooter status left; 98–107 spacer (off)
 *   <li>Group 3 (108–157): game piece tracking left; 148–157 spacer (off)
 *   <li>Group 4 (158–207): right front limelight status; 198–207 spacer (off)
 *   <li>Group 5 (208–257): shooter status right; 248–257 spacer (off)
 *   <li>Group 6 (258–307): game piece tracking right; 298–307 spacer (off)
 * </ul>
 */
public class LEDSubsystem extends SubsystemBase {

  private final boolean m_isReal;
  private final Vision m_vision;
  private final Shooter m_shooter;

  // Hardware — null when not real
  private final CANdle m_candle;

  // Throttle: run LED state logic at 10 Hz (every 5 cycles of the 50 Hz loop)
  private static final int UPDATE_PERIOD_CYCLES = 5;
  private int m_periodicCount = 0;

  // Pre-allocated SolidColor controls for active groups (reused every cycle, avoids per-cycle GC)
  private final SolidColor m_solidGroup1 = new SolidColor(GROUP1_ACTIVE_START, GROUP1_ACTIVE_END);
  private final SolidColor m_solidGroup2 = new SolidColor(GROUP2_ACTIVE_START, GROUP2_ACTIVE_END);
  private final SolidColor m_solidGroup3 = new SolidColor(GROUP3_ACTIVE_START, GROUP3_ACTIVE_END);
  private final SolidColor m_solidGroup4 = new SolidColor(GROUP4_ACTIVE_START, GROUP4_ACTIVE_END);
  private final SolidColor m_solidGroup5 = new SolidColor(GROUP5_ACTIVE_START, GROUP5_ACTIVE_END);
  private final SolidColor m_solidGroup6 = new SolidColor(GROUP6_ACTIVE_START, GROUP6_ACTIVE_END);

  // Pre-allocated StrobeAnimation controls with fixed slot and frame-rate assignments
  private final StrobeAnimation m_strobeGroup2 =
      new StrobeAnimation(GROUP2_ACTIVE_START, GROUP2_ACTIVE_END)
          .withSlot(SLOT_SHOOTER_LEFT)
          .withFrameRate(STROBE_FRAME_RATE_HZ);
  private final StrobeAnimation m_strobeGroup3 =
      new StrobeAnimation(GROUP3_ACTIVE_START, GROUP3_ACTIVE_END)
          .withSlot(SLOT_GAMEPIECE_LEFT)
          .withFrameRate(STROBE_FRAME_RATE_HZ);
  private final StrobeAnimation m_strobeGroup5 =
      new StrobeAnimation(GROUP5_ACTIVE_START, GROUP5_ACTIVE_END)
          .withSlot(SLOT_SHOOTER_RIGHT)
          .withFrameRate(STROBE_FRAME_RATE_HZ);
  private final StrobeAnimation m_strobeGroup6 =
      new StrobeAnimation(GROUP6_ACTIVE_START, GROUP6_ACTIVE_END)
          .withSlot(SLOT_GAMEPIECE_RIGHT)
          .withFrameRate(STROBE_FRAME_RATE_HZ);

  /**
   * Creates the LED subsystem.
   *
   * @param isReal true when running on real hardware; false in sim/replay (stubs all CANdle calls)
   * @param vision Vision subsystem used to query per-camera AprilTag counts
   * @param shooter Shooter subsystem used to query readiness and shooting state
   */
  public LEDSubsystem(boolean isReal, Vision vision, Shooter shooter) {
    m_isReal = isReal;
    m_vision = vision;
    m_shooter = shooter;

    if (m_isReal) {
      m_candle = new CANdle(CANDLE_CAN_ID);
      var config = new CANdleConfiguration();
      config.LED.StripType = StripTypeValue.GRB;
      config.LED.BrightnessScalar = 1.0;
      m_candle.getConfigurator().apply(config);

      // Onboard LEDs and spacers never change — set them once here instead of every periodic.
      m_candle.setControl(new SolidColor(ONBOARD_START, ONBOARD_END).withColor(GREEN));
      m_candle.setControl(new SolidColor(GROUP1_SPACER_START, GROUP1_SPACER_END).withColor(OFF));
      m_candle.setControl(new SolidColor(GROUP2_SPACER_START, GROUP2_SPACER_END).withColor(OFF));
      m_candle.setControl(new SolidColor(GROUP3_SPACER_START, GROUP3_SPACER_END).withColor(OFF));
      m_candle.setControl(new SolidColor(GROUP4_SPACER_START, GROUP4_SPACER_END).withColor(OFF));
      m_candle.setControl(new SolidColor(GROUP5_SPACER_START, GROUP5_SPACER_END).withColor(OFF));
      m_candle.setControl(new SolidColor(GROUP6_SPACER_START, GROUP6_SPACER_END).withColor(OFF));
    } else {
      m_candle = null;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDs/IsStubbed", !m_isReal);

    // Run LED state logic at 10 Hz to limit CAN traffic and CPU overhead
    if (m_periodicCount++ % UPDATE_PERIOD_CYCLES != 0) {
      return;
    }

    // Limelight status groups — active in all robot modes including disabled
    updateGroup1();
    updateGroup4();

    // Shooter status groups
    updateShooterGroups();

    // Game piece tracking groups
    updateGamePieceGroups();
  }

  // ---------------------------------------------------------------------------
  // Per-zone update methods
  // ---------------------------------------------------------------------------

  /** Group 1 (8–47): left front limelight (camera index 0). */
  private void updateGroup1() {
    RGBWColor color = limelightColor(m_vision.getTagCount(0));
    setSolid(m_solidGroup1, color);
    Logger.recordOutput("LEDs/Group1/TagCount", m_vision.getTagCount(0));
    Logger.recordOutput("LEDs/Group1/Color", colorName(color));
  }

  /** Group 4 (158–197): right front limelight (camera index 1). */
  private void updateGroup4() {
    RGBWColor color = limelightColor(m_vision.getTagCount(1));
    setSolid(m_solidGroup4, color);
    Logger.recordOutput("LEDs/Group4/TagCount", m_vision.getTagCount(1));
    Logger.recordOutput("LEDs/Group4/Color", colorName(color));
  }

  /**
   * Groups 2 & 5: shooter status. Always identical.
   *
   * <ul>
   *   <li>Disabled → alliance color (or Maroon if undetermined)
   *   <li>Enabled solid → Red = not ready, Green = ready
   *   <li>Enabled strobing → StrobeAnimation when {@link Shooter#isShooting()} is true
   * </ul>
   */
  private void updateShooterGroups() {
    if (DriverStation.isDisabled()) {
      RGBWColor allianceColor = getAllianceColor();
      setSolid(m_solidGroup2, allianceColor);
      setSolid(m_solidGroup5, allianceColor);
      Logger.recordOutput("LEDs/Group2/Color", colorName(allianceColor));
      Logger.recordOutput("LEDs/Group2/Strobing", false);
      Logger.recordOutput("LEDs/Group5/Color", colorName(allianceColor));
      Logger.recordOutput("LEDs/Group5/Strobing", false);
      return;
    }

    boolean shooterReady = m_shooter.isReady();
    boolean isShooting = m_shooter.isShooting();
    RGBWColor baseColor = shooterReady ? GREEN : RED;

    if (isShooting) {
      setStrobe(m_strobeGroup2, baseColor);
      setStrobe(m_strobeGroup5, baseColor);
    } else {
      setSolid(m_solidGroup2, baseColor);
      setSolid(m_solidGroup5, baseColor);
    }

    Logger.recordOutput("LEDs/Group2/Color", colorName(baseColor));
    Logger.recordOutput("LEDs/Group2/Strobing", isShooting);
    Logger.recordOutput("LEDs/Group5/Color", colorName(baseColor));
    Logger.recordOutput("LEDs/Group5/Strobing", isShooting);
  }

  /**
   * Groups 3 & 6: game piece tracking. Always identical.
   *
   * <ul>
   *   <li>Disabled → alliance color (or Maroon if undetermined)
   *   <li>Enabled solid → Yellow = has game piece, White = no game piece
   *   <li>Enabled strobing → StrobeAnimation during last 5 s of each TeleOp shift
   * </ul>
   *
   * <p>Game piece detection is a stub (always false) until sensor hardware is available.
   */
  private void updateGamePieceGroups() {
    if (DriverStation.isDisabled()) {
      RGBWColor allianceColor = getAllianceColor();
      setSolid(m_solidGroup3, allianceColor);
      setSolid(m_solidGroup6, allianceColor);
      Logger.recordOutput("LEDs/Group3/Color", colorName(allianceColor));
      Logger.recordOutput("LEDs/Group3/Strobing", false);
      Logger.recordOutput("LEDs/Group6/Color", colorName(allianceColor));
      Logger.recordOutput("LEDs/Group6/Strobing", false);
      return;
    }

    // TODO: replace stub with real game piece detection (beam break, vision, etc.)
    boolean hasGamePiece = false;
    boolean inBlinkWindow = isInBlinkWindow();
    RGBWColor baseColor = hasGamePiece ? YELLOW : WHITE;

    if (inBlinkWindow) {
      setStrobe(m_strobeGroup3, baseColor);
      setStrobe(m_strobeGroup6, baseColor);
    } else {
      setSolid(m_solidGroup3, baseColor);
      setSolid(m_solidGroup6, baseColor);
    }

    Logger.recordOutput("LEDs/Group3/Color", colorName(baseColor));
    Logger.recordOutput("LEDs/Group3/Strobing", inBlinkWindow);
    Logger.recordOutput("LEDs/Group6/Color", colorName(baseColor));
    Logger.recordOutput("LEDs/Group6/Strobing", inBlinkWindow);
  }

  // ---------------------------------------------------------------------------
  // Helper methods
  // ---------------------------------------------------------------------------

  /** Maps AprilTag count to the corresponding limelight status color. */
  private RGBWColor limelightColor(int tagCount) {
    if (tagCount >= 2) return GREEN;
    if (tagCount == 1) return BLUE;
    return RED;
  }

  /**
   * Returns the alliance color for use while disabled. Falls back to Maroon if the alliance has not
   * yet been determined by the Driver Station.
   */
  private RGBWColor getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return MAROON;
    }
    return alliance.get() == Alliance.Red ? RED : BLUE;
  }

  /**
   * Returns true if the current match time falls within a defined blink window (last 5 s of any
   * TeleOp shift). Returns false during Autonomous, practice mode (matchTime < 0), or non-TeleOp.
   */
  private boolean isInBlinkWindow() {
    if (!DriverStation.isTeleop()) {
      return false;
    }
    double matchTimeSec = DriverStation.getMatchTime();
    if (matchTimeSec < 0.0) {
      return false;
    }
    return isInWindow(
            matchTimeSec, BLINK_WINDOW_TRANSITION_LOW_SEC, BLINK_WINDOW_TRANSITION_HIGH_SEC)
        || isInWindow(matchTimeSec, BLINK_WINDOW_SHIFT1_LOW_SEC, BLINK_WINDOW_SHIFT1_HIGH_SEC)
        || isInWindow(matchTimeSec, BLINK_WINDOW_SHIFT2_LOW_SEC, BLINK_WINDOW_SHIFT2_HIGH_SEC)
        || isInWindow(matchTimeSec, BLINK_WINDOW_SHIFT3_LOW_SEC, BLINK_WINDOW_SHIFT3_HIGH_SEC)
        || isInWindow(matchTimeSec, BLINK_WINDOW_SHIFT4_LOW_SEC, BLINK_WINDOW_SHIFT4_HIGH_SEC)
        || isInWindow(matchTimeSec, BLINK_WINDOW_ENDGAME_LOW_SEC, BLINK_WINDOW_ENDGAME_HIGH_SEC);
  }

  private boolean isInWindow(double matchTimeSec, double lowSec, double highSec) {
    return matchTimeSec >= lowSec && matchTimeSec <= highSec;
  }

  /** Returns a telemetry-friendly name for the given color constant. */
  private String colorName(RGBWColor color) {
    if (color == GREEN) return "GREEN";
    if (color == RED) return "RED";
    if (color == BLUE) return "BLUE";
    if (color == WHITE) return "WHITE";
    if (color == YELLOW) return "YELLOW";
    if (color == MAROON) return "MAROON";
    if (color == OFF) return "OFF";
    return "UNKNOWN";
  }

  /**
   * Sends a SolidColor control to the CANdle for the LED range configured in the request. No-ops
   * when not on real hardware.
   */
  private void setSolid(SolidColor request, RGBWColor color) {
    if (!m_isReal) return;
    m_candle.setControl(request.withColor(color));
  }

  /**
   * Sends a StrobeAnimation control to the CANdle for the LED range configured in the request.
   * No-ops when not on real hardware.
   */
  private void setStrobe(StrobeAnimation request, RGBWColor color) {
    if (!m_isReal) return;
    m_candle.setControl(request.withColor(color));
  }
}
