package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.TimedSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem backed by a CTRE CANdle. Runs in all robot modes including disabled.
 *
 * <p>LED layout (308 total):
 *
 * <ul>
 *   <li>Onboard 0–7: always solid green
 *   <li>Group 1 (8–57): combined vision status (both cameras); 48–57 spacer (off)
 *   <li>Group 2 (58–107): shooter status; 98–107 spacer (off)
 *   <li>Group 3 (108–157): game piece tracking; 148–157 spacer (off)
 *   <li>158–307: unused, set to off once at startup
 * </ul>
 */
public class LEDSubsystem extends TimedSubsystem {

  private final boolean m_isReal;
  private final Subsystems m_subsystems;

  // Hardware — null when not real
  private final CANdle m_candle;

  // Throttle: run LED state logic at 10 Hz (every 5 cycles of the 50 Hz loop)
  private static final int UPDATE_PERIOD_CYCLES = 5;
  private int m_periodicCount = 0;

  // Pre-allocated SolidColor controls for active groups (reused every cycle,
  // avoids per-cycle GC)
  private final SolidColor m_solidGroup1 = new SolidColor(GROUP1_ACTIVE_START, GROUP1_ACTIVE_END);
  private final SolidColor m_solidGroup2 = new SolidColor(GROUP2_ACTIVE_START, GROUP2_ACTIVE_END);
  private final SolidColor m_solidGroup3 = new SolidColor(GROUP3_ACTIVE_START, GROUP3_ACTIVE_END);

  // Pre-allocated StrobeAnimation control with fixed slot and frame-rate
  // assignment
  private final StrobeAnimation m_strobeGroup3 =
      new StrobeAnimation(GROUP3_ACTIVE_START, GROUP3_ACTIVE_END)
          .withSlot(SLOT_GAMEPIECE)
          .withFrameRate(STROBE_FRAME_RATE_HZ);

  // EmptyAnimation clears a CANdle animation slot. Pre-allocated so we don't
  // allocate on the periodic path.
  private final EmptyAnimation m_emptyGroup3 = new EmptyAnimation(SLOT_GAMEPIECE);

  // Track whether Group 3 is currently running a slot animation so we can
  // explicitly clear the slot before transitioning back to solid color.
  // SolidColor is not a slot-based control and does not cancel a running
  // animation slot on its own.
  private boolean m_group3Strobing = false;

  /**
   * Creates the LED subsystem.
   *
   * @param isReal true when running on real hardware; false in sim/replay (stubs all CANdle calls)
   * @param subsystems Bundle of all subsystems; used to query vision, shooter, and game piece state
   */
  public LEDSubsystem(boolean isReal, Subsystems subsystems) {
    m_isReal = isReal;
    m_subsystems = subsystems;

    if (m_isReal) {
      m_candle = new CANdle(CANDLE_CAN_ID);
      var config = new CANdleConfiguration();
      config.LED.StripType = StripTypeValue.GRB;
      config.LED.BrightnessScalar = 1.0;
      m_candle.getConfigurator().apply(config);

      // Onboard LEDs and spacers never change — set them once here instead of every
      // periodic.
      m_candle.setControl(new SolidColor(ONBOARD_START, ONBOARD_END).withColor(GREEN));
      m_candle.setControl(new SolidColor(GROUP1_SPACER_START, GROUP3_SPACER_END).withColor(OFF));

      // Indices 158–307 are unused — turn them off once.
      m_candle.setControl(new SolidColor(GROUP3_SPACER_END + 1, 307).withColor(OFF));
    } else {
      m_candle = null;
    }
  }

  @Override
  protected void periodicImpl() {
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("LEDs/IsStubbed", !m_isReal);
    }

    // Run LED state logic at 10 Hz to limit CAN traffic and CPU overhead
    if (m_periodicCount++ % UPDATE_PERIOD_CYCLES != 0) {
      return;
    }

    // Vision status group — active in all robot modes including disabled
    updateGroup1();

    // Shooter status group
    updateShooterGroup();

    // Game piece tracking group
    updateGamePieceGroup();
  }

  // ---------------------------------------------------------------------------
  // Per-zone update methods
  // ---------------------------------------------------------------------------

  /**
   * Group 1 (8–47): combined vision status for both cameras.
   *
   * <ul>
   *   <li>Blue — both cameras see ≥ 1 AprilTag
   *   <li>Lime — left camera only sees ≥ 1 AprilTag
   *   <li>Rose — right camera only sees ≥ 1 AprilTag
   *   <li>Red — neither camera sees any AprilTag
   * </ul>
   */
  private void updateGroup1() {
    boolean leftHasTarget = m_subsystems.vision().getTagCount(0) >= 1;
    boolean rightHasTarget = m_subsystems.vision().getTagCount(1) >= 1;

    RGBWColor color;
    if (leftHasTarget && rightHasTarget) {
      color = BLUE;
    } else if (leftHasTarget) {
      color = LIME;
    } else if (rightHasTarget) {
      color = ROSE;
    } else {
      color = RED;
    }

    setSolid(m_solidGroup1, color);
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("LEDs/Group1/LeftTagCount", m_subsystems.vision().getTagCount(0));
      Logger.recordOutput("LEDs/Group1/RightTagCount", m_subsystems.vision().getTagCount(1));
      Logger.recordOutput("LEDs/Group1/Color", colorName(color));
    }
  }

  /**
   * Group 2: shooter status.
   *
   * <ul>
   *   <li>Disabled → alliance color (or Maroon if undetermined)
   *   <li>Idle (no shoot command active) → solid Amber
   *   <li>Targeting, not ready → solid Red
   *   <li>Targeting, ready → solid Green
   * </ul>
   */
  private void updateShooterGroup() {
    if (DriverStation.isDisabled()) {
      RGBWColor allianceColor = getAllianceColor();
      setSolid(m_solidGroup2, allianceColor);
      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput("LEDs/Group2/Color", colorName(allianceColor));
      }
      return;
    }

    boolean isShooting = m_subsystems.shooter().isShooting();
    RGBWColor color;

    if (!isShooting) {
      color = AMBER;
    } else {
      color = m_subsystems.shooter().isReady() ? GREEN : RED;
    }

    setSolid(m_solidGroup2, color);
    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("LEDs/Group2/Color", colorName(color));
    }
  }

  /**
   * Group 3: game piece tracking.
   *
   * <ul>
   *   <li>Disabled → alliance color (or Maroon if undetermined)
   *   <li>Enabled solid → Yellow = has game piece, White = no game piece
   *   <li>Enabled strobing → StrobeAnimation during last 5 s of each TeleOp shift
   * </ul>
   *
   * <p>Game piece detection uses {@link frc.robot.subsystems.gamepiece.GamePieceVisionSubsystem} —
   * Yellow when the intake-cam sees a fuel game piece, White when none are detected.
   */
  private void updateGamePieceGroup() {
    if (DriverStation.isDisabled()) {
      RGBWColor allianceColor = getAllianceColor();
      if (m_group3Strobing) {
        clearGroup3Animation();
      }
      setSolid(m_solidGroup3, allianceColor);
      if (Constants.VERBOSE_TELEMETRY) {
        Logger.recordOutput("LEDs/Group3/Color", colorName(allianceColor));
        Logger.recordOutput("LEDs/Group3/Strobing", false);
      }
      return;
    }

    boolean hasGamePiece = m_subsystems.gamePieceVision().hasTarget();
    boolean inBlinkWindow = isInBlinkWindow();
    RGBWColor baseColor = hasGamePiece ? YELLOW : WHITE;

    if (inBlinkWindow) {
      setStrobe(m_strobeGroup3, baseColor);
      m_group3Strobing = true;
    } else {
      if (m_group3Strobing) {
        clearGroup3Animation();
      }
      setSolid(m_solidGroup3, baseColor);
    }

    if (Constants.VERBOSE_TELEMETRY) {
      Logger.recordOutput("LEDs/Group3/Color", colorName(baseColor));
      Logger.recordOutput("LEDs/Group3/Strobing", inBlinkWindow);
    }
  }

  // ---------------------------------------------------------------------------
  // Helper methods
  // ---------------------------------------------------------------------------

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
    if (color == LIME) return "LIME";
    if (color == ROSE) return "ROSE";
    if (color == WHITE) return "WHITE";
    if (color == YELLOW) return "YELLOW";
    if (color == AMBER) return "AMBER";
    if (color == MAROON) return "MAROON";
    if (color == OFF) return "OFF";
    return "UNKNOWN";
  }

  /**
   * Clears the Group 3 animation slot so a subsequent SolidColor control takes visual effect.
   * No-ops when not on real hardware.
   */
  private void clearGroup3Animation() {
    m_group3Strobing = false;
    if (!m_isReal) return;
    m_candle.setControl(m_emptyGroup3);
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
