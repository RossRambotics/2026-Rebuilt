package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;

/** All constants for the LED subsystem. */
public final class LEDConstants {
  private LEDConstants() {}

  // CAN ID
  public static final int CANDLE_CAN_ID = 50;

  // ------------------------------------------------------------------
  // LED index layout
  // ------------------------------------------------------------------
  // Onboard LEDs: indices 0–7 (always solid Green)
  public static final int ONBOARD_START = 0;
  public static final int ONBOARD_END = 7;

  // External strip: indices 8–307 (6 groups of 50 LEDs each)
  // Each group: 40 active LEDs + 10 spacer (always off)

  // Group 1 — left front limelight (camera index 0)
  public static final int GROUP1_ACTIVE_START = 8;
  public static final int GROUP1_ACTIVE_END = 47;
  public static final int GROUP1_SPACER_START = 48;
  public static final int GROUP1_SPACER_END = 57;

  // Group 2 — shooter status (left side)
  public static final int GROUP2_ACTIVE_START = 58;
  public static final int GROUP2_ACTIVE_END = 97;
  public static final int GROUP2_SPACER_START = 98;
  public static final int GROUP2_SPACER_END = 107;

  // Group 3 — game piece tracking (left side)
  public static final int GROUP3_ACTIVE_START = 108;
  public static final int GROUP3_ACTIVE_END = 147;
  public static final int GROUP3_SPACER_START = 148;
  public static final int GROUP3_SPACER_END = 157;

  // Group 4 — right front limelight (camera index 1)
  public static final int GROUP4_ACTIVE_START = 158;
  public static final int GROUP4_ACTIVE_END = 197;
  public static final int GROUP4_SPACER_START = 198;
  public static final int GROUP4_SPACER_END = 207;

  // Group 5 — shooter status (right side)
  public static final int GROUP5_ACTIVE_START = 208;
  public static final int GROUP5_ACTIVE_END = 247;
  public static final int GROUP5_SPACER_START = 248;
  public static final int GROUP5_SPACER_END = 257;

  // Group 6 — game piece tracking (right side)
  public static final int GROUP6_ACTIVE_START = 258;
  public static final int GROUP6_ACTIVE_END = 297;
  public static final int GROUP6_SPACER_START = 298;
  public static final int GROUP6_SPACER_END = 307;

  // ------------------------------------------------------------------
  // Colors — RGBWColor(red, green, blue, white), W=0 for GRB strips
  // ------------------------------------------------------------------
  public static final RGBWColor GREEN = new RGBWColor(0, 217, 0, 0);
  public static final RGBWColor RED = new RGBWColor(217, 0, 0, 0);
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 217, 0);
  public static final RGBWColor WHITE = new RGBWColor(217, 217, 217, 0);
  public static final RGBWColor YELLOW = new RGBWColor(217, 183, 0, 0);
  public static final RGBWColor AMBER = new RGBWColor(217, 120, 0, 0);
  public static final RGBWColor MAROON = new RGBWColor(128, 0, 0, 0);
  public static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);

  // ------------------------------------------------------------------
  // Animation parameters
  // ------------------------------------------------------------------
  /** Strobe frame rate in Hz (number of on/off transitions per second). */
  public static final double STROBE_FRAME_RATE_HZ = 24.0;

  /** CANdle animation slot for Groups 3 and 6 (game piece strobe, left). */
  public static final int SLOT_GAMEPIECE_LEFT = 0;

  /** CANdle animation slot for Groups 3 and 6 (game piece strobe, right). */
  public static final int SLOT_GAMEPIECE_RIGHT = 1;

  // ------------------------------------------------------------------
  // TeleOp blink windows
  //
  // DriverStation.getMatchTime() returns seconds remaining (counting down).
  // TeleOp is ~150 s (2:30). Blink windows = last 5 s of each shift period.
  //
  // Shift boundaries (time remaining):
  // Transition: 2:20–2:10 (matchTimeSec 140–130) → blink 130–135
  // Shift 1: 2:10–1:45 (matchTimeSec 130–105) → blink 105–110
  // Shift 2: 1:45–1:20 (matchTimeSec 105–80) → blink 80–85
  // Shift 3: 1:20–0:55 (matchTimeSec 80–55) → blink 55–60
  // Shift 4: 0:55–0:30 (matchTimeSec 55–30) → blink 30–35
  // End Game: 0:30–0:00 (matchTimeSec 30–0) → blink 0–5
  // ------------------------------------------------------------------
  public static final double BLINK_WINDOW_TRANSITION_LOW_SEC = 130.0;
  public static final double BLINK_WINDOW_TRANSITION_HIGH_SEC = 135.0;

  public static final double BLINK_WINDOW_SHIFT1_LOW_SEC = 105.0;
  public static final double BLINK_WINDOW_SHIFT1_HIGH_SEC = 110.0;

  public static final double BLINK_WINDOW_SHIFT2_LOW_SEC = 80.0;
  public static final double BLINK_WINDOW_SHIFT2_HIGH_SEC = 85.0;

  public static final double BLINK_WINDOW_SHIFT3_LOW_SEC = 55.0;
  public static final double BLINK_WINDOW_SHIFT3_HIGH_SEC = 60.0;

  public static final double BLINK_WINDOW_SHIFT4_LOW_SEC = 30.0;
  public static final double BLINK_WINDOW_SHIFT4_HIGH_SEC = 35.0;

  public static final double BLINK_WINDOW_ENDGAME_LOW_SEC = 0.0;
  public static final double BLINK_WINDOW_ENDGAME_HIGH_SEC = 5.0;
}
