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

  // External strip: indices 8–157 (3 groups of 50 LEDs each)
  // Each group: 40 active LEDs + 10 spacer (always off)
  // Indices 158–307 are unused and set to off once at startup.

  // Group 1 — combined vision status (both cameras)
  public static final int GROUP1_ACTIVE_START = ONBOARD_END + 1;
  public static final int GROUP1_ACTIVE_END = GROUP1_ACTIVE_START + 53;
  public static final int GROUP1_SPACER_START = GROUP1_ACTIVE_END + 1;
  public static final int GROUP1_SPACER_END = GROUP1_SPACER_START + 9;

  // Group 2 — shooter status
  public static final int GROUP2_ACTIVE_START = GROUP1_SPACER_END + 11;
  public static final int GROUP2_ACTIVE_END = GROUP2_ACTIVE_START + 39;
  public static final int GROUP2_SPACER_START = GROUP2_ACTIVE_END - 10;
  public static final int GROUP2_SPACER_END = GROUP2_SPACER_START + 21;

  // Group 3 — game piece tracking
  public static final int GROUP3_ACTIVE_START = GROUP2_SPACER_END + 11;
  public static final int GROUP3_ACTIVE_END = GROUP3_ACTIVE_START + 39;
  public static final int GROUP3_SPACER_START = GROUP3_ACTIVE_END + 1;
  public static final int GROUP3_SPACER_END = GROUP3_SPACER_START + 9;

  // ------------------------------------------------------------------
  // Colors — RGBWColor(red, green, blue, white), W=0 for GRB strips
  // ------------------------------------------------------------------
  public static final RGBWColor GREEN = new RGBWColor(0, 217, 0, 0);
  public static final RGBWColor RED = new RGBWColor(217, 0, 0, 0);
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 217, 0);
  public static final RGBWColor LIME = new RGBWColor(50, 205, 50, 0);
  public static final RGBWColor ROSE = new RGBWColor(217, 0, 100, 0);
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

  /** CANdle animation slot for Group 3 (game piece strobe). */
  public static final int SLOT_GAMEPIECE = 0;

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
