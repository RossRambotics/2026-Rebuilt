# LED Subsystem (CANdle)

This document describes the LED subsystem, what each group of LEDs displays, and how the implementation works.

---

## Hardware

- **Device:** CTRE CANdle (CAN ID 50)
- **Strip type:** GRB (WS2812B)
- **Total LEDs:** 308 (8 onboard + 300 external)
- **Strip update rate:** 10 Hz (state logic runs every 5 robot loop cycles)

The onboard LEDs (indices 0–7) are on the CANdle PCB itself. The 300 external LEDs (indices 8–307) are divided into six groups of 50. The last 10 LEDs of each group are dark spacers that visually separate the groups.

---

## LED Layout

```
Index   Zone                    Purpose
──────  ──────────────────────  ────────────────────────────────────────
0–7     Onboard                 Always solid green
8–47    Group 1 — active (40)   Left front limelight status
48–57   Group 1 — spacer (10)   Always off
58–97   Group 2 — active (40)   Shooter status (left)
98–107  Group 2 — spacer (10)   Always off
108–147 Group 3 — active (40)   Game piece tracking (left)
148–157 Group 3 — spacer (10)   Always off
158–197 Group 4 — active (40)   Right front limelight status
198–207 Group 4 — spacer (10)   Always off
208–247 Group 5 — active (40)   Shooter status (right)
248–257 Group 5 — spacer (10)   Always off
258–297 Group 6 — active (40)   Game piece tracking (right)
298–307 Group 6 — spacer (10)   Always off
```

---

## What Each Group Shows

### Onboard LEDs (0–7)
Always solid **green**. No logic — set once at startup.

### Groups 1 & 4 — Limelight Status
Active in **all robot modes, including disabled.**

| Color | Meaning |
|-------|---------|
| Red | 0 AprilTags detected |
| Blue | 1 AprilTag detected |
| Green | 2 or more AprilTags detected |

- Group 1 reflects **camera index 0** (`limelight-frontl`, left front)
- Group 4 reflects **camera index 1** (`limelight-frontr`, right front)

### Groups 2 & 5 — Shooter Status
Groups 2 and 5 always show the same thing (left and right sides are mirrors).

**While disabled:** shows the alliance color — Red for Red alliance, Blue for Blue alliance, or Maroon (`#800000`) if the alliance has not yet been set by the Driver Station.

**While enabled:**

| Color | Meaning |
|-------|---------|
| Solid amber | Idle — no shoot command active (driver not holding RT) |
| Solid red | Targeting — shoot command active, waiting for turret/hood/flywheel to align |
| Solid green | Targeting — ready to fire (all subsystems on target) |

"Ready" means all of: turret on target, hood on target, flywheel at speed, and robot speed under the SpeedNanny limit — all debounced for 100 ms.

### Groups 3 & 6 — Game Piece Tracking
Groups 3 and 6 always show the same thing.

**While disabled:** shows the alliance color or Maroon (same rules as Groups 2 & 5).

**While enabled:**

| Color / Effect | Meaning |
|----------------|---------|
| Solid white | No game piece detected |
| Solid yellow | Game piece detected |
| Strobing | Last 5 seconds of a TeleOp shift period |

Game piece detection uses `GamePieceVisionSubsystem.hasTarget()` — YELLOW when the `intake-cam` PhotonVision coprocessor sees a fuel game piece on the floor, WHITE when none are detected.

#### TeleOp Shift Blink Windows
Groups 3 & 6 strobe during the last 5 seconds of each TeleOp shift to alert the drivers that the period is about to change. The shift boundaries use `DriverStation.getMatchTime()` (seconds remaining, counting down from ~150 s):

| Shift | Time range | Blink window (matchTime) |
|-------|-----------|--------------------------|
| Transition | 2:20 – 2:10 | 130 – 135 s |
| Shift 1 | 2:10 – 1:45 | 105 – 110 s |
| Shift 2 | 1:45 – 1:20 | 80 – 85 s |
| Shift 3 | 1:20 – 0:55 | 55 – 60 s |
| Shift 4 | 0:55 – 0:30 | 30 – 35 s |
| End Game | 0:30 – 0:00 | 0 – 5 s |

Strobing only occurs during TeleOp. In Autonomous or practice mode (where `getMatchTime()` returns −1) the groups show their normal solid color.

---

## Color Reference

| Color | R | G | B | Used for |
|-------|---|---|---|---------|
| Green | 0 | 217 | 0 | 2+ AprilTags, shooter ready, onboard |
| Red | 217 | 0 | 0 | 0 AprilTags, shooter targeting not ready, Red alliance |
| Amber | 217 | 120 | 0 | Shooter idle |
| Blue | 0 | 0 | 217 | 1 AprilTag, Blue alliance |
| White | 217 | 217 | 217 | No game piece |
| Yellow | 217 | 183 | 0 | Game piece detected |
| Maroon | 128 | 0 | 0 | Alliance undetermined (disabled) |
| Off | 0 | 0 | 0 | Spacers |

W channel is 0 for all colors (GRB strip, no dedicated white channel).

---

## Implementation Notes

### Architecture
`LEDSubsystem` does not use the IO abstraction pattern used by Drive, Vision, and Shooter. Because the CANdle is output-only (no meaningful inputs to log), the simpler `boolean isReal` pattern (same as `Spindexer.java`) is used instead. The `CANdle` instance is created only when `isReal == true`; all `setControl()` calls are no-ops otherwise.

### Always-on (disabled mode)
`SubsystemBase.periodic()` is called by the `CommandScheduler` in all robot modes, including disabled. No `.ignoringDisable()` is needed.

### Update rate throttle
The state-dependent LED logic runs at **10 Hz** (every 5 cycles of the 50 Hz robot loop) to limit CAN traffic. The `Logger.recordOutput("LEDs/IsStubbed")` call still runs every cycle.

```java
private static final int UPDATE_PERIOD_CYCLES = 5; // 50 Hz ÷ 5 = 10 Hz
private int m_periodicCount = 0;

if (m_periodicCount++ % UPDATE_PERIOD_CYCLES != 0) return;
```

### Static zones set once at startup
Onboard LEDs and spacers never change, so they are commanded in the constructor (not in `periodic()`). This eliminates 7 redundant CAN frames per cycle.

### Pre-allocated control objects
All `SolidColor` and `StrobeAnimation` control objects are pre-allocated as class fields. The `withColor()` method mutates and returns the same object, avoiding per-cycle heap allocations. Each animated group has a dedicated CANdle animation slot:

| Slot | Zone |
|------|------|
| 0 | Group 3 — game piece strobe (left) |
| 1 | Group 6 — game piece strobe (right) |

### Dependencies injected via constructor
`LEDSubsystem` receives the full `Subsystems` record at construction time (instantiated after `Subsystems` is assembled in `RobotContainer`). It accesses `subsystems.vision()`, `subsystems.shooter()`, and `subsystems.gamePieceVision()` directly.

---

## Telemetry

All LED state is recorded with `Logger.recordOutput()` and available in AdvantageScope:

| Key | Type | Description |
|-----|------|-------------|
| `LEDs/IsStubbed` | boolean | true in sim/replay (no real CANdle) |
| `LEDs/Group1/TagCount` | int | AprilTags seen by left limelight |
| `LEDs/Group1/Color` | string | Color name shown |
| `LEDs/Group4/TagCount` | int | AprilTags seen by right limelight |
| `LEDs/Group4/Color` | string | Color name shown |
| `LEDs/Group2/Color` | string | Shooter color (left): AMBER / RED / GREEN |
| `LEDs/Group5/Color` | string | Shooter color (right): AMBER / RED / GREEN |
| `LEDs/Group3/Color` | string | Game piece color (left) |
| `LEDs/Group3/Strobing` | boolean | Strobing during shift blink window |
| `LEDs/Group6/Color` | string | Game piece color (right) |
| `LEDs/Group6/Strobing` | boolean | Strobing during shift blink window |

---

## Tuning / Placeholder Values

These values are deliberate placeholders and should be updated when hardware is finalized:

| Constant | Location | Current value | Notes |
|----------|----------|---------------|-------|
| `ONBOARD_END` | `LEDConstants.java` | 7 | 8 onboard LEDs on CANdle |
| `GROUP*_SPACER_END` | `LEDConstants.java` | 10 LEDs per group | Spacer count — adjust as needed |
| `STROBE_FRAME_RATE_HZ` | `LEDConstants.java` | 24 Hz | Strobe speed |
| `LED.BrightnessScalar` | `LEDSubsystem.java` constructor | 1.0 | Reduce if LEDs are too bright |
| `LED.BrightnessScalar` | `LEDSubsystem.java` constructor | 1.0 | Reduce if LEDs are too bright |

---

## Code Locations

| File | Purpose |
|------|---------|
| `src/main/java/frc/robot/subsystems/leds/LEDSubsystem.java` | Main subsystem — all periodic logic |
| `src/main/java/frc/robot/subsystems/leds/LEDConstants.java` | LED indices, colors, animation slots, blink windows |
| `src/main/java/frc/robot/subsystems/vision/Vision.java` | `getTagCount(int cameraIndex)` — exposes per-camera tag count |
| `src/main/java/frc/robot/subsystems/shooter/Shooter.java` | `isReady()` / `isShooting()` — shooter state queries |
| `src/main/java/frc/robot/RobotContainer.java` | `m_ledSubsystem` field and instantiation |
