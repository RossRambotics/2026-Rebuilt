# Driver & Operator Certification Test

Answer all 25 questions. Passing score: 20/25 (80%).

Refer to [`DRIVER_OPERATOR_GUIDE.md`](DRIVER_OPERATOR_GUIDE.md) to study.

---

## Section 1 — Driver Controls (Questions 1–8)

**1.** The driver wants to deploy the intake arm and start the roller. Which button do they press?

- A) Left Bumper (LB)
- B) Left Trigger (LT)
- C) Right Trigger (RT)
- D) B Button

---

**2.** The driver presses Left Trigger (LT) once to start intaking. How do they stop the roller without retracting the arm?

- A) Press Left Bumper (LB)
- B) Press Left Trigger (LT) a second time
- C) Press X button
- D) The roller stops automatically after 5 seconds

---

**3.** How does the driver start Auto Shoot?

- A) Hold Right Trigger (RT) — it runs only while held
- B) Press Right Trigger (RT) once to toggle on; press again to toggle off
- C) Press Y button
- D) The robot starts shooting automatically when ready

---

**4.** The robot's gyro heading is wrong after a collision. The intake is currently facing the alliance wall. What does the driver press to correct heading only (without changing X/Y position)?

- A) Start button
- B) Back button
- C) Y button
- D) Right Bumper (RB)

---

**5.** The driver wants to reset the robot's full pose (position + heading) at the hub. What must be true before pressing Start?

- A) The intake must be facing the alliance wall
- B) The robot must be enabled
- C) The shooter side must be flush against the hub wall
- D) The turret must be centered

---

**6.** What does holding the B button do?

- A) Runs the kicker and spindexer manually
- B) Drives the robot backward toward detected game pieces and auto-deploys the intake
- C) Locks wheels in an X pattern
- D) Engages the climb elevator

---

**7.** When Auto Shoot (RT) is toggled on, the driver notices the robot feels sluggish to drive. Is this a problem?

- A) Yes — something is stuck and the operator should press Shooter / Off
- B) No — SpeedNanny automatically reduces drive speed and rotation while Auto Shoot is active
- C) Yes — the flywheel is drawing too much power
- D) No — this only happens when battery voltage is low

---

**8.** The driver wants to drive the robot to a pre-selected field position. Which button triggers the auto-drive?

- A) B button
- B) A button
- C) Y button
- D) Start button

---

## Section 2 — Operator Controls & Shooter (Questions 9–17)

**9.** The shooter is landing shots consistently short. Which button should the operator press first?

- A) Shooter / Hood +1deg
- B) Shooter / RPM +50
- C) Shooter / Turret Right 5deg
- D) Shooter / RPM Reset

---

**10.** The operator has pressed RPM +50 three times. How many RPM have been added to the flywheel target?

- A) 50 RPM
- B) 100 RPM
- C) 150 RPM
- D) 200 RPM

---

**11.** Shots are consistently landing too high (overshooting vertically). What is the correct operator action?

- A) Press Shooter / Hood +1deg
- B) Press Shooter / RPM -50
- C) Press Shooter / Hood -1deg
- D) Press Shooter / Turret Left 5deg

---

**12.** The operator presses Hood +1deg twelve times. What happens if that would push the commanded hood angle above the maximum safe angle?

- A) The hood motor overheats and faults
- B) The trim is clamped — the hood command never exceeds the maximum safe angle
- C) The robot disables itself
- D) The trim resets automatically

---

**13.** The operator presses Shooter / Retract Hood. Does the hood trim offset affect where the hood goes?

- A) Yes — the hood moves to the minimum angle plus the current trim offset
- B) No — retract commands always go directly to the minimum safe angle, ignoring the trim
- C) Yes, but only if the trim is positive
- D) Only if the robot is enabled

---

**14.** The middle LEDs have been RED for 4 seconds after the driver toggled RT on. The top LEDs are GREEN. What are three possible causes? *(Short answer — list all three)*

---

**15.** The operator needs to select a specific shooting target manually because auto-targeting is picking the wrong one. After selecting the target, how does the operator return the robot to automatic zone-based target selection?

- A) Press Shooter / Off, then Shooter / Normal Targeting
- B) Press Shooter / Target Enable Auto
- C) Releasing RT automatically re-enables auto targeting
- D) Auto targeting re-enables on its own after 10 seconds

---

**16.** Which of the following is true about the Manual Shoot mode (Right Bumper)?

- A) It waits for the turret, hood, and flywheel to reach their targets before firing
- B) It runs the kicker and spindexer only, without checking shooter readiness
- C) It overrides the turret and locks it to 0°
- D) It disables SpeedNanny

---

**17.** What does pressing Shooter / Shot 2m do?

- A) Commands the robot to auto-drive to a position 2 meters from the target
- B) Locks turret to 0°, applies a pre-tuned hood angle and RPM for a 2-meter shot
- C) Adds 2 meters to the estimated distance to target
- D) Runs a 2-meter calibration path

---

## Section 3 — LEDs & Telemetry (Questions 18–20)

**18.** Match each top LED color to its meaning:

| Color | Meaning |
|-------|---------|
| BLUE | ___ |
| LIME | ___ |
| ROSE | ___ |
| RED | ___ |

*(Fill in: "No cameras see AprilTags", "Both cameras see ≥ 1 AprilTag", "Left camera only sees ≥ 1 AprilTag", "Right camera only sees ≥ 1 AprilTag")*

---

**19.** The bottom LEDs are YELLOW. What does this indicate?

- A) The robot is disabled
- B) A game piece is detected by the intake camera
- C) The shooter is ready to fire
- D) The match shift warning is active

---

**20.** The operator wants to monitor flywheel RPM on the dashboard without it jumping around every loop. Which telemetry key should they use?

- A) `Shooter/FlyWheelActualRPM`
- B) `Shooter/FlyWheelTargetRPM`
- C) `Shooter/FlyWheelAverageRPM`
- D) `BottomFlyWheelMotor/Velocity`

---

## Section 4 — Pre-Match & Calibration (Questions 21–23)

**21.** Which of the following items must the operator verify on the pre-match checklist? *(Select ALL that apply)*

- A) RPM trim is reset (Shooter / RPM Reset)
- B) Hood trim is reset (Shooter / Hood Trim Reset)
- C) Turret nudge is cleared
- D) Top LEDs are GREEN before the match starts
- E) Correct auto routine is selected on the dashboard

---

**22.** During disabled, the Field widget shows the white X (auto start pose) is not overlapping the green square (robot pose). What are the two correct actions?

- A) Physically move the robot to match the white X, OR use Calibration / Reseed from Vision to correct the pose
- B) Restart the robot code, OR press Back button
- C) Change the auto routine, OR press Start button
- D) Press Shooter / Off, OR drive the robot in a circle

---

**23.** Hood Home & Calibrate must be run while the robot is __________.

- A) Disabled
- B) Enabled
- C) Either — it works in both states
- D) Only during autonomous

---

## Section 5 — Scenario-Based (Questions 24–25)

**24.** It is teleop, 45 seconds remaining. Shots have been going consistently to the right. The top LEDs are GREEN. The Field widget shows the robot's position is correct. What is the single best operator action?

- A) Press Shooter / RPM +50
- B) Press Shooter / Turret Right 5deg
- C) Press Shooter / Turret Left 5deg
- D) Press Calibration / Reseed from Vision

---

**25.** During a match the robot takes a hard hit. After the hit, the top LEDs turn RED and the Field widget shows the robot in completely the wrong position. The robot is near the hub. Place the following recovery steps in the correct order:

- ① Driver presses Start button to snap pose to hub wall
- ② Driver pushes shooter side flush against the hub wall
- ③ Robot drives normally — top LEDs should turn LIME, ROSE, or BLUE as cameras see tags
- ④ If LEDs stay RED after pose reset, rotate so cameras face the field

A) ②→①→④→③
B) ①→②→③→④
C) ③→②→①→④
D) ②→①→③→④

---

## Answer Key

*(Instructor copy — do not distribute to students)*

| Q | Answer |
|---|--------|
| 1 | B |
| 2 | B |
| 3 | B |
| 4 | B |
| 5 | C |
| 6 | B |
| 7 | B |
| 8 | C |
| 9 | B |
| 10 | C |
| 11 | C |
| 12 | B |
| 13 | B |
| 14 | Turret cannot reach target angle (hit a hard stop or facing wrong way); Hood did not reach target angle; Flywheel has not spun up to target RPM yet |
| 15 | B |
| 16 | B |
| 17 | B |
| 18 | BLUE = Both cameras see ≥ 1 AprilTag; LIME = Left camera only; ROSE = Right camera only; RED = No cameras see AprilTags |
| 19 | B |
| 20 | C |
| 21 | A, B, C, D, E |
| 22 | A |
| 23 | B |
| 24 | C |
| 25 | A |
