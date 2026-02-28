package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems;
import frc.robot.subsystems.ShooterTargetSelector;
import frc.robot.subsystems.SpotSelector;
import frc.robot.subsystems.climb.ClimbElevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * OperatorControls — Joystick button bindings and button box publishing for teleop.
 *
 * <p>{@link #configureControls()} sets up all joystick bindings and the Elastic button box. Call it
 * once from {@code RobotContainer.configureButtonBindings()}.
 *
 * <h2>Xbox Controller Quick Reference</h2>
 *
 * <pre>
 *   Buttons:                   Triggers and Bumpers:
 *       Y                       LB = leftBumper()     RB = rightBumper()
 *     X   B                     LT = leftTrigger()    RT = rightTrigger()
 *       A
 *
 *   Sticks:                    Other:
 *     Left  = getLeftY/X()      back()   start()
 *     Right = getRightY/X()     povUp()  povDown()  povLeft()  povRight()
 * </pre>
 *
 * <h2>Command Binding Cheat Sheet</h2>
 *
 * <ul>
 *   <li>{@code button.onTrue(command)} — runs ONCE when the button is first pressed
 *   <li>{@code button.whileTrue(command)} — runs while held, stops when released
 *   <li>{@code button.toggleOnTrue(command)} — press to start, press again to stop
 * </ul>
 *
 * <h2>Lambda Primer (the {@code () ->} syntax)</h2>
 *
 * A "lambda" is a tiny unnamed function you write inline. For example:
 *
 * <pre>
 *   () -> controller.getLeftY()       // takes no input, returns the left stick Y value
 *   () -> Rotation2d.fromDegrees(45)  // takes no input, returns a Rotation2d of 45 degrees
 *   () -> { ... multiple lines ... }  // takes no input, runs several statements
 * </pre>
 *
 * They are used as "suppliers" — the robot calls your lambda every 20 ms to get the latest value.
 */
public class OperatorControls {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final ClimbElevator climbElevator;
  private final Vision vision;
  private final CommandXboxController controller;
  private final SpotSelector spotSelector;
  private final ShooterTargetSelector targetSelector;

  public OperatorControls(
      Subsystems subsystems,
      CommandXboxController controller,
      SpotSelector spotSelector,
      ShooterTargetSelector targetSelector) {
    this.drive = subsystems.drive();
    this.shooter = subsystems.shooter();
    this.intake = subsystems.intake();
    this.climbElevator = subsystems.climbElevator();
    this.vision = subsystems.vision();
    this.controller = controller;
    this.spotSelector = spotSelector;
    this.targetSelector = targetSelector;
  }

  /**
   * Call this once from RobotContainer.configureButtonBindings() to activate all controls.
   *
   * <p>Uncomment each line below as you finish implementing it.
   */
  public void configureControls() {
    // === WORKING EXAMPLES (study these first) ===
    // configureHoodRetraction(); // Y button — hold to retract hood
    configureTrenchZoneAutoRetract(); // Automatic — retracts hood when robot enters zone
    // configureBumpTraversal(); // Right bumper — hold to lock heading to 45
    // degrees

    // === YOUR TURN (uncomment as you complete each one) ===
    // configureBumpTraversal(); // Right bumper — hold to lock heading to 45
    // degrees
    // configurePassingMode(); // Left bumper — hold to aim at nearest passing
    // target

    configureButtonBox();
  }

  // ========================================================================
  // Button Box — Elastic Dashboard
  //
  // Commands published here appear as toggle buttons in the Elastic dashboard.
  // Click once to start, click again to cancel.
  // Commands that run while disabled are wrapped with .ignoringDisable(true).
  // ========================================================================
  private void configureButtonBox() {
    // ── Spots ───────────────────────────────────────────────────────────────
    SmartDashboard.putData(
        "Spots/Alliance Trench Left", spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_LEFT));
    SmartDashboard.putData(
        "Spots/Alliance Trench Right", spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_RIGHT));
    SmartDashboard.putData(
        "Spots/Opponent Trench Left", spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_LEFT));
    SmartDashboard.putData(
        "Spots/Opponent Trench Right", spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_RIGHT));
    SmartDashboard.putData(
        "Spots/Hub Alliance", spotSelector.cmdSelectSpot(FieldSpot.HUB_ALLIANCE));
    SmartDashboard.putData("Spots/Hub Neutral", spotSelector.cmdSelectSpot(FieldSpot.HUB_NEUTRAL));
    SmartDashboard.putData("Spots/Climb Left", spotSelector.cmdSelectSpot(FieldSpot.CLIMB_LEFT));
    SmartDashboard.putData(
        "Spots/Climb Center", spotSelector.cmdSelectSpot(FieldSpot.CLIMB_CENTER));
    SmartDashboard.putData("Spots/Climb Right", spotSelector.cmdSelectSpot(FieldSpot.CLIMB_RIGHT));
    SmartDashboard.putData("Spots/Clear Selection", spotSelector.cmdClearSelection());

    // ── Climb ────────────────────────────────────────────────────────────────
    SmartDashboard.putData(
        "Climb/Left", ClimbCommands.superFastClimb(drive, climbElevator, FieldSpot.CLIMB_LEFT));
    SmartDashboard.putData(
        "Climb/Center", ClimbCommands.superFastClimb(drive, climbElevator, FieldSpot.CLIMB_CENTER));
    SmartDashboard.putData(
        "Climb/Right", ClimbCommands.superFastClimb(drive, climbElevator, FieldSpot.CLIMB_RIGHT));
    SmartDashboard.putData("Climb/Extend Arm", climbElevator.getExtendCmd());
    SmartDashboard.putData("Climb/Retract Arm", climbElevator.getRetractCmd());

    // ── Shooter ──────────────────────────────────────────────────────────────
    SmartDashboard.putData("Shooter/Normal Targeting", shooter.getEnableNormalTargetingModeCmd());
    SmartDashboard.putData(
        "Shooter/Predictive Targeting", shooter.getEnablePredictiveTargetingModeCmd());

    SmartDashboard.putData(
        "Shooter/Target Hub", targetSelector.cmdSelectManual(ShootingTarget.HUB));
    SmartDashboard.putData(
        "Shooter/Target Neutral Left", targetSelector.cmdSelectManual(ShootingTarget.NEUTRAL_LEFT));
    SmartDashboard.putData(
        "Shooter/Target Neutral Right",
        targetSelector.cmdSelectManual(ShootingTarget.NEUTRAL_RIGHT));
    SmartDashboard.putData(
        "Shooter/Target Opposition Left",
        targetSelector.cmdSelectManual(ShootingTarget.OPPOSITION_LEFT));
    SmartDashboard.putData(
        "Shooter/Target Opposition Right",
        targetSelector.cmdSelectManual(ShootingTarget.OPPOSITION_RIGHT));
    SmartDashboard.putData("Shooter/Target Enable Auto", targetSelector.cmdEnableAuto());

    SmartDashboard.putData("Shooter/Auto Shoot", shooter.getAutoShootCmd());
    SmartDashboard.putData("Shooter/Off", shooter.getOffCmd());
    SmartDashboard.putData("Shooter/Retract Hood", shooter.getRetractHoodCmd("ButtonBox"));

    SmartDashboard.putData("Shooter/Shot 1m", shooter.getFixedShot1mCmd());
    SmartDashboard.putData("Shooter/Shot 2m", shooter.getFixedShot2mCmd());
    SmartDashboard.putData("Shooter/Shot 3m", shooter.getFixedShot3mCmd());
    SmartDashboard.putData("Shooter/Turret Left 5deg", shooter.getTurretNudgeCmd(Degrees.of(-5)));
    SmartDashboard.putData("Shooter/Turret Center", shooter.getTurretCenterCmd());
    SmartDashboard.putData("Shooter/Turret Right 5deg", shooter.getTurretNudgeCmd(Degrees.of(5)));
    SmartDashboard.putData("Shooter/RPM +100", shooter.getRPMTrimCmd(+100));
    SmartDashboard.putData("Shooter/RPM -100", shooter.getRPMTrimCmd(-100));
    SmartDashboard.putData("Shooter/RPM Reset", shooter.getRPMTrimResetCmd());

    // ── Intake & Spindexer ───────────────────────────────────────────────────
    SmartDashboard.putData("Intake/Deploy", intake.getDeployCmd());
    SmartDashboard.putData("Intake/Retract", intake.getRetractCmd());
    SmartDashboard.putData("Intake/Roller On", intake.getRollerOnCmd());
    SmartDashboard.putData("Intake/Roller Off", intake.getRollerOffCmd());
    SmartDashboard.putData("Intake/Eject", intake.getReverseCmd());

    SmartDashboard.putData("Intake/Run Spindexer", shooter.getSpindexerRunCmd());
    SmartDashboard.putData("Intake/Stop Spindexer", shooter.getSpindexerStopCmd());
    SmartDashboard.putData("Intake/Reverse Spindexer", shooter.getSpindexerReverseCmd());

    // ── Calibration ──────────────────────────────────────────────────────────
    // TODO: Add prematch alignment readouts to the Calibration page on the button box.
    // During disabled, the Field widget already shows a green X at the selected auto's starting
    // pose (via SpotSelector/SelectedPose). For a text summary on the Calibration page, add
    // SmartDashboard entries showing position error (m) and heading error (deg) between the
    // robot's current pose and the auto start pose so drivers get a pass/fail at a glance.
    SmartDashboard.putData(
        "Calibration/Reset Gyro",
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .withName("Calibration/Reset Gyro")
            .ignoringDisable(true));

    SmartDashboard.putData(
        "Calibration/Turret Home & Calibrate", shooter.getTurretHomeAndCalibrateCmd());
    SmartDashboard.putData(
        "Calibration/Turret Calibrate In Place", shooter.getTurretCalibrateInPlaceCmd());

    SmartDashboard.putData(
        "Calibration/Hood Home & Calibrate", shooter.getHoodHomeAndCalibrateCmd());
    SmartDashboard.putData(
        "Calibration/Hood Calibrate In Place", shooter.getHoodCalibrateInPlaceCmd());
    SmartDashboard.putData("Calibration/Hood Zero", shooter.getHoodZeroCmd());

    SmartDashboard.putData(
        "Calibration/Intake Arm Home & Calibrate", intake.getDeployArmHomeAndCalibrateCmd());
    SmartDashboard.putData(
        "Calibration/Intake Arm Calibrate In Place", intake.getDeployArmCalibrateInPlaceCmd());

    SmartDashboard.putData("Calibration/Climb Home & Calibrate", climbElevator.homeAndCalibrate());
    SmartDashboard.putData(
        "Calibration/Climb Calibrate In Place",
        climbElevator.calibrateInPlace().ignoringDisable(true));

    SmartDashboard.putData(
        "Calibration/Enable Vision Gyro",
        Commands.runOnce(vision::enableVisionGyro)
            .withName("Calibration/Enable Vision Gyro")
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Calibration/Enable Normal Gyro",
        Commands.runOnce(vision::enableNormalGyro)
            .withName("Calibration/Enable Normal Gyro")
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Calibration/Reseed from Vision",
        Commands.runOnce(vision::reseedFromVision)
            .withName("Calibration/Reseed from Vision")
            .ignoringDisable(true));
  }

  // ========================================================================
  // EXAMPLE 1: Hood Retraction Button
  //
  // Hold Y to retract the hood to its lowest angle (safe for going under
  // obstacles). When you release Y, the hood returns to auto-aiming.
  //
  // This works because whileTrue() interrupts the hood's default command
  // while held, and when it ends, the default command resumes.
  // ========================================================================
  private void configureHoodRetraction() {
    // Y button is reserved for DriveToSpot (RobotContainer). Hood retraction is
    // available via the button box ("Shooter/Retract Hood") instead.
  }

  // ========================================================================
  // EXAMPLE 2: Automatic Hood Retraction in a Zone (Rectangle2d + Trigger)
  //
  // Instead of a button, this uses the robot's POSITION to trigger an
  // action. We define a rectangular zone on the field (the "trench"),
  // and whenever the robot drives into it, the hood retracts automatically.
  //
  // Key concepts:
  // - Rectangle2d defines a rectangle on the field (center, width, height)
  // - A Trigger checks a condition every 20 ms
  // - whileTrue() runs a command as long as the trigger condition is true
  // ========================================================================
  private void configureTrenchZoneAutoRetract() {
    // Define the trench zone as a rectangle on the field, in meters.
    //
    // Field coordinate system:
    // X = 0 at your alliance wall, increases toward the far wall (~16.5 m)
    // Y = 0 at the right wall (driver's perspective), increases left (~8.2 m)
    //
    // Rectangle2d(center, xWidth, yWidth):
    // center = a Pose2d (position + rotation) for the rectangle's center
    // Use Rotation2d.kZero for an axis-aligned (non-rotated) rectangle.
    // xWidth = how wide along the X axis (meters)
    // yWidth = how wide along the Y axis (meters)
    //
    Rectangle2d trenchZoneBL =
        new Rectangle2d(
            new Pose2d(4.5, 0.5, Rotation2d.kZero), // center of the trench zone (not rotated)
            3.0, // 3 meters wide along X
            1.5 // 1.5 meters wide along Y
            );
    Rectangle2d trenchZoneBR =
        new Rectangle2d(
            new Pose2d(4.5, 7.45, Rotation2d.kZero), // center of the trench zone (not rotated)
            3.0, // 3 meters wide along X
            1.5 // 1.5 meters wide along Y
            );
    Rectangle2d trenchZoneRL =
        new Rectangle2d(
            new Pose2d(12.0, 0.5, Rotation2d.kZero), // center of the trench zone (not rotated)
            3.0, // 3 meters wide along X
            1.5 // 1.5 meters wide along Y
            );
    Rectangle2d trenchZoneRR =
        new Rectangle2d(
            new Pose2d(12.0, 7.45, Rotation2d.kZero), // center of the trench zone (not rotated)
            3.0, // 3 meters wide along X
            1.5 // 1.5 meters wide along Y
            );
    // A Trigger that checks: "is the robot inside the trench zone right now?"
    // drive.getPose() returns the robot's Pose2d (position + heading).
    // .getTranslation() gives just the position (Translation2d), which is what
    // Rectangle2d.contains() needs.
    Trigger inTrenchZoneBL =
        new Trigger(() -> trenchZoneBL.contains(drive.getPose().getTranslation()));
    Trigger inTrenchZoneBR =
        new Trigger(() -> trenchZoneBR.contains(drive.getPose().getTranslation()));
    Trigger inTrenchZoneRL =
        new Trigger(() -> trenchZoneRL.contains(drive.getPose().getTranslation()));
    Trigger inTrenchZoneRR =
        new Trigger(() -> trenchZoneRR.contains(drive.getPose().getTranslation()));
    // While the robot is in the zone, retract the hood.
    // When the robot drives out, the hood returns to normal auto-aiming.
    inTrenchZoneBL.whileTrue(shooter.getRetractHoodCmd("TrenchZone/BL"));
    inTrenchZoneBR.whileTrue(shooter.getRetractHoodCmd("TrenchZone/BR"));
    inTrenchZoneRL.whileTrue(shooter.getRetractHoodCmd("TrenchZone/RL"));
    inTrenchZoneRR.whileTrue(shooter.getRetractHoodCmd("TrenchZone/RR"));
  }

  // ========================================================================
  // TODO 1: Bump Traversal at 45 Degrees
  //
  // When the right bumper is held, lock the robot heading to 45 degrees
  // so the swerve wheels cross the bump at an angle. The driver can still
  // translate freely with the left stick — only rotation is locked.
  //
  // HINTS:
  // - Button: controller.rightBumper()
  // - Binding: .whileTrue(...)
  // - Command: DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier,
  // angleSupplier)
  // This is already written — it drives with joystick translation but locks
  // rotation.
  //
  // - For X and Y suppliers, use the same ones as the normal drive:
  // () -> -controller.getLeftY() // forward/backward
  // () -> -controller.getLeftX() // left/right
  //
  // - For the angle supplier, create a Rotation2d of 45 degrees:
  // () -> Rotation2d.fromDegrees(45)
  //
  // Your code should look like:
  // controller.rightBumper().whileTrue(
  // DriveCommands.joystickDriveAtAngle(
  // drive,
  // <x supplier>,
  // <y supplier>,
  // <angle supplier>
  // )
  // );
  // ========================================================================
  private void configureBumpTraversal() {
    // Right bumper is reserved for manual shoot (RobotContainer). The zone-based
    // auto-tilt triggers below still apply without a button binding.

    Rectangle2d rampZoneBL =
        new Rectangle2d(
            new Pose2d(4.5, 2.5, Rotation2d.kZero), // center of the ramp zone (not rotated)
            3.0, // 3 meters wide along X
            2.0 // 2.0 meters wide along Y
            );
    Rectangle2d rampZoneBR =
        new Rectangle2d(
            new Pose2d(4.5, 5.5, Rotation2d.kZero), // center of the ramp zone (not rotated)
            3.0, // 3 meters wide along X
            2.0 // 2.0 meters wide along Y
            );
    Rectangle2d rampZoneRL =
        new Rectangle2d(
            new Pose2d(12.0, 2.5, Rotation2d.kZero), // center of the ramp zone (not rotated)
            3.0, // 3 meters wide along X
            2.0 // 2.0 meters wide along Y
            );
    Rectangle2d rampZoneRR =
        new Rectangle2d(
            new Pose2d(12.0, 5.5, Rotation2d.kZero), // center of the ramp zone (not rotated)
            3.0, // 3 meters wide along X
            2.0 // 2.0 meters wide along Y
            );
    // Suppress zone triggers during autonomous and while DriveToSpot is actively
    // pathfinding.
    // DriverStation.isTeleop() handles the autonomous period;
    // spotSelector.isAutoDriving()
    // handles teleop pathfinding via the Y button.
    Trigger notAutoDriving =
        new Trigger(() -> DriverStation.isTeleop() && !spotSelector.isAutoDriving());

    Trigger inRampZoneBL =
        new Trigger(() -> rampZoneBL.contains(drive.getPose().getTranslation()))
            .and(notAutoDriving);
    Trigger inRampZoneBR =
        new Trigger(() -> rampZoneBR.contains(drive.getPose().getTranslation()))
            .and(notAutoDriving);
    Trigger inRampZoneRL =
        new Trigger(() -> rampZoneRL.contains(drive.getPose().getTranslation()))
            .and(notAutoDriving);
    Trigger inRampZoneRR =
        new Trigger(() -> rampZoneRR.contains(drive.getPose().getTranslation()))
            .and(notAutoDriving);
    // While the robot is in the zone, retract the hood.
    // When the robot drives out, the hood returns to normal auto-aiming.
    inRampZoneBL.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(45)));
    inRampZoneBR.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(45)));
    inRampZoneRL.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(45)));
    inRampZoneRR.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(45)));
  }

  // ========================================================================
  // TODO 2: Passing Mode — Aim at the Nearest Passing Target
  //
  // When the robot moves to various field locations, switch the shooter to
  // "passing mode."
  // Every 20 ms, check the robot's position and aim at the closest
  // passing target from the list below.
  //
  // When the left bumper is released, switch back to the default target
  // (the hub).
  //
  // STEP-BY-STEP GUIDE:
  //
  // 1. Get the robot's position:
  // Translation2d robotPos = drive.getPose().getTranslation();
  //
  // 2. Loop through the targets and find the closest one:
  // double closestDistanceMeters = Double.MAX_VALUE;
  // int closestIndex = 0;
  //
  // for (int i = 0; i < PASSING_TARGET_POSES.length; i++) {
  // Translation2d targetPos =
  // PASSING_TARGET_POSES[i].getTranslation().toTranslation2d();
  // double distMeters = robotPos.getDistance(targetPos);
  // if (distMeters < closestDistanceMeters) {
  // closestDistanceMeters = distMeters;
  // closestIndex = i;
  // }
  // }
  //
  // 3. Tell the shooter to aim at it:
  // shooter.setTargetPose(
  // PASSING_TARGET_NAMES[closestIndex],
  // PASSING_TARGET_POSES[closestIndex]
  // );
  //
  // 4. Wrap steps 1-3 in a command that runs continuously:
  // Commands.run(() -> {
  // // steps 1-3 go here
  // })
  //
  // 5. When the button is released, reset to the hub:
  // .finallyDo(() -> shooter.setTargetPose("Blue Hub", DEFAULT_TARGET))
  //
  // 6. Bind it:
  // controller.leftBumper().whileTrue( <your command> );
  // ========================================================================

  /** Passing targets — field positions the robot can pass to. */
  private static final String[] PASSING_TARGET_NAMES = {
    "Neutral Left", "Neutral Right", "Opposition Left", "Opposition Right"
  };

  private static final Pose3d[] PASSING_TARGET_POSES = {
    new Pose3d(0.85, 1.5, 0.0, new Rotation3d()), // Neutral Left
    new Pose3d(0.85, 6.5, 0.0, new Rotation3d()), // Neutral Right
    new Pose3d(2.0, 2.0, 0.0, new Rotation3d()), // Opposition Left
    new Pose3d(2.0, 6.0, 0.0, new Rotation3d()), // Opposition Right
  };

  /** Default target — the hub. Used to reset after passing mode ends. */
  private static final Pose3d DEFAULT_TARGET = new Pose3d(4.5, 4.1, 1.83, new Rotation3d());

  private void configurePassingMode() {
    // YOUR CODE HERE
  }
}
