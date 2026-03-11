// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoDriveToFuel;
import frc.robot.commands.AutoTargetSelector;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToSpot;
import frc.robot.commands.FieldSpot;
import frc.robot.commands.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterTargetSelector;
import frc.robot.subsystems.SpeedNanny;
import frc.robot.subsystems.SpotSelector;
import frc.robot.subsystems.climb.ClimbElevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.gamepiece.GamePieceVisionConstants;
import frc.robot.subsystems.gamepiece.GamePieceVisionIO;
import frc.robot.subsystems.gamepiece.GamePieceVisionIOPhoton;
import frc.robot.subsystems.gamepiece.GamePieceVisionIOSim;
import frc.robot.subsystems.gamepiece.GamePieceVisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public SpeedNanny m_speedNanny = new SpeedNanny();

  public final SpotSelector m_spotSelector = new SpotSelector();

  public Shooter m_shooter = null;

  // Shared AutoShoot command instance — used by the driver trigger toggle and by
  // the "Shoot.Start" / "Shoot.Stop" PathPlanner named commands.
  private Command m_autoShootCmd = null;

  public ShooterTargetSelector m_targetSelector = null;

  public Intake m_intake = null;

  public ClimbElevator m_climbElevator = null;

  public LEDSubsystem m_ledSubsystem = null;

  public GamePieceVisionSubsystem m_gamePieceVision = null;

  // All subsystems bundled for passing to OperatorControls, commands, etc.
  private Subsystems m_subsystems;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    m_shooter = new Shooter(true);
    m_autoShootCmd = m_shooter.getAutoShootCmd();
    m_targetSelector = new ShooterTargetSelector(m_shooter);
    m_intake = new Intake(true);
    m_climbElevator = new ClimbElevator(false);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // drive =
        // new Drive(
        // new GyroIO() {},
        // new ModuleIOSim(TunerConstants.FrontLeft),
        // new ModuleIOSim(TunerConstants.FrontRight),
        // new ModuleIOSim(TunerConstants.BackLeft),
        // new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::setPose,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        // vision =
        // new Vision(
        // demoDrive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0),
        // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        DriverStation.silenceJoystickConnectionWarning(true);

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::setPose,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::setPose,
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    // Instantiate game piece vision subsystem (dedicated object-detection camera)
    switch (Constants.currentMode) {
      case REAL:
        m_gamePieceVision =
            new GamePieceVisionSubsystem(
                new GamePieceVisionIOPhoton(
                    GamePieceVisionConstants.CAMERA_NAME,
                    GamePieceVisionConstants.ROBOT_TO_INTAKE_CAM));
        break;
      case SIM:
        m_gamePieceVision = new GamePieceVisionSubsystem(new GamePieceVisionIOSim());
        break;
      default:
        m_gamePieceVision = new GamePieceVisionSubsystem(new GamePieceVisionIO() {});
        break;
    }

    // Bundle subsystems for OperatorControls and other consumers
    m_subsystems =
        new Subsystems(drive, m_shooter, m_intake, m_climbElevator, vision, m_gamePieceVision);

    // Instantiate LED subsystem after m_subsystems so it can receive the full
    // bundle
    m_ledSubsystem = new LEDSubsystem(true, m_subsystems);

    // Register SuperFastClimb as PathPlanner named commands for use in .auto files.
    // Must be registered before AutoBuilder.buildAutoChooser().
    NamedCommands.registerCommand(
        "SuperFastClimb/Left",
        ClimbCommands.superFastClimb(drive, m_climbElevator, FieldSpot.CLIMB_LEFT));
    NamedCommands.registerCommand(
        "SuperFastClimb/Center",
        ClimbCommands.superFastClimb(drive, m_climbElevator, FieldSpot.CLIMB_CENTER));
    NamedCommands.registerCommand(
        "SuperFastClimb/Right",
        ClimbCommands.superFastClimb(drive, m_climbElevator, FieldSpot.CLIMB_RIGHT));
    NamedCommands.registerCommand("Intake.Deploy", m_intake.getDeployInstantCmd());
    NamedCommands.registerCommand("Intake.Start", getAutonomousCommand());
    // Shoot.Start / Shoot.Stop: instant commands for use in PathPlanner auto routines.
    // They schedule/cancel the same shared AutoShoot command instance used by the driver trigger,
    // so the toggle state stays consistent across both teleop and auto use.
    NamedCommands.registerCommand(
        "Shoot.Start",
        Commands.runOnce(() -> CommandScheduler.getInstance().schedule(m_autoShootCmd)));
    NamedCommands.registerCommand("Shoot.Stop", Commands.runOnce(m_autoShootCmd::cancel));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Publish spot-selection commands to Shuffleboard
    configureSpotSelectorDashboard();

    // Vision/gyro calibration commands are published by
    // OperatorControls.configureButtonBox().

    // Configure FuelSim for sim
    if (Constants.currentMode == Constants.Mode.SIM) {
      configureFuelSim();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            // () -> -controller.getRightX()
            // () -> -controller.getRawAxis(2)));
            () -> -controller.getRawAxis(4)));

    // // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> Rotation2d.kZero));

    // Left trigger (toggle): deploy arm on every press; toggle roller
    // independently.
    // Press 1 — arm deploys, roller starts. Press 2 — roller stops, arm stays
    // deployed.
    controller.leftTrigger().onTrue(m_intake.getDeployArmCmd());
    controller.leftTrigger().toggleOnTrue(m_intake.getRollerOnCmd());
    // Left bumper: retract intake arm
    controller.leftBumper().onTrue(m_intake.getRetractCmd());

    // Right trigger (toggle): auto shoot — predictive targeting mode
    // Hood and flywheel override their idle defaults; kicker and spindexer engage
    // once ready.
    controller.rightTrigger().toggleOnTrue(m_autoShootCmd);

    // Right bumper (hold): manual shoot — kicker and spindexer only
    // Assumes turret, hood, and flywheel are already at the correct setpoints.
    controller.rightBumper().whileTrue(m_shooter.getManualShootCmd());

    // Y button: auto-select a target spot based on robot position, then drive to
    // it.
    // Blue/Red home zone → HUB_NEUTRAL; Neutral zone → SHOOTING_TOP/BOTTOM by field
    // half.
    controller
        .y()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> m_spotSelector.autoSelectForRobot(drive.getPose())),
                new DriveToSpot(drive, m_spotSelector)));

    // TODO: Wire button box buttons to spot selection once hardware is available.
    // Example bindings (replace JoystickButton refs with your actual button box
    // inputs):
    // buttonBox.button(1).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.BLUE_TRENCH_TOP));
    // buttonBox.button(2).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.BLUE_TRENCH_BOTTOM));
    // buttonBox.button(3).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.RED_TRENCH_TOP));
    // buttonBox.button(4).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.RED_TRENCH_BOTTOM));
    // buttonBox.button(5).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.HUB_ALLIANCE));
    // buttonBox.button(6).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.HUB_NEUTRAL));
    // buttonBox.button(7).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_LEFT));
    // buttonBox.button(8).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_CENTER));
    // buttonBox.button(9).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_RIGHT));
    // buttonBox.button(10).onTrue(m_spotSelector.cmdClearSelection());

    // A button (hold): pathfind to the start of "Left_Alliance_Neutral", then run
    // it.
    // Release to cancel.
    PathPlannerAuto leftAllianceNeutral = new PathPlannerAuto("Left_Alliance_Neutral");
    controller
        .a()
        .whileTrue(
            Commands.sequence(
                // AutoBuilder.pathfindToPose(
                // leftAllianceNeutral.getStartingPose(),
                // new PathConstraints(4.0, 3.0, Math.toRadians(540), Math.toRadians(720))),
                leftAllianceNeutral));

    // B button (hold): auto-drive toward detected game pieces and deploy intake.
    // Release to cancel. Intake stays deployed after release — retract manually if
    // needed.
    controller.b().whileTrue(new AutoDriveToFuel(drive, m_intake, m_gamePieceVision));

    // AutoTargetSelector is the default command: selects the correct ShootingTarget
    // based on
    // the robot's field zone. Manual operator selections override it until
    // re-engaged.
    m_targetSelector.setDefaultCommand(new AutoTargetSelector(m_targetSelector, drive));

    // Student-owned operator controls (hood retraction, bump traversal, passing
    // mode, etc.)
    new OperatorControls(m_subsystems, controller, m_spotSelector, m_targetSelector)
        .configureControls();

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Back button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      boolean isRed =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              !isRed ? Rotation2d.kZero : Rotation2d.fromDegrees(180)));
                    },
                    drive)
                .ignoringDisable(true));

    // Reset pose to hub wall face (X=3.6, Y=4.0) and reset gyro heading when Start
    // button is
    // pressed. Same action as the Calibration/Reset Pose Hub Wall button on the
    // dashboard.
    controller.start().onTrue(drive.getResetPoseHubWallCmd());

    // D-pad: turret trim (driver)
    controller.povUp().onTrue(m_shooter.getTurretCenterCmd()); // center turret
    controller.povRight().onTrue(m_shooter.getTurretTrimCmd(+1)); // trim right
    controller.povLeft().onTrue(m_shooter.getTurretTrimCmd(-1)); // trim left
    controller.povDown().onTrue(m_shooter.getTurretTrimResetCmd()); // reset trim to 0
  }

  /**
   * Publishes all spot-selection commands to a dedicated Shuffleboard tab so operators can trigger
   * them from the dashboard (or verify button box wiring during testing).
   */
  private void configureSpotSelectorDashboard() {
    var tab = Shuffleboard.getTab("SpotSelector");
    tab.add("Blue Trench Top", m_spotSelector.cmdSelectSpot(FieldSpot.BLUE_TRENCH_TOP));
    tab.add("Blue Trench Bottom", m_spotSelector.cmdSelectSpot(FieldSpot.BLUE_TRENCH_BOTTOM));
    tab.add("Red Trench Top", m_spotSelector.cmdSelectSpot(FieldSpot.RED_TRENCH_TOP));
    tab.add("Red Trench Bottom", m_spotSelector.cmdSelectSpot(FieldSpot.RED_TRENCH_BOTTOM));
    tab.add("Hub Alliance", m_spotSelector.cmdSelectSpot(FieldSpot.HUB_ALLIANCE));
    tab.add("Hub Neutral", m_spotSelector.cmdSelectSpot(FieldSpot.HUB_NEUTRAL));
    tab.add("Climb Left", m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_LEFT));
    tab.add("Climb Center", m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_CENTER));
    tab.add("Climb Right", m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_RIGHT));
    tab.add("Clear Selection", m_spotSelector.cmdClearSelection());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    // PathPlannerAuto instances are single-use — WPILib marks composed commands as
    // non-reusable. Rebuild from the auto name so a fresh command tree is created
    // every autonomous period.
    if (selected instanceof PathPlannerAuto) {
      return new PathPlannerAuto(selected.getName());
    }
    return selected;
  }

  /**
   * Returns the starting pose of the currently selected PathPlanner auto, if the selected auto is a
   * {@link PathPlannerAuto}. Returns empty for SysId routines and other non-path commands.
   */
  public Optional<Pose2d> getSelectedAutoStartPose() {
    Command selected = autoChooser.get();
    if (selected instanceof PathPlannerAuto auto) {
      return Optional.ofNullable(auto.getStartingPose());
    }
    return Optional.empty();
  }

  /**
   * Publishes the selected auto's starting pose to telemetry so the Field widget can show a marker
   * for prematch alignment. Publishes the origin when no starting pose is available (e.g., "None"
   * or a SysId routine is selected).
   */
  public void publishAutoStartPose() {
    Optional<Pose2d> autoStart = getSelectedAutoStartPose();
    Logger.recordOutput("AutoStart/Pose", autoStart.orElse(new Pose2d()));
  }

  public Pose2d getRobotPose2d() {
    return drive.getPose();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return drive.getChassisSpeeds();
  }

  private static RobotContainer instance = null;

  public static RobotContainer getInstance() {
    return instance;
  }

  /** Immutable robot dimensions for FuelSim / simulation */
  public record Dimensions(
      Distance fullLength, // meters, front-to-back
      Distance fullWidth, // meters, side-to-side
      Distance bumperHeight, // meters
      Distance intakeDepth // meters, how far the intake extends past frame
      ) {

    /** Convenience factory for your 26.75 in square robot with 1 intake at back */
    public static Dimensions createRobot() {
      return new Dimensions(
          Inches.of(26.75), // fullLength
          Inches.of(26.75), // fullWidth
          Inches.of(10.0), // bumperHeight
          Inches.of(7.0) // intakeDepth
          );
    }
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    Dimensions dimensions = Dimensions.createRobot();

    // Start with no fuel
    instance.clearFuel();

    // Register the robot with FuelSim
    instance.registerRobot(
        dimensions.fullWidth.in(Meters),
        dimensions.fullLength.in(Meters),
        dimensions.bumperHeight.in(Meters),
        drive::getPose,
        drive::getFieldSpeeds);

    // Register a single back intake
    instance.registerIntake(
        -dimensions.fullLength.div(2).in(Meters), // xMin (rear of robot)
        dimensions.fullLength.div(2).in(Meters), // xMax (front of robot)
        -dimensions.fullWidth.div(2).in(Meters), // yMin (back edge)
        -dimensions.fullWidth.div(2).plus(Inches.of(7)).in(Meters), // yMax (intake depth)
        () -> true, // only one intake check
        () -> {}) // callback when fuel is collected
    ;

    // Start the simulation
    instance.start();

    // Dashboard reset button
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));

    // Dashboard clear fuel button
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                })
            .withName("Clear Fuel")
            .ignoringDisable(true));
  }
}
