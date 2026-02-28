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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import java.util.Optional;
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

  public ShooterTargetSelector m_targetSelector = null;

  public Intake m_intake = null;

  public ClimbElevator m_climbElevator = null;

  public LEDSubsystem m_ledSubsystem = null;

  // All subsystems bundled for passing to OperatorControls, commands, etc.
  private Subsystems m_subsystems;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    m_shooter = new Shooter(true);
    m_targetSelector = new ShooterTargetSelector(m_shooter);
    m_intake = new Intake(false);
    m_climbElevator = new ClimbElevator(false);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(TunerConstants.FrontLeft),
        // new ModuleIOTalonFX(TunerConstants.FrontRight),
        // new ModuleIOTalonFX(TunerConstants.BackLeft),
        // new ModuleIOTalonFX(TunerConstants.BackRight));

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

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

    // Instantiate LED subsystem — real only on REAL hardware, stubbed in sim/replay
    m_ledSubsystem = new LEDSubsystem(false, vision, m_shooter);

    // Bundle subsystems for OperatorControls and other consumers
    m_subsystems = new Subsystems(drive, m_shooter, m_intake, m_climbElevator, vision);

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

    // Left trigger (hold): deploy intake arm and run intake roller
    // getDeployCmd() runs the roller internally — no .alongWith() needed.
    controller.leftTrigger().whileTrue(m_intake.getDeployCmd());

    // Left bumper: retract intake arm
    controller.leftBumper().onTrue(m_intake.getRetractCmd());

    // Right trigger (hold): auto shoot — predictive targeting mode
    // Hood and flywheel override their idle defaults; kicker and spindexer engage
    // once ready.
    controller.rightTrigger().whileTrue(m_shooter.getAutoShootCmd());

    // Right bumper (hold): manual shoot — kicker and spindexer only
    // Assumes turret, hood, and flywheel are already at the correct setpoints.
    controller.rightBumper().whileTrue(m_shooter.getManualShootCmd());

    // Y button: drive to the currently selected field spot via PathPlanner
    // on-the-fly pathfinding.
    // Use the button box (see TODO below) to select a spot before pressing Y.
    controller.y().whileTrue(new DriveToSpot(drive, m_spotSelector));

    // TODO: Wire button box buttons to spot selection once hardware is available.
    // Example bindings (replace JoystickButton refs with your actual button box
    // inputs):
    // buttonBox.button(1).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_LEFT));
    // buttonBox.button(2).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_RIGHT));
    // buttonBox.button(3).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_LEFT));
    // buttonBox.button(4).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_RIGHT));
    // buttonBox.button(5).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.HUB_ALLIANCE));
    // buttonBox.button(6).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.HUB_NEUTRAL));
    // buttonBox.button(7).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_LEFT));
    // buttonBox.button(8).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_CENTER));
    // buttonBox.button(9).onTrue(m_spotSelector.cmdSelectSpot(FieldSpot.CLIMB_RIGHT));
    // buttonBox.button(10).onTrue(m_spotSelector.cmdClearSelection());

    // Robot centric drive when B button is held
    // controller
    // .b()
    // .whileTrue(
    // DriveCommands.joystickDriveRobotCentric(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // // () -> -controller.getRightX()));
    // // () -> -controller.getRawAxis(2)));
    // () -> -controller.getRawAxis(4)));

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
                              isRed ? Rotation2d.kZero : Rotation2d.fromDegrees(180)));
                    },
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Publishes all spot-selection commands to a dedicated Shuffleboard tab so operators can trigger
   * them from the dashboard (or verify button box wiring during testing).
   */
  private void configureSpotSelectorDashboard() {
    var tab = Shuffleboard.getTab("SpotSelector");
    tab.add("Alliance Trench Left", m_spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_LEFT));
    tab.add("Alliance Trench Right", m_spotSelector.cmdSelectSpot(FieldSpot.ALLIANCE_TRENCH_RIGHT));
    tab.add("Opponent Trench Left", m_spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_LEFT));
    tab.add("Opponent Trench Right", m_spotSelector.cmdSelectSpot(FieldSpot.OPPONENT_TRENCH_RIGHT));
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
    return autoChooser.get();
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
   * During disabled, keeps the SpotSelector pointed at the selected auto's starting pose so the
   * Field widget shows a green X for prematch alignment. If no auto start pose is available (e.g.,
   * "None" or a SysId routine is selected), falls back to the robot's current pose so the green X
   * sits on the robot and effectively disappears without changing the key's data type.
   */
  public void updatePrematchAutoStartPose() {
    Optional<Pose2d> autoStart = getSelectedAutoStartPose();
    m_spotSelector.selectCustomPose(
        autoStart.orElse(drive.getPose()), autoStart.isPresent() ? "Auto Start" : "Robot");
  }

  /** Clears the prematch auto-start-pose visualization when teleop begins. */
  public void clearPrematchAutoStartPose() {
    m_spotSelector.clearSelection();
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
