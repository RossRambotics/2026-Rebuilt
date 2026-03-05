package frc.robot;

import frc.robot.subsystems.climb.ClimbElevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.GamePieceVisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * Bundle of all subsystems, passed to classes that need to bind controls or create commands.
 *
 * <p>When you add a new subsystem, add a field here so OperatorControls (and any other consumer)
 * can access it without changing constructor signatures everywhere.
 */
public record Subsystems(
    Drive drive,
    Shooter shooter,
    Intake intake,
    ClimbElevator climbElevator,
    Vision vision,
    GamePieceVisionSubsystem gamePieceVision) {}
