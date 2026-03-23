package frc.robot.subsystems.gamepiece;

/**
 * Simulation stub for GamePieceVisionIO.
 *
 * <p>Returns a connected camera with no detections. The {@link frc.robot.commands.AutoDriveToFuel}
 * command will run in "TargetLost" state (zero forward speed) during simulation.
 *
 * <p>Future enhancement: query {@link frc.robot.util.FuelSim} for the positions of simulated fuel
 * balls and compute synthetic yaw and area values based on the robot's simulated pose, enabling
 * realistic command testing in simulation.
 */
public class GamePieceVisionIOSim implements GamePieceVisionIO {

  @Override
  public void updateInputs(GamePieceVisionIOInputs inputs) {
    inputs.connected = true;
    inputs.targetYawsDeg = new double[0];
    inputs.targetPitchesDeg = new double[0];
    inputs.targetAreaPercents = new double[0];
    inputs.targetConfidences = new float[0];
    inputs.targetClassIds = new int[0];
    inputs.latestResultTimestampSec = 0.0;
  }
}
