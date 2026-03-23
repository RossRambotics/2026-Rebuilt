package frc.robot.subsystems.gamepiece;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for a real PhotonVision object-detection camera (intake-cam). */
public class GamePieceVisionIOPhoton implements GamePieceVisionIO {

  private final PhotonCamera m_camera;
  // Stored for future field-relative game piece position estimation.
  @SuppressWarnings("unused")
  private final Transform3d m_robotToCamera;

  /**
   * Creates a new GamePieceVisionIOPhoton.
   *
   * @param cameraName The configured name of the camera on the coprocessor.
   * @param robotToCamera The 3D transform from robot center to the camera.
   */
  public GamePieceVisionIOPhoton(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    m_robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(GamePieceVisionIOInputs inputs) {
    inputs.connected = m_camera.isConnected();

    var results = m_camera.getAllUnreadResults();
    if (results.isEmpty()) {
      // No new frames — leave arrays from the last cycle in place so staleness
      // detection (based on latestResultTimestampSec) works correctly.
      return;
    }

    // Use only the most recent result (same pattern as VisionIOPhotonVision).
    var latest = results.get(results.size() - 1);
    inputs.latestResultTimestampSec = latest.getTimestampSeconds();

    if (!latest.hasTargets()) {
      inputs.targetYawsDeg = new double[0];
      inputs.targetPitchesDeg = new double[0];
      inputs.targetAreaPercents = new double[0];
      inputs.targetConfidences = new float[0];
      inputs.targetClassIds = new int[0];
      return;
    }

    // Collect accepted detections: filter by class ID, confidence, and area.
    List<PhotonTrackedTarget> accepted = new ArrayList<>();
    for (var t : latest.getTargets()) {
      if (t.getDetectedObjectClassID() != GamePieceVisionConstants.FUEL_CLASS_ID) continue;
      if (t.getDetectedObjectConfidence() < GamePieceVisionConstants.MIN_CONFIDENCE) continue;
      if (t.getArea() < GamePieceVisionConstants.MIN_TARGET_AREA_PERCENT) continue;
      accepted.add(t);
    }

    int n = accepted.size();
    inputs.targetYawsDeg = new double[n];
    inputs.targetPitchesDeg = new double[n];
    inputs.targetAreaPercents = new double[n];
    inputs.targetConfidences = new float[n];
    inputs.targetClassIds = new int[n];
    for (int i = 0; i < n; i++) {
      var t = accepted.get(i);
      inputs.targetYawsDeg[i] = t.getYaw();
      inputs.targetPitchesDeg[i] = t.getPitch();
      inputs.targetAreaPercents[i] = t.getArea();
      inputs.targetConfidences[i] = t.getDetectedObjectConfidence();
      inputs.targetClassIds[i] = t.getDetectedObjectClassID();
    }
  }
}
