// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> poseResetter;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Gyro yaw validation state
  private final double[] latestMegatag1YawDeg;
  private final double[] latestMegatag1TimestampSec;
  private Pose2d bestMegatag1Pose = null;
  private double bestMegatag1TimestampSec = 0.0;
  private int bestMegatag1TagCount = 0;
  private boolean visionGyroMode = false;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> poseResetter,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.poseResetter = poseResetter;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize yaw tracking arrays
    latestMegatag1YawDeg = new double[io.length];
    latestMegatag1TimestampSec = new double[io.length];
    Arrays.fill(latestMegatag1YawDeg, Double.NaN);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Reset best pose tracking each cycle so stale high-tag-count observations
    // don't block future lower-tag-count (but still valid) observations
    bestMegatag1TagCount = 0;

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        // Track accepted MegaTag 1 yaw for gyro validation
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          latestMegatag1YawDeg[cameraIndex] =
              Math.toDegrees(observation.pose().getRotation().getZ());
          latestMegatag1TimestampSec[cameraIndex] = Timer.getFPGATimestamp();

          // Track best pose for reseeding (prefer more tags, then most recent)
          if (observation.tagCount() >= reseedMinTagCount
              && observation.averageTagDistance() <= reseedMaxAverageDistanceMeters
              && (observation.tagCount() > bestMegatag1TagCount
                  || observation.timestamp() > bestMegatag1TimestampSec)) {
            bestMegatag1Pose = observation.pose().toPose2d();
            bestMegatag1TimestampSec = Timer.getFPGATimestamp();
            bestMegatag1TagCount = observation.tagCount();
          }
        }
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Gyro yaw validation (every cycle)
    double nowSec = Timer.getFPGATimestamp();
    double gyroYawDeg = poseSupplier.get().getRotation().getDegrees();
    double freshnessThresholdSec = 2.0;

    // Check which camera yaws are fresh
    boolean isGyroYawGood = true; // Trust gyro unless vision disagrees
    for (int i = 0; i < io.length; i++) {
      boolean fresh =
          !Double.isNaN(latestMegatag1YawDeg[i])
              && (nowSec - latestMegatag1TimestampSec[i]) < freshnessThresholdSec;
      if (fresh) {
        // Compare this camera's yaw against gyro
        double diffDeg =
            Math.abs(MathUtil.inputModulus(latestMegatag1YawDeg[i] - gyroYawDeg, -180.0, 180.0));
        if (diffDeg >= gyroYawToleranceDeg) {
          isGyroYawGood = false;
        }
      }
    }

    // Also check pairwise camera agreement if both are fresh
    if (io.length >= 2
        && !Double.isNaN(latestMegatag1YawDeg[0])
        && !Double.isNaN(latestMegatag1YawDeg[1])
        && (nowSec - latestMegatag1TimestampSec[0]) < freshnessThresholdSec
        && (nowSec - latestMegatag1TimestampSec[1]) < freshnessThresholdSec) {
      double cameraDiffDeg =
          Math.abs(
              MathUtil.inputModulus(
                  latestMegatag1YawDeg[0] - latestMegatag1YawDeg[1], -180.0, 180.0));
      if (cameraDiffDeg >= gyroYawToleranceDeg) {
        isGyroYawGood = false;
      }
    }

    // Log yaw values, agreement, and mode
    Logger.recordOutput("Vision/GyroYawDeg", gyroYawDeg);
    for (int i = 0; i < io.length; i++) {
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(i) + "/Megatag1YawDeg", latestMegatag1YawDeg[i]);
    }
    Logger.recordOutput("Vision/IsGyroYawGood", isGyroYawGood);
    Logger.recordOutput("Vision/GyroMode", visionGyroMode ? "VisionGyro" : "NormalGyro");

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /** Enables VisionGyro mode — MegaTag 1 yaw monitoring active, reseed available. */
  public void enableVisionGyro() {
    visionGyroMode = true;
    Logger.recordOutput("Vision/GyroMode", "VisionGyro");
  }

  /** Enables NormalGyro mode — gyro trusted for heading, standard operation. */
  public void enableNormalGyro() {
    visionGyroMode = false;
    Logger.recordOutput("Vision/GyroMode", "NormalGyro");
  }

  /**
   * Reseeds the odometry heading from the best recent high-quality MegaTag 1 observation. Keeps
   * current XY position. Logs before/after yaw values.
   *
   * @return true if reseed was performed, false if no valid observation available
   */
  public boolean reseedFromVision() {
    if (bestMegatag1Pose == null || Timer.getFPGATimestamp() - bestMegatag1TimestampSec > 5.0) {
      return false;
    }
    Pose2d currentPose = poseSupplier.get();
    double beforeYawDeg = currentPose.getRotation().getDegrees();
    double afterYawDeg = bestMegatag1Pose.getRotation().getDegrees();
    poseResetter.accept(new Pose2d(currentPose.getTranslation(), bestMegatag1Pose.getRotation()));
    Logger.recordOutput("Vision/ReseedBeforeYawDeg", beforeYawDeg);
    Logger.recordOutput("Vision/ReseedAfterYawDeg", afterYawDeg);
    return true;
  }

  /**
   * Returns the rotation from the best recent high-quality MegaTag 1 observation, or empty if none
   * is available or the data is stale (>5 seconds old).
   */
  public Optional<Rotation2d> getBestMegatag1Rotation() {
    if (bestMegatag1Pose == null) {
      return Optional.empty();
    }
    if (Timer.getFPGATimestamp() - bestMegatag1TimestampSec > 5.0) {
      return Optional.empty();
    }
    return Optional.of(bestMegatag1Pose.getRotation());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
