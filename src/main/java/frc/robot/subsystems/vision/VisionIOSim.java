// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Simulated vision IO using PhotonVision simulation. */
public class VisionIOSim implements VisionIO {
  private static final double[] kDefaultStddevs =
      new double[] {0, 0, 0, 0, 0, 0, 0.5, 0.5, 0, 0, 0, 0.1};

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;

  private final PhotonPoseEstimator photonEstimator;
  private final Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier;
  private final CameraConfig config;

  public VisionIOSim(CameraConfig config, Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier) {
    this.dynamicCameraPoseSupplier = dynamicCameraPoseSupplier;
    this.config = config;
    this.camera = new PhotonCamera(config.name());
    this.photonEstimator =
        new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
    cameraProp.setExposureTimeMs(120);
    cameraProp.setFPS(60);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);

    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isConnected = true;

    if (dynamicCameraPoseSupplier.isPresent()) {
      Pose3d cameraPose = dynamicCameraPoseSupplier.get().get();
      Transform3d cameraTransform =
          new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
      visionSim.adjustCamera(cameraSim, cameraTransform);
      photonEstimator.setRobotToCameraTransform(cameraTransform);
    }

    visionSim.update(RobotState.getInstance().getSimulatedPose());

    PhotonPipelineResult result = camera.getLatestResult();
    Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    }

    if (visionEst.isPresent()) {
      var est = visionEst.get();
      inputs.poseEstimate = est.estimatedPose.toPose2d();
      inputs.numTags = est.targetsUsed.size();
      inputs.timestamp = est.timestampSeconds;

      // Calculate average tag distance
      double totalDist = 0.0;
      for (PhotonTrackedTarget target : est.targetsUsed) {
        totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
      }
      inputs.avgTagDist = est.targetsUsed.isEmpty() ? 0.0 : totalDist / est.targetsUsed.size();

      // Calculate standard deviations using the same hybrid formula as hardware
      inputs.stdDevs = calculateHybridStdDevs(kDefaultStddevs, inputs.avgTagDist, inputs.numTags);
    }
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  private Matrix<N3, N1> calculateHybridStdDevs(
      double[] rawStdDevs, double avgTagDist, int numTags) {
    double baseX = rawStdDevs[6];
    double baseY = rawStdDevs[7];

    double distanceScalar =
        1.0
            + Math.pow(
                avgTagDist / VisionConstants.maxReliableDistance,
                VisionConstants.distanceScalingExponent);

    double tagScalar = (numTags == 1) ? VisionConstants.singleTagPenalty : 1.0;
    double multiplier = VisionConstants.baseStddevMultiplier * distanceScalar * tagScalar;
    return VecBuilder.fill(baseX * multiplier, baseY * multiplier, Float.POSITIVE_INFINITY);
  }
}
