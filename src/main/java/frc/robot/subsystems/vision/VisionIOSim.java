// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;

  // Our camera is mounted at...
  // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
  private static final Translation3d robotToCameraTrl =
      new Translation3d(Inches.of(-6.0), Inches.of(0.0), Inches.of(18.667));
  // and pitched 30 degrees up.
  private static final Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-30.0), 0);
  private static final Transform3d robotToCamera =
      new Transform3d(robotToCameraTrl, robotToCameraRot);

  private final PhotonPoseEstimator photonEstimator =
      new PhotonPoseEstimator(FieldConstants.aprilLayout, robotToCamera);
  private final Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier;
  private final CameraConfig config;

  public VisionIOSim(CameraConfig config, Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier) {
    this.dynamicCameraPoseSupplier = dynamicCameraPoseSupplier;
    this.config = config;
    this.camera = new PhotonCamera(config.name());

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

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (dynamicCameraPoseSupplier.isPresent()) {
      Pose3d cameraPose = dynamicCameraPoseSupplier.get().get();
      Transform3d cameraTransform =
          new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
      visionSim.adjustCamera(cameraSim, cameraTransform);
      photonEstimator.setRobotToCameraTransform(cameraTransform);
    }

    visionSim.update(RobotState.getInstance().getSimulatedPose());

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    PhotonPipelineResult result = camera.getLatestResult();
    visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
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
    double baseYaw = rawStdDevs[11];

    double distanceScalar =
        1.0
            + Math.pow(
                avgTagDist / VisionConstants.maxReliableDistance,
                VisionConstants.distanceScalingExponent);

    double tagScalar = (numTags == 1) ? VisionConstants.singleTagPenalty : 1.0;
    double multiplier = VisionConstants.baseStddevMultiplier * distanceScalar * tagScalar;
    return VecBuilder.fill(baseX * multiplier, baseY * multiplier, baseYaw * multiplier);
  }
}
