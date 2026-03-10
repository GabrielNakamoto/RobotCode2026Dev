// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.VisionConstants;
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
  private final PhotonCamera camera = new PhotonCamera("turret");
  private final PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  // Our camera is mounted at...
  // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
  Translation3d robotToCameraTrl =
      new Translation3d(Inches.of(-6.0), Inches.of(0.0), Inches.of(18.667));
  // and pitched 30 degrees up.
  Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-30.0), 0);
  Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

  PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);

  private final Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier;
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOSim(
      Supplier<Pose2d> poseSupplier, Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier) {
    this.dynamicCameraPoseSupplier = dynamicCameraPoseSupplier;
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(FieldConstants.aprilLayout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
    cameraProp.setExposureTimeMs(120);
    cameraProp.setFPS(60);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(new PhotonCamera("turret"), cameraProp);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable the raw and processed streams. These are enabled by default.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // The turret the camera is mounted on is rotated 5 degrees
    if (dynamicCameraPoseSupplier.isPresent()) {
      Pose3d cameraPose = dynamicCameraPoseSupplier.get().get();
      Transform3d cameraTransform =
          new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
      visionSim.adjustCamera(cameraSim, cameraTransform);
      photonEstimator.setRobotToCameraTransform(cameraTransform);
    }

    // Update with the simulated drivetrain pose. This should be called every loop in simulation.
    visionSim.update(poseSupplier.get());

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    PhotonPipelineResult result = camera.getLatestResult();
    visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    }
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
