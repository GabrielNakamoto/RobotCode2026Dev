package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] cameraInputs;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.cameraInputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; ++i) {
      cameraInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(cameraInputs[i]);
      Logger.processInputs("Vision/" + cameras[i].getConfig().name(), cameraInputs[i]);

      processCamera(cameras[i].getConfig(), cameraInputs[i])
          .ifPresent(obsv -> RobotState.getInstance().addVisionMeasurement(obsv));
    }
  }

  private static Optional<VisionObservation> processCamera(
      CameraConfig config, VisionIOInputs inputs) {
    if (!isTrustableMeasurement(inputs)) return Optional.empty();

    double scale = 1.0 / inputs.mt1.quality();

    double xStd = inputs.stddevs[0] * scale;
    double yStd = inputs.stddevs[1] * scale;
    double yawStd = inputs.stddevs[5] * scale;

    return Optional.of(
        new VisionObservation(
            config,
            inputs.mt1.pose(),
            VecBuilder.fill(Math.max(xStd, yStd), Math.max(xStd, yStd), yawStd),
            Seconds.of(inputs.mt1.timestamp())));
  }

  private static boolean isTrustableMeasurement(VisionIOInputs inputs) {
    var estimate = inputs.mt1;
    Pose2d pose = estimate.pose();

    boolean tagless = estimate.tagCount() == 0;
    boolean tooHigh =
        inputs.pose3d == null ? false : Math.abs(inputs.pose3d.getZ()) > VisionConstants.zThreshold;
    boolean outofbounds =
        pose.getX() < 0.0
            || pose.getMeasureX().gt(FieldConstants.fieldLength)
            || pose.getY() < 0.0
            || pose.getMeasureY().gt(FieldConstants.fieldWidth);

    if (tagless || tooHigh || outofbounds) {
      return false;
    }

    if (estimate.tagCount() < 2) {
      boolean ambiguous =
          Arrays.stream(inputs.fiducials)
              .filter(f -> f.ambiguity() > VisionConstants.maxSingleTagAmbiguity)
              .findAny()
              .isPresent();
      boolean smallArea = estimate.avgTagArea() < VisionConstants.minSingleTagArea;

      if (ambiguous || smallArea) return false;
    }

    return true;
  }

  public boolean setRobotToCamera(String cameraName, Transform3d robotToCamera) {
    for (int i = 0; i < cameras.length; ++i) {
      if (cameras[i].getConfig().name().equals(cameraName)) {
        cameras[i].setRobotToCamera(robotToCamera);
        return true;
      }
    }
    return false;
  }

  public void captureRewind(double duration) {
    for (var camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
