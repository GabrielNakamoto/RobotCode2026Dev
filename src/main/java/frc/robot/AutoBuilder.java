package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.HashMap;
import java.util.Set;

public class AutoBuilder {
  private static final HashMap<String, Trajectory<SwerveSample>> autoTrajectories = new HashMap<>();

  static {
    String[] trajNames = {"FullFuelSwipe", "FullSecondSwipe", "CleanSwipe"};

    for (String name : trajNames) {
      autoTrajectories.put(name, (Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get());
    }
  }

  private static Command fuelSwipeFull(
      String pathName, Rotation2d endHeading, Drive drive, SuperStructure superStructure) {
    return Commands.sequence(
        superStructure.intake(),
        // superStructure.setTarget(TurretTarget.NEAREST_TAG),
        drive.followChoreoTrajectoryCommand(autoTrajectories.get(pathName)),
        superStructure.idle(),
        drive.driveToPoseCommand(AllianceFlip.apply(new Pose2d(3.5784, 0.663, endHeading))),
        Commands.runOnce(() -> drive.setIdle())
        // superStructure.setTarget(TurretTarget.HUB),
        );
  }

  public static Command doubleSwipeTestAuto(Drive drive, SuperStructure superStructure) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetOdometry(
                        autoTrajectories
                            .get("FullFuelSwipe")
                            .getInitialPose(!FieldConstants.isBlueAlliance())
                            .get())),
        Commands.defer(
            () -> fuelSwipeFull("FullFuelSwipe", Rotation2d.kZero, drive, superStructure),
            Set.of(drive, superStructure)),
        superStructure.shoot(),
        Commands.waitSeconds(2.0),
        Commands.defer(
            () -> fuelSwipeFull("CleanSwipe", Rotation2d.k180deg, drive, superStructure),
            Set.of(drive, superStructure)),
        superStructure.shoot(),
        Commands.waitSeconds(2.0),
        superStructure.idle());
  }
}
