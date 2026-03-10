package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

public class AutoBuilder {
  private static final HashMap<String, Trajectory<SwerveSample>> autoTrajectories = new HashMap<>();

  static {
    String[] trajNames = {"FullFuelSwipe", "FullSecondSwipe", "CleanSwipe"};

    for (String name : trajNames) {
      Trajectory<SwerveSample> traj = (Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get();
      autoTrajectories.put(name, traj);
      autoTrajectories.put(name + "_flipped", traj.flipped());
    }
  }

  public static class Auto {
    enum NodeType {
      STATE_CHANGE,
      TARGET_TRACK,
      DRIVE_TO_POSE,
      CHOREO_TRAJ,
      WAIT
    }

    private final Drive drive;
    private final SuperStructure superStructure;

    private List<Supplier<Pose2d>> targetPoses = new ArrayList<>();
    private List<String> trajectoryNames = new ArrayList<>();
    private List<SuperStructureState> superStates = new ArrayList<>();
    private List<TurretTarget> turretTargets = new ArrayList<>();
    private List<Double> delays = new ArrayList<>();

    private List<NodeType> graph = new ArrayList<>();
    private boolean flipped = false;

    public Auto(Drive drive, SuperStructure superStructure) {
      this.drive = drive;
      this.superStructure = superStructure;
    }

    public Command generate() {
      List<Command> commands = new ArrayList<>();
      int[] nodeFreqs = new int[NodeType.values().length];
      NodeType firstMove =
          graph.stream()
              .filter(n -> n == NodeType.DRIVE_TO_POSE || n == NodeType.CHOREO_TRAJ)
              .findFirst()
              .get();
      Supplier<Pose2d> initialPose =
          () ->
              switch (firstMove) {
                case DRIVE_TO_POSE -> targetPoses.get(0).get();
                case CHOREO_TRAJ ->
                    autoTrajectories
                        .get(trajectoryNames.get(0) + (flipped ? "_flipped" : ""))
                        .getInitialPose(!FieldConstants.isBlueAlliance())
                        .get();
                default -> Pose2d.kZero;
              };
      commands.add(
          Commands.defer(
              () ->
                  Commands.runOnce(() -> RobotState.getInstance().resetOdometry(initialPose.get())),
              Set.of()));
      for (NodeType node : graph) {
        int ctxIndex = nodeFreqs[node.ordinal()];
        commands.add(
            switch (node) {
              case WAIT -> Commands.waitSeconds(delays.get(ctxIndex));
              case DRIVE_TO_POSE -> {
                // TODO: flip
                yield drive.driveToPoseCommandDeferred(targetPoses.get(ctxIndex));
              }
              case TARGET_TRACK -> superStructure.setTarget(turretTargets.get(ctxIndex));
              case STATE_CHANGE ->
                  Commands.runOnce(() -> superStructure.setState(superStates.get(ctxIndex)));
              case CHOREO_TRAJ ->
                  drive.followChoreoTrajectoryCommand(
                      autoTrajectories.get(
                          trajectoryNames.get(ctxIndex) + (flipped ? "_flipped" : "")));
            });
        nodeFreqs[node.ordinal()] += 1;
      }
      return Commands.sequence(commands.toArray(Command[]::new));
    }

    public Auto flipped() {
      flipped = !flipped;
      return this;
    }

    public Auto withStateChange(SuperStructureState state) {
      graph.add(NodeType.STATE_CHANGE);
      superStates.add(state);
      return this;
    }

    public Auto withTrackTarget(TurretTarget target) {
      graph.add(NodeType.TARGET_TRACK);
      turretTargets.add(target);
      return this;
    }

    public Auto withDelay(double seconds) {
      graph.add(NodeType.WAIT);
      delays.add(seconds);
      return this;
    }

    public Auto withDriveToPose(Supplier<Pose2d> pose) {
      graph.add(NodeType.DRIVE_TO_POSE);
      targetPoses.add(pose);
      return this;
    }

    public Auto withChoreoTraj(String name) {
      graph.add(NodeType.CHOREO_TRAJ);
      trajectoryNames.add(name);
      return this;
    }
  }

  public static Command doubleSwipeCleanup(Drive drive, SuperStructure superStructure) {
    return new Auto(drive, superStructure)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("FullFuelSwipe")
        .withDriveToPose(() -> AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.kZero)))
        .withStateChange(SuperStructureState.SHOOT)
        .withDelay(2.0)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CleanSwipe")
        .withDriveToPose(() -> AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.k180deg)))
        .withStateChange(SuperStructureState.SHOOT)
        .generate();
  }

  public static Command doubleSwipeHumanStation(Drive drive, SuperStructure superStructure) {
    return new Auto(drive, superStructure)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("FullFuelSwipe")
        .withDriveToPose(() -> AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.kZero)))
        .withStateChange(SuperStructureState.SHOOT)
        .withDelay(2.0)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CleanSwipe")
        .withDriveToPose(() -> AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.k180deg)))
        .withStateChange(SuperStructureState.SHOOT)
        .withDelay(1.0)
				.withDriveToPose(() -> FieldConstants.getHumanStation())
        .withDelay(0.8)
				.withDriveToPose(() -> AllianceFlip.apply(new Pose2d(1.75, 1.75, Rotation2d.kZero)))
        .generate();
  }
}
