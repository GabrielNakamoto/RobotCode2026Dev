package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.TurretState;
import frc.robot.subsystems.azimuth.Azimuth;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SuperStructureState {
  IDLE,
  INTAKE,
  SHOOT
}

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private final Spindexer spindexer;
  private final Hood hood;
  private final Azimuth azimuth;
  private final Launcher launcher;
  private final Intake intake;

  public SuperStructure(
      Spindexer spindexer, Hood hood, Azimuth azimuth, Launcher launcher, Intake intake) {
    this.spindexer = spindexer;
    this.hood = hood;
    this.azimuth = azimuth;
    this.launcher = launcher;
    this.intake = intake;

    setState(SuperStructureState.IDLE);
  }

  @Override
  public void periodic() {
    applyState();
  }

  public Command idle() {
    return Commands.runOnce(() -> setState(SuperStructureState.IDLE));
  }

  public Command intake() {
    return Commands.runOnce(() -> setState(SuperStructureState.INTAKE));
  }

  public Command shoot() {
    return Commands.runOnce(() -> setState(SuperStructureState.SHOOT));
  }

  @Override
  public void applyState() {
    Logger.recordOutput("SuperStructure/state", getCurrentState());
    Angle testAzimuth =
        RobotState.getInstance().getEstimatedPose().getRotation().getMeasure().unaryMinus();
    TurretState setpoints = new TurretState(testAzimuth, Radians.of(0), RotationsPerSecond.of(25));
    /*TurretState setpoints =
    RobotState.getInstance()
        .getTurretSetpoints(
            new TurretState(azimuth.getAngle(), hood.getAngle(), launcher.getSpeed()));*/

    // Always track hub
    azimuth.setAngle(setpoints.azimuthAngle());
    hood.setAngle(setpoints.hoodAngle());

    switch (getCurrentState()) {
      case IDLE:
        launcher.setSpeed(RotationsPerSecond.of(0.0));
        intake.stay();
        spindexer.hold();
        break;
      case INTAKE:
        intake.run();
        spindexer.hold();
        break;
      case SHOOT:
        // TODO: Ensure azimuth + hood are within tolerance to shoot
        launcher.setSpeed(setpoints.launchSpeed());
        intake.retract();
        // TODO: Wait until shooter up to speed before feeding
        spindexer.feed();
        break;
    }
  }
}
