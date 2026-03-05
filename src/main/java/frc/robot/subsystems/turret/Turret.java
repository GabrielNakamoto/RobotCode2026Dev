package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.StateSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

enum TurretState {
  IDLE,
  TRACK,
  SHOOT
}

public class Turret extends StateSubsystem<TurretState> {
  public static record ShotParameters(Angle yawAngle, Angle hoodAngle, double flywheelSpeed) {}

  private Supplier<ShotParameters> parameters;

  private final TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretIOOutputs outputs = new TurretIOOutputs();

  public Turret(TurretIO io, Supplier<ShotParameters> parameters) {
    this.io = io;
    this.parameters = parameters;

    setState(TurretState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    RobotState.getInstance()
        .updateTurretState(
            new RobotState.TurretState(
                Radians.of(inputs.hoodPositionRad),
                Radians.of(inputs.yawPositionRad),
                MetersPerSecond.of(inputs.shootVelocityRadsPerSec)));

    applyState();
    io.applyOutputs(outputs);
  }

  public void track() {
    setState(TurretState.TRACK);
  }

  public void shoot() {
    setState(TurretState.SHOOT);
  }

  @Override
  public void applyState() {
    var state = getCurrentState();
    var params = parameters.get();
    switch (state) {
      case IDLE:
        outputs.hoodAngle = inputs.hoodPositionRad;
        outputs.yawAngle = inputs.yawPositionRad;
        outputs.flywheelSpeed = 0.0;
        break;
      case TRACK:
      case SHOOT:
        outputs.hoodAngle = params.hoodAngle().in(Units.Radian);
        outputs.yawAngle = params.yawAngle().in(Units.Radian);
        outputs.flywheelSpeed = state == TurretState.SHOOT ? params.flywheelSpeed() : 0.0;
        break;
    }
  }
}
