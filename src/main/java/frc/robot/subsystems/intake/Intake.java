package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotConfig.IntakeConstants;

enum IntakeState {
  IDLE,
  RETRACT,
  INTAKE
}

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs;

  public Intake(IntakeIO io) {
    this.io = io;
    setState(IntakeState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    applyState();
    io.applyOutputs(outputs);
  }

  public void run() {
    setState(IntakeState.INTAKE);
  }

  public void retract() {
    setState(IntakeState.RETRACT);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case IDLE:
        outputs.extendMeters = inputs.extendPositionRads / IntakeConstants.extensionRadius;
        outputs.intakeVoltage = 0.0;
        break;
      case RETRACT:
        // TODO: run intake slowly while retracting, unstuck balls
        outputs.extendMeters = IntakeConstants.maxRetractionMeters;
        outputs.intakeVoltage = 0.5;
        break;
      case INTAKE:
        outputs.extendMeters = IntakeConstants.maxExtensionMeters;
        outputs.intakeVoltage = 4.0;
        break;
    }
  }
}
