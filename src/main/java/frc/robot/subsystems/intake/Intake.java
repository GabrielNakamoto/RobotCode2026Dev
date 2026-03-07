package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum IntakeState {
  IDLE,
  RETRACT,
  INTAKE
}

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs = new IntakeIOOutputs();

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

  public void stay() {
    setState(IntakeState.IDLE);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case IDLE:
        // outputs.extendMeters = Units.inchesToMeters(inputs.extendPositionInches);
        outputs.extendVoltage = 0.0;
        outputs.intakeVoltage = 0.0;
        break;
      case RETRACT:
        // TODO: run intake slowly while retracting, unstuck balls
        outputs.extendVoltage = -6.0;
        // outputs.extendMeters = IntakeConstants.maxRetractionMeters;
        outputs.intakeVoltage = 1.5;
        break;
      case INTAKE:
        outputs.extendVoltage = 6.0;
        // outputs.extendMeters = IntakeConstants.maxExtensionMeters;
        if (inputs.extendPositionInches > 5.5) {
          outputs.intakeVoltage = 8.0;
        } else {
          outputs.intakeVoltage = 0.0;
        }
        break;
    }
  }
}
