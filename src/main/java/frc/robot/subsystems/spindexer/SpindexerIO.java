package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
    public boolean indexConnected = false;
    public double indexMotorVoltsApplied = 0.0;
    public double indexMotorVelocityRadsPerSec = 0.0;

    public boolean feedConnected = false;
    public double feedMotorVoltsApplied = 0.0;
    public double feedMotorVelocityRadsPerSec = 0.0;
  }

  public class SpindexerIOOutputs {
    public double indexMotorVoltageRequested = 0.0;
    public double feedMotorVoltageRequested = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
