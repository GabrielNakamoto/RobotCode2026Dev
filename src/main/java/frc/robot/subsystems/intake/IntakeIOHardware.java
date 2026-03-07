package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

public class IntakeIOHardware implements IntakeIO {
  private final TalonFX extendMotor;
  private final TalonFXSignals extendSignals;

  private final SparkFlex intakeMotor;
  private final RelativeEncoder intakeEncoder;

  // private PositionTorqueCurrentFOC extendRequest = new PositionTorqueCurrentFOC(0.0);
  private PositionVoltage extendRequest = new PositionVoltage(0.0);

  public IntakeIOHardware(int extendId, int intakeId) {
    this.extendMotor = new TalonFX(extendId);
    this.intakeMotor = new SparkFlex(intakeId, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    var intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    intakeConfig.encoder.positionConversionFactor(IntakeConstants.intakeGearRatio);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    var extendConfig = new TalonFXConfiguration();
    extendConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.3);
    extendConfig.Feedback.withSensorToMechanismRatio(IntakeConstants.extendGearRatio);
    extendConfig.withSlot0(IntakeConstants.extendGains.toSlot0Configs());
    extendConfig.CurrentLimits.withStatorCurrentLimit(10);
    extendConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Units.metersToInches(IntakeConstants.maxExtensionMeters))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.0);
    extendMotor.getConfigurator().apply(extendConfig);
    extendMotor.setPosition(0.0);

    extendSignals = PhoenixSync.registerTalonFX(extendMotor, 50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extendConnected = extendSignals.isConnected();
    inputs.extendPositionInches = extendSignals.getPositionRads();
    inputs.extendVelocityRadsPerSec = extendSignals.getVelocityRadsPerSec();
    inputs.extendVoltageApplied = extendSignals.getVoltage();

    inputs.intakeConnected = intakeMotor.getLastError() == REVLibError.kOk;
    inputs.intakeVoltageApplied = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder.getVelocity());
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    Logger.recordOutput("Intake/extensionSetpoint", outputs.extendMeters);
    Logger.recordOutput("Intake/intakeSetpoint", outputs.intakeVoltage);

    extendMotor.setControl(extendRequest.withPosition(Units.metersToInches(outputs.extendMeters)));
    intakeMotor.setVoltage(outputs.intakeVoltage);
  }
}
