// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConfig.TurretConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AzimuthIOSimSimple implements AzimuthIO {
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.07867078, TurretConstants.azimuthGearRatio),
          gearbox);

  private PIDController turretPID = new PIDController(200, 0, 0);

  public AzimuthIOSimSimple() {
    turretSim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    double motorVoltage = turretPID.calculate(turretSim.getAngularPositionRotations());
    turretSim.setInputVoltage(motorVoltage);
    turretSim.update(0.020);

    inputs.isConnected = true;
    inputs.voltageApplied = Volts.of(motorVoltage);
    inputs.current = Amps.zero();
    inputs.velocity = turretSim.getAngularVelocity();
    inputs.position = turretSim.getAngularPosition();
  }

  @Override
  public void setAngle(Angle angle) {
    Logger.recordOutput("Azimuth/Setpoint", angle);
    turretPID.setSetpoint(angle.in(Rotations));
  }
}
