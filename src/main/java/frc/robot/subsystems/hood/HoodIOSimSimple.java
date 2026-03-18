// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

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
public class HoodIOSimSimple implements HoodIO {
  private DCMotor gearbox = DCMotor.getKrakenX44(1);
  private DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.07867078, TurretConstants.hoodGearRatio),
          gearbox);

  private PIDController hoodPID = new PIDController(200, 0, 0);

  public HoodIOSimSimple() {
    hoodSim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double motorVoltage = hoodPID.calculate(hoodSim.getAngularPositionRotations());
    hoodSim.setInputVoltage(motorVoltage);
    hoodSim.update(0.020);

    inputs.isConnected = true;
    inputs.voltageApplied = Volts.of(motorVoltage);
    inputs.current = Amps.zero();
    inputs.velocity = hoodSim.getAngularVelocity();
    inputs.position = hoodSim.getAngularPosition();
  }

  @Override
  public void setAngle(Angle angle) {
    Logger.recordOutput("Hood/setpoint", angle);
    hoodPID.setSetpoint(angle.in(Rotations));
  }
}
