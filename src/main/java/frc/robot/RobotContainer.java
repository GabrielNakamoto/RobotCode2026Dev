// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or
// share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.azimuth.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.util.FuelSim;
import frc.robot.util.PhoenixSync;

public class RobotContainer {
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final Drive drive;
  public Spindexer spindexer;
  public Intake intake;
  public Azimuth azimuth;
  public Hood hood;
  public Launcher launcher;
  public SuperStructure superStructure;
  // public final Vision vision;
  public FuelSim fuelSim = null;

  public RobotContainer() {
    switch (RobotConfig.getMode()) {
      case SIM:
        drive =
            new Drive(
                driveController,
                new DriveIOSim(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));
        spindexer = new Spindexer(new SpindexerIO() {});
        intake = new Intake(new IntakeIO() {});
        azimuth = new Azimuth(new AzimuthIO() {});
        hood = new Hood(new HoodIO() {});
        launcher = new Launcher(new LauncherIO() {});
        configureFuelSim();
        break;
      case REAL:
        drive =
            new Drive(
                driveController,
                new DriveIOHardware(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontRight,
                    TunerConstants.FrontLeft,
                    TunerConstants.BackRight,
                    TunerConstants.BackLeft));

        spindexer =
            new Spindexer(
                new SpindexerIOHardware(
                    RobotConfig.SpindexerConstants.indexMotorId,
                    RobotConfig.SpindexerConstants.feedMotorId));
        intake =
            new Intake(
                new IntakeIOHardware(IntakeConstants.extendMotorId, IntakeConstants.intakeMotorId));
        azimuth =
            new Azimuth(
                new AzimuthIOHardware(
                    TurretConstants.azimuthMotorId, TurretConstants.azimuthEncoderId));
        hood = new Hood(new HoodIOHardware(TurretConstants.hoodMotorId));
        launcher = new Launcher(new LauncherIOHardware(TurretConstants.launcherMotorId));
        break;
      default:
        drive = new Drive(driveController, new DriveIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intake = new Intake(new IntakeIO() {});
        azimuth = new Azimuth(new AzimuthIO() {});
        hood = new Hood(new HoodIO() {});
        launcher = new Launcher(new LauncherIO() {});
        break;
    }
    superStructure = new SuperStructure(spindexer, hood, azimuth, launcher, intake);
    PhoenixSync.optimizeAll();

    configureBindings();
  }

  private void configureBindings() {
    driveController.leftTrigger(0.3).onTrue(superStructure.intake());
    driveController.rightTrigger(0.3).onTrue(superStructure.shoot());
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.in(Units.Meters),
        RobotConfig.bumperWidthX.in(Units.Meters),
        edu.wpi.first.math.util.Units.inchesToMeters(3.0),
        RobotState.getInstance()::getSimulatedPose,
        RobotState.getInstance()::getFieldVelocity);

    fuelSim.enableAirResistance();
    fuelSim.start();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
