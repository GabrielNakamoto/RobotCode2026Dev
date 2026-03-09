package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoBuilder {
  public interface AutoWaypoint {
    Command getCommand(Drive drive, SuperStructure superStructure);
  }
}
