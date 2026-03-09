package frc.robot;

import java.util.HashMap;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.RobotConfig.*;

public class AutoBuilder {
	private static HashMap<String, Trajectory<SwerveSample>> loadTrajectoryMap(String... trajNames) {
		HashMap<String, Trajectory<SwerveSample>> trajs = new HashMap<>();
		for (String name : trajNames) {
			trajs.put(name, (Trajectory<SwerveSample>)Choreo.loadTrajectory(name).get());
		}
		return trajs;
	}

	public static Command testAuto(Drive drive, SuperStructure superStructure) {
		var trajMap = loadTrajectoryMap("FuelSwipe", "TrenchToNeutral", "ExitSwipe");

		return Commands.sequence(
			superStructure.setTarget(TurretTarget.NEAREST_TAG),
			drive.followChoreoTrajectoryCommand(trajMap.get("TrenchToNeutral")),
			superStructure.intake(),
			drive.followChoreoTrajectoryCommand(trajMap.get("FuelSwipe")),
			superStructure.idle(),
			drive.followChoreoTrajectoryCommand(trajMap.get("ExitSwipe")),
			superStructure.setTarget(TurretTarget.HUB),
			superStructure.shoot()
		);
	}
}
