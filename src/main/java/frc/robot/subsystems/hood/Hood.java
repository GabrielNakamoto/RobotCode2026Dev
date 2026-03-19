package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final Supplier<Angle> azimuthAngle;

  public Hood(HoodIO io, Supplier<Angle> azimuthAngle) {
    this.io = io;
    this.azimuthAngle = azimuthAngle;
  }

  public Angle getAngle() {
    return inputs.position;
  }

  public void setAngle(Angle angle) {
    Logger.recordOutput("Hood/setpoint", angle);
    this.io.setAngle(angle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput(
        "Hood/HoodPose",
        new Pose3d(
                Inches.of(-6),
                Inches.of(-6),
                Inches.of(18.66694637),
                new Rotation3d(Rotations.zero(), getAngle(), azimuthAngle.get()))
            .transformBy(
                new Transform3d(
                    Inches.of(3.77617094), Inches.zero(), Inches.zero(), Rotation3d.kZero)));
  }
}
