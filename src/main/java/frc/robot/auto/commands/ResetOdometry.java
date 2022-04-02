package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.Drivetrain;

public class ResetOdometry extends InstantCommand {
    public ResetOdometry(Drivetrain drivetrain, Pose2d pose) {
        super(() -> drivetrain.resetOdometry(pose));
    }
}
