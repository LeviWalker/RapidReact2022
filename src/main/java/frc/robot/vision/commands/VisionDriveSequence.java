package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class VisionDriveSequence extends SequentialCommandGroup {
    public VisionDriveSequence(Drivetrain drivetrain, VisionSystem vision) {
        addCommands(
            new SequentialCommandGroup(
                new VisionOn(vision),
                new WaitCommand(0.20),
                new VisionDrive(drivetrain, vision),
                new VisionOff(vision)
            )
        );
    }
}
