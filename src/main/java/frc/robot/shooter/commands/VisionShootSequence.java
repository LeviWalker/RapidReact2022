package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.Drivetrain;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.shooter.Shooter;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.commands.VisionDrive;
import frc.robot.vision.commands.VisionOff;
import frc.robot.vision.commands.VisionOn;

public class VisionShootSequence extends SequentialCommandGroup {
    public VisionShootSequence(Drivetrain drivetrain, Indexer indexer, Shooter shooter, VisionSystem vision) {
        addCommands(
            new SequentialCommandGroup(
                new VisionOn(vision),
                new WaitCommand(0.20),
                new VisionDrive(drivetrain, vision),
                new ParallelCommandGroup(
                    new VisionShoot(shooter, vision),
                    new ShootIndexCommand(indexer, shooter)),
                new VisionOff(vision)
            )
        );
    }
}
