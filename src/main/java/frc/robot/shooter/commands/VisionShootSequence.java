package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shooter.Shooter;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.commands.VisionOff;
import frc.robot.vision.commands.VisionOn;

public class VisionShootSequence extends SequentialCommandGroup {
    public VisionShootSequence(Shooter shooter, VisionSystem vision) {
        super(new VisionOn(vision),
              new WaitCommand(0.12), // wait for vision to process once light is turned on
              new VisionSpinUpShooter(shooter, vision),
              new VisionOff(vision)
        );
        this.withInterrupt(vision::isVisionClientOperational);
    }
}
