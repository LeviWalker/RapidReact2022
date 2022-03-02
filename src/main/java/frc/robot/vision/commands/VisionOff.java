package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.VisionSystem;

public class VisionOff extends InstantCommand {
    public VisionOff(VisionSystem vision) {
        super(vision::setLightOff, vision);
    }
}
