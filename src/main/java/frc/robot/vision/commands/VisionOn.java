package frc.robot.vision.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.VisionSystem;

public class VisionOn extends InstantCommand {
    public VisionOn(VisionSystem vision) {
        super(vision::setLightOn, vision);
    }
}
