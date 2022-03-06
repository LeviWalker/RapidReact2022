package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.Drivetrain;

public class HighGear extends InstantCommand {
    public HighGear(Drivetrain drivetrain) {
        super(()-> drivetrain.setHighGear(true));
    }
}
