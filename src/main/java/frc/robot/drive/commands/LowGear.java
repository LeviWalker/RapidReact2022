package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.Drivetrain;

public class LowGear extends InstantCommand {
    public LowGear(Drivetrain drivetrain) {
        super(()-> drivetrain.setHighGear(false));
    }
}