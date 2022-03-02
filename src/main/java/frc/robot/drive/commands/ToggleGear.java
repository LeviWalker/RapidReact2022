package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.Drivetrain;

public class ToggleGear extends InstantCommand {
    public ToggleGear(Drivetrain drivetrain) {
        super(drivetrain::toggleGear);
    }
}
