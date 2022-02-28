package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.drive.Drivetrain;

public class ToggleGear extends CommandBase {
    Drivetrain drivetrain;

    public ToggleGear(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.toggleGear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
