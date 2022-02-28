package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;

public class JoystickDrive extends CommandBase {
    Drivetrain drivetrain;
    Joystick driver;

    public JoystickDrive(Drivetrain drivetrain, Joystick driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
    }

    @Override
    public void execute() {
        drivetrain.curveDrive(driver.getRawAxis(4), driver.getRawAxis(5));
    }
}
