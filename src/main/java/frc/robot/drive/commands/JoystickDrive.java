package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.drive.Drivetrain;

public class JoystickDrive extends CommandBase {
    Drivetrain drivetrain;
    Joystick driver;

    public JoystickDrive(Drivetrain drivetrain, Joystick driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // GTA Control
        drivetrain.curveDrive(
            driver.getRawAxis(OIConstants.kRightTrigger) - driver.getRawAxis(OIConstants.kLeftTrigger),
            driver.getRawAxis(OIConstants.kLeftXJoystick)
        );

        // Sticks Control
        // drivetrain.curveDrive(
        //     driver.getRawAxis(OIConstants.kLeftYJoystick),
        //     driver.getRawAxis(OIConstants.kRightXJoystick)
        // );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
