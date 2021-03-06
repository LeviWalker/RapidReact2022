package frc.robot;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.Drivetrain;

public class DriveStraightCommand extends WaitCommand {

    Drivetrain drive;

    public DriveStraightCommand(Drivetrain drive) {
        super(3);
        this.drive = drive;
    }

    @Override
    public void execute() {
        super.execute();
        drive.driveStraightVolts(-3.0);
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.tankDriveVolts(0, 0);
    }
}
