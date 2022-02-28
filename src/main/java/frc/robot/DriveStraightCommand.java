package frc.robot;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveStraightCommand extends WaitCommand {

    DrivetrainSubsystem drive;

    public DriveStraightCommand(DrivetrainSubsystem drive) {
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
