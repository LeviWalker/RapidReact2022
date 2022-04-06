package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;

public class DriveFPS extends CommandBase {
    private Drivetrain drivetrain;
    public DriveFPS(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // SmartDashboard.putNumber("drive fps", 0);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
