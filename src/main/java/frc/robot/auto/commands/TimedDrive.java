package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.Drivetrain;

public class TimedDrive extends WaitCommand {

    private Drivetrain drivetrain;
    private double motorPower;

    public TimedDrive(Drivetrain drivetrain, double seconds, double motorPower) {
        super(seconds);
        this.drivetrain = drivetrain;
        this.motorPower = motorPower;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.autoPercentArcadeDrive(motorPower, 0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.autoPercentArcadeDrive(0, 0);
    }
    
}
