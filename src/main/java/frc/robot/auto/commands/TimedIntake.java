package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.intake.Intake;



public class TimedIntake extends AutoIntake {

    Timer timer;
    double seconds;

    public TimedIntake(Intake intake, double seconds) {
        super(intake);
        timer = new Timer();
        this.seconds = seconds;
        new WaitCommand(seconds);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }
    
}
