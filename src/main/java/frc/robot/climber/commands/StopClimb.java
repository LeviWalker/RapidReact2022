package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

public class StopClimb extends CommandBase {

    Climber climber;

    public StopClimb(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setClimbMotors(0);
    }

    public boolean isFinished() {
        return true;
    }
}
