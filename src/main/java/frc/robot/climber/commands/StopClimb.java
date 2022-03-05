package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.climber.Climber;

public class StopClimb extends InstantCommand {

    Climber climber;

    public StopClimb(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        climber.setClimbMotors(0);
    }
}
