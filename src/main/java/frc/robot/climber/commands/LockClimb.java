package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.climber.Climber;

public class LockClimb extends InstantCommand {
    public LockClimb(Climber climber) {
        super(climber::unlockClimb, climber);
    }
}
