package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.climber.Climber;

/**
 * Unlocks the climb ratchet so that we move the climb hooks
 */
public class UnlockClimb extends InstantCommand {
    public UnlockClimb(Climber climber) {
        super(climber::unlockClimb, climber);
    }
}
