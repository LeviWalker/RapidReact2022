package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.climber.Climber;

/**
 * Inches the climb motors down until they hit the limit switch
 */
public class ResetClimbSequence extends SequentialCommandGroup {
    public ResetClimbSequence(Climber climber) {
        super(
            new UnlockClimb(climber),
            new ParallelCommandGroup(
                new ResetLeft(climber), 
                new ResetRight(climber)
            ),
            new LockClimb(climber)
        );
    }
}
