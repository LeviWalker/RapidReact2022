package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.climber.Climber;

public class L2ClimbSequence extends SequentialCommandGroup {
    public L2ClimbSequence(Climber climber, Joystick operator) {
        super(
            new UnlockClimb(climber),
            new ParallelCommandGroup(
                new L2ClimbUp(climber),
                new WaitUntilCommand(() -> operator.getRawButtonPressed(OIConstants.kShare))
            ),
            new L2ClimbDown(climber), 
            new LockClimb(climber)
        );
    }
}
