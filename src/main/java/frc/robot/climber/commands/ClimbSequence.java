package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.climber.Climber;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence(Climber climber, Joystick operator) {
        super(
            new UnlockClimb(climber),
            new ClimbUp(climber) {
                @Override
                public boolean isFinished() {
                    return operator.getRawButtonPressed(OIConstants.kShare);
                }
            },
            new ClimbDown(climber), 
            new LockClimb(climber)
        );
    }
}
