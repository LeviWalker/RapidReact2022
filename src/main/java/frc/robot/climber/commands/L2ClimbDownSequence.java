package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.climber.Climber;

public class L2ClimbDownSequence extends SequentialCommandGroup {
    public L2ClimbDownSequence(Climber climber, Joystick operator) {
        super(
            new L2ClimbDown(climber), 
            new LockClimb(climber)
        );
    }
}
