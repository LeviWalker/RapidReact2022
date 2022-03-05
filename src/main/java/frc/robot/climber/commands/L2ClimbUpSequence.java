package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.climber.Climber;

public class L2ClimbUpSequence extends SequentialCommandGroup {
    public L2ClimbUpSequence(Climber climber, Joystick operator) {
        super(
            new UnlockClimb(climber),
            new L2ClimbUp(climber)
        );
    }
}
