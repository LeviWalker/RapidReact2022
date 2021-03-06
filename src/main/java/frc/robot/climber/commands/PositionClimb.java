package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.climber.Climber;

public class PositionClimb extends ParallelCommandGroup {
    public PositionClimb(Climber climber, double targetPosition) {
        super(
            new PositionLeftClimb(climber, targetPosition),
            new PositionRightClimb(climber, targetPosition)
        );
    }
}
