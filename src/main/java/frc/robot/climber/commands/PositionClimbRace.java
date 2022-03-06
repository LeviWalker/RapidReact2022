package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.climber.Climber;

public class PositionClimbRace extends ParallelRaceGroup {
    PositionClimbRace(Climber climber, double targetPosition) {
        super(
            new PositionLeftClimb(climber, targetPosition),
            new PositionRightClimb(climber, targetPosition)
        );
    }
}
