package frc.robot.climber.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class L2ClimbDown extends PositionClimb {
    Climber climber;
    public L2ClimbDown(Climber climber) {
        super(climber, ClimbConstants.kL2ClimbDownHallSensorValue);
        this.climber = climber;
    }

    @Override
    public boolean isFinished() {
        return climber.isLeftAtBottom() && climber.isRightAtBottom() && super.isFinished();
    }
}
