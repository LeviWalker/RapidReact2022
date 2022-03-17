package frc.robot.climber.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class L2ClimbUp extends PositionClimbRace {
    Climber climber;

    public L2ClimbUp(Climber climber) {
        super(climber, ClimbConstants.kL2ClimbUpHallSensorValue);
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.unlockClimb();
        super.initialize();
    }
}
