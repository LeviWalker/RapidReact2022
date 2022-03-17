package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class L2ClimbDown extends CommandBase {
    Climber climber;
    public L2ClimbDown(Climber climber) {
        // super(climber, ClimbConstants.kL2ClimbDownHallSensorValue + 10); //from PosiitonClimb
        // making sure we reach the bottom, don't worry the limit switches will stop it

        // super.withInterrupt(() -> climber.isLeftAtBottom() && climber.isRightAtBottom());
        this.climber = climber;
    }

    @Override
    public void execute() {
        if (!climber.isLeftAtBottom())
            climber.setLeftClimbMotor((climber.getLeftEncoderPosition() > 20)? -0.80 : -0.40);
        if (!climber.isRightAtBottom())
            climber.setRightClimbMotor((climber.getRightEncoderPosition() > 20)? -0.80 : -0.40);
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        climber.setClimbMotors(0);
        climber.lockClimb();
    }

    @Override
    public boolean isFinished() {
        return climber.isLeftAtBottom() && climber.isRightAtBottom();
    }
}
