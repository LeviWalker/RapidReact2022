package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

public class ResetRight extends CommandBase {
    private Climber climber;

    public ResetRight(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRightClimbMotor(-0.10);
    }

    @Override
    public void execute() {
        if (climber.isRightAtBottom()) climber.setRightClimbMotor(0);
    }

    @Override
    public void end(boolean interupted) {
        climber.setRightClimbMotor(0);
        climber.resetRightEncoder();
    }

    @Override
    public boolean isFinished() {
        return climber.isRightAtBottom();
    }
}
