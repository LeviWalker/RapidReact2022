package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

public class ResetLeft extends CommandBase {
    private Climber climber;

    public ResetLeft(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setLeftClimbMotor(-0.10);
    }

    @Override
    public void execute() {
        if (climber.isLeftAtBottom()) climber.setLeftClimbMotor(0);
    }

    @Override
    public void end(boolean interupted) {
        climber.setLeftClimbMotor(0);
        climber.resetLeftEncoder();
    }

    @Override
    public boolean isFinished() {
        return climber.isLeftAtBottom();
    }
}
