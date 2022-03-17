package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

/**
 * Runs the left side of the climb down until it hits the limit switch
 */
public class ResetLeft extends CommandBase {
    private Climber climber;

    public ResetLeft(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setLeftClimbMotor(-0.20); // Let's let the climb reset a little faster
    }

    @Override
    public void execute() {
        // make sure that we are forcing the climber to stop at the limit switch
        if (climber.isLeftAtBottom()) climber.setLeftClimbMotor(0);
    }

    @Override
    public void end(boolean interupted) {
        // once again, setting motor to zero
        climber.setLeftClimbMotor(0);
        // reset the climb encoder
        climber.resetLeftEncoder();
    }

    @Override
    public boolean isFinished() {
        // limit switch should end the command
        return climber.isLeftAtBottom();
    }
}
