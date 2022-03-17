package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

/**
 * Runs the right climb hook down until it hits the limit switch
 */
public class ResetRight extends CommandBase {
    private Climber climber;

    public ResetRight(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRightClimbMotor(-0.20); // Let's let the climb reset a little faster
    }

    @Override
    public void execute() {
        // make sure that we are forcing the climber to stop at the limit switch
        if (climber.isRightAtBottom()) climber.setRightClimbMotor(0);
    }

    @Override
    public void end(boolean interupted) {
        // once again, setting motor to zero
        climber.setRightClimbMotor(0);
        // reset the climb encoder
        climber.resetRightEncoder();
    }

    @Override
    public boolean isFinished() {
        // limit switch should end the command
        return climber.isRightAtBottom();
    }
}
