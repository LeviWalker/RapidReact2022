package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climber.Climber;

public class SlowUp extends CommandBase {

    Climber climber;

    public SlowUp(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        climber.setLeftClimbMotor(0.10);
    }
}
