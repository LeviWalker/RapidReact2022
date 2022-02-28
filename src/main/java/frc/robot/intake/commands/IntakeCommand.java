package frc.robot.intake.commands;

import java.lang.reflect.Executable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class IntakeCommand extends CommandBase {

    private Intake intake;
    private Joystick operator;

    public IntakeCommand(Intake intake, Joystick operator) {
        this.intake = intake;
        this.operator = operator;
    }

    @Override
    public void execute() {
        
    }
}