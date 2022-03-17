package frc.robot.intake.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.Intake;

public class RunIntake extends CommandBase {
    private Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public static CommandBase createRunIntakeCommand(Intake intake) {
        return new RunIntake(intake);
    }
    
    public static CommandBase createRunIntakeCommand(Intake intake, double seconds) {
        return new RunIntake(intake).withTimeout(seconds);
    }
    
    public static CommandBase createRunIntakeCommand(Intake intake, BooleanSupplier endCondition) {
        return new RunIntake(intake).withInterrupt(endCondition);
    }
    
    public static CommandBase createRunIntakeCommandWithDelay(Intake intake, double delaySeconds) {
        return new RunIntake(intake).beforeStarting(new WaitCommand(delaySeconds));
    }

    public static CommandBase createRunIntakeCommandWithDelay(Intake intake, BooleanSupplier startCondition) {
        return new RunIntake(intake).beforeStarting(new WaitUntilCommand(startCondition));
    }
    
    @Override
    public void initialize() {
        // when the command starts, we are going to set the intake
        // the motor will continue to run this way
        intake.setIntake(-0.75);
    }

    @Override
    public void end(boolean interrupted) {
        // when command ends, we are turning the intake off
        intake.setIntake(0);
    }
}
