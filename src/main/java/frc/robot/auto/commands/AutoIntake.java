package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.intake.Intake;

public class AutoIntake extends CommandBase {

    Intake intake;

    public AutoIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deploy();
    }

    @Override
    public void execute() {
        intake.setIntake(IntakeConstants.kMaxIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(0);
        intake.retract();
    }
}
