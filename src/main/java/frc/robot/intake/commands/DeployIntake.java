package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.intake.Intake;

public class DeployIntake extends InstantCommand {
    public DeployIntake(Intake intake) {
        super(intake::deploy, intake);
    }
}
