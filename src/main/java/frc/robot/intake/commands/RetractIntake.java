package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.intake.Intake;

public class RetractIntake extends InstantCommand {
    public RetractIntake(Intake intake) {
        super(intake::retract, intake);
    }
}
