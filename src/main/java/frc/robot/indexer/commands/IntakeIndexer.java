package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;

public class IntakeIndexer extends CommandBase {
    Intake intake;
    Indexer indexer;

    public IntakeIndexer(Intake intake, Indexer indexer) {
        addRequirements(indexer);
        this.intake = intake;
        this.indexer = indexer;
    }

    @Override
    public void execute() {
        if (intake.isIntaking() && indexer.limitSwitchTripped()) {
            indexer.intakeIndex();
        } else {
            indexer.stopIndex();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}