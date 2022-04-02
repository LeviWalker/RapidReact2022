package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.indexer.Indexer;

public class AutoIndex extends CommandBase {
    Indexer indexer;
    public AutoIndex(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        this.indexer.intakeIndex();
    }

    @Override
    public void end(boolean interrupted) {
        this.indexer.stopIndex();
    }

    @Override
    public boolean isFinished() {
        return this.indexer.hasCargo();
    }
}
