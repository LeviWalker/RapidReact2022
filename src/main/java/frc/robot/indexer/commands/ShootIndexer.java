package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class ShootIndexer extends CommandBase {
    Shooter shooter;
    Indexer indexer;
    Intake intake;

    public ShootIndexer(Shooter shooter, Indexer indexer, Intake intake) {
        addRequirements(indexer);
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
    }

    @Override
    public void execute() {
        indexer.shootIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}