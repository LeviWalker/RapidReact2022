package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class ShootIndexer extends CommandBase {
    Shooter shooter;
    Indexer indexer;

    public ShootIndexer(Shooter shooter, Indexer indexer) {
        addRequirements(indexer);
        this.shooter = shooter;
        this.indexer = indexer;
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