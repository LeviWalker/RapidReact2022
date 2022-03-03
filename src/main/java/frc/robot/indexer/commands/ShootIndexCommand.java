package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.indexer.Indexer;
import frc.robot.shooter.Shooter;

public class ShootIndexCommand extends CommandBase {
    Shooter shooter;
    Indexer indexer;

    public ShootIndexCommand(Shooter shooter, Indexer indexer) {
        addRequirements(indexer);
        this.shooter = shooter;
        this.indexer = indexer;
    }

    @Override
    public void execute() {
        if (shooter.isAtTarget()) indexer.shootIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}