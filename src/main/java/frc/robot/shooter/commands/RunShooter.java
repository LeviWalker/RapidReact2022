package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.indexer.Indexer;
import frc.robot.shooter.Shooter;

/**
 * runs the flywheel on a peroidic basis, spinning up in initialize and stopping in end
 */
public class RunShooter extends CommandBase {

    private Shooter shooter;
    private double rpm;
    private boolean extendHood;

    public RunShooter(Shooter shooter, Indexer indexer, double rpm, boolean extendHood) {
        this.shooter = shooter;
        this.rpm = rpm;
        this.extendHood = extendHood;
    }

    public static CommandBase createSpitOut(Shooter shooter, Indexer indexer) {
        return
            new RunShooter(shooter,
                           indexer,
                           ShooterConstants.kOppositeColorRPM,
                           ShooterConstants.kOppositeColorHoodExtended
                          )
        .withInterrupt(indexer::hasCargo);
    }

    public static CommandBase createSpitOut(Shooter shooter, Indexer indexer, double timeout) {
        return
            new RunShooter(shooter,
                           indexer,
                           ShooterConstants.kOppositeColorRPM,
                           ShooterConstants.kOppositeColorHoodExtended
                          )
        .withInterrupt(indexer::hasCargo)
        .withTimeout(timeout);
    }
}
