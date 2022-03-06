package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.Drivetrain;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class TimedAutoSequence extends SequentialCommandGroup {
    Timer timer;
    public TimedAutoSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter) {
        super(
            new ParallelRaceGroup(
                new TimedShoot(shooter, 1.5),
                new ShootIndexCommand(indexer, shooter)
            ),   // 1.5 sec
            new ParallelRaceGroup(
                new AutoIntake(intake),
                new SequentialCommandGroup(
                    new TimedDrive(drivetrain, 2.5, 0.6),                                      // 4.5 sec
                    new TimedDrive(drivetrain, 2.75, -0.6))
            ),    // 7.5 sec
            new ParallelRaceGroup(
                new TimedShoot(shooter, 1.5),
                new ShootIndexCommand(indexer, shooter)
            )                                 // 10 sec
        );
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(15) || super.isFinished();
    }
}
