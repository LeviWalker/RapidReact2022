package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.drive.Drivetrain;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.AutoIndex;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.DeployIntake;
import frc.robot.intake.commands.RetractIntake;
import frc.robot.intake.commands.RunIntake;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.SpinUpShooter;

import static frc.robot.auto.TrajectoryCenter.makeWaypoint;

import java.util.ArrayList;

public class FourCargoAuto extends SequentialCommandGroup {
    Shooter shooter;
    public FourCargoAuto(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter) {
        this.shooter = shooter;
        ArrayList<Translation2d> noMiddleWaypoints = new ArrayList<>();

        TrajectoryConfig config = drivetrain.getFasterAccelTrajectoryConfiguration();
        TrajectoryConfig reverseConfig = drivetrain.getFasterAccelTrajectoryConfiguration();
        reverseConfig.setReversed(true);
        
        Pose2d startingPosition = makeWaypoint(0, 0, 0);
        // Pose2d atSecondCargo = makeWaypoint(4, 0, 0);
        Pose2d atThirdCargo = makeWaypoint(17.3, -5.06, 16);
        Pose2d atFourthCargo = makeWaypoint(16.2, -5.29, 16);
        Pose2d shootingPosition = makeWaypoint(4, 0, 6);
        Pose2d secondShootingPosition = makeWaypoint(3.5, 0, 6);

        Trajectory getSecondCargo = TrajectoryGenerator.generateTrajectory(
            startingPosition,
            noMiddleWaypoints,
            shootingPosition,
            config);

        Trajectory getThirdCargo = TrajectoryGenerator.generateTrajectory(
            shootingPosition, // start at shooting position
            noMiddleWaypoints,
            atThirdCargo, // backup for the fourth
            config);
        
        // Trajectory getFourthCargo = TrajectoryGenerator.generateTrajectory(
        //     atThirdCargo, // start at third cargo
        //     noMiddleWaypoints,
        //     atFourthCargo, // backup for the fourth
        //     reverseConfig);

        Trajectory goShootThirdAndFourthCargo = TrajectoryGenerator.generateTrajectory(
            atThirdCargo, // start at fourth cargo
            noMiddleWaypoints,
            secondShootingPosition, // end at shooting position
            reverseConfig);

        addCommands(
            new ResetOdometry(drivetrain, startingPosition),
            new DeployIntake(intake),
            new ParallelDeadlineGroup(
                drivetrain.generateRamseteCommand(getSecondCargo),
                new RunIntake(intake),
                new AutoIndex(indexer),
                new SpinUpShooter(shooter, 4000, false)),
            new RetractIntake(intake),
            new ShootIndexCommand(indexer, shooter).alongWith(new RunIntake(intake)).withTimeout(1.75),
            new DeployIntake(intake),
            new ParallelDeadlineGroup(
                drivetrain.generateRamseteCommand(getThirdCargo),
                new RunIntake(intake)
            ),
            new ParallelDeadlineGroup(new AutoIndex(indexer), new RunIntake(intake)),
            // new ParallelDeadlineGroup(
            //     drivetrain.generateRamseteCommand(getFourthCargo),
            //     new RunIntake(intake)),
            new RunIntake(intake).withTimeout(0.25),
            new RetractIntake(intake),
            new ParallelDeadlineGroup(
                drivetrain.generateRamseteCommand(goShootThirdAndFourthCargo),
                new RunIntake(intake),
                new SpinUpShooter(shooter, 4000, false)),
            new ShootIndexCommand(indexer, shooter).alongWith(new RunIntake(intake))
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.shooter.stopFlywheel();
    }
}
