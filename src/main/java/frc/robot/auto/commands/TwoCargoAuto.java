package frc.robot.auto.commands;

import static frc.robot.auto.TrajectoryCenter.makeWaypoint;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.drive.Drivetrain;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.AutoIndex;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.DeployIntake;
import frc.robot.intake.commands.RetractIntake;
import frc.robot.intake.commands.RunIntake;
import frc.robot.shooter.Shooter;

public class TwoCargoAuto extends SequentialCommandGroup {
    private static final double kTwoCargoShotRPM = 0;
    private static final boolean kTwoCargoShotHoodExtended = false;

    public TwoCargoAuto(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter) {
        TrajectoryConfig config = new TrajectoryConfig(6.5, 4);
        ArrayList<Translation2d> empty = new ArrayList<>();
        config.addConstraint(
            new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 6.5)
        );

        Pose2d startingPoint = makeWaypoint(0, 0, 0);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
                startingPoint,
                empty,
                makeWaypoint(5.5, 0, 0),
                config
            );

        addCommands(
            new ResetOdometry(drivetrain, startingPoint),
            new DeployIntake(intake),
            new ParallelDeadlineGroup(
                drivetrain.generateRamseteCommand(traj),
                new RunIntake(intake),
                new AutoIndex(indexer)),
            new RunIntake(intake).withTimeout(0.7),
            new RetractIntake(intake),
            new ParallelRaceGroup(
                new TimedShoot(shooter, 3770, false, 2),
                new ShootIndexCommand(indexer, shooter),
                new RunIntake(intake)
            )
        );
    }
}
