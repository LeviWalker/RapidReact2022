package frc.robot.auto.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.TrajectoryCenter;
import frc.robot.drive.Drivetrain;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.AutoIndex;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.DeployIntake;
import frc.robot.intake.commands.RetractIntake;
import frc.robot.intake.commands.RunIntake;
import frc.robot.shooter.Shooter;

import static frc.robot.auto.TrajectoryCenter.makeWaypoint;

public class ShortTwoCargoAuto extends SequentialCommandGroup {
    private static final double kTwoCargoShotRPM = 0;
    private static final boolean kTwoCargoShotHoodExtended = false;

    public ShortTwoCargoAuto(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter) {
        TrajectoryConfig config = TrajectoryCenter.getTrajectoryConfiguration();
        ArrayList<Translation2d> empty = new ArrayList<>();
        config.addConstraint(
            new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 6.5)
        );

        Pose2d startingPoint = makeWaypoint(0, 0, 0);
        Pose2d secondCargoPoint = makeWaypoint(4, 0, 0);
        Pose2d closeShotPosition = makeWaypoint(-1, 0, 0);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        //         startingPoint,
        //         empty,
        //         startingPoint,
        //         config
        //     );

        Trajectory goGetSecondCargo = TrajectoryGenerator.generateTrajectory(
            startingPoint,
            empty,
            secondCargoPoint,
            config
        );

        config.setReversed(true);

        Trajectory comeBackToShoot = TrajectoryGenerator.generateTrajectory(
            secondCargoPoint,
            empty,
            closeShotPosition,
            config
        );


        addCommands(
            new ResetOdometry(drivetrain, startingPoint),
            new DeployIntake(intake),
            new ParallelDeadlineGroup(
                drivetrain.generateRamseteCommand(goGetSecondCargo),
                new RunIntake(intake),
                new AutoIndex(indexer)),
            new RetractIntake(intake),
            drivetrain.generateRamseteCommand(comeBackToShoot).alongWith(new RunIntake(intake)).withTimeout(4),
            new ParallelRaceGroup(
                new TimedShoot(shooter, ShooterConstants.kCloseShotRPM, ShooterConstants.kCloseShotHoodExtended, 2),
                new ShootIndexCommand(indexer, shooter),
                new RunIntake(intake)
            )
        );
    }
}
