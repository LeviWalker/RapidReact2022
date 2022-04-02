package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.robot.Constants.DriveConstants;

public class TrajectoryCenter {
    
    public Trajectory twoCargoAuto;
    public Trajectory fourCargoAutoGetCargo2;

    public static void main(String[] args) {
        System.out.println(distance(0, 0.01, 1, 1));
        System.out.println(angleBetweenDegrees(0, 0.01, 1, 0));
    }

    public TrajectoryCenter() {
        TrajectoryConfig config = new TrajectoryConfig(6.5, 4);
        ArrayList<Translation2d> empty = new ArrayList<>();
        config.addConstraint(new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 6.5));

        twoCargoAuto = TrajectoryGenerator.generateTrajectory(
                makeWaypoint(0, 0, 0),
                empty,
                makeWaypoint(5, 0, 0),
                config
            );
    }

    public static Pose2d makeWaypoint(double x, double y, double horizontalScalar, double verticalScalar) {
        return new Pose2d(x, y, new Rotation2d(horizontalScalar, verticalScalar));
    }

    public static Pose2d makeWaypoint(double x, double y, double angleDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }

    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    private static double angleBetween(double x1, double y1, double x2, double y2) {

        // vector a = <x1, y1>, vector b = <x2, y2>
        // a dot b = x1 * x2 + y1 * y2
        // theta = arccos((a dot b)/(magn(a) * magn(b))

        double a = Math.sqrt((x1 * x1) + (y1 * y1)),
               b = Math.sqrt((x2 * x2) + (y2 * y2)),
               dotProduct = (x1 * x2) + (y1 * y2);
        return Math.acos(dotProduct / (a * b));
    }

    private static double angleBetweenDegrees(double x1, double y1, double x2, double y2) {
        return angleBetween(x1, y1, x2, y2) * 180 / Math.PI;
    }

    public static TrajectoryConfig getTrajectoryConfiguration() {
        TrajectoryConfig config = new TrajectoryConfig(
          DriveConstants.kMaxAutoSpeedFPS,
          DriveConstants.kMaxAutoAccelerationFPS2
        );
        config.addConstraint(
          new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 5)
        );
        return config;
      }
    
    public static TrajectoryConfig getFasterAccelTrajectoryConfiguration() {
        TrajectoryConfig config = new TrajectoryConfig(
          DriveConstants.kMaxAutoSpeedFPS,
          DriveConstants.kMaxAutoAccelerationFPS2 + 1
        );
        config.addConstraint(
          new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 5)
        );
        return config;
      }
}
