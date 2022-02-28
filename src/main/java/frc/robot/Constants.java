package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static class DriveConstants {
        public static final int kLeftMotor1Port = 4;
        public static final int kLeftMotor2Port = 5;
        public static final int kLeftMotor3Port = 6;
        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 2;
        public static final int kRightMotor3Port = 3;
    
        // how much faster are the drive wheels going than the motors?
        public static final double kGearRatio = 9.07; // 16.91; // low gear
        public static final double kWheelDiameterInches = 6;
        public static final double kInchesToMeters = 0.0254;
        public static final double kRotationsToMetersConversion = Math.PI * kWheelDiameterInches * kInchesToMeters / kGearRatio;
    
        public static final double kTrackWidthMeters = 1.0; // TODO Find out with SysId?
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 0; // TODO math // jvn
        public static final double kMaxAccelerationMetersPerSecondSquared = 0; // TODO math // jvn
    
        public static final double ksVolts = 0; // TODO sys id
        public static final double kvVoltSecondsPerMeter = 0; // TODO sys id
        public static final double kaVoltSecondsSquaredPerMeter = 0; // TODO sys id
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kPDriveVel = 0; // TODO sys id
          
      }
    
    public static final class OIConstants {

        public static final int squarePS4 = 1;
        public static final int circlePS4 = 3;
        public static final int trianglePS4 = 4;
        public static final int xPS4 = 2;

        public static final int leftXPS4 = 0;
        public static final int leftYPS4 = 1;
        public static final int rightXPS4 = 2;
        public static final int rightYPS4 = 5;
        public static final int leftTriggerPS4 = 3;
        public static final int rightTriggerPS4 = 4;
    }

    public static class VisionConstants {

        // Vision Client information

        /**
         * TCP port the vision is hosted on
         */
        public static final short kVisionPort = 5050;

        /**
         * Identifier for the Vision Raspberry Pi
         */
        public static final String kVisionIPAddress = "pi@wpilibpi";


        // Data value indexes

        /**
         * Index of the distance value from the vision server
         */
        public static final byte kDistanceIndex = 0;

        /**
         * Index of the angle value from the vision server
         */
        public static final byte kAngleIndex = 1;
    }
}