package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Constants {

    public static final int kPDH = 40;
    public static final ModuleType kPDHType = ModuleType.kRev;
    public static final int kPH = 41;
    public static final PneumaticsModuleType kPHType = PneumaticsModuleType.REVPH;

    public static class DriveConstants {
        public static final int kLeftMasterID = 4;
        public static final int kLeftFrontID = 5;
        public static final int kLeftRearID = 6;
        public static final int kRightMasterID = 1;
        public static final int kRightFrontID = 2;
        public static final int kRightRearID = 3;
    
        public static final int kGearShifterChannel = 4;

        public static final double kRegularMaxThrottle = 0.70;
        public static final double kRegularMaxTurn = 0.60;
        public static final double kSlowMaxThrottle = 0.50;
        public static final double kSlowMaxTurn = 0.40;

        // how much faster are the drive wheels going than the motors?
        public static final double kHighGearRatio = 9.07;
        public static final double kLowGearRatio = 16.91;
        public static final double kWheelDiameterInches = (6.0 * 7.0 / 8.0);
        public static final double kInchesToMeters = 0.0254;
        public static final double kLowGearRotationsToMetersConversion = Math.PI * kWheelDiameterInches * kInchesToMeters / kLowGearRatio;
        public static final double kHighGearRotationsToMetersConversion = Math.PI * kWheelDiameterInches * kInchesToMeters / kHighGearRatio;

        public static final double kLowGearRotationsToFeetConversion = Math.PI * kWheelDiameterInches /(12 * kLowGearRatio);
        public static final double kHighGearRotationsToFeetConversion = Math.PI * kWheelDiameterInches /(12 * kHighGearRatio);
    
        // public static final double kTrackWidthMeters = 2.5;
        public static final double kTrackwidth = 22.75 / 12;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);
        public static final double kMaxAutoSpeedFPS = 6.5;
        public static final double kMaxAutoAccelerationFPS2 = 4;
        public static final double kMaxSpeedMetersPerSecond = 0; // TODO math // jvn
        public static final double kMaxAccelerationMetersPerSecondSquared = 0; // TODO math // jvn
    
        public static final double kAutoS = 0.7; // TODO sys id
        public static final double kAutoV = 1.388; // TODO sys id
        public static final double kA = 0; // TODO sys id
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kAutoP = 0.31;
    }
    
    public static final class OIConstants {
        // controller ID constants
        public static final int kDriverID = 0;
        public static final int kOperatorID = 1;

        // joystick button numbers
        public static final int kSquare = 1;
        public static final int kX = 2;
        public static final int kCircle = 3;
        public static final int kTriangle = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftTriggerButton = 7;
        public static final int kRightTriggerButton = 8;
        public static final int kShare = 9;
        public static final int kOptions = 10;

        // joystick axis numbers
        public static final int kLeftXJoystick = 0;
        public static final int kLeftYJoystick = 1;
        public static final int kRightXJoystick = 2;
        public static final int kRightYJoystick = 5;
        public static final int kLeftTrigger = 3;
        public static final int kRightTrigger = 4;
    }

    public static class IntakeConstants {
        // Motor CAN IDs
        public static final byte kIntakeMotorID = 9;

        // Solenoid Pneumatic Hub Channels
        public static final byte kIntakeDeployerForwardChannel = 2;
        public static final byte kIntakeDeployerReverseChannel = 3;

        // Acceptable speeds for intake
        public static final double kMinIntakeSpeed = 0.25;
        public static final double kMaxIntakeSpeed = 0.60;
        public static final double kDefaultIntakeSpeed = 0.50;

        // Values for intake positions
        public static final DoubleSolenoid.Value kRetracted = Value.kReverse;
        public static final DoubleSolenoid.Value kDeployed = Value.kForward;
    }

    public static class ShooterConstants {
        // Motor CAN ID
        public static final byte kFlywheelMotorID = 7;

        // Solenoid Pneumatic Hub Channels
        public static final byte kHoodForwardChannelID = 0;
        public static final byte kHoodReverseChannelID = 1;


        // RoboRio Sensor Ports
        public static final byte kShooterSwitchID = 0;
        public static final I2C.Port kColorSensorPort = I2C.Port.kOnboard;


        // Solenoid Values
        public static final DoubleSolenoid.Value kHoodRetracted = Value.kReverse;
        public static final DoubleSolenoid.Value kHoodExtended = Value.kForward;

        // Opposite Alliance Cargo Shots
        public static final boolean kOppositeColorHoodExtended = false; // TODO Find this
        public static final double kOppositeColorRPM = 800; // TODO Find this

        // Autonomous Cargo Shots
        public static final boolean kTARMACEdgeHoodExtended = false;
        public static final double kTARMACEdgeShotRPM = 3800;


        // Tranfer wheel speeds
        public static final double kDefaultTransferSpeed = 0.40; // TODO Find this

        // Tolerances on the flywheel
        public static final double kVelocityToleranceRPM = 40;
        public static final int kMinCount = 4;

        public static final boolean kAutoShotHoodExtended = true;
		public static final double kAutoShotRPM = 3650;

        public static final boolean kCloseShotHoodExtended = true;
		public static final double kCloseShotRPM = 3500;

        public static final boolean kSafeZoneHoodExtended = true;
		public static final double kSafeZoneRPM = 4300;
    }

    public static class ClimbConstants {

        // Motor CAN IDs
        public static final byte kLeftClimbMotorID = 11;
        public static final byte kRightClimbMotorID = 12;


        // Servo PWM IDs
        public static final byte kLeftClimbServoID = 0;
        public static final byte kRightClimbServoID = 1;
        
        // Limit Switch DIO IDs
        public static final int kLeftClimbLowLimitSwitchID = 1;
        public static final int kRightClimbLowLimitSwitchID = 2;

        // Servo Angle Values
        public static final short kLeftServoNeutralAngle = 90; // 15;
        public static final short kRightServoNeutralAngle = 90;
        public static final short kLeftServoClimbAngle = 50; // -5;
        public static final short kRightServoClimbAngle = 130;
        
        // Estimated Climb Parameters
        public static final float kL2ClimbDownHallSensorValue = 0.0f;
        public static final float kL2ClimbUpHallSensorValue = 229.77f;
        public static final float kClimbSpeedDeadband = 0.07f;
        public static final float kErrorAtDeadband = 3f;
        public static final float kAbsoluteMaxSpeed = 0.70f;
        // speed = kP * error => kP = speed / error
        public static final float kP = kClimbSpeedDeadband / kErrorAtDeadband;
        public static final double kPositionTolerance = 4;
        public static final int kRatchetSolenoidChannel = 5;
    }

    public static class VisionConstants {

        // Vision Client information
        public static final short kVisionPort = 5800;
        public static final String kVisionIPAddress = "wpilibpi"; // "10.1.22.53";

        // Data value indexes
        public static final byte kDistanceIndex = 0;
        public static final byte kAngleIndex = 1;
    }
}