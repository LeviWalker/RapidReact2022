package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.NKTalonFX;

public class DrivetrainSubsystem extends SubsystemBase {

  private NKTalonFX leftMaster, leftFront, leftRear;
  private NKTalonFX rightMaster, rightFront, rightRear;

  private SupplyCurrentLimitConfiguration currentLimitConfiguration;

  private final DifferentialDrive m_drive;

  private AHRS imu;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {

    leftMaster = new NKTalonFX(DriveConstants.kLeftMotor1Port);
    rightMaster = new NKTalonFX(DriveConstants.kRightMotor1Port);
    leftFront = new NKTalonFX(DriveConstants.kLeftMotor2Port);
    rightFront = new NKTalonFX(DriveConstants.kRightMotor2Port);
    leftRear = new NKTalonFX(DriveConstants.kLeftMotor3Port);
    rightRear = new NKTalonFX(DriveConstants.kRightMotor3Port);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    leftMaster.setInverted(true);
    leftFront.setInverted(true);
    leftRear.setInverted(true);

    leftFront.follow(leftMaster);
    leftRear.follow(leftMaster);
    rightFront.follow(rightMaster);
    rightRear.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftFront.setNeutralMode(NeutralMode.Coast);
    leftRear.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);
    rightRear.setNeutralMode(NeutralMode.Coast);

    currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 49, 38, 0);

    leftMaster.configSupplyCurrentLimit(currentLimitConfiguration);
    leftFront.configSupplyCurrentLimit(currentLimitConfiguration);
    leftRear.configSupplyCurrentLimit(currentLimitConfiguration);
    rightMaster.configSupplyCurrentLimit(currentLimitConfiguration);
    rightFront.configSupplyCurrentLimit(currentLimitConfiguration);
    rightRear.configSupplyCurrentLimit(currentLimitConfiguration);

    m_drive = new DifferentialDrive(leftMaster, rightMaster);
    m_drive.setSafetyEnabled(false);

    imu = new AHRS(SPI.Port.kMXP);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(imu.getRotation2d());

    SmartDashboard.putNumber("kDriveStraightP", kDriveStraightP);
  }

  public DifferentialDrive getDifferentialDrive() {
    return m_drive;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public AHRS getIMU() {
    return imu;
  }

  @Override
  public void periodic() {

    final double p = SmartDashboard.getNumber("kDriveStraightP", kDriveStraightP);

    if (kDriveStraightP != p) {
      kDriveStraightP = p;
      SmartDashboard.putNumber("kDriveStraightP", kDriveStraightP);
    }

    // Update the odometry in the periodic block
    updateOdometry();
  }

  public void updateOdometry() {
    m_odometry.update(
        imu.getRotation2d(),
        this.getLeftEncoderDistanceMeters(),
        this.getRightEncoderDistanceMeters()
    );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        this.getLeftEncoderVelocityMetersPerSecond(),
        this.getRightEncoderVelocityMetersPerSecond()
    );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, imu.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void curveDrive(double fwd, double rot) {
    m_drive.curvatureDrive(fwd, rot, true);
  }

  public void tankDrive(double l, double r) {
    m_drive.tankDrive(l, r, false);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    m_drive.feed();
  }

  private double kDriveStraightP = SmartDashboard.getNumber("kDriveStraightP", 0.001);

  public void driveStraightVolts(double volts) {

    // positive when left is greater => turning to the right
    double error = this.getLeftEncoderDistanceMeters() - this.getRightEncoderDistanceMeters();
    double correction = kDriveStraightP * error;

    leftMaster.setVoltage(volts + ((volts > 0) ? correction : -correction));
    rightMaster.setVoltage(volts);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
      this.leftMaster.resetPosition();
      this.rightMaster.resetPosition();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (this.getLeftEncoderDistanceMeters() + this.getRightEncoderDistanceMeters()) / 2;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return imu.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -imu.getRate();
  }

  public double getLeftEncoderDistanceMeters() {
    return -this.leftMaster.getPositionRotations() * DriveConstants.kRotationsToMetersConversion;
  }

  public double getRightEncoderDistanceMeters() {
    return this.rightMaster.getPositionRotations() * DriveConstants.kRotationsToMetersConversion;
  }

  public double getLeftEncoderVelocityMetersPerSecond() {
      return -this.leftMaster.getVelocityRPM() * DriveConstants.kRotationsToMetersConversion / 60;
  }

  public double getRightEncoderVelocityMetersPerSecond() {
    return this.rightMaster.getVelocityRPM() * DriveConstants.kRotationsToMetersConversion / 60;
  }

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

    public static final double kTrackWidthMeters = 0; // TODO Find out with SysId?
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10
    );

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );
    // Add kinematics to ensure max speed is actually obeyed
    config.setKinematics(DriveConstants.kDriveKinematics);
    // Apply the voltage constraint
    config.addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        this::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        this::tankDriveVolts,
        this
    );

    // Reset odometry to the starting pose of the trajectory.
    this.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }
}