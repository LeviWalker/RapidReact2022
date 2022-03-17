package frc.robot.drive;

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
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.control.NKSolenoid;
import frc.robot.util.control.NKTalonFX;

public class Drivetrain extends SubsystemBase {

  private NKSolenoid gear;
  private NKTalonFX leftMaster, leftFront, leftRear;
  private NKTalonFX rightMaster, rightFront, rightRear;

  private SupplyCurrentLimitConfiguration currentLimitConfiguration;

  private final DifferentialDrive drive;

  private AHRS imu;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  private PIDController leftPID, rightPID;
  private SimpleMotorFeedforward leftFF, rightFF;

  private double maxThrottle = DriveConstants.kRegularMaxThrottle,
                 maxTurn = DriveConstants.kRegularMaxTurn;

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    gear = new NKSolenoid(Constants.kPH, Constants.kPHType, DriveConstants.kGearShifterChannel);

    leftMaster = new NKTalonFX(DriveConstants.kLeftMasterID);
    rightMaster = new NKTalonFX(DriveConstants.kRightMasterID);
    leftFront = new NKTalonFX(DriveConstants.kLeftFrontID);
    rightFront = new NKTalonFX(DriveConstants.kRightFrontID);
    leftRear = new NKTalonFX(DriveConstants.kLeftRearID);
    rightRear = new NKTalonFX(DriveConstants.kRightRearID);

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

    currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 40, 0);

    leftMaster.configSupplyCurrentLimit(currentLimitConfiguration);
    leftFront.configSupplyCurrentLimit(currentLimitConfiguration);
    leftRear.configSupplyCurrentLimit(currentLimitConfiguration);
    rightMaster.configSupplyCurrentLimit(currentLimitConfiguration);
    rightFront.configSupplyCurrentLimit(currentLimitConfiguration);
    rightRear.configSupplyCurrentLimit(currentLimitConfiguration);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);

    imu = new AHRS(SPI.Port.kMXP);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(imu.getRotation2d());

    leftPID = new PIDController(0, 0, 0);
    rightPID = new PIDController(0, 0, 0);

    leftFF = new SimpleMotorFeedforward(0, 0, 0);
    rightFF = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void setHighGear(boolean highGear) {
    gear.set(highGear);
  }

  public void toggleGear() {
    gear.set(!gear.get());
  }

  public DifferentialDrive getDifferentialDrive() {
    return drive;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public AHRS getIMU() {
    return imu;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    // SmartDashboard.putNumber("left meters", getLeftEncoderDistanceMeters());
    // SmartDashboard.putNumber("right meters", getRightEncoderDistanceMeters());

    // SmartDashboard.putNumber("left m/s", getLeftEncoderVelocityMetersPerSecond());
    // SmartDashboard.putNumber("right m/s", getRightEncoderVelocityMetersPerSecond());
  }

  public void updateOdometry() {
    odometry.update(
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
    return odometry.getPoseMeters();
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
    odometry.resetPosition(pose, imu.getRotation2d());
  }

  public void setMaxSpeeds(double throttle, double turn) {
    maxThrottle = throttle;
    maxTurn = turn;
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void autoPercentArcadeDrive(double throttle, double turn) {
    drive.arcadeDrive(throttle, turn);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double throttle, double turn) {
    drive.arcadeDrive(maxThrottle * throttle, maxTurn * turn);
  }

  public void curveDrive(double throttle, double turn) {
    drive.curvatureDrive(maxThrottle * throttle, maxTurn * turn, true);
  }

  public void tankDrive(double l, double r) {
    drive.tankDrive(l, r, false);
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
    // drive.feed();
  }

  private double kDriveStraightP = 0.001; // SmartDashboard.getNumber("kDriveStraightP", 0.001);

  public void driveStraightVolts(double volts) {

    // positive when left is greater => turning to the right
    double error = this.getLeftEncoderDistanceMeters() - this.getRightEncoderDistanceMeters();
    double correction = kDriveStraightP * error;

    leftMaster.setVoltage(volts + ((volts > 0) ? correction : -correction));
    rightMaster.setVoltage(volts);
  }

  public void tankDriveMetersPerSecond(double left, double right) {
    // TODO Implement characterization
    leftMaster.setVoltage(leftFF.calculate(left));
    rightMaster.setVoltage(rightFF.calculate(right));
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
    drive.setMaxOutput(maxOutput);
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
      DriveConstants.kDriveKinematics,
      this::tankDriveMetersPerSecond, // in case the ff constants differ
      this
    );

    // Reset odometry to the starting pose of the trajectory.
    this.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }
}