package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.AutoSelectorSwitch;
import frc.robot.auto.commands.AlternateTwoCargoAuto;
import frc.robot.auto.commands.FourCargoAuto;
import frc.robot.auto.commands.ShortTwoCargoAuto;
import frc.robot.auto.commands.TimedAutoSequence;
import frc.robot.auto.commands.TimedDrive;
import frc.robot.auto.commands.TwoCargoAuto;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.*;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.DrivetrainCharacterization;
import frc.robot.drive.commands.HighGear;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.drive.commands.LowGear;
import frc.robot.drive.commands.RegularDrive;
import frc.robot.drive.commands.SlowDrive;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.*;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.commands.*;

public class RobotContainer {
    AutoSelectorSwitch autoSwitch;
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    Indexer indexer;
    Climber climber;
    PowerDistribution pdh;
    Compressor compressor;
    VisionSystem vision;
    Joystick driver, operator;

    CommandBase autoCommand;

    private ShortTwoCargoAuto shortAuto;
    private AlternateTwoCargoAuto altAuto;
    private FourCargoAuto four;

    public RobotContainer() {
        // initialize subsystems
        autoSwitch = new AutoSelectorSwitch();
        drivetrain = new Drivetrain();
        shooter = new Shooter();
        intake = new Intake();
        indexer = new Indexer();
        climber = new Climber();

        pdh = new PowerDistribution(Constants.kPDH, Constants.kPDHType);
        compressor = new Compressor(Constants.kPH, Constants.kPHType);

        vision = new VisionSystem(pdh);
        
        configureButtonBindings();

        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, driver));
        intake.setDefaultCommand(new IntakeCommand(intake, indexer, operator));
        indexer.setDefaultCommand(new IntakeIndexCommand(intake, indexer));

        shortAuto = new ShortTwoCargoAuto(drivetrain, intake, indexer, shooter);
        altAuto = new AlternateTwoCargoAuto(drivetrain, intake, indexer, shooter);
        four = new FourCargoAuto(drivetrain, intake, indexer, shooter);
    }

    public CommandBase getAuto() {
        if (autoSwitch.doTwoCargoLayup()) {
            autoCommand = shortAuto;
        } else if (autoSwitch.doAltTwoCargo()) {
            autoCommand = altAuto;
        } else if (autoSwitch.doFourCargo()) {
            autoCommand = four;
        } else {
            autoCommand = new TimedDrive(drivetrain, 3, 0.6);
        }
        return autoCommand;
        // return new AlternateTwoCargoAuto(drivetrain, intake, indexer, shooter);
        // return new ShortTwoCargoAuto(drivetrain, intake, indexer, shooter);
    }

    public void initRobotCommands() {
        SequentialCommandGroup visionOffSeq = new SequentialCommandGroup(
            new WaitCommand(0.10),
            new VisionOff(vision)
        );
        visionOffSeq.schedule();
        Shuffleboard.getTab("Vision").addBoolean("is client good?", vision::isVisionClientOperational);
    }

    public void autoInit() {
        drivetrain.reset();
        drivetrain.resetOdometry(new Pose2d());
        if (vision.isVisionClientHavingProblems()) vision.initClient();
    }

    public void teleopInit() {
        drivetrain.resetOdometry(new Pose2d());
        if (vision.isVisionClientHavingProblems()) vision.initClient();
    }

    public void periodic() {
        boolean[] rawAutoResult = autoSwitch.getRaw();
        for (int i = 0; i < rawAutoResult.length; i++) {
            SmartDashboard.putBoolean("auto switch value " + (i + 1), rawAutoResult[i]);
        }
    }
    
    public void configureButtonBindings() {
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);
        
        new JoystickButton(driver, OIConstants.kSquare)
            .whenPressed(new HighGear(drivetrain))
            .whenReleased(new LowGear(drivetrain));
            
        new JoystickButton(driver, OIConstants.kX)
            .whenPressed(new VisionDriveSequence(drivetrain, vision))
            .whenReleased(new VisionOff(vision)); // making sure vision light is off

        new JoystickButton(driver, OIConstants.kTriangle)
            .whenPressed(new SlowDrive(drivetrain))
            .whenReleased(new RegularDrive(drivetrain));

        new JoystickButton(operator, OIConstants.kSquare)
            .whenPressed(new SpinUpShooter(shooter, ShooterConstants.kCloseShotRPM, ShooterConstants.kCloseShotHoodExtended))
            .whenHeld(new ShootIndexCommand(indexer, shooter))
            .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kCircle)
            .whenPressed(new SpinUpShooter(shooter, 4000, false)) //was 4300
            .whenHeld(new ShootIndexCommand(indexer, shooter))
            .whenReleased(new StopShooter(shooter));

        // new JoystickButton(operator, OIConstants.kX)
        //     .whileHeld(
        //         new ParallelCommandGroup(
        //             new SmartDashShooter(shooter, false),
        //             new ShootIndexCommand(indexer, shooter)
        //         )
        //     ).whenReleased(new StopShooter(shooter));

        // new JoystickButton(operator, OIConstants.kCircle)
        //     .whileHeld(
        //         new ParallelCommandGroup(
        //             new SmartDashShooter(shooter, true),
        //             new ShootIndexCommand(indexer, shooter)
        //         )
        //     ).whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kTriangle)
            .whileHeld(new VisionShootSequence(drivetrain, indexer, shooter, vision))
            .whenReleased(new VisionOff(vision)); // making sure vision light ring is off


            // TODO Change close to 3500

        // auto shot => 3650
        // safe shot => 4300
        // min vision shot => 3900
        // max vision shot => 5200

        new POVButton(operator, 0).whenPressed(new ResetClimbSequence(climber));

        // new POVButton(operator, 180).whenPressed(new VisionOn(vision)).whenReleased(new VisionOff(vision));

        new JoystickButton(operator, OIConstants.kShare)
            .whenPressed(new L2ClimbUp(climber));
        new JoystickButton(operator, OIConstants.kOptions)
            .whenPressed(new L2ClimbDown(climber));
    }
}