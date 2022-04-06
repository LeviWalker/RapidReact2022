package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.AutoSelectorSwitch;
import frc.robot.auto.commands.*;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.*;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.*;
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
    private FourCargoAuto fourAuto;

    private int switchPosition;

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
        fourAuto = new FourCargoAuto(drivetrain, intake, indexer, shooter);
    }

    public CommandBase getAuto() {
        if (switchPosition == 1) {
            autoCommand = shortAuto;
        } else if (switchPosition == 2) {
            autoCommand = altAuto;
        } else if (switchPosition == 3) {
            autoCommand = fourAuto;
        } else {
            autoCommand = new TimedDrive(drivetrain, 3, 0.6);
        }
        return autoCommand;
    }

    public void initRobotCommands() {
        SequentialCommandGroup visionOffSeq = new SequentialCommandGroup(
            new WaitCommand(0.10),
            new VisionOff(vision)
        );
        visionOffSeq.schedule();
    }

    public void autoInit() {
        drivetrain.reset();
        drivetrain.resetOdometry(new Pose2d());
        if (vision.isVisionClientHavingProblems()) vision.initClient();
        SmartDashboard.putNumber("Auto Switch Position", switchPosition);
    }

    public void teleopInit() {
        drivetrain.resetOdometry(new Pose2d());
        if (vision.isVisionClientHavingProblems()) vision.initClient();
    }

    public void disabledPeriodic() {
        switchPosition = autoSwitch.getPosition();
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