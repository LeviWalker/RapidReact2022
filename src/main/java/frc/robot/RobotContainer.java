package frc.robot;

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
import frc.robot.Constants.OIConstants;
import frc.robot.auto.commands.TimedAutoSequence;
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
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    Indexer indexer;
    Climber climber;
    PowerDistribution pdh;
    Compressor compressor;
    VisionSystem vision;
    Joystick driver, operator;

    public RobotContainer() {
        // initialize subsystems
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

        
    }

    public CommandBase getAuto() {
        return new TimedAutoSequence(drivetrain, intake, indexer, shooter);
        // return new DrivetrainCharacterization(drivetrain);
    }

    public void initRobotCommands() {
        SequentialCommandGroup visionOffSeq = new SequentialCommandGroup(
            new WaitCommand(0.10),
            new VisionOff(vision)
        );
        visionOffSeq.schedule();
        Shuffleboard.getTab("Vision").addBoolean("is client good?", vision::isVisionClientOperational);
    }

    public void periodic() {
    }
    
    public void configureButtonBindings() {
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);
        
        new JoystickButton(driver, OIConstants.kSquare)
            .whenPressed(new HighGear(drivetrain))
            .whenReleased(new LowGear(drivetrain));

        new JoystickButton(driver, OIConstants.kX)
            .whenPressed(new SlowDrive(drivetrain))
            .whenReleased(new RegularDrive(drivetrain));

        // new JoystickButton(operator, OIConstants.kTriangle) // throws IllegalArgumentException
        //     .whenPressed(new VisionShootSequence(shooter, vision))
        //     .whenHeld(new ShootIndexCommand(indexer, shooter))
        //     .whenReleased(new StopShooter(shooter));

        // new JoystickButton(operator, OIConstants.kSquare)
        //     .whenPressed(new SpinUpShooter(shooter, 3500, true))
        //     .whenHeld(new ShootIndexCommand(indexer, shooter))
        //     .whenReleased(new StopShooter(shooter));

        // new JoystickButton(operator, OIConstants.kCircle)
        //     .whenPressed(new SpinUpShooter(shooter, 4000, false)) //was 4300
        //     .whenHeld(new ShootIndexCommand(indexer, shooter))
        //     .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kX)
            .whileHeld(
                new ParallelCommandGroup(
                    new SmartDashShooter(shooter, false),
                    new ShootIndexCommand(indexer, shooter)
                )
            ).whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kCircle)
            .whileHeld(
                new ParallelCommandGroup(
                    new SmartDashShooter(shooter, true),
                    new ShootIndexCommand(indexer, shooter)
                )
            ).whenReleased(new StopShooter(shooter));


            // TODO Change close to 3500

        // auto shot => 3650
        // safe shot => 4300
        // min vision shot => 3900
        // max vision shot => 5200

        new POVButton(operator, 0).whenPressed(new ResetClimbSequence(climber));

        new POVButton(operator, 180).whenPressed(new VisionOn(vision)).whenReleased(new VisionOff(vision));

        new JoystickButton(operator, OIConstants.kShare)
            .whenPressed(new L2ClimbUp(climber));
        new JoystickButton(operator, OIConstants.kOptions)
            .whenPressed(new L2ClimbDown(climber));
    }
}