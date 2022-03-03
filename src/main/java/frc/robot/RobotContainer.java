package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.L2ClimbDown;
import frc.robot.climber.commands.L2ClimbSequence;
import frc.robot.climber.commands.L2ClimbUp;
import frc.robot.climber.commands.ResetClimbSequence;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.IntakeIndexCommand;
import frc.robot.indexer.commands.ShootIndexCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.SmartDashSpinUpShooter;
import frc.robot.shooter.commands.SpinUpShooter;
import frc.robot.shooter.commands.StopShooter;
import frc.robot.vision.VisionClient;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.VisionClient.VisionClientException;
import frc.robot.vision.commands.VisionOff;
import frc.robot.vision.commands.VisionOn;

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
    JoystickButton driverCircle, driverTriangle, driverSquare, driverX;

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

        new VisionOff(vision).schedule();
    }
    
    public void configureButtonBindings() {
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);

        driverCircle = new JoystickButton(driver, OIConstants.kCircle);
        driverSquare = new JoystickButton(driver, OIConstants.kSquare);
        driverTriangle = new JoystickButton(driver, OIConstants.kTriangle);
        driverX = new JoystickButton(driver, OIConstants.kX);

        // new JoystickButton(driver, OIConstants.kShare)
        //     .whenPressed(new VisionOff(vision));
        // new JoystickButton(driver, Constants.OIConstants.kOptions)
        //     .whenPressed(new VisionOn(vision));

        driverCircle
            .whenPressed(new SpinUpShooter(shooter, 500, false))
            .whenHeld(new ShootIndexCommand(shooter, indexer))
            .whenReleased(new StopShooter(shooter));

        driverSquare
            .whenPressed(new SpinUpShooter(shooter, 4269, true))
            .whenHeld(new ShootIndexCommand(shooter, indexer))
            .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kTriangle)
            .whenPressed(new SmartDashSpinUpShooter(shooter))
            .whenHeld(new ShootIndexCommand(shooter, indexer))
            .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kSquare)
            .whenPressed(
                new ParallelCommandGroup(
                    new PrintCommand("running climb reset"),
                    new ResetClimbSequence(climber)
                )
            );

        new JoystickButton(operator, OIConstants.kShare)
            .whenPressed(new L2ClimbSequence(climber, operator));
    }
}