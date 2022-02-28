package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.SpinUpShooter;
import frc.robot.shooter.commands.StopShooter;
import frc.robot.util.vision.VisionClient;
import frc.robot.util.vision.VisionClient.VisionClientException;

public class RobotContainer {
    Drivetrain drivetrain;
    Shooter shooter;
    Intake intake;
    Compressor compressor;
    VisionClient vision;
    Joystick driver, operator;
    JoystickButton driverCircle, driverTriangle, driverSquare, driverX;

    public RobotContainer() {

        drivetrain = new Drivetrain();
        shooter = new Shooter();
        intake = new Intake();

        compressor = new Compressor(41, PneumaticsModuleType.REVPH);

        try {
            vision = new VisionClient(VisionConstants.kVisionIPAddress, VisionConstants.kVisionPort);
            Shuffleboard.getTab("Vision").addNumber("Distance", vision::getDistance);
            Shuffleboard.getTab("Vision").addNumber("Angle", vision::getAngle);
        } catch (VisionClientException e) {
            e.printStackTrace();
        }
        
        configureButtonBindings();

        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, driver));
    }

    public void teleopInit() {
        new IntakeCommand(intake, operator).schedule();
    }

    public void configureButtonBindings() {
        driver = new Joystick(0);
        operator = new Joystick(1);

        driverCircle = new JoystickButton(driver, OIConstants.circlePS4);
        driverSquare = new JoystickButton(driver, OIConstants.squarePS4);
        driverTriangle = new JoystickButton(driver, OIConstants.trianglePS4);
        driverX = new JoystickButton(driver, OIConstants.xPS4);

        driverCircle.whenPressed(new SpinUpShooter(shooter, 500));
        driverCircle.whenReleased(new StopShooter(shooter));
    }
}