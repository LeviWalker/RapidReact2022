package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.*;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.drive.commands.ToggleGear;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.*;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.*;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.commands.VisionOff;

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

        new VisionOff(vision).schedule();
    }
    
    public void configureButtonBindings() {
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);
        
        new JoystickButton(driver, OIConstants.kSquare)
            .whenPressed(new ToggleGear(drivetrain));

        new JoystickButton(operator, OIConstants.kTriangle)
            .whenPressed(new SmartDashSpinUpShooter(shooter))
            .whenHeld(new ShootIndexCommand(shooter, indexer))
            .whenReleased(new StopShooter(shooter));

        /**
         * Controls:
         * 
         *   Driver:
         *      Right trigger -> drive forward
         *      Left trigger -> drive backwards
         *      Left X -> turning
         *      Square -> Toggle drive gear default should be low gear
         * 
         *   Operator:
         *      Right Y Axis -> Intake
         *          Pull towards you to deploy and intake
         *          Push away to deploy and outtake
         *      Triangle -> Shooting
         *          To use this, open Shuffleboard and set
         *          the SmartDashboard/RPM tab and
         *          SmartDashboard/Hood Extended. Then, press
         *          and hold the triangle button. Both the
         *          target flywheel rpm and the hood position
         *          should be set to whatever is on Shuffleboard.
         *          Extended Hood as true correlates to the steeper
         *          shooting angle. To find the SmartDashboard tabs
         *          on Shuffleboard, you may have to open the dialog
         *          on the left side of the application (if not
         *          already open). Once you have opened the dialog,
         *          make sure you have selected the "Sources" option.
         *          You should see two dropdowns below. Select the
         *          one named "NetworkTables". If connected to the
         *          robot, you should see more dropdowns underneath.
         *          Find the one that is labeled SmartDashboard
         *          (should be near the bottom of the NetworkTables
         *          sections) and look beneath it to find RPM and
         *          Hood Extended. Please note that these values
         *          should only take effect in the robot program
         *          after you have pressed and held the triangle
         *          button. I recommend that you start with a low
         *          RPM of about 1000 and increase/decrease from
         *          there (especially because of the new super
         *          grippy flywheel).
         */
    }
}