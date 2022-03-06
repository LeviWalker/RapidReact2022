package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.drive.commands.HighGear;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.drive.commands.LowGear;
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

        // new VisionOff(vision).schedule();
    }

    public CommandBase getAuto() {
        return new TimedAutoSequence(drivetrain, intake, indexer, shooter);
    }

    public void initRobotCommands() {
        SequentialCommandGroup visionOffSeq = new SequentialCommandGroup(
            new WaitCommand(0.10),
            new VisionOff(vision)
        );
        visionOffSeq.schedule();
        Shuffleboard.getTab("Vision").addBoolean("is client good?", vision::isVisionClientOperational);
        Shuffleboard.getTab("Vision").addBoolean("is code working???", visionOffSeq::isScheduled);
    }

    public void periodic() {
        if (Timer.getFPGATimestamp() == 0.20) vision.setLightOff();
    }
    
    public void configureButtonBindings() {
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);
        
        new JoystickButton(driver, OIConstants.kSquare)
            .whenPressed(new HighGear(drivetrain))
            .whenReleased(new LowGear(drivetrain));

        new JoystickButton(operator, OIConstants.kTriangle)
            .whenPressed(new VisionSpinUpShooter(shooter, vision))
            .whenHeld(new ShootIndexCommand(indexer, shooter))
            .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kSquare)
            .whenPressed(new SpinUpShooter(shooter, 3650, true))
            .whenHeld(new ShootIndexCommand(indexer, shooter))
            .whenReleased(new StopShooter(shooter));

        new JoystickButton(operator, OIConstants.kCircle)
            .whenPressed(new SpinUpShooter(shooter, 4300, false))
            .whenHeld(new ShootIndexCommand(indexer, shooter))
            .whenReleased(new StopShooter(shooter));

        // auto shot => 3650
        // safe shot => 4300
        // min vision shot => 3900
        // max vision shot => 5200

        // new JoystickButton(operator, OIConstants.kX).whenPressed(new ResetClimbSequence(climber));

        new POVButton(operator, 0).whenPressed(new ResetClimbSequence(climber));

        new JoystickButton(operator, OIConstants.kShare)
            .whenPressed(new L2ClimbUpSequence(climber, operator));
        new JoystickButton(operator, OIConstants.kOptions)
            .whenPressed(new L2ClimbDownSequence(climber, operator));
        // new JoystickButton(operator, OIConstants.kSquare).whenPressed(new StopClimb(climber));

        // new JoystickButton(operator)
        
        /**
         * Controls:
         * 
         *   Driver:
         *      Right trigger -> drive forward
         *      Left trigger -> drive backwards
         *      Left X -> turning
         *      Square -> hold for high drive gear, default should be low gear
         * 
         *   Operator:
         *      Right Y Axis -> Intake
         *          Pull towards you to deploy and intake
         *          Push away to deploy and outtake
         *      Square -> Close shot in high goal
         *      Circle -> Safe Zone in high goal
         *      POV top -> Reset Climb to limit switches
         *      Share -> Climb Up
         *      Options -> Climb Down
         * 
         * 
         *    Testing:
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