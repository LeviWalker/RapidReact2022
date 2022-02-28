package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.SpinUpShooter;
import frc.robot.shooter.commands.StopShooter;
import frc.robot.util.oi.NKPS4Controller;

public class RobotContainer {
    Shooter shooter;
    Joystick driver, operator;
    JoystickButton driverCircle, driverTriangle, driverSquare, driverX;

    public RobotContainer() {
        shooter = new Shooter();
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