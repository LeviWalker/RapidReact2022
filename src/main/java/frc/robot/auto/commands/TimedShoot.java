package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shooter.Shooter;

public class TimedShoot extends WaitCommand {
    Shooter shooter;
    public TimedShoot(Shooter shooter, double seconds) {
        super(seconds);
        this.shooter = shooter;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.shooter.setHoodExtended(ShooterConstants.kAutoShotHoodExtended);
        this.shooter.setFlywheelRPM(ShooterConstants.kAutoShotRPM);
    }

    @Override
    public void end(boolean interrupted) {
       this.shooter.stopFlywheel();
    }
}
