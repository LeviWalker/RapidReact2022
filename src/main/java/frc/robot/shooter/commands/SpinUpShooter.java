package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;

public class SpinUpShooter extends CommandBase {
    Shooter shooter;
    double rpm;
    boolean hood;

    public SpinUpShooter(Shooter shooter, double rpm, boolean hoodExtended) {
        this.shooter = shooter;
        this.rpm = rpm;
        this.hood = hoodExtended;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFlywheelRPM(rpm);
        shooter.setHood(hood);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}