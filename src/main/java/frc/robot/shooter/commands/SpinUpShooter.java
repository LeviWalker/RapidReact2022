package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;

public class SpinUpShooter extends CommandBase {
    Shooter shooter;
    double rpm;

    public SpinUpShooter(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFlywheelRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}