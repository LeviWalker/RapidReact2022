package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;

public class StopShooter extends CommandBase {
    Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}