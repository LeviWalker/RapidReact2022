package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;

public class StopShooter extends InstantCommand {
    Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stopFlywheel();
    }
}