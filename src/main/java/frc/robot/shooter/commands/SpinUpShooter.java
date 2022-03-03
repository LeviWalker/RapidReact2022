package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;

public class SpinUpShooter extends InstantCommand {
    public SpinUpShooter(Shooter shooter, double rpm, boolean hoodExtended) {
        super(() -> {
            shooter.setFlywheelRPM(rpm);
            shooter.setHoodExtended(hoodExtended);
        }, shooter);
        addRequirements(shooter);
    }
}