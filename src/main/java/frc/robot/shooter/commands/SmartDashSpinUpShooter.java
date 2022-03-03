package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;

public class SmartDashSpinUpShooter extends InstantCommand {
    public SmartDashSpinUpShooter(Shooter shooter) {
        super(() -> new SpinUpShooter(
                shooter,
                SmartDashboard.getNumber("RPM", shooter.getFlywheelRPM()),
                SmartDashboard.getBoolean("Hood Extended", shooter.getHoodExtended())
            ).initialize()
        );
        SmartDashboard.putNumber("RPM", shooter.getFlywheelRPM());
        SmartDashboard.putBoolean("Hood Extended", shooter.getHoodExtended());
    }
}
