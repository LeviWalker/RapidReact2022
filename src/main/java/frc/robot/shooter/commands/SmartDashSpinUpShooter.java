package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;

public class SmartDashSpinUpShooter extends CommandBase {
    Shooter shooter;
    public SmartDashSpinUpShooter(Shooter shooter) {
        this.shooter = shooter;
        SmartDashboard.putNumber("RPM", shooter.getFlywheelRPM());
        SmartDashboard.putBoolean("Hood Extended", shooter.getHoodExtended());
    }

    @Override
    public void initialize() {
        shooter.setFlywheelRPM(SmartDashboard.getNumber("RPM", shooter.getFlywheelRPM()));
        // shooter.setHoodExtended(SmartDashboard.putBoolean("Hood Extended", shooter.getHoodExtended()));
    }
}
