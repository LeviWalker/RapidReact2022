package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;

public class SmartDashShooter extends CommandBase {
    Shooter shooter;
    double rpm;
    boolean hoodExtended;
    public SmartDashShooter(Shooter shooter, boolean hood) {
        this.shooter = shooter;
        this.hoodExtended = hood;
        // SmartDashboard.putNumber("RPM", shooter.getFlywheelRPM());
        // SmartDashboard.putBoolean("Hood Extended", shooter.getHoodExtended());
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.rpm = shooter.getFlywheelRPM();
        shooter.setHoodExtended(hoodExtended);
    }

    @Override
    public void execute() {
        // double newRPM = SmartDashboard.getNumber("RPM", shooter.getFlywheelRPM());
        // boolean newHoodExtended = SmartDashboard.putBoolean("Hood Extended", false);

        // if (rpm != newRPM) {
        //     shooter.setFlywheelRPM(newRPM);
        //     this.rpm = newRPM;
        // }

        // if (hoodExtended != newHoodExtended) {
        //     shooter.setHoodExtended(newHoodExtended);
        //     this.hoodExtended = newHoodExtended;
        // }

        // shooter.setHoodExtended(newHoodExtended);

        // SmartDashboard.putNumber("Measured RPM", shooter.getFlywheelRPM());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) shooter.setFlywheelRPM(0);
    }
}
