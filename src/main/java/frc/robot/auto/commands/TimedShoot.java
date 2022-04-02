package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shooter.Shooter;

public class TimedShoot extends WaitCommand {
    Shooter shooter;

    double rpm;
    boolean hood;

    public TimedShoot(Shooter shooter, double seconds) {
        this(
            shooter,
            ShooterConstants.kAutoShotRPM,
            ShooterConstants.kAutoShotHoodExtended,
            seconds
        );
    }

    public TimedShoot(Shooter shooter, double rpm, boolean hoodExtended, double seconds) {
        super(seconds);
        this.shooter = shooter;
        this.addRequirements(shooter);

        this.hood = hoodExtended;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.shooter.setHoodExtended(this.hood);
        this.shooter.setFlywheelRPM(this.rpm);
    }

    @Override
    public void execute() {
        super.execute();
        this.shooter.setFlywheelRPM(this.rpm);
        this.shooter.setHoodExtended(this.hood);

        SmartDashboard.putBoolean("timed shoot hood", this.hood);
        SmartDashboard.putNumber("timed shoot rpm", this.rpm);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.shooter.stopFlywheel();
    }
}
