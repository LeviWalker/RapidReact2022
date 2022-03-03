package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;
import frc.robot.vision.VisionSystem;

public class VisionSpinUpShooter extends InstantCommand {
    public VisionSpinUpShooter(Shooter shooter, VisionSystem vision) {
        super(() -> {
            shooter.setFlywheelRPM(calculateRPM(vision.getDistance()));
            shooter.setHoodExtended(false); // can only use vision for far shooting
        }, shooter);
        addRequirements(shooter);
    }

    private static double calculateRPM(double distance) {
        return 0; // TODO curve fitting/linear interpolation
    }
}
