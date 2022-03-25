package frc.robot.shooter.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;
import frc.robot.util.math.DoubleInterpolater;
import frc.robot.vision.VisionSystem;

public class VisionSpinUpShooter extends InstantCommand {

    private static DoubleInterpolater rpmInterpolater, angleInterpolater;

    public VisionSpinUpShooter(Shooter shooter, VisionSystem vision) {
        super(() -> {
            shooter.setFlywheelRPM(calculateRPM(vision.getDistance()));
            shooter.setHoodExtended(false); // can only use vision for far shooting
        }, shooter);
        // addRequirements(shooter);
    }

    private static double calculateRPM(double distance) {
        double rpm = 0;
        if (distance > 9.5 && distance <= 14.2)
            rpm = Math.max(3900 + ((4300-3900)/(14.2-9.7)) * (distance - 9.7), 3900);
        else if (distance > 14.2 && distance < 16.6)
            rpm = Math.min(4300 + ((5200-4300)/(16.4-14.2)) * (distance - 14.2), 5200);
        return rpm;
    }

    private static double rpm(double distance) {
        rpmInterpolater = new DoubleInterpolater()
            .add(0, 0)
            .add(0, 0)
            .add(0, 0)
            .add(0, 0)
            .add(0, 0);
        return rpmInterpolater.interpolate(distance);
    }

    private static double angle() {
        angleInterpolater = new DoubleInterpolater()
            .add(0, 0)
            .add(0, 0)
            .add(0, 0)
            .add(0, 0)
            .add(0, 0);
        return 0;
    }
}
