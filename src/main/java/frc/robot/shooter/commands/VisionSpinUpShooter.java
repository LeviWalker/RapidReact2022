package frc.robot.shooter.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooter.Shooter;
import frc.robot.vision.VisionSystem;

public class VisionSpinUpShooter extends InstantCommand {

    private static final Map<Double, Double> calibration = new HashMap<>();

    static {
        calibration.put(9.5, 3900.);
        calibration.put(9.7, 3900.);
        calibration.put(13.2, 4300.);
        calibration.put(14.2, 4300.);
        calibration.put(16.6, 5200.);
    }
    
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

    private static double linearInterpolation(double distance) {
        double retVal;
        Double[] keys = (Double[]) calibration.keySet().toArray();
        Double[] vals = (Double[]) calibration.keySet().toArray();
        int i = 0;
        for (i = 0; i < keys.length; i++) {
            if (distance <= keys[i]) break;
        }

        if (i == keys.length - 1)
            retVal = vals[i];
        else
        //           intercept                      slope                              difference  
            retVal = vals[i] + ((vals[i + 1] - vals[i])/(keys[i + 1] - keys[i])) * (distance - keys[i]);
        

        return MathUtil.clamp(retVal, 3900, 5200);
    }
}
