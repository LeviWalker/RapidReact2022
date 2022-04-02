package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.Shooter;
import frc.robot.util.math.DoubleInterpolater;
import frc.robot.vision.VisionSystem;

public class VisionShoot extends CommandBase {
    private DoubleInterpolater rpmInterpolater;
    private Shooter shooter;

    private double distance;
    private VisionSystem vision;

    public VisionShoot(Shooter shooter, VisionSystem vision) {
        addRequirements(shooter);

        rpmInterpolater = new DoubleInterpolater()
            .add(10.09, 3925)
            .add(7.2, 3700)
            .add(19.9, 4800)
            .add(13.2, 4700)
            .add(14.09, 4400);
        rpmInterpolater.sort();

        this.shooter = shooter;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        distance = vision.getDistance();
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(rpmInterpolater.interpolate(distance));
        shooter.setHoodExtended(false);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelRPM(0);
    }
}
