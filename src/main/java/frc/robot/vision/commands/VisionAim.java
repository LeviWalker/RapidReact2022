package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;
import frc.robot.util.math.DoubleInterpolater;
import frc.robot.vision.VisionSystem;

public class VisionAim extends CommandBase {
    DoubleInterpolater interpolater;
    PIDController controller;
    public VisionAim(Drivetrain drivetrain, VisionSystem vision) {
        addRequirements(drivetrain);
    }
}
