package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class VisionTurnToAngle extends CommandBase {

    private PIDController controller;

    private Drivetrain drivetrain;
    private VisionSystem vision;
    private double targetAngle;

    public VisionTurnToAngle(Drivetrain drivetrain, VisionSystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        targetAngle = vision.getAngle();
        drivetrain.getIMU().reset();
    }

    @Override
    public void execute() {
        drivetrain.autoPercentArcadeDrive(
            0,
            controller.calculate(drivetrain.getIMU().getAngle(), targetAngle)
        );
    }


}
