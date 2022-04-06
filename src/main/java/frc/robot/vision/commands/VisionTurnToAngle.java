package frc.robot.vision.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class VisionTurnToAngle extends CommandBase {

    private PIDController controller;

    private Drivetrain drivetrain;
    private VisionSystem vision;
    private double targetAngle;
    private static final double kMaxTurn = 0.80;

    public VisionTurnToAngle(Drivetrain drivetrain, VisionSystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        controller.reset();
        // double newP = SmartDashboard.getNumber("angle p", controller.getP());
        // double newI = SmartDashboard.getNumber("angle i", controller.getI());
        // double newD = SmartDashboard.getNumber("angle d", controller.getD());

        // if (newP != controller.getP()) controller.setP(newP);
        // if (newI != controller.getI()) controller.setI(newI);
        // if (newD != controller.getD()) controller.setD(newD);
        
        // targetAngle = SmartDashboard.getNumber("target angle", 0);
        // targetAngle = vision.getAngle();
        drivetrain.getIMU().reset();
    }

    @Override
    public void execute() {
        drivetrain.autoPercentArcadeDrive(
            0,
            MathUtil.clamp(controller.calculate(drivetrain.getIMU().getAngle(), targetAngle), -kMaxTurn, kMaxTurn)
        );
    }


}
