package frc.robot.vision.commands;

import com.fasterxml.jackson.databind.ser.std.AsArraySerializerBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;
import frc.robot.shooter.commands.SmartDashShooter;
import frc.robot.vision.VisionSystem;

public class VisionDrive extends CommandBase {

    final double p = 0.03, i = 0, d = 0;
    final double minTurnVoltage = 0.75, maxTurnVolage = 6;
    PIDController controller;
    Drivetrain drivetrain;
    VisionSystem vision;

    double count = 0;

    final double targetAngle = -1.5;

    public VisionDrive(Drivetrain drivetrain, VisionSystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain);

        controller = new PIDController(p, i, d);
        controller.setSetpoint(targetAngle);

        SmartDashboard.putNumber("vision p", p);
        SmartDashboard.putNumber("vision i", i);
        SmartDashboard.putNumber("vision d", d);
    }

    @Override
    public void initialize() {
        count = 0;
        double p = SmartDashboard.getNumber("vision p", this.controller.getP());
        double i = SmartDashboard.getNumber("vision i", this.controller.getI());
        double d = SmartDashboard.getNumber("vision d", this.controller.getD());

        if (p != this.controller.getP()) this.controller.setP(p);
        if (i != this.controller.getI()) this.controller.setI(i);
        if (d != this.controller.getD()) this.controller.setD(d);

        controller.reset();
        controller.setSetpoint(targetAngle);

        SmartDashboard.putNumber("vision p", p);
        SmartDashboard.putNumber("vision i", i);
        SmartDashboard.putNumber("vision d", d);
    }


    @Override
    public void execute() {
        double output = -MathUtil.clamp(controller.calculate(vision.getAngle()), -(maxTurnVolage - minTurnVoltage), maxTurnVolage - minTurnVoltage);
        output += (Math.signum(vision.getAngle() - targetAngle) * minTurnVoltage);
        drivetrain.tankDriveVolts(output, -output);
        SmartDashboard.putNumber("drive output", output);

        if (Math.abs(vision.getAngle() - targetAngle) < 0.3) count++;
        else count = 0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        return count > 10;
    }

}
