package frc.robot.climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class PositionRightClimb extends CommandBase {
    private Climber climber;

    private static double p = ClimbConstants.kP, absMaxSpeed = ClimbConstants.kAbsoluteMaxSpeed;
    private double rightError, rightSpeed;

    private double targetPosition;

    public PositionRightClimb(Climber climber, double targetPosition) {
        this.targetPosition = targetPosition;
        this.climber = climber;
        SmartDashboard.putNumber("Right Climb kP", ClimbConstants.kP);
        SmartDashboard.putNumber("Right Abs Max Speed from kP", ClimbConstants.kP * ClimbConstants.kL2ClimbUpHallSensorValue);
        SmartDashboard.putNumber("Right Abs Max Climb Speed", ClimbConstants.kAbsoluteMaxSpeed);
    }

    @Override
    public void execute() {

        double newP = SmartDashboard.getNumber("Right Climb kP", this.p);
        if (this.p != newP) {
            this.p = newP;
            SmartDashboard.putNumber("Right Abs Max Speed from kP", this.p * ClimbConstants.kL2ClimbUpHallSensorValue);
        }
        
        double newAbsMaxSpeed = SmartDashboard.getNumber("Right Abs Max Climb Speed", absMaxSpeed);
        if (this.absMaxSpeed != newAbsMaxSpeed) {
            this.absMaxSpeed = newAbsMaxSpeed;
        }
        
        rightError = targetPosition - climber.getRightEncoderPosition();
        rightSpeed = MathUtil.clamp(
            this.p * rightError,
            -ClimbConstants.kAbsoluteMaxSpeed,
            ClimbConstants.kAbsoluteMaxSpeed
        );
        SmartDashboard.putNumber("Right Climb Error", rightError);
        SmartDashboard.putNumber("Right Climb Speed", rightSpeed);
        
        if (rightSpeed < 0 && climber.isRightAtBottom()) rightSpeed = 0;
        
        climber.setRightClimbMotor(Math.signum(rightError) * 0.40);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRightClimbMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rightError) < ClimbConstants.kPositionTolerance;
    }
}
