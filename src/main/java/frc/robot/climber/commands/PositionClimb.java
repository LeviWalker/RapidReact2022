package frc.robot.climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class PositionClimb extends CommandBase {
    private Climber climber;

    private static double p = ClimbConstants.kP, absMaxSpeed = ClimbConstants.kAbsoluteMaxSpeed;
    private double leftError, leftSpeed;
    private double rightError, rightSpeed;

    private double targetPosition;

    public PositionClimb(Climber climber, double targetPosition) {
        addRequirements(climber);
        this.targetPosition = targetPosition;
        this.climber = climber;
        SmartDashboard.putNumber("Climb kP", ClimbConstants.kP);
        SmartDashboard.putNumber("Abs Max Speed from kP", ClimbConstants.kP * ClimbConstants.kL2ClimbUpHallSensorValue);
        SmartDashboard.putNumber("Abs Max Climb Speed", ClimbConstants.kAbsoluteMaxSpeed);
    }

    @Override
    public void execute() {

        double newP = SmartDashboard.getNumber("Climb kP", this.p);
        if (this.p != newP) {
            this.p = newP;
            SmartDashboard.putNumber("Abs Max Speed from kP", this.p * ClimbConstants.kL2ClimbUpHallSensorValue);
        }
        
        double newAbsMaxSpeed = SmartDashboard.getNumber("Abs Max Climb Speed", absMaxSpeed);
        if (this.absMaxSpeed != newAbsMaxSpeed) {
            this.absMaxSpeed = newAbsMaxSpeed;
        }

        leftError = targetPosition - climber.getLeftEncoderPosition();
        leftSpeed = MathUtil.clamp(
            this.p * leftError,
            -ClimbConstants.kAbsoluteMaxSpeed,
            ClimbConstants.kAbsoluteMaxSpeed
        );
        SmartDashboard.putNumber("Left Climb Error", leftError);
        SmartDashboard.putNumber("Left Climb Speed", leftSpeed);

        if (leftSpeed < 0 && climber.isLeftAtBottom()) leftSpeed = 0;
        climber.setClimbMotors(leftSpeed);
        
        rightError = targetPosition - climber.getRightEncoderPosition();
        rightSpeed = MathUtil.clamp(
            this.p * rightError,
            -ClimbConstants.kAbsoluteMaxSpeed,
            ClimbConstants.kAbsoluteMaxSpeed
        );
        SmartDashboard.putNumber("Right Climb Error", rightError);
        SmartDashboard.putNumber("Right Climb Speed", rightSpeed);
        
        if (rightSpeed < 0 && climber.isRightAtBottom()) rightSpeed = 0;
        climber.setClimbMotors(rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(leftError) < ClimbConstants.kPositionTolerance)
                && (Math.abs(rightError) < ClimbConstants.kPositionTolerance);
    }
}
