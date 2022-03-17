package frc.robot.climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class PositionLeftClimb extends CommandBase {
    private Climber climber;

    private static double p = ClimbConstants.kP, absMaxSpeed = ClimbConstants.kAbsoluteMaxSpeed;
    private double leftError, leftSpeed;

    private double targetPosition;

    public PositionLeftClimb(Climber climber, double targetPosition) {
        this.targetPosition = targetPosition;
        this.climber = climber;
        // SmartDashboard.putNumber("Left Climb kP", ClimbConstants.kP);
        // SmartDashboard.putNumber("Left Abs Max Speed from kP", ClimbConstants.kP * ClimbConstants.kL2ClimbUpHallSensorValue);
        // SmartDashboard.putNumber("Left Abs Max Climb Speed", ClimbConstants.kAbsoluteMaxSpeed);
    }

    @Override
    public void execute() {

        // double newP = SmartDashboard.getNumber("Left Climb kP", this.p);
        // if (this.p != newP) {
        //     this.p = newP;
        //     SmartDashboard.putNumber("Left Abs Max Speed from kP", this.p * ClimbConstants.kL2ClimbUpHallSensorValue);
        // }
        
        // double newAbsMaxSpeed = SmartDashboard.getNumber("Left Abs Max Climb Speed", absMaxSpeed);
        // if (this.absMaxSpeed != newAbsMaxSpeed) {
        //     this.absMaxSpeed = newAbsMaxSpeed;
        // }

        // if error is positive, needs to go in the positive direction
        leftError = targetPosition - climber.getLeftEncoderPosition();
        // leftSpeed = MathUtil.clamp(
        //     this.p * leftError,
        //     -ClimbConstants.kAbsoluteMaxSpeed,
        //     ClimbConstants.kAbsoluteMaxSpeed
        // );
        // SmartDashboard.putNumber("Left Climb Error", leftError);
        // SmartDashboard.putNumber("Left Climb Speed", leftSpeed);

        leftSpeed = Math.signum(leftError) * ((Math.abs(leftError) < 20)? 0.40 : ClimbConstants.kAbsoluteMaxSpeed);
        if (leftSpeed < 0 && climber.isLeftAtBottom()) leftSpeed = 0;

        climber.setLeftClimbMotor(leftSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        climber.setLeftClimbMotor(0);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Math.abs(leftError) < ClimbConstants.kPositionTolerance;
        // SmartDashboard.putBoolean("Left isFinished", finished);
        return finished;
    }
}
