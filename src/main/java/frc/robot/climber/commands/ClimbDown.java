package frc.robot.climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.climber.Climber;

public class ClimbDown extends CommandBase {
    private Climber climber;

    private double leftError, rightError, leftSpeed, rightSpeed;

    public ClimbDown(Climber climber) {
        addRequirements(climber);
        this.climber = climber;
    }

    @Override
    public void execute() {

        // leftError = ClimbConstants.kClimbDownHallSensorValue - climber.getLeftEncoderPosition();
        // leftSpeed = MathUtil.clamp(
        //     ClimbConstants.kP * leftError,
        //     -ClimbConstants.kAbsoluteMaxSpeed,
        //     ClimbConstants.kAbsoluteMaxSpeed
        // );
        // SmartDashboard.putNumber("Right Climb Error", leftError);
        // SmartDashboard.putNumber("Right Climb Speed", leftSpeed);
        // climber.setLeftClimbMotor(leftSpeed);

        rightError = ClimbConstants.kClimbDownHallSensorValue - climber.getRightEncoderPosition();
        rightSpeed = MathUtil.clamp(
            ClimbConstants.kP * rightError,
            -ClimbConstants.kAbsoluteMaxSpeed,
            ClimbConstants.kAbsoluteMaxSpeed
        );
        SmartDashboard.putNumber("Right Climb Error", rightError);
        SmartDashboard.putNumber("Right Climb Speed", rightSpeed);
        climber.setRightClimbMotor(rightSpeed);
    }
}
