package frc.robot.climber;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.control.NKSparkMax;

public class Climber extends SubsystemBase {
    NKSparkMax left, right;
    RelativeEncoder leftEnc, rightEnc;
    Servo lServo, rServo;
    DigitalInput leftLowLimit, rightLowLimit;

    boolean neutral;

    public Climber() {
        left = new NKSparkMax(ClimbConstants.kLeftClimbMotorID, true); // left climb down -> increasing position
        right = new NKSparkMax(ClimbConstants.kRightClimbMotorID, true); // right climb up -> decreasing position

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setInverted(false);
        right.setInverted(true);

        leftEnc = left.getEncoder();
        rightEnc = right.getEncoder();
        
        lServo = new Servo(ClimbConstants.kLeftClimbServoID);
        rServo = new Servo(ClimbConstants.kRightClimbServoID);
        
        leftLowLimit = new DigitalInput(ClimbConstants.kLeftClimbLowLimitSwitchID);
        rightLowLimit = new DigitalInput(ClimbConstants.kRightClimbLowLimitSwitchID);
        
        neutral = true;
        SmartDashboard.putBoolean("neutral", neutral);
    }
    
    public void unlockClimb() {
        neutral = false;
    }
    
    public void lockClimb() {
        neutral = true;
    }

    public void setClimbMotors(double speed) {
        left.set(speed);
        right.set(speed);
    }
    
    public void setLeftClimbMotor(double speed) {
        if (isLeftAtBottom() && speed < 0) left.set(0);
        else left.set(speed);
    }

    public void setRightClimbMotor(double speed) {
        if (isRightAtBottom() && speed < 0) right.set(0);
        else right.set(speed);
    }
    
    public double getLeftEncoderPosition() {
        return leftEnc.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEnc.getPosition();
    }
    
    public boolean isLeftAtBottom() {
        return !leftLowLimit.get();
    }

    public boolean isRightAtBottom() {
        return !rightLowLimit.get();
    }
    
    @Override
    public void periodic() {
        neutral = SmartDashboard.getBoolean("neutral", neutral);
        if (neutral) {
            lServo.setAngle(ClimbConstants.kLeftServoNeutralAngle);
            rServo.setAngle(ClimbConstants.kRightServoNeutralAngle);
        } else {
            lServo.setAngle(ClimbConstants.kLeftServoClimbAngle);
            rServo.setAngle(ClimbConstants.kRightServoClimbAngle);
        }

        SmartDashboard.putBoolean("Left Low Limit", isLeftAtBottom());
        SmartDashboard.putBoolean("Right Low Limit", isRightAtBottom());
    }

    public void resetLeftEncoder() {
        leftEnc.setPosition(0);
    }

    public void resetRightEncoder() {
        rightEnc.setPosition(0);
    }
}
