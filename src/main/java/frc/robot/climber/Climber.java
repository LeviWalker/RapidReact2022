package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.control.NKSolenoid;
import frc.robot.util.control.NKSparkMax;

public class Climber extends SubsystemBase {
    NKSparkMax left, right;
    RelativeEncoder leftEnc, rightEnc;
    NKSolenoid climbShifter; // neutral -> false , climb -> true
    DigitalInput leftLowLimit, rightLowLimit;

    boolean neutral;

    public Climber() {
        left = new NKSparkMax(ClimbConstants.kLeftClimbMotorID, true); // left climb down -> increasing position
        right = new NKSparkMax(ClimbConstants.kRightClimbMotorID, true); // right climb up -> decreasing position

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setSmartCurrentLimit(40);
        right.setSmartCurrentLimit(40);

        left.setInverted(false);
        right.setInverted(true);

        leftEnc = left.getEncoder();
        rightEnc = right.getEncoder();
        
        climbShifter = new NKSolenoid(Constants.kPH, Constants.kPHType, ClimbConstants.kRatchetSolenoidChannel);
        
        leftLowLimit = new DigitalInput(ClimbConstants.kLeftClimbLowLimitSwitchID);
        rightLowLimit = new DigitalInput(ClimbConstants.kRightClimbLowLimitSwitchID);
        
        // SmartDashboard.putBoolean("neutral", neutral);
    }
    
    public void unlockClimb() {
        climbShifter.set(true);
    }
    
    public void lockClimb() {
        climbShifter.set(false);
    }

    public boolean isLocked() {
        return !climbShifter.get();
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
        // neutral = SmartDashboard.getBoolean("neutral", neutral);

        // SmartDashboard.putBoolean("Left Low Limit", isLeftAtBottom());
        // SmartDashboard.putBoolean("Right Low Limit", isRightAtBottom());

        // SmartDashboard.putNumber("left climb position", leftEnc.getPosition());
    }

    public void resetLeftEncoder() {
        leftEnc.setPosition(0);
    }

    public void resetRightEncoder() {
        rightEnc.setPosition(0);
    }
}
