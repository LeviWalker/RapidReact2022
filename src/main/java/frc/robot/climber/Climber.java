package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
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
    PIDController leftController, rightController;

    boolean neutral;

    public Climber() {
        left = new NKSparkMax(ClimbConstants.kLeftClimbMotorID, true); // left climb down -> increasing position
        right = new NKSparkMax(ClimbConstants.kRightClimbMotorID, true); // right climb up -> decreasing position

        // set these to break so that we can stop fast
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        // current limits 
        left.setSmartCurrentLimit(40);
        right.setSmartCurrentLimit(40);

        // invert the motors if needed
        left.setInverted(false);
        right.setInverted(true);

        // let's create and store our integrated encoders
        leftEnc = left.getEncoder();
        rightEnc = right.getEncoder();
        
        // solenoid for the ratchet action
        climbShifter = new NKSolenoid(Constants.kPH, Constants.kPHType, ClimbConstants.kRatchetSolenoidChannel);
        
        // limit switches because the robot will break without them
        leftLowLimit = new DigitalInput(ClimbConstants.kLeftClimbLowLimitSwitchID);
        rightLowLimit = new DigitalInput(ClimbConstants.kRightClimbLowLimitSwitchID);

        // PID controllers for velocity???
        leftController = new PIDController(0, 0, 0);
        rightController = new PIDController(0, 0, 0);
        
        // SmartDashboard.putBoolean("neutral", neutral);
    }
    
    /**
     * Unlocks the ratchet
     */
    public void unlockClimb() {
        climbShifter.set(true);
    }
    
    /**
     * Locks the ratchet
     */
    public void lockClimb() {
        climbShifter.set(false);
    }

    /**
     * Lets everyone who wants to know if the climb is locked or not
     * @return true if locked
     */
    public boolean isLocked() {
        // invert becasue setting the solenoid to true corresponds to unlocked
        return !climbShifter.get();
    }

    /**
     * Sets the speed of both climb motors
     * @param speed the speed in percent, |speed| <= 1
     */
    public void setClimbMotors(double speed) {
        setLeftClimbMotor(speed);
        setRightClimbMotor(speed);
    }
    
    /**
     * Sets the speed of the left climb motors,
     * should stop when limit switch is hit
     * @param speed the speed in percent, |speed| <= 1
     */
    public void setLeftClimbMotor(double speed) {
        if (isLeftAtBottom() && speed < 0) left.set(0);
        else left.set(speed);
    }

    /**
     * Sets the speed of the right climb motors,
     * should stop when limit switch is hit
     * @param speed the speed in percent, |speed| <= 1
     */
    public void setLeftClimbVelocity(double velocity) {
        if (isLeftAtBottom() && velocity < 0) left.set(0);
    }

    
    public void setRightClimbMotor(double speed) {
        if (isRightAtBottom() && speed < 0) right.set(0);
        else right.set(speed);
    }
    
    public void setRightClimbVelocity(double velocity) {
        if (isRightAtBottom() && velocity < 0) left.set(0); // 2270
        else right.set(rightController.calculate(rightEnc.getVelocity(), velocity));
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
