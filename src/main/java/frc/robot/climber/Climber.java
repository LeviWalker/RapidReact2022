package frc.robot.climber;


import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.control.NKSparkMax;

public class Climber extends SubsystemBase {
    NKSparkMax left, right;
    RelativeEncoder leftEnc, rightEnc;
    Servo lServo, rServo;

    boolean neutral;

    public Climber() {
        left = new NKSparkMax(ClimbConstants.kLeftClimbMotorID, true); // left climb down -> increasing position
        right = new NKSparkMax(ClimbConstants.kRightClimbMotorID, true); // right climb up -> decreasing position

        left.setInverted(true);
        right.setInverted(true);

        leftEnc = left.getEncoder();
        rightEnc = right.getEncoder();

        lServo = new Servo(ClimbConstants.kLeftClimbServoID);
        rServo = new Servo(ClimbConstants.kRightClimbServoID);

        // TODO Bring motors together once robot is fixed

        neutral = true;
    }

    public void unlockClimb() {
        neutral = false;
    }

    public void lockClimb() {
        neutral = true;
    }

    public void setLeftClimbMotor(double speed) {
        left.set(speed);
    }

    public void setRightClimbMotor(double speed) {
        right.set(speed);
    }

    public double getLeftEncoderPosition() {
        return leftEnc.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEnc.getPosition();
    }

    @Override
    public void periodic() {
        if (neutral) {
            lServo.setAngle(ClimbConstants.kLeftServoNeutralAngle);
            rServo.setAngle(ClimbConstants.kRightServoNeutralAngle);
        } else {
            lServo.setAngle(ClimbConstants.kLeftServoClimbAngle);
            rServo.setAngle(ClimbConstants.kRightServoClimbAngle);
        }
    }
}
