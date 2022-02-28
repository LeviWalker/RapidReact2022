package frc.robot;

import javax.swing.plaf.basic.BasicBorders.SplitPaneBorder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.NKSparkMax;

public class Climber extends SubsystemBase {
    NKSparkMax left, right;
    RelativeEncoder lEncoder, rEncoder;
    Servo lServo, rServo;

    boolean neutral;

    private final double climbExtended = 247.793, climbRetracted = 0;

    private double deadbandError = 7, deadbandSpeed = 0.02;
    private final double kP = deadbandError / deadbandSpeed;

    private double targetPosition;

    private double MaxSpeed = 0.70;

    double error, speed;

    public Climber() {
        left = new NKSparkMax(11, true); // left climb down -> increasing position
        right = new NKSparkMax(12, true); // right climb up -> decreasing position

        left.setInverted(true);
        right.setInverted(true);
    }

    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public double getError() {
        return error;
    }

    @Override
    public void periodic() {
        if (neutral) {
            lServo.setAngle(90);
            rServo.setAngle(90);
        } else {
            lServo.setAngle(50);
            rServo.setAngle(130);
        }

        if (!neutral) {
            error = targetPosition - right.getEncoder().getPosition();
            speed = MathUtil.clamp(kP * error, -MaxSpeed, MaxSpeed);
            right.set(speed);
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
