package frc.robot.util.control;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NKServo extends Servo implements Subsystem {

    NKServoMode mode = NKServoMode.kValue;
    double value = 0;

    public NKServo(int channel) {
        super(channel);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void set(double value) {
        this.mode = NKServoMode.kValue;
        this.value = value;
    }

    @Override
    public void setAngle(double degrees) {
        this.mode = NKServoMode.kAngle;
        this.value = degrees;
    }

    @Override
    public void setRaw(int value) {
        this.mode = NKServoMode.kRaw;
        this.value = value;
    }

    @Override
    public void setPosition(double pos) {
        this.mode = NKServoMode.kPosition;
        this.value = pos;
    }

    @Override
    public void setSpeed(double speed) {
        this.mode = NKServoMode.kSpeed;
        this.value = speed;
    }

    public String GET() {
        return this.mode.name().substring(1) + ": " + this.value;
    }

    public void printSatus() {
        System.out.println(this.GET());
    }
    
    @Override
    public void periodic() {
        switch (mode) {
            case kAngle:
                super.setAngle(value);
            case kRaw:
                super.setRaw((int) value);
            case kPosition:
                super.setPosition(value);
            case kSpeed:
                super.setSpeed(value);
            case kValue:
            default:
                super.set(value);

        }
    }

    private enum NKServoMode {
        kRaw, kAngle, kValue, kPosition, kSpeed;
    }
}
