package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.NKDoubleSolenoid;
import frc.robot.util.control.NKTalonFX;

public class Shooter extends SubsystemBase {

    private double 
                   kP = 0.15,
                   kI = 0.0,
                   kD = 0.0,
                   kF = 0.0475;

    double targetRPM;

    private NKTalonFX flywheel;
    private NKDoubleSolenoid hood;

    private double kFlywheelTolerance = 20;

    public Shooter() {
        flywheel = new NKTalonFX(7);
        flywheel.setPIDF(0, kP, kI, kD, kF);
        flywheel.enableVoltageCompensation(true);
        flywheel.configVoltageCompSaturation(12);
        hood = new NKDoubleSolenoid(41, PneumaticsModuleType.REVPH, 0, 1);
    }

    public void setFlywheelRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        flywheel.setVelocityRPM(targetRPM);
    }

    public double getFlywheelRPM() {
        return flywheel.getVelocityRPM();
    }

    public void stopFlywheel() {
        flywheel.set(ControlMode.PercentOutput, 0);
    }

    public void setHood(boolean closeShot) {
        hood.set(closeShot ? Value.kForward : Value.kReverse);
    }

    public boolean getHood() {
        return hood.get() == Value.kForward;
    }

    public void toggleHood() {
        setHood(!getHood());
    }

    public boolean isAtTarget() {
        return Math.abs(flywheel.getVelocityRPM() - targetRPM) < kFlywheelTolerance;
    }

    @Override
    public void periodic() {
        log();
    }

    private void log() {
        SmartDashboard.putBoolean("Is at target", isAtTarget());
        SmartDashboard.putNumber("Actual RPM", flywheel.getVelocityRPM());
        SmartDashboard.putNumber("RPM Error", flywheel.getVelocityRPM());
        SmartDashboard.putNumber("Voltage Output", flywheel.getMotorOutputVoltage());
    }
}
