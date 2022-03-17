package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.control.NKDoubleSolenoid;
import frc.robot.util.control.NKTalonFX;

public class Shooter extends SubsystemBase {

    private double kP = 0.15,
                   kI = 0.0,
                   kD = 0.0,
                   kF = 0.0480;

    double targetRPM, targetPercent;

    private NKTalonFX flywheel;
    private NKDoubleSolenoid hood;

    private double kFlywheelTolerance = 40;

    public Shooter() {
        flywheel = new NKTalonFX(ShooterConstants.kFlywheelMotorID);
        flywheel.setPIDF(0, kP, kI, kD, kF);
        flywheel.configVoltageCompSaturation(12);
        flywheel.enableVoltageCompensation(true);
        hood = new NKDoubleSolenoid(
            Constants.kPH,
            Constants.kPHType,
            ShooterConstants.kHoodForwardChannelID,
            ShooterConstants.kHoodReverseChannelID
        );

        // SmartDashboard.putNumber("setFlywheelRPM", targetRPM);
        // SmartDashboard.putBoolean("setHoodExtended", getHoodExtended());
    }

    public void setFlywheelRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        flywheel.setVelocityRPM(targetRPM);
    }

    public double getFlywheelRPM() {
        return flywheel.getVelocityRPM();
    }

    public void stopFlywheel() {
        targetPercent = 0;
        flywheel.set(0); // removed ControlMode.PercentOutput to use the get() method
    }

    public void setHoodExtended(boolean extended) {
        hood.set(extended? ShooterConstants.kHoodExtended : ShooterConstants.kHoodRetracted);
    }

    public boolean getHoodExtended() {
        return hood.get() == ShooterConstants.kHoodExtended;
    }

    public void toggleHood() {
        setHoodExtended(!getHoodExtended());
    }

    public boolean isAtTarget() {
        return (targetPercent > 0.05 || targetRPM > 100) && Math.abs(flywheel.getVelocityRPM() - targetRPM) < kFlywheelTolerance;
    }

    @Override
    public void periodic() {
        log();
    }

    private void log() {
        // SmartDashboard.putNumber("Flywheel RPM", flywheel.getVelocityRPM());
        // SmartDashboard.putNumber("Voltage Output", flywheel.getMotorOutputVoltage());
    }
}
