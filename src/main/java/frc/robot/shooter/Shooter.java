package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
                   kF = 0.0475;

    double targetRPM;

    private NKTalonFX flywheel;
    private NKDoubleSolenoid hood;

    private double kFlywheelTolerance = 20;

    public Shooter() {
        flywheel = new NKTalonFX(ShooterConstants.kFlywheelMotorID);
        flywheel.setPIDF(0, kP, kI, kD, kF);
        flywheel.enableVoltageCompensation(true);
        flywheel.configVoltageCompSaturation(12);
        hood = new NKDoubleSolenoid(
            Constants.kPH,
            Constants.kPHType,
            ShooterConstants.kHoodForwardChannelID,
            ShooterConstants.kHoodReverseChannelID
        );

        SmartDashboard.putNumber("setFlywheelRPM", targetRPM);
        SmartDashboard.putBoolean("setHoodExtended", getHoodExtended());
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
        return targetRPM > 100 && Math.abs(flywheel.getVelocityRPM() - targetRPM) < kFlywheelTolerance;
    }

    @Override
    public void periodic() {
        log();
    }

    private void log() {
        SmartDashboard.putNumber("Flywheel RPM", flywheel.getVelocityRPM());
        SmartDashboard.putNumber("Voltage Output", flywheel.getMotorOutputVoltage());

        // double newRPM = SmartDashboard.getNumber("setFlywheelRPM", targetRPM);
        // if (targetRPM != newRPM) setFlywheelRPM(newRPM);

        // boolean newHood = SmartDashboard.getBoolean("setHoodExtended", getHoodExtended());
        // if (getHoodExtended() != newHood) setHoodExtended(newHood);
    }
}
