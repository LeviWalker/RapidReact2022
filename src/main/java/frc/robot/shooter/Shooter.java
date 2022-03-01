package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    private DigitalInput limitSwitch;

    ColorSensorV3 colorSensor;

    boolean redCargo, blueCargo, hadCargoLastTime;

    public Shooter() {
        flywheel = new NKTalonFX(7);
        flywheel.setPIDF(0, kP, kI, kD, kF);
        flywheel.enableVoltageCompensation(true);
        flywheel.configVoltageCompSaturation(12);
        hood = new NKDoubleSolenoid(41, PneumaticsModuleType.REVPH, 0, 1);
        limitSwitch = new DigitalInput(0);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        updateSmartDashPIDF();
    }

    public void setFlywheelRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        flywheel.setVelocityRPM(targetRPM);
        // flywheel.setVelocityRPM(this.targetRPM);
        // if (Math.abs(targetRPM) > 100 && !isAllianceColorCargo())
        //     flywheel.set(ControlMode.PercentOutput, 0.40);
        // else if (Math.abs(targetRPM) < 100)
        //     flywheel.set(ControlMode.PercentOutput, 0);
        // else
        //     flywheel.set(ControlMode.PercentOutput, NKTalonFX.Math.rpmToPercent(targetRPM));
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

    public boolean hasCargo() {
        return !this.limitSwitch.get();
    }

    public boolean hasRedCargo() {
        return this.hasCargo() && this.redCargo;
    }

    public boolean hasBlueCargo() {
        return this.hasCargo() && this.blueCargo;
    }

    public boolean isAllianceColorCargo() {
        return (DriverStation.getAlliance() == Alliance.Red)? this.redCargo : this.blueCargo;
    }

    public boolean isAtTarget() {
        return Math.abs(flywheel.getVelocityRPM() - targetRPM) < kFlywheelTolerance;
    }

    @Override
    public void periodic() {
        targetRPM = SmartDashboard.getNumber("Target RPM", 0);
        flywheel.setVelocityRPM(targetRPM);
        setFlywheelP(SmartDashboard.getNumber("Flyweel P", this.kP));
        setFlywheelI(SmartDashboard.getNumber("Flyweel I", this.kI));
        setFlywheelD(SmartDashboard.getNumber("Flyweel D", this.kD));
        setFlywheelF(SmartDashboard.getNumber("Flyweel F", this.kF));

        if (this.hasCargo() && !this.hadCargoLastTime) {
            if (colorSensor.getRed() > 8000) {
                redCargo = true;
                blueCargo = false;
            } else if (colorSensor.getBlue() > 8000) {
                redCargo = false;
                blueCargo = true;
            }
        }

        this.hadCargoLastTime = this.hasCargo();

        log();
    }

    private void log() {
        SmartDashboard.putBoolean("Is at target", isAtTarget());
        SmartDashboard.putNumber("Actual RPM", flywheel.getVelocityRPM());
        SmartDashboard.putNumber("RPM Error", flywheel.getVelocityRPM());
        SmartDashboard.putNumber("Voltage Output", flywheel.getMotorOutputVoltage());
    }

    private void updateSmartDashPIDF() {
        SmartDashboard.putNumber("Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel P", kP);
        SmartDashboard.putNumber("Flywheel I", kI);
        SmartDashboard.putNumber("Flywheel D", kD);
        SmartDashboard.putNumber("Flywheel F", kF);
    }

    public void setFlywheelP(double p) {
        if (kP != p) {
            kP = p;
            flywheel.setP(0, p);
        }
    }
    
    public void setFlywheelI(double i) {
        if (kI != i) {
            kI = i;
            flywheel.setI(0, i);
        }
    }
    
    public void setFlywheelD(double d) {
        if (kD != d) {
            kD = d;
            flywheel.setD(0, d);
        }
    }
    
    public void setFlywheelF(double f) {
        if (kF != f) {
            kF = f;
            flywheel.setF(0, f);
        }
    }

}
