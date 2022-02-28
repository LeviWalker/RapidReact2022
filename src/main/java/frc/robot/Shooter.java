package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.control.NKDoubleSolenoid;
import frc.robot.util.control.NKTalonFX;

public class Shooter extends SubsystemBase {

    private double kP = 0.0,
                   kI = 0.0,
                   kD = 0.0,
                   kF = 0.0;

    double targetRPM;

    private NKTalonFX flywheel;
    private NKDoubleSolenoid hood;

    private DigitalInput limitSwitch;

    ColorSensorV3 colorSensor;

    boolean redCargo, blueCargo, hadCargoLastTime;

    public Shooter() {
        flywheel = new NKTalonFX(7);
        flywheel.setPIDF(0, kP, kI, kD, kF);
        hood = new NKDoubleSolenoid(41, PneumaticsModuleType.REVPH, 0, 1);
        limitSwitch = new DigitalInput(0);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        Shuffleboard.getTab("Shooter").addNumber("Target RPM", () -> targetRPM);
        Shuffleboard.getTab("Shooter").addNumber("Actual RPM", flywheel::getVelocityRPM);
        Shuffleboard.getTab("Shooter").addNumber("RPM Error", () -> targetRPM - this.flywheel.getVelocityRPM());
        Shuffleboard.getTab("Shooter").addNumber("Voltage Output", flywheel::getMotorOutputVoltage);

        updateSmartDashPIDF();
    }

    public void setFlywheelTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        // flywheel.setVelocityRPM(this.targetRPM);
        if (Math.abs(targetRPM) > 100 && !isAllianceColorCargo())
            flywheel.set(ControlMode.PercentOutput, 0.40);
        else if (Math.abs(targetRPM) < 100)
            flywheel.set(ControlMode.PercentOutput, 0);
        else
            flywheel.set(ControlMode.PercentOutput, NKTalonFX.Math.rpmToPercent(targetRPM));
    }

    public void setTargetRPM(double rpm) {
        flywheel.setVelocityRPM(rpm);
    }

    public boolean isUpToSpeed() {
        return (targetRPM > 100) && flywheel.getVelocityRPM() == MathUtil.clamp(flywheel.getVelocityRPM(), (1 - 0.20) * targetRPM, (1 + 0.20) * targetRPM);
    }

    public void setHood(boolean closeShot) {
        hood.set(closeShot? Value.kForward : Value.kReverse);
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
        return Robot.isRedAlliance()? this.redCargo : this.blueCargo;
    }

    @Override
    public void periodic() {

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

        updateSmartDashPIDF();
    }

    private void updateSmartDashPIDF() {
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
