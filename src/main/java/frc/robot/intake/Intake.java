package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.NKDoubleSolenoid;
import frc.robot.util.control.NKVictorSPX;

public class Intake extends SubsystemBase {

    NKVictorSPX intake;
    NKDoubleSolenoid deployer;

    public Intake() {
        intake = new NKVictorSPX(9);
        deployer = new NKDoubleSolenoid(41, PneumaticsModuleType.REVPH, 2, 3);
    }

    public void setIntake(double power) {
        intake.set(
            (power > 0.07)?
            MathUtil.clamp(power, 0.25, 0.60) :
            (power < -0.07) ? MathUtil.clamp(power, -0.60, -0.25) : 0
        );
    }

    public boolean isDeployed() {
        return deployer.get() == Value.kForward;
    }

    public void deploy() {
        deployer.set(Value.kForward);
    }

    public void retract() {
        deployer.set(Value.kReverse);
    }

    public boolean isIntaking() {
        return intake.get() > 0;
    }

    public boolean isIntakeRunning() {
        return Math.abs(intake.get()) > 0;
    }
    
}
