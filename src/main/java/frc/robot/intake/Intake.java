package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.control.NKDoubleSolenoid;
import frc.robot.util.control.NKVictorSPX;

public class Intake extends SubsystemBase {

    NKVictorSPX intake;
    NKDoubleSolenoid deployer;

    public Intake() {
        intake = new NKVictorSPX(IntakeConstants.kIntakeMotorID);
        deployer = new NKDoubleSolenoid(
            Constants.kPH,
            Constants.kPHType,
            IntakeConstants.kIntakeDeployerForwardChannel,
            IntakeConstants.kIntakeDeployerReverseChannel
        );
    }

    public void setIntake(double power) {
        intake.set(power);
    }

    public boolean isDeployed() {
        return deployer.get() == IntakeConstants.kDeployed;
    }

    public void deploy() {
        deployer.set(IntakeConstants.kDeployed);
    }

    public void retract() {
        deployer.set(IntakeConstants.kRetracted);
    }
    
}
