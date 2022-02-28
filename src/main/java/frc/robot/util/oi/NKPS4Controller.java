package frc.robot.util.oi;

import edu.wpi.first.wpilibj.PS4Controller;

public class NKPS4Controller extends PS4Controller {

    public NKPS4Controller(int port) {
        super(port);
    }

    public double getLeftTrigger() {
        return this.getL2Axis();
    }

    public boolean getLeftTriggerRawButton() {
        return this.getL2Button();
    }

    public boolean getLeftTriggerRawPressed() {
        return this.getL2ButtonPressed();
    }

    public boolean getLeftTriggerRawReleased() {
        return this.getL2ButtonReleased();
    }

    public double getRightTrigger() {
        return this.getR2Axis();
    }

    public boolean getRightTriggerRawButton() {
        return this.getR2Button();
    }

    public boolean getRightTriggerRawPressed() {
        return this.getR2ButtonPressed();
    }
    
    public boolean getRightTriggerRawReleased() {
        return this.getR2ButtonReleased();
    }
}
