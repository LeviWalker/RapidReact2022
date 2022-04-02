package frc.robot.auto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSelectorSwitch extends SubsystemBase {
    private DigitalInput one, two, three, four;

    public AutoSelectorSwitch() {
        one = new DigitalInput(6);
        two = new DigitalInput(7);
        three = new DigitalInput(8);
        four = new DigitalInput(9);
    }

    public boolean[] getRaw() {
        boolean[] result = {one.get(), two.get(), three.get(), four.get()};
        return result;
    }

    public int getPosition() {
        if (!one.get()) return 1;
        else if (!two.get()) return 2;
        else if (!three.get()) return 3;
        else if (!four.get()) return 4;
        else return 0;
    }

    public boolean doFourCargo() {
        return !three.get();
    }

    public boolean doAltTwoCargo() {
        return !two.get();
    }

    public boolean doTwoCargoLayup() {
        return !one.get();
    }
}
