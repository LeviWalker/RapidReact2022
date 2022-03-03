package frc.robot.indexer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.control.NKVictorSPX;

public class Indexer extends SubsystemBase {
    private NKVictorSPX transfer;
    private ColorSensorV3 sensor;
    private DigitalInput limit;

    public Indexer() {
        transfer = new NKVictorSPX(10);
        transfer.setNeutralMode(NeutralMode.Brake);
        sensor = new ColorSensorV3(I2C.Port.kOnboard);
        limit = new DigitalInput(0);
    }

    public void intakeIndex() {
        transfer.set(0.20);
    }

    public void shootIndex() {
        transfer.set(0.50);
    }

    public void stopIndex() {
        transfer.set(0.0);
    }

    public boolean hasCargo() {
        return !limit.get();
    }

    @Override
    public void periodic() {
        // if (this.hasCargo() && !this.hadCargoLastTime) {
        //     if (colorSensor.getRed() > 8000) {
        //         redCargo = true;
        //         blueCargo = false;
        //     } else if (colorSensor.getBlue() > 8000) {
        //         redCargo = false;
        //         blueCargo = true;
        //     }
        // }

        // this.hadCargoLastTime = this.hasCargo();

    }
}
