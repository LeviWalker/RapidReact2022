package frc.robot.util.tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class TunablePIDController extends PIDController implements Tunable {

    private String name, pName, iName, dName, oName, eName;
    private static final String DefaultName = "PID Tuning";
    private boolean tuning;
    double output = 0;

    public TunablePIDController(double p, double i, double d) {
        super(p, i, d);
        this.setTunableName(DefaultName);
        this.tuning = false;
        TunableRegistry.register(this);
    }

    public void setTuningEnabled(boolean enable) {
        this.tuning = enable;
    }

    public void enableTuning(String name) {
        setTunableName(name);
        setTuningEnabled(true);
    }

    @Override
    public double calculate(double measurement) {
        output = super.calculate(measurement);
        if (tuning) {
            SmartDashboard.putNumber(oName, output);
            SmartDashboard.putNumber(eName, Math.abs(measurement - super.getSetpoint()));
        }
        return output;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    private void updatePID() {
        SmartDashboard.putNumber(this.pName, this.getP());
        SmartDashboard.putNumber(this.iName, this.getI());
        SmartDashboard.putNumber(this.dName, this.getD());
    }

    @Override
    public void tune() {
        final double p = SmartDashboard.getNumber(this.pName, 0);
        final double i = SmartDashboard.getNumber(this.iName, 0);
        final double d = SmartDashboard.getNumber(this.dName, 0);

        if ((p != this.getP())) this.setP(p);
        if ((i != this.getI())) this.setI(i);
        if ((d != this.getP())) this.setD(d);

        updatePID();
    }

    @Override
    public boolean isTuningEnabled() {
        return tuning;
    }

    @Override
    public void setTunableName(String name) {
        this.name = name;
        this.pName = name + " P Gain";
        this.iName = name + " I Gain";
        this.dName = name + " D Gain";
        this.oName = name + " Output";
        this.eName = name + " Absolute Error";
    }

    @Override
    public String getTunableName() {
        return this.name;
    }

}