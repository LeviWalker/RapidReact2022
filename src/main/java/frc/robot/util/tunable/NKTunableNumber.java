package frc.robot.util.tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class NKTunableNumber implements Tunable {

    /**
     * Standard default return value for all instances
     * of the {@link NKTunableNumber} class.
     */
    protected static final double kDefaultValue = 0;
    protected static boolean kDefaultEnable = false;
    protected String m_name;
    protected double m_value, m_defaultValue;
    protected boolean m_enabled;

    public NKTunableNumber(String name, double defaultValue) {
        this.m_name = name;
        this.m_defaultValue = defaultValue;
        TunableRegistry.register(this);
    }

    public NKTunableNumber(String name) {
        this(name, kDefaultValue);
    }

    protected abstract void updateValue(double newValue);

    @Override
    public void tune() {
        this.updateValue(SmartDashboard.getNumber(this.m_name, this.m_defaultValue));
        SmartDashboard.putNumber(this.m_name, this.m_value);
    }

    @Override
    public void setTuningEnabled(boolean enable) {
        this.m_enabled = enable;
    }

    @Override
    public boolean isTuningEnabled() {
        return m_enabled;
    }

    @Override
    public void setTunableName(String name) {
        this.m_name = name;
    }

    @Override
    public String getTunableName() {
        return m_name;
    }

    public double getValue() {
        return m_value;
    }
}