package frc.robot.util.tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class NKTunableBoolean implements Tunable {

    /**
     * Standard default return value for all instances
     * of the {@link NKTunableNumber} class.
     */
    protected static final boolean kDefaultValue = false;
    protected static boolean kDefaultEnable = false;
    protected String m_name;
    protected boolean m_value, m_defaultValue;
    protected boolean m_enabled;

    public NKTunableBoolean(String name, boolean defaultValue) {
        this.m_name = name;
        this.m_defaultValue = defaultValue;
        TunableRegistry.register(this);
    }

    public NKTunableBoolean(String name) {
        this(name, kDefaultValue);
    }

    protected abstract void updateValue(boolean newValue);

    @Override
    public void tune() {
        this.updateValue(SmartDashboard.getBoolean(this.m_name, this.m_defaultValue));
        SmartDashboard.putBoolean(this.m_name, this.m_value);
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

    public boolean getValue() {
        return m_value;
    }
}
