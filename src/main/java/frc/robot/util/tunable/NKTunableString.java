package frc.robot.util.tunable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class NKTunableString implements Tunable {

    /**
     * Standard default return value for all instances
     * of the {@link NKTunableNumber} class.
     */
    protected static final String kDefaultValue = "";
    protected static boolean kDefaultEnable = false;
    protected String m_name;
    protected String m_value, m_defaultValue;
    protected boolean m_enabled;

    public NKTunableString(String name, String defaultValue) {
        this.m_name = name;
        this.m_defaultValue = defaultValue;
        TunableRegistry.register(this);
    }

    public NKTunableString(String name) {
        this(name, kDefaultValue);
    }

    protected abstract void updateValue(String newValue);

    @Override
    public void tune() {
        this.updateValue(SmartDashboard.getString(this.m_name, this.m_defaultValue));
        SmartDashboard.putString(this.m_name, this.m_value);
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
}
