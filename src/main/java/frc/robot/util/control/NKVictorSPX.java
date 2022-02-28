package frc.robot.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.tunable.NKTunableMotorBase;

public class NKVictorSPX extends WPI_VictorSPX implements NKTunableMotorBase {
    
    private ControlMode m_lastMode;
    private double m_lastOutput;
    protected int m_id;
    private boolean m_lock = false;
    private boolean m_useOutputRange = false;
    private double m_minOutput, m_maxOutput;

    public NKVictorSPX(int deviceNumber) {
        super(deviceNumber);
        this.m_id = deviceNumber;
    }

    @Override
    public void set(ControlMode mode, double output) {
        if (mode != m_lastMode || output != m_lastOutput) {
            m_lastMode = mode;
            m_lastOutput = this.m_useOutputRange? MathUtil.clamp(output, this.m_minOutput, this.m_maxOutput) : output;
            super.set(mode, output);
        }
    }

    public void set(double power) {
        this.set(ControlMode.PercentOutput, power);
    }

    @Override
    public int getID() {
        return m_id;
    }

    @Override
    public boolean isTuningLocked() {
        return m_lock;
    }

    @Override
    public void setTuningLock(boolean lock) {
        m_lock = lock;
    }

    @Override
    public void setP(int slot, double p) {
        this.config_kP(slot, p);
    }

    @Override
    public void setI(int slot, double i) {
        this.config_kI(slot, i);
    }

    @Override
    public void setD(int slot, double d) {
        this.config_kD(slot, d);
    }

    @Override
    public void setF(int slot, double f) {
        this.config_kF(slot, f);
    }

    @Override
    public void setIZone(int slot, int iZone) {
        this.config_IntegralZone(slot, iZone);
    }

    @Override
    public void setMaxIntegralAccumulator(int slot, double maxAccumulator) {
        this.configMaxIntegralAccumulator(slot, maxAccumulator);
    }

    @Override
    public void setOutputRange(double min, double max) {
        this.m_useOutputRange = true;
        this.m_minOutput = min;
        this.m_maxOutput = max;
    }

    @Override
    public void resetOutputRange() {
        this.m_useOutputRange = false;
    }

}