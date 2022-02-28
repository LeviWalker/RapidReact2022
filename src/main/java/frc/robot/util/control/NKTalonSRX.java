package frc.robot.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.tunable.NKTunableMotorBase;

public class NKTalonSRX extends WPI_TalonSRX implements NKTunableMotorBase {
    /**
     * The default slot for PID control is zero.
     * If you would like to run more than one PID
     * loop on this motor, you may create a integer
     * with the number you want for your slot and
     * pass that in instead of this.
     * For the purposes of this program, we will use
     * the default PID slot. DO NOT CREATE A SLOT
     * WITH A VALUE OF ZERO! I have also created some
     * other PID slot ID numbers for you that do not
     * conflict with each other.
     * @see NKTalonFX#kSecondaryPIDSlot
     * @see NKTalonFX#kTertiaryPIDSlot
     * @see NKTalonFX#kQuaternaryPIDSlot
     */
    public static final int kDefaultPIDSlot = 0;

    public static final int kSecondaryPIDSlot = 1;
    public static final int kTertiaryPIDSlot = 2;
    public static final int kQuaternaryPIDSlot = 3;

    private ControlMode m_lastMode;
    private double m_lastOutput;
    private int m_id;
    private boolean m_lock;
    private boolean m_useOutputRange = false;
    private double m_maxOutput;
    private double m_minOutput;

    public NKTalonSRX(int deviceNumber) {
        super(deviceNumber);
        this.m_id = deviceNumber;
        this.m_handle = deviceNumber;
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
        return this.m_id;
    }

    @Override
    public boolean isTuningLocked() {
        return this.m_lock;
    }

    @Override
    public void setTuningLock(boolean lock) {
        this.m_lock = lock;
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
        m_useOutputRange = false;
    }
}