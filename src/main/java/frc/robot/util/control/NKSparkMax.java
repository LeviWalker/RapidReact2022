package frc.robot.util.control;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.util.tunable.NKTunableMotorBase;

import com.revrobotics.CANSparkMax;

public class NKSparkMax extends CANSparkMax implements NKTunableMotorBase {

    protected SparkMaxPIDController controller;
    protected double m_lastSpeed;
    private boolean m_lock;
    protected int m_id;

    public NKSparkMax(int deviceID, boolean isBrushless) {
        super(deviceID, isBrushless? MotorType.kBrushless : MotorType.kBrushed);
        this.controller = this.getPIDController();
        this.m_id = deviceID;
    }

    @Override
    public void set(double speed) {
        if (this.m_lastSpeed != speed) {
            this.m_lastSpeed = speed;
            super.set(speed);
        }
        
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

    /**
     * @return the onboard PID controller for the motor
     */
    public SparkMaxPIDController getController() {
        return controller;
    }

    @Override
    public void setP(int slot, double p) {
        this.controller.setP(p, slot);
    }

    @Override
    public void setI(int slot, double i) {
        this.controller.setI(i, slot);
    }

    @Override
    public void setD(int slot, double d) {
        this.controller.setD(d, slot);
    }

    @Override
    public void setF(int slot, double f) {
        this.controller.setFF(f, slot);
    }

    @Override
    public void setIZone(int slot, int iZone) {
        this.controller.setIZone(iZone, slot);
    }

    @Override
    public void setMaxIntegralAccumulator(int slot, double maxAccumulator) {
        this.controller.setIMaxAccum(maxAccumulator, slot);
    }

    @Override
    public void setOutputRange(double min, double max) {
        this.controller.setOutputRange(min, max);
    }

    @Override
    public void resetOutputRange() {
        this.controller.setOutputRange(Double.MIN_VALUE, Double.MAX_VALUE);
    }

}