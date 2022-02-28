package frc.robot.util.control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.tunable.NKTunableMotorBase;

import static frc.robot.util.control.NKTalonFX.Math.*;

public class NKTalonFX extends WPI_TalonFX implements NKTunableMotorBase {

    public static final double kMaxFreeSpeedVelocityTicksPer100Milliseconds = 21777;
    public static final double kMaxFreeSpeedVelocityRPM = 6380;
    public static final double kTicksPerRotation = 2048;

    private ControlMode m_lastMode;
    private double m_lastOutput;
    private boolean m_lock = false;
    protected int m_id;
    private double m_minOutput;
    private double m_maxOutput;
    private boolean m_useOutputRange = false;

    public NKTalonFX(int deviceNumber) {
        super(deviceNumber);
        m_id = deviceNumber;
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

    public void setVelocity(double ticksPer100Milliseconds) {
        this.set(
            ControlMode.Velocity,
            MathUtil.clamp(
                ticksPer100Milliseconds,
                -NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds,
                NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds
            )
        );
    }

    public void setVelocityRPM(double rpm) {
        this.setVelocity(rpmToTicksPer100Milliseconds(rpm));
    }

    public double getPosition() {
        return super.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getPositionRotations() {
        return this.getPosition() / NKTalonFX.kTicksPerRotation;
    }

    public double getAbsolutePosition() {
        return super.getSensorCollection().getIntegratedSensorAbsolutePosition();
    }

    public double getAbsolutePositionRotations() {
        return this.getAbsolutePosition() / NKTalonFX.kTicksPerRotation;
    }

    public double getVelocity() {
        return super.getSensorCollection().getIntegratedSensorVelocity();
    }

    public double getVelocityRPM() {
        return ticksPer100MillisecondsToRPM(this.getVelocity());
    }

    public void resetPosition() {
        this.setPosition(0, 0);
    }

    public void setPosition(double newPosition, int timeoutMs) {
        super.getSensorCollection().setIntegratedSensorPosition(newPosition, timeoutMs);
    }

    @Override
    public void setP(int slot, double p) {
        this.config_kP(slot, p, 0);
    }

    @Override
    public void setI(int slot, double i) {
        this.config_kI(slot, i, 0);
    }
    @Override
    public void setD(int slot, double d) {
        this.config_kD(slot, d, 0);
    }

    @Override
    public void setF(int slot, double f) {
        this.config_kF(slot, f, 0);
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
    public boolean isTuningLocked() {
        return m_lock;
    }

    @Override
    public void setTuningLock(boolean lock) {
        this.m_lock = lock;
    }

    @Override
    public int getID() {
        return m_id;
    }

    @Override
    public void resetOutputRange() {
        this.m_useOutputRange = false;
    }

    public static class Math {

        /**
         * A rough estimate of the percent output
         * for a certain velocity in ticks per
         * 100 ms (default for ControlMode.Velocity)
         * @param ticksPer100Milliseconds velocity
         * @return the rough estimate
         */
        public static double ticksPer100MillisecondsToPercent(double ticksPer100Milliseconds) {
            return ticksPer100Milliseconds / NKTalonFX.kMaxFreeSpeedVelocityTicksPer100Milliseconds;
        }

        /**
         * A rough estimate of the percent output
         * for a certain velocity in RPM
         * @param ticksPer100Milliseconds velocity
         * @return the rough estimate
         */
        public static double rpmToPercent(double rpm) {
            return rpm / NKTalonFX.kMaxFreeSpeedVelocityRPM;
        }

        /**
         * Converts RPM to ticks per 100 ms.
         * @param rpm
         * @return ticks per 100 ms
         */
        public static double rpmToTicksPer100Milliseconds(double rpm) {
            return rpm * 3.41333333333;
        }

        /**
         * Converts ticks per 100 ms to RPM.
         * @param ticksPer100Milliseconds
         * @return RPM
         */
        public static double ticksPer100MillisecondsToRPM(double ticksPer100Milliseconds) {
            return ticksPer100Milliseconds / 3.41333333333;
        }
    }
}