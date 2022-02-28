package frc.robot.util.tunable;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.control.NKSparkMax;
import frc.robot.util.control.NKTalonFX;
import frc.robot.util.control.NKTalonSRX;
import frc.robot.util.control.NKVictorSPX;

public class NKTunablePIDFLoop implements Tunable {

    private double p, i, d, f, maxAccum;
    private int iZone;
    private String name, pName, iName, dName, fName, maxAccumName = null, iZoneName = null;
    ArrayList<NKTunableMotorBase> motors;
    private int slot;
    private boolean tuningEnabled;

    public NKTunablePIDFLoop(String name, int slot) throws Exception {
        this(name, slot, 0, 0, 0, 0);
    }

    public NKTunablePIDFLoop(String name, int slot, double p, double i, double d, double f, NKTunableMotorBase... motors) throws Exception {
        setTunableName(name);
        this.motors = new ArrayList<NKTunableMotorBase>();
        this.slot = slot;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        TunableRegistry.register(this);
        if (motors != null) {
            this.add(motors);
        }
    }

    public void add(NKTunableMotorBase... motors) throws Exception {
        if (motors.length > 0) { // if there are any motors
            String motorClassName = motors[0].getClass().getName();
            for (NKTunableMotorBase motor : motors) { // for each motor
                if (motor.getClass().getName().equals(motorClassName)) { // if same class as the first one
                    if (!motor.isTuningLocked()) { // is another loop tuning this motor?
                        motor.setTuningLock(true); // set the lock so other loops will know if they try anything
                        this.motors.add(motor); // now we add it after so much work
                    } else {
                        throw new NKTunableMotorAlreadyLockedException(motor); // you can't do this, exception
                    }
                } else {
                    throw new NKPIDLoopException(
                        "You cannot add two motors with different motor"
                         + "controllers to the same NKTunablePIDLoop"
                    ); // you can't do this either, please do not do this
                }
            }
        }
    }

    public void remove(NKTunableMotorBase... motors) {
        for (NKTunableMotorBase motor : motors) {
            this.motors.removeIf(otherMotor -> {
                boolean remove = motor.is(otherMotor);
                motor.setTuningLock(!remove); // release from the lock if removing
                return remove;
            });
        }
    }

    private void updateSmartDash() {
        SmartDashboard.putNumber(this.pName, this.p);
        SmartDashboard.putNumber(this.iName, this.i);
        SmartDashboard.putNumber(this.dName, this.d);        
        SmartDashboard.putNumber(this.fName, this.f);
        SmartDashboard.putNumber(this.iZoneName, this.iZone);
        SmartDashboard.putNumber(this.maxAccumName, this.maxAccum);
    }

    @Override
    public void tune() {
        final double smartdashP = SmartDashboard.getNumber(this.pName, 0);
        final double smartdashI = SmartDashboard.getNumber(this.iName, 0);
        final double smartdashD = SmartDashboard.getNumber(this.dName, 0);
        final double smartdashF = SmartDashboard.getNumber(this.fName, 0);
        final int smartdashIZone = (int) SmartDashboard.getNumber(this.iZoneName, 0);
        final double smartdashMaxAccum = SmartDashboard.getNumber(this.maxAccumName, 0);

        if (smartdashP != this.p) this.setP(smartdashP);
        if (smartdashI != this.i) this.setI(smartdashI);
        if (smartdashD != this.d) this.setD(smartdashD);
        if (smartdashF != this.f) this.setF(smartdashF);

        if (smartdashIZone > 0 && this.iZone != smartdashIZone)
            this.setIZone(smartdashIZone);
        else if (smartdashIZone == 0 && this.iZone != smartdashIZone)
            this.setIZone(Integer.MAX_VALUE);

        if (smartdashMaxAccum > 0 && this.maxAccum != smartdashMaxAccum)
            this.setMaxIntegralAccumulator(smartdashMaxAccum);
        else if (smartdashMaxAccum == 0 && this.maxAccum != smartdashMaxAccum)
            this.setMaxIntegralAccumulator(Double.MAX_VALUE);

        updateSmartDash();
    }

    public void setP(double p) {
        this.p = p;
        this.motors.forEach(motor -> motor.setP(this.slot, p));
    }

    public void setI(double i) {
        this.i = i;
        this.motors.forEach(motor -> motor.setD(this.slot, i));
    }

    public void setD(double d) {
        this.d = d;
        this.motors.forEach(motor -> motor.setD(this.slot, d));
    }

    public void setF(double f) {
        this.f = f;
        this.motors.forEach(motor -> motor.setD(this.slot, f));
    }

    public void setIZone(int iZone) {
        this.iZone = iZone;
        this.motors.forEach(motor -> motor.setIZone(this.slot, iZone));
    }

    public void setMaxIntegralAccumulator(double maxAccumulator) {
        this.maxAccum = maxAccumulator;
        this.motors.forEach(
            motor -> motor.setMaxIntegralAccumulator(this.slot, maxAccumulator)
        );
    }

    public void setOutputRange(double min, double max) {
        this.motors.forEach(motor -> motor.setOutputRange(min, max));
    }

    public void setOutputRange(double max) {
        this.setOutputRange(-max, max);
    };

    public void resetOutputRange() {
        this.motors.forEach(motor -> motor.resetOutputRange());
    }

    @Override
    public boolean isTuningEnabled() {
        return this.tuningEnabled;
    }

    @Override
    public void setTunableName(String name) {
        String slotString = "(Slot: " + this.slot + ")";
        this.name = name;
        this.pName = name + " P Gain" + slotString;
        this.iName = name + " I Gain" + slotString;
        this.dName = name + " D Gain" + slotString;
        this.fName = name + " FV Gain" + slotString;
        this.maxAccumName = name + "Max I Term " + slotString;
    }

    @Override
    public String getTunableName() {
        return this.name;
    }

    @Override
    public void setTuningEnabled(boolean enable) {
        this.tuningEnabled = enable;
    }

    @SuppressWarnings("all")
    private static class NKTunableMotorAlreadyLockedException extends Exception {
        public NKTunableMotorAlreadyLockedException(NKTunableMotorBase motor)
        {
            super(
                "Motor controller (Type: "
                 + ((motor instanceof NKTalonFX)? "TalonFX" :
                    (motor instanceof NKTalonSRX)? "TalonSRX" :
                    (motor instanceof NKVictorSPX)? "VictorSPX" : 
                    (motor instanceof NKSparkMax)? "SparkMAX" : "Unknown")
                 + ", ID Number: " + motor.getID() + ") is already locked."
            );
        }
    }

    @SuppressWarnings("all")
    private static class NKPIDLoopException extends Exception {
        public NKPIDLoopException(String error)
        {
            super("Error in PID loop object: " + error);
        }
    }
}