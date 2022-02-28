package frc.robot.util.tunable;

public interface NKMotorBase {
    public int getID();
    public void setP(int slot, double p);
    public void setI(int slot, double i);
    public void setD(int slot, double d);
    public void setF(int slot, double f);
    public void setIZone(int slot, int iZone);
    public void setMaxIntegralAccumulator(int slot, double maxAccumulator);
    public void setOutputRange(double min, double max);
    public default void setOutputRange(double max) {
        this.setOutputRange(-max, max);
    };
    public void resetOutputRange();
    public default void setPIDF(int slot, double p, double i, double d, double f) {
        setP(slot, p);
        setI(slot, i);
        setD(slot, d);
        setF(slot, f);
    }

    public default boolean is(NKMotorBase other) {
        return (this.getID() == other.getID()) // true if same CAN ID and motor controller class
                && (this.getClass().getName().equals(other.getClass().getName())); 
    }
}