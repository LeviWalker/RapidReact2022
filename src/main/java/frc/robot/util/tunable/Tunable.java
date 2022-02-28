package frc.robot.util.tunable;

public interface Tunable {
    public void tune();
    public void setTuningEnabled(boolean enable);
    public default void enableTuning(String name) {
        setTunableName(name);
        setTuningEnabled(true);
    }
    public default void enableTuning(String name, boolean enable) {
        setTunableName(name);
        setTuningEnabled(enable);
    }
    public default void disableTuning() {
        setTuningEnabled(false);
    }
    public boolean isTuningEnabled();
    public void setTunableName(String name);
    public String getTunableName();
}