package frc.robot.util.tunable;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>This is a implementation of a feedforward contoller that is designed
 * to be easily tunable from SmartDashboard or ShuffleBoard for faster tuning.
 * <p>NOTE: It is imperative that you run the
 * {@link edu.wpi.first.wpilibj2.command.CommandSceduler#run()} method in your
 * {@code robotPeriodic()} method for this to work. If you only want to have it in tele-op,
 * you could run the CommandScheduler in {@code teleopPeriodic()} (and the same for
 * autonomous), but you really should run the CommandScheduler from {@code robotPeriodic()}.
 */
public class TunableFeedForward extends SimpleMotorFeedforward implements Tunable {

    private static final String DefaultName = "FF Tuning";
    private String name, oName, sName, vName, aName;
    private double s, v, a;
    private boolean tuning;

    public TunableFeedForward(String name, double kS, double kV, double kA) {
        super(kS, kV, kA);
        this.setTunableName(name);
        this.tuning = false;
        TunableRegistry.register(this);
    }

    public TunableFeedForward(double kS, double kV, double kA) {
        this(DefaultName, kS, kV, kA);
    }

    public TunableFeedForward(String name, double kS, double kV) {
        this(name, kS, kV, 0);
    }

    public TunableFeedForward(double kS, double kV) {
        this(DefaultName, kS, kV, 0);
    }

    public void setTuningEnabled(boolean enable) {
        this.tuning = enable;
    }

    public void enableTuning(String name) {
        this.name = name;
        setTuningEnabled(true);
    }

    public void setS(double s) {
        this.s = s;
    }

    public void setV(double v) {
        this.v = v;
    }

    public void setA(double a) {
        this.a = a;
    }

    public double getS() {
        return this.s;
    }

    public double getV() {
        return this.v;
    }

    public double getA() {
        return this.a;
    }

    @Override
    public double calculate(double velocity, double acceleration) {
        double output = (this.s * Math.signum(velocity)) + (this.v * velocity) + (this.a * acceleration);
        if (tuning) SmartDashboard.putNumber(oName, output);
        return output;
    }

    @Override
    public double calculate(double velocity) {
        return this.calculate(velocity, 0);
    }

    @Override
    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - this.s - acceleration * this.a) / this.v;
    }

    @Override
    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume min velocity is negative, sGain flips sign
        return (-maxVoltage + this.s - acceleration * this.a) / this.v;
    }

    @Override
    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - this.s * Math.signum(velocity) - velocity * this.v) / this.a;
    }

    @Override
    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return this.maxAchievableAcceleration(-maxVoltage, velocity);
    }

    private void updateFF() {
        SmartDashboard.putNumber(this.sName, this.getS());
        SmartDashboard.putNumber(this.vName, this.getV());
        SmartDashboard.putNumber(this.aName, this.getA());
    }

    @Override
    public void tune() {
        final double s = SmartDashboard.getNumber(this.sName, 0);
        final double v = SmartDashboard.getNumber(this.vName, 0);
        final double a = SmartDashboard.getNumber(this.aName, 0);

        if ((s != this.getS())) this.setS(s);
        if ((v != this.getV())) this.setV(v);
        if ((a != this.getA())) this.setA(a);

        updateFF();
    }

    @Override
    public boolean isTuningEnabled() {
        return tuning;
    }

    @Override
    public void setTunableName(String name) {
        this.name = name;
        this.sName = name + " S Gain";
        this.vName = name + " V Gain";
        this.aName = name + " A Gain";
        this.oName = name + " Output";
    }

    @Override
    public String getTunableName() {
        return name;
    }

}