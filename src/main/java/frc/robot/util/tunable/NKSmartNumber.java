package frc.robot.util.tunable;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class NKSmartNumber extends NKTunableNumber {

    private Supplier<Double> getter;
    private Consumer<Double> setter;

    public NKSmartNumber(String name, Supplier<Double> getter, Consumer<Double> setter, double defaultValue) {
        super(name, defaultValue);
        this.getter = getter;
        this.setter = setter;
    }

    public NKSmartNumber(String name, Supplier<Double> getter, Consumer<Double> setter) {
        this(name, getter, setter, kDefaultValue);
    }

    public NKSmartNumber(String name, Supplier<Double> getter, double defaultValue) {
        this(name, getter, null, defaultValue);
    }

    public NKSmartNumber(String name, Consumer<Double> setter, double defaultValue) {
        this(name, null, setter, defaultValue);
    }

    public NKSmartNumber(String name, Supplier<Double> getter) {
        this(name, getter, null);
    }

    public NKSmartNumber(String name, Consumer<Double> setter) {
        this(name, null, setter);
    }

    @Override
    protected void updateValue(double newValue) {
        if (this.getter != null && this.getter.get() != newValue && this.setter != null)
            this.setter.accept(newValue);
        else if (this.setter != null)
            this.setter.accept(newValue);
    }
}