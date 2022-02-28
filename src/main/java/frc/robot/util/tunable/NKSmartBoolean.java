package frc.robot.util.tunable;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class NKSmartBoolean extends NKTunableBoolean {

    private Supplier<Boolean> getter;
    private Consumer<Boolean> setter;

    public NKSmartBoolean(String name, Supplier<Boolean> getter, Consumer<Boolean> setter, boolean defaultValue) {
        super(name, defaultValue);
        this.getter = getter;
        this.setter = setter;
    }

    public NKSmartBoolean(String name, Supplier<Boolean> getter, Consumer<Boolean> setter) {
        this(name, getter, setter, kDefaultValue);
    }

    public NKSmartBoolean(String name, Supplier<Boolean> getter, boolean defaultValue) {
        this(name, getter, null, defaultValue);
    }

    public NKSmartBoolean(String name, Consumer<Boolean> setter, boolean defaultValue) {
        this(name, null, setter, defaultValue);
    }

    public NKSmartBoolean(String name, Supplier<Boolean> getter) {
        this(name, getter, null);
    }

    public NKSmartBoolean(String name, Consumer<Boolean> setter) {
        this(name, null, setter);
    }

    @Override
    protected void updateValue(boolean newValue) {
        if (this.getter != null && this.getter.get() != newValue && this.setter != null)
            this.setter.accept(newValue);
        else if (this.setter != null)
            this.setter.accept(newValue);
    }
    
}
