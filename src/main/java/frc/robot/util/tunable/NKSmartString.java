package frc.robot.util.tunable;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class NKSmartString extends NKTunableString {

    private Supplier<String> getter;
    private Consumer<String> setter;

    public NKSmartString(String name, Supplier<String> getter, Consumer<String> setter, String defaultValue) {
        super(name, defaultValue);
        this.getter = getter;
        this.setter = setter;
    }

    public NKSmartString(String name, Supplier<String> getter, Consumer<String> setter) {
        this(name, getter, setter, kDefaultValue);
    }

    public NKSmartString(String name, Supplier<String> getter, String defaultValue) {
        this(name, getter, null, defaultValue);
    }

    public NKSmartString(String name, Consumer<String> setter, String defaultValue) {
        this(name, null, setter, defaultValue);
    }

    public NKSmartString(String name, Supplier<String> getter) {
        this(name, getter, null, kDefaultValue);
    }

    public NKSmartString(String name, Consumer<String> setter) {
        this(name, null, setter, kDefaultValue);
    }

    @Override
    protected void updateValue(String newValue) {
        if (this.getter != null && this.getter.get() != newValue && this.setter != null)
            this.setter.accept(newValue);
        else if (this.setter != null)
            this.setter.accept(newValue);
    }
    
}
