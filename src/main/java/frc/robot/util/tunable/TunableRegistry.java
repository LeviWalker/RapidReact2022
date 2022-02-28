package frc.robot.util.tunable;

import java.util.ArrayList;

public class TunableRegistry {

    private static final TunableRegistry instance;
    private ArrayList<Tunable> tunables;

    static {
        instance = new TunableRegistry();
    }

    private TunableRegistry() {
        tunables = new ArrayList<>();
    }

    private void add(Tunable tunable) {
        tunables.add(tunable);
    }

    public static void register(Tunable tunable) {
        instance.add(tunable);
    }

    public static final TunableRegistry getInstance() {
        return instance;
    }

    public void run() {
        instance.tunables.forEach(tunable -> {
            if (tunable.isTuningEnabled()) tunable.tune();
        });
    }
}