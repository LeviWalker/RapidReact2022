package frc.robot.util.math;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

public class DoubleInterpolater {
    private Map<Double, Double> map;
    private LinkedList<Double> keys;
    private boolean sorted = false;

    // results
    public double dx, dy, x0, y0, slope;

    public DoubleInterpolater() {
        this.map = new HashMap<>();
        this.keys = new LinkedList<>();
    }

    public DoubleInterpolater(Map<Double,Double> map) {
        this.map = map;
    }

    public double interpolate(double x) {
        // if the list isn't sorted, sort it
        if (!sorted) {
            keys.clear();
            map.forEach((key, value) -> keys.add(key));
            keys.sort((k1, k2) -> {
                return (int) (k1 - k2);
            });
        }

        double retVal;

        int i = 0;
        for (i = 0; i < keys.size() - 1; i++) {
            if (valueIsBetween(x, keys.get(i), keys.get(i + 1)))
                break;
        }

        x0 = keys.get(i);
        y0 = map.get(keys.get(i));

        if (i == keys.size() - 1) {
            retVal = y0;
            slope = 0;
        } else {
            // change in x
            dx = keys.get(i + 1) - x0;
            // change in y
            dy = map.get(keys.get(i + 1)) - y0;
            slope = dy/dx;

            // point-slope form of a line with point (x0, y0) and slope
            retVal = y0 + (slope * (x - x0));
        }  

        return retVal;
    }

    public DoubleInterpolater add(double key, double value) {
        map.put(key, value);
        sorted = false;
        return this;
    }

    public void addAll(Map<Double, Double> map) {
        map.forEach((k, v) -> this.map.put(k, v));
        sorted = false;
    }

    private static boolean valueIsBetween(double value, double min, double max) {
        return value == Math.min(max, Math.max(value, min));
    }
}
