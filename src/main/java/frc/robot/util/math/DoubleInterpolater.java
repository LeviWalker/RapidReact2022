package frc.robot.util.math;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

public class DoubleInterpolater {
    private Map<Double, Double> map;
    private LinkedList<Double> keys;
    private boolean sorted = false;

    // results
    private double x0, y0, x, y, dx, dy, slope;

    public DoubleInterpolater() {
        this.map = new HashMap<>();
        this.keys = new LinkedList<>();
    }

    public DoubleInterpolater(Map<Double, Double> map) {
        this.map = map;
    }

    public double interpolate(double x) {
        // if the list isn't sorted, sort it
        if (!sorted) this.sort();

        double retVal;

        // let's find the values that surround our x value at index i and i + 1
        int i = 0;
        for (i = 0; i < this.keys.size() - 1; i++) {
            if (valueIsBetween(x, this.keys.get(i), this.keys.get(i + 1)))
                break;
        }

        // save x0 and y0 for our line

        double firstX = this.keys.get(0);

        if (x < firstX) {
            this.x0 = firstX;
            this.y0 = this.map.get(firstX);
            this.x = this.x0;
            this.y = this.y0;
            this.dx = 0;
            this.dy = 0;
            this.slope = 0;
            return this.y;
        } else { 
            this.x0 = this.keys.get(i);
            this.y0 = this.map.get(this.keys.get(i));
        }

        // make sure that we don't exceed our array bounds
        if (i == this.keys.size() - 1) {
            this.x = this.x0;
            this.y = this.y0;
            this.dx = 0;
            this.dy = 0;
            this.slope = 0;
            return this.y;
        } else {
            // ("change in" x) = (delta x) = dx
            this.dx = this.keys.get(i + 1) - x0;
            // ("change in" y) = (delta y) = dy
            this.dy = this.map.get(this.keys.get(i + 1)) - y0;
            // ("change in" y)/("change in" x) = (delta y)/(delta x) = dy/dx
            this.slope = this.dy / this.dx;

            // point-slope form of a line with point (x0, y0) and slope
            this.y = this.y0 + (this.slope * (x - this.x0));
        }  

        // return our result
        this.x = x;
        return this.y;
    }

    public void sort() {
        this.keys.clear(); // make sure we don't have any duplicates
        this.map.forEach((key, value) -> this.keys.add(key)); // add everything back in
        this.keys.sort((k1, k2) -> {
            return (int) (k1 - k2); // sort for least to greatest
        });

        this.sorted = true;
    }

    public Result getResult() {
        return new Result(this.x0, this.y0, this.x, this.y, this.dx, this.dy, this.slope);
    }

    public Result getResult(double x) {
        this.interpolate(x);
        return new Result(this.x0, this.y0, this.x, this.y, this.dx, this.dy, this.slope);
    }

    public DoubleInterpolater add(double key, double value) {
        this.map.put(key, value);
        this.sorted = false;
        return this;
    }

    public void addAll(Map<Double, Double> map) {
        this.map.forEach((k, v) -> this.map.put(k, v));
        this.sorted = false;
    }

    /**
     * Checks to see if a value is between another
     * @param value the value
     * @param min minimum bound
     * @param max maximum bound
     * @return true if value is between the minimum bound and the maximum bound
     */
    private static boolean valueIsBetween(double value, double min, double max) {
        return value == Math.min(max, Math.max(value, min));
    }
    
    public static class Result {
        private double x0, y0, x, y, dx, dy, slope;

        private Result(double x0, double y0, double x, double y, double dx, double dy, double slope) {
            this.x0 = x0;
            this.y0 = y0;
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.slope = slope;
        }
        
        public double getX0() {
            return this.x0;
        }

        public double getY0() {
            return this.y0;
        }

        public double getX() {
            return this.x;
        }

        public double getY() {
            return this.y;
        }

        public double getDeltaX() {
            return this.dx;
        }

        public double getDeltaY() {
            return this.dy;
        }

        public double getSlope() {
            return this.slope;
        }

        @Override
        public String toString() {
            return "x0 = " + this.x0 + ", y0 = " + this.y0 + ", "
                    + "x = " + this.x + ", y = " + this.y + ", "
                    + "dx = " + this.dx + ", dy = " + this.dy + ", "
                    + "slope = " + this.slope;
        }

    }
}
