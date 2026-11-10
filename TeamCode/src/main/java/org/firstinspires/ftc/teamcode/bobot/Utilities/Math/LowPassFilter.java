package org.firstinspires.ftc.teamcode.bobot.Utilities.Math;

public class LowPassFilter {
    private double alpha;
    private double value;

    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double calculate(double input) {
        value += alpha * (input - value);
        return value;
    }
}