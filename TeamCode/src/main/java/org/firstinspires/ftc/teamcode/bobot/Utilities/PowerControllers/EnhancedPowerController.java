package org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Math.LowPassFilter;

public class EnhancedPowerController {
    public enum State { spinUp, hold }

    private final PSVController psv = new PSVController(0, 0, 0);
    private final LowPassFilter filter = new LowPassFilter(1);
    private double threshold, kHyst;
    private State state = State.spinUp;

    public EnhancedPowerController(double kP, double kS, double kV, double threshold, double kHyst, double alpha) {
        set(kP, kS, kV, threshold, kHyst, alpha);
    }

    public void set(double kP, double kS, double kV, double threshold, double kHyst, double alpha) {
        psv.setPSV(kP, kS, kV);
        filter.setAlpha(alpha);
        this.threshold = threshold;
        this.kHyst = kHyst;
    }

    public State getState() {
        return state;
    }

    public double calculate(double current, double target, double voltage) {
        double velocity = filter.calculate(current);
        double error = target - velocity;

        if (target == 0) return 0.0;

        state = error > (state == State.spinUp ? threshold * kHyst : threshold) ? State.spinUp : State.hold;
        return state == State.spinUp ? 1.0 : Math.max(-1.0, Math.min(1.0, psv.calculate(velocity, target, voltage)));
    }
}