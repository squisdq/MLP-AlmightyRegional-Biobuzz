package org.firstinspires.ftc.teamcode.bobot.Utilities.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class MLPGamepad {
    public final Axes gamepad1, gamepad2;

    private final Gamepad g1, g2;
    private final Map<String, BindMap> maps = new HashMap<>();
    private BindMap active, pending;
    private BindMapEx ex;

    public MLPGamepad(Gamepad g1, Gamepad g2) {
        this.g1 = g1;
        this.g2 = g2;
        gamepad1 = new Axes(g1);
        gamepad2 = new Axes(g2);
    }

    public MLPGamepad register(BindMap map) {
        maps.put(map.name, map);
        return this;
    }

    public MLPGamepad setBindMap(BindMap map) {
        register(map);
        pending = map;
        return this;
    }

    public void switchTo(String name) {
        if (maps.containsKey(name)) pending = maps.get(name);
    }

    public BindMap getBindMap() {
        return active;
    }

    public double forward() {
        return ex == null ? 0 : ex.forward();
    }

    public double strafe() {
        return ex == null ? 0 : ex.strafe();
    }

    public double turn() {
        return ex == null ? 0 : ex.turn();
    }

    public void update() {
        if (pending != null) {
            if (active != null) active.deactivate();
            active = pending;
            pending = null;
            ex = active instanceof BindMapEx ? (BindMapEx) active : null;
            active.activate(g1, g2);
        }
        if (active != null) active.update(g1, g2, System.nanoTime() / 1000000);
    }

    private static DoubleSupplier slew(DoubleSupplier source, double rise, double fall) {
        double[] last = {0};
        long[] lastNanos = {-1};
        return () -> {
            long now = System.nanoTime();
            double dt = lastNanos[0] < 0 ? 0 : (now - lastNanos[0]) / 1e9;
            lastNanos[0] = now;
            if (dt > 0.25) {
                last[0] = 0;
                dt = 0;
            }
            double delta = source.getAsDouble() - last[0];
            double step = (last[0] * delta < 0 ? fall : rise) * dt;
            last[0] += Math.max(-step, Math.min(step, delta));
            return last[0];
        };
    }

    public static class Axes {
        private final Gamepad g;

        Axes(Gamepad g) {
            this.g = g;
        }

        public DoubleSupplier leftX() { return () -> g.left_stick_x; }
        public DoubleSupplier leftY() { return () -> -g.left_stick_y; }
        public DoubleSupplier rightX() { return () -> g.right_stick_x; }
        public DoubleSupplier rightY() { return () -> -g.right_stick_y; }
        public DoubleSupplier leftTrigger() { return () -> g.left_trigger; }
        public DoubleSupplier rightTrigger() { return () -> g.right_trigger; }
        public DoubleSupplier triggers() { return () -> g.right_trigger - g.left_trigger; }

        public DoubleSupplier leftX(double rise, double fall) { return slew(leftX(), rise, fall); }
        public DoubleSupplier leftY(double rise, double fall) { return slew(leftY(), rise, fall); }
        public DoubleSupplier rightX(double rise, double fall) { return slew(rightX(), rise, fall); }
        public DoubleSupplier rightY(double rise, double fall) { return slew(rightY(), rise, fall); }
        public DoubleSupplier leftTrigger(double rise, double fall) { return slew(leftTrigger(), rise, fall); }
        public DoubleSupplier rightTrigger(double rise, double fall) { return slew(rightTrigger(), rise, fall); }
        public DoubleSupplier triggers(double rise, double fall) { return slew(triggers(), rise, fall); }
    }
}
