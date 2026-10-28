package org.firstinspires.ftc.teamcode.bobot.Utilities.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class BindMapEx extends BindMap {
    private final List<DoubleSupplier> sources = new ArrayList<>();
    private final List<DoubleConsumer> sinks = new ArrayList<>();
    private DoubleSupplier forward, strafe, turn;

    public BindMapEx(String name) {
        super(name);
    }

    public BindMapEx drive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        return this;
    }

    public BindMapEx axis(DoubleSupplier source, DoubleConsumer sink) {
        sources.add(source);
        sinks.add(sink);
        return this;
    }

    double forward() {
        return forward == null ? 0 : forward.getAsDouble();
    }

    double strafe() {
        return strafe == null ? 0 : strafe.getAsDouble();
    }

    double turn() {
        return turn == null ? 0 : turn.getAsDouble();
    }

    @Override
    void update(Gamepad g1, Gamepad g2, long now) {
        super.update(g1, g2, now);
        for (int i = 0; i < sinks.size(); i++) sinks.get(i).accept(sources.get(i).getAsDouble());
    }

    @Override
    void deactivate() {
        for (DoubleConsumer sink : sinks) sink.accept(0);
    }
}
