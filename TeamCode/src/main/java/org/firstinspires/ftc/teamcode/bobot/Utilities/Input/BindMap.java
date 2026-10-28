package org.firstinspires.ftc.teamcode.bobot.Utilities.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BiConsumer;

public class BindMap {
    public static long holdMs = 400, doubleMs = 250;

    public final String name;
    public final PadBinds gamepad1 = new PadBinds(), gamepad2 = new PadBinds();

    public BindMap(String name) {
        this.name = name;
    }

    void update(Gamepad g1, Gamepad g2, long now) {
        gamepad1.each(g1, (bind, down) -> bind.update(down, now));
        gamepad2.each(g2, (bind, down) -> bind.update(down, now));
    }

    void activate(Gamepad g1, Gamepad g2) {
        gamepad1.each(g1, (bind, down) -> bind.sync(down));
        gamepad2.each(g2, (bind, down) -> bind.sync(down));
    }

    void deactivate() {
    }

    private static void run(Runnable r) {
        if (r != null) r.run();
    }

    public class PadBinds {
        public final Bind a = new Bind(), b = new Bind(), x = new Bind(), y = new Bind(),
                leftBumper = new Bind(), rightBumper = new Bind(),
                leftTrigger = new Bind(), rightTrigger = new Bind(),
                dpadUp = new Bind(), dpadDown = new Bind(), dpadLeft = new Bind(), dpadRight = new Bind(),
                leftStickButton = new Bind(), rightStickButton = new Bind(),
                start = new Bind(), back = new Bind();

        void each(Gamepad g, BiConsumer<Bind, Boolean> f) {
            f.accept(a, g.a);
            f.accept(b, g.b);
            f.accept(x, g.x);
            f.accept(y, g.y);
            f.accept(leftBumper, g.left_bumper);
            f.accept(rightBumper, g.right_bumper);
            f.accept(leftTrigger, g.left_trigger > 0.5);
            f.accept(rightTrigger, g.right_trigger > 0.5);
            f.accept(dpadUp, g.dpad_up);
            f.accept(dpadDown, g.dpad_down);
            f.accept(dpadLeft, g.dpad_left);
            f.accept(dpadRight, g.dpad_right);
            f.accept(leftStickButton, g.left_stick_button);
            f.accept(rightStickButton, g.right_stick_button);
            f.accept(start, g.start);
            f.accept(back, g.back);
        }
    }

    public class Bind {
        private Runnable click, doubleClick, hold, held;
        private boolean down, holdFired, consumed;
        private int clicks;
        private long pressedAt, releasedAt;

        public BindMap onClick(Runnable r) {
            click = r;
            return BindMap.this;
        }

        public BindMap onDoubleClick(Runnable r) {
            doubleClick = r;
            return BindMap.this;
        }

        public BindMap onHold(Runnable r) {
            hold = r;
            return BindMap.this;
        }

        public BindMap whileHeld(Runnable r) {
            held = r;
            return BindMap.this;
        }

        void update(boolean pressed, long now) {
            if (pressed) {
                if (!down) {
                    down = true;
                    pressedAt = now;
                    holdFired = false;
                }
                if (consumed) return;
                run(held);
                if (hold != null && !holdFired && now - pressedAt >= holdMs) {
                    holdFired = true;
                    clicks = 0;
                    hold.run();
                }
            } else if (down) {
                down = false;
                if (consumed) {
                    consumed = false;
                } else if (holdFired) {
                    holdFired = false;
                } else if (doubleClick == null) {
                    run(click);
                } else if (++clicks >= 2) {
                    clicks = 0;
                    doubleClick.run();
                } else {
                    releasedAt = now;
                }
            } else if (clicks == 1 && now - releasedAt >= doubleMs) {
                clicks = 0;
                run(click);
            }
        }

        void sync(boolean pressed) {
            down = pressed;
            consumed = pressed;
            holdFired = false;
            clicks = 0;
        }
    }
}
