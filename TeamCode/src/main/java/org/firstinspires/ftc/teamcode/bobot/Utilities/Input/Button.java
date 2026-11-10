package org.firstinspires.ftc.teamcode.bobot.Utilities.Input;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Button {
    public static double holdMs = 400, doubleMs = 250;

    public boolean down, rising, falling, click, doubleClick, hold;

    private final ElapsedTime pressTimer = new ElapsedTime();
    private final ElapsedTime gapTimer = new ElapsedTime();
    private boolean pending, secondPress, holdFired;

    public void update(boolean pressed) {
        rising = pressed && !down;
        falling = !pressed && down;
        down = pressed;
        click = false;
        doubleClick = false;
        hold = false;

        if (rising) {
            pressTimer.reset();
            holdFired = false;
            if (pending && gapTimer.milliseconds() < doubleMs) {
                doubleClick = true;
                secondPress = true;
                pending = false;
            }
        }

        if (down && !holdFired && !secondPress && pressTimer.milliseconds() >= holdMs) {
            hold = true;
            holdFired = true;
        }

        if (falling) {
            if (!holdFired && !secondPress) {
                pending = true;
                gapTimer.reset();
            }
            secondPress = false;
        }

        if (pending && gapTimer.milliseconds() >= doubleMs) {
            click = true;
            pending = false;
        }
    }
}