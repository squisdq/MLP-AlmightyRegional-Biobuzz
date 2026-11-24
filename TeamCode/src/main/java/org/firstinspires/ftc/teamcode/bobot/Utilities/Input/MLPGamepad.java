package org.firstinspires.ftc.teamcode.bobot.Utilities.Input;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MLPGamepad {
    public static double triggerThreshold = 0.5, slewK = 100, deadzone = 0.05;

    public final Gamepad gamepad;
    public final Button a = new Button(), b = new Button(), x = new Button(), y = new Button();
    public final Button leftBumper = new Button(), rightBumper = new Button();
    public final Button leftTrigger = new Button(), rightTrigger = new Button();
    public final Button dpadUp = new Button(), dpadDown = new Button(), dpadLeft = new Button(), dpadRight = new Button();
    public final Button leftStick = new Button(), rightStick = new Button();
    public final Button start = new Button(), back = new Button();

    public double leftX, leftY, rightX, rightY;

    private final ElapsedTime slewTimer = new ElapsedTime();

    public MLPGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        double step = slewK * slewTimer.seconds();
        slewTimer.reset();

//        leftX += Math.max(-step, Math.min(step, gamepad.left_stick_x - leftX));
//        leftY += Math.max(-step, Math.min(step, gamepad.left_stick_y - leftY));
//        rightX += Math.max(-step, Math.min(step, gamepad.right_stick_x - rightX));
//        rightY += Math.max(-step, Math.min(step, gamepad.right_stick_y - rightY));

        leftX += Math.max(-step, Math.min(step, (Math.abs(gamepad.left_stick_x) < deadzone ? 0 : gamepad.left_stick_x * Math.abs(gamepad.left_stick_x)) - leftX));
        leftY += Math.max(-step, Math.min(step, (Math.abs(gamepad.left_stick_y) < deadzone ? 0 : gamepad.left_stick_y * Math.abs(gamepad.left_stick_y)) - leftY));
        rightX += Math.max(-step, Math.min(step, (Math.abs(gamepad.right_stick_x) < deadzone ? 0 : gamepad.right_stick_x * Math.abs(gamepad.right_stick_x)) - rightX));
        rightY += Math.max(-step, Math.min(step, (Math.abs(gamepad.right_stick_y) < deadzone ? 0 : gamepad.right_stick_y * Math.abs(gamepad.right_stick_y)) - rightY));

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        leftTrigger.update(gamepad.left_trigger > triggerThreshold);
        rightTrigger.update(gamepad.right_trigger > triggerThreshold);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        leftStick.update(gamepad.left_stick_button);
        rightStick.update(gamepad.right_stick_button);
        start.update(gamepad.start);
        back.update(gamepad.back);
    }
}