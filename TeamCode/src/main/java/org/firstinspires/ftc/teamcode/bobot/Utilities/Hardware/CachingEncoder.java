package org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CachingEncoder {
    private final DcMotorEx motor;
    private boolean freshPosition;
    private boolean freshVelocity;
    private int position;
    private double velocity;
    private int offset;
    private int sign = 1;

    public CachingEncoder(DcMotorEx motor) {
        this.motor = motor;
    }

    public void update() {
        freshPosition = false;
        freshVelocity = false;
    }

    public int getPosition() {
        if (!freshPosition) {
            position = motor.getCurrentPosition();
            freshPosition = true;
        }
        return sign * (position - offset);
    }

    public double getVelocity() {
        if (!freshVelocity) {
            velocity = motor.getVelocity();
            freshVelocity = true;
        }
        return sign * velocity;
    }

    public void reset() {
        offset = motor.getCurrentPosition();
        position = offset;
        freshPosition = true;
    }

    public void setReversed(boolean reversed) {
        sign = reversed ? -1 : 1;
    }
}