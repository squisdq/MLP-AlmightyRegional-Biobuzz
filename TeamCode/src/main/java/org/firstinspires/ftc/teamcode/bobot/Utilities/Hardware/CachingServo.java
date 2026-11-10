package org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CachingServo {
    private final ServoImplEx servo;
    private double lastPosition = -1;
    private double threshold = 0.002;

    public CachingServo(HardwareMap map, String id) {
        servo = map.get(ServoImplEx.class, id);
    }

    public void set(double position) {
        if (Math.abs(position - lastPosition) > threshold) {
            lastPosition = position;
            servo.setPosition(position);
        }
    }

    public void setPwm(double minMicros, double maxMicros) {
        servo.setPwmRange(new PwmControl.PwmRange(minMicros, maxMicros));
        lastPosition = -1;
    }

    public void disable() {
        servo.setPwmDisable();
    }

    public void enable() {
        servo.setPwmEnable();
        lastPosition = -1;
    }

    public double getPosition() {
        return lastPosition;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public ServoImplEx get() {
        return servo;
    }
}