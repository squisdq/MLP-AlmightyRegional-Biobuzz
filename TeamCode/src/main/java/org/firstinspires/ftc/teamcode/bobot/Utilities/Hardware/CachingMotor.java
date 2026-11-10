package org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachingMotor {
    private final DcMotorEx motor;
    private double lastPower;
    private double threshold = 0.01;

    public CachingMotor(HardwareMap map, String id) {
        motor = map.get(DcMotorEx.class, id);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void set(double power) {
        if (Math.abs(power - lastPower) > threshold || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    public double getPower() {
        return lastPower;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public DcMotorEx get() {
        return motor;
    }
}