package org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachingCurrentSensor {
    private final DcMotorEx motor;
    private double current;
    private long lastTime;
    private long interval = 50000000L;

    public CachingCurrentSensor(HardwareMap map, String id) {
        motor = map.get(DcMotorEx.class, id);
    }

    public double getCurrent() {
        if (System.nanoTime() - lastTime > interval) {
            current = motor.getCurrent(CurrentUnit.AMPS);
            lastTime = System.nanoTime();
        }
        return current;
    }

    public void setInterval(long millis) {
        interval = millis * 1000000L;
    }
}