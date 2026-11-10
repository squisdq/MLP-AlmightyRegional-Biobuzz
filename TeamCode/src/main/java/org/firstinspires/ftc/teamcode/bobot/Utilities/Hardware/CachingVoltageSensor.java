package org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class CachingVoltageSensor {
    private final VoltageSensor sensor;
    private double voltage = 12;
    private long lastTime;
    private long interval = 200000000L;

    public CachingVoltageSensor(HardwareMap map, String id) {
        sensor = map.voltageSensor.get(id);
    }

    public double getVoltage() {
        if (System.nanoTime() - lastTime > interval) {
            voltage = sensor.getVoltage();
            lastTime = System.nanoTime();
        }
        return voltage;
    }

    public void setInterval(long millis) {
        interval = millis * 1000000L;
    }
}