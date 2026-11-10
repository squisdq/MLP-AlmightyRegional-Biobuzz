package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingServo;

@Config
public class TurretController {
    private final CachingServo tServo1, tServo2;

    private double tAngle = 0, tPos = 0.5;

    public TurretController(HardwareMap hMap){
        tServo1 = new CachingServo(hMap, "tServo1");
        tServo1.setPwm(500, 2500);
        tServo1.setThreshold(0.0003);
        tServo2 = new CachingServo(hMap, "tServo2");
        tServo2.setPwm(500, 2500);
        tServo2.setThreshold(0.0003);
    }

    public void setAngle(double tAngle){
        this.tAngle = Range.clip(AngleUnit.normalizeDegrees(tAngle), -175, 175);
    }

    public void update(){
        tPos = (175 - tAngle) / 350.0;

        tServo1.set(tPos);
        tServo2.set(tPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- TURRET CONTROLLER -----");
        telemetry.addData("tPos", tPos);
        telemetry.addData("tAngle", tAngle);
    }
}