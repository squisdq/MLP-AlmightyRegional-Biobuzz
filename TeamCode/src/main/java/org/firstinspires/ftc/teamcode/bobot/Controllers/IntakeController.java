package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingMotor;

public class IntakeController {
    private CachingMotor iMotor1, iMotor2;

    private double iPower;
    public IntakeController(HardwareMap hMap){
        iMotor1 = new CachingMotor(hMap, "iMotor1");
        iMotor2 = new CachingMotor(hMap, "iMotor2");
        iMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        iMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        iPower = 0;
    }

    public void intake(double power){
        this.iPower = power;
    }

    public void update(){
        iMotor1.set(iPower);
        iMotor2.set(iPower);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- INTAKE CONTROLLER -----");
        telemetry.addData("IC: power: ", iPower);
    }
}