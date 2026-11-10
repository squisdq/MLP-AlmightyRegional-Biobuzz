package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingServo;

public class BlockerController {
    private final CachingServo bServo;

    private final double blockPos = 0.52;
    private final double supplyPos = 0.35;
    private double bPos = blockPos;

    public BlockerController(HardwareMap hMap){
        bServo = new CachingServo(hMap, "bServo");
        bServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setBlockPos(){
        bPos = blockPos;
    }

    public void setSupplyPos(){
        bPos = supplyPos;
    }

    public void setPos(double pos){
        bPos = pos;
    }

    public void update(){
        bServo.set(bPos);
    }
}