package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingServo;

public class PushBackController {
    private final CachingServo armServo;

    private final double upPos = 0.99;
    private final double flowerPos = 0.42;
    private double pos = upPos;

    public PushBackController(HardwareMap hMap){
        armServo = new CachingServo(hMap, "armServo");
        armServo.setDirection(Servo.Direction.FORWARD);
        armServo.setPwm(500, 2500);
    }

    public void initServo(double pos){
        this.pos = pos;
        armServo.set(pos);
    }

    public void setUpPos(){
        pos = upPos;
    }

    public void setFlowerPos(){
        pos = flowerPos;
    }

    public void setPos(double pos){
        this.pos = pos;
    }

    public void update(){
        armServo.set(pos);
    }
}