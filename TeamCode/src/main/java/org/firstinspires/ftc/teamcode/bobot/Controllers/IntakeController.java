package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class IntakeController {
    private Motor iMotor1, iMotor2;
    private ServoEx rServo;

    private double iPower;
    private double rPos;
    private final double upPos = 0.35;
    private final double flowerPos = 1;

    private rStates state;

    public IntakeController(HardwareMap hMap){
        iMotor1 = new Motor(hMap, "iMotor1");
        iMotor2 = new Motor(hMap, "iMotor2");
        iMotor1.setRunMode(Motor.RunMode.VelocityControl);
        iMotor2.setRunMode(Motor.RunMode.VelocityControl);
//        iMotor1.setInverted(true);
        iMotor2.setInverted(true);

        rServo = new ServoEx(hMap, "armServo");
        rServo.setInverted(true);
        rServo.setPwm(new PwmControl.PwmRange(500, 2500));

        iPower = 0;
        rPos = upPos;
        state = rStates.up;
    }

    public void initAServo(){
        rServo.set(rPos);}

    public void intake(double power){
        this.iPower = power;
    }

    public void setPos(double rPos){
        this.rPos = rPos;
    }

    public void setState(rStates state){
        this.state = state;

        if(state == rStates.up)
            setPos(upPos);
        else
            setPos(flowerPos);
    }

    public void switchState(){
        if(state == rStates.up)
            setState(rStates.flower);
        else
            setState(rStates.up);
    }

    public void update(){
        iMotor1.set(iPower);
        iMotor2.set(iPower);

        rServo.set(rPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- INTAKE CONTROLLER -----");
        telemetry.addData("IC: power: ", iPower);
        telemetry.addData("IC: rPos: ", rPos);
        telemetry.addData("IC: state: ", state);
    }

    public enum rStates {
        up, flower
    }
}