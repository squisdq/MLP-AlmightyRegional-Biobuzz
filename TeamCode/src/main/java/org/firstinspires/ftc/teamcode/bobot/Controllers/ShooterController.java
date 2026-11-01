package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Math.MLPLut;
import org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers.PSVController;

@Config
public class ShooterController {
    private final Motor sMotor1, sMotor2;
    private final ServoEx bServo;

    private PSVController controller;
    private VoltageSensor battery;

    private MLPLut sLut;

    public static double fP = 0.003, fS = 0.09, fV = 0.00046;

    private double target = 0, current = 0;

    private double bPos = 0;
    private bStates state;

    private final double bBlockPos = 0.2, bSupplyPos = 0.36;

    public ShooterController(HardwareMap hMap){
        sMotor1 = new Motor(hMap, "sMotor");
//        fMotor.setInverted(true);
        sMotor2 = new Motor(hMap, "sMotor2");
        sMotor2.setInverted(true);

        bServo = new ServoEx(hMap, "bServo");
        bServo.setInverted(true);

        battery = hMap.voltageSensor.iterator().next();
        controller = new PSVController(fP, fS, fV);

        sLut = MLPLut.builder()
                .add(36, 660)
                .add(40, 670)
                .add(45, 690)
                .add(72, 780)
                .allowExtrapolation(true)
                .build();

        state = bStates.block;
    }

    public void setBPos(double bPos){
        this.bPos = bPos;
    }

    public void initBServo(){
        bServo.set(bPos);
        state = bStates.block;
    }

    public boolean fAtTarget(){
        return (Math.abs(target - current) <= 40);
    }

    public void setState(bStates state){
        this.state = state;

        if(state == bStates.block)
            setBPos(bBlockPos);
        else
            setBPos(bSupplyPos);
    }

    public void setFTarget(double fTarget){
        this.target = fTarget;
    }


    public double getSLut(double distance){
        return sLut.get(distance);
    }

    public void update(){
        current =  sMotor1.getCorrectedVelocity();

        controller.setPSV(fP, fS, fV);
        double voltage = battery.getVoltage();
        double fPower = controller.calculate(current, target, voltage);

        if(target == 0){
            sMotor1.set(0);
            sMotor2.set(0);
        } else{
            sMotor1.set(fPower);
            sMotor2.set(fPower);
        }

        bServo.set(bPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: fAtTarget", fAtTarget());
        telemetry.addData("SC: fCurrent", current);
        telemetry.addData("SC: fTarget", target);
        telemetry.addData("bPos", bPos);
    }

    public enum bStates {
        block, supply
    }
}