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
    private final Motor fMotor, cMotor;
    private final ServoEx bServo;

    private PSVController fController, cController;
    private VoltageSensor battery;

    private MLPLut sLut, hLut;

    public static double fP = 0.003, fS = 0.07, fV = 0.0004,
            cP = 0.0003, cS = 0.03, cV = 0.00036;

    private double fCurrent = 0, cCurrent = 0, fTarget = 0, cTarget = 0;

    private double bPos = 0;
    private bStates state;

    private final double bBlockPos = 0.2, bSupplyPos = 0.36;

    public ShooterController(HardwareMap hMap){
        fMotor = new Motor(hMap, "sMotor");
        fMotor.setInverted(true);
        cMotor = new Motor(hMap, "csMotor");
        cMotor.setInverted(true);
        bServo = new ServoEx(hMap, "bServo");
        bServo.setInverted(true);

        battery = hMap.voltageSensor.iterator().next();
        fController = new PSVController(fP, fS, fV);
        cController = new PSVController(cP, cS, cV);

        sLut = MLPLut.builder()
                .add(36, 660)
                .add(40, 670)
                .add(45, 690)
                .add(72, 780)
                .allowExtrapolation(true)
                .build();

        hLut = MLPLut.builder()
                .add(36, 1.7)
                .add(40, 1.7)
                .add(72, 1.7)
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
        return (Math.abs(fTarget - fCurrent) <= 40);
    }

    public boolean cAtTarget(){
        return (Math.abs(cTarget - cCurrent) <= 40);
    }

    public void setState(bStates state){
        this.state = state;

        if(state == bStates.block)
            setBPos(bBlockPos);
        else
            setBPos(bSupplyPos);
    }

    public void setFTarget(double fTarget){
        this.fTarget = fTarget;
    }

    public void setCTarget(double cTarget){
        this.cTarget = cTarget;
    }

    public double getSLut(double distance){
        return sLut.get(distance);
    }

    public double getHLut(double distance){
        return hLut.get(distance);
    }

    public void update(){
        fCurrent =  fMotor.getCorrectedVelocity();
        cCurrent = cMotor.getCorrectedVelocity();

        fController.setPSV(fP, fS, fV);
        cController.setPSV(cP, cS, cV);
        double voltage = battery.getVoltage();
        double fPower = fController.calculate(fCurrent, fTarget, voltage);
        double cPower = cController.calculate(cCurrent, cTarget, voltage);

        if(fTarget == 0){
            fMotor.set(0);
        } else{
            fMotor.set(fPower);
        }

        if(cTarget == 0){
            cMotor.set(0);
        } else{
            cMotor.set(cPower);
        }

        bServo.set(bPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: cAtTarget", cAtTarget());
        telemetry.addData("SC: cCurrent", cCurrent);
        telemetry.addData("SC: cTarget", cTarget);
        telemetry.addData("SC: fAtTarget", fAtTarget());
        telemetry.addData("SC: fCurrent", fCurrent);
        telemetry.addData("SC: fTarget", fTarget);
        telemetry.addData("bPos", bPos);
    }

    public enum bStates {
        block, supply
    }
}