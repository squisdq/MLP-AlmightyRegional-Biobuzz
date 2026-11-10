package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingEncoder;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingMotor;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Hardware.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Math.MLPLut;
import org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers.EnhancedPowerController;

@Config
public class ShooterController {
    private final CachingMotor sMotor1, sMotor2;
    private final CachingEncoder sEncoder1, sEncoder2;
    private static String hubId = "Control Hub";

    private EnhancedPowerController controller;
    private CachingVoltageSensor battery;

    private MLPLut sLut;

    public static double kP = 0.001, kS = 0.04, kV = 0.00039, threshold = 100, kHysteresis = 0.5, alpha = 1.00;

    private double target = 0, current = 0;

    public ShooterController(HardwareMap hMap){
        sMotor1 = new CachingMotor(hMap, "sMotor1");
        sMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        sMotor2 = new CachingMotor(hMap, "sMotor2");
        sMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        sEncoder1 = new CachingEncoder(sMotor1.get());
        sEncoder2 = new CachingEncoder(sMotor2.get());

        battery = new CachingVoltageSensor(hMap, hubId);
        controller = new EnhancedPowerController(kP, kS, kV, threshold, kHysteresis, alpha);

        sLut = MLPLut.builder()
                .add(36, 660)
                .add(40, 670)
                .add(45, 690)
                .add(72, 780)
                .allowExtrapolation(true)
                .build();
    }

    public boolean fAtTarget(){
        return (Math.abs(target - current) <= 40);
    }

    public void setTarget(double fTarget){
        this.target = fTarget;
    }

    public double getSLut(double distance){
        return sLut.get(distance);
    }

    public void update(){
        sEncoder2.update();
        sEncoder1.update();

        current = sEncoder1.getVelocity();

        controller.set(kP, kS, kV, threshold, kHysteresis, alpha);
        double voltage = battery.getVoltage();
        double fPower = controller.calculate(current, target, voltage);

        if(target == 0){
            sMotor1.set(0);
            sMotor2.set(0);
        } else{
            sMotor1.set(fPower);
            sMotor2.set(fPower);
        }
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: fAtTarget", fAtTarget());
        telemetry.addData("SC: fCurrent", current);
        telemetry.addData("SC: fTarget", target);
    }
}