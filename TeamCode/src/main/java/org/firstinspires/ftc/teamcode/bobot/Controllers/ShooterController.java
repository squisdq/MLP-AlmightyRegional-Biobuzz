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

    public static double kP = 0.005, kS = 0.07, kV = 0.00038, threshold = 170, kHysteresis = 0.5, alpha = 1.00;

    private double target = 0, current = 0;
    private boolean idle;

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
                .add(33, 1320)
                .add(40, 1325)
                .add(45,1330)
                .add(50, 1370)
                .add(60, 1420)
                .add(65, 1450)
                .allowExtrapolation(true)
                .build();
    }

    public boolean atTarget(){
        return (Math.abs(target - current) <= 40);
    }

    public void setTarget(double fTarget){
        idle = false;
        this.target = fTarget;
    }

    public void setIdle(double idleTarget){
        idle = true;
        this.target = idleTarget;
    }

    public double getSLut(double distance){
        return sLut.get(distance);
    }

    public void update(){
        sEncoder2.update();

        current = sEncoder2.getVelocity();

        double voltage = battery.getVoltage();
        double power;

        if(target == 0)
            power = 0;
        else if(idle)
            power = current > target ? 0 : (kS + kV * target) * 12.0 / voltage;
        else{
            controller.set(kP, kS, kV, threshold, kHysteresis, alpha);
            power = controller.calculate(current, target, voltage);
        }

        sMotor1.set(power);
        sMotor2.set(power);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: AtTarget", atTarget());
        telemetry.addData("SC: Current", current);
        telemetry.addData("SC: Target", target);
        telemetry.addData("SC: Idle", idle);
    }
}