package org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers;

public class PSVController {
    private double kP, kS, kV;

    public PSVController(double kP, double kS, double kV) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
    }

    public void setPSV(double kP, double kS, double kV) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
    }

    public double calculate(double current, double target, double voltage) {
        double output = kV * target + kP * (target - current) + (target != 0.0 ? Math.signum(target) * kS : 0.0);

        return output * 12.0 / voltage;
    }
}