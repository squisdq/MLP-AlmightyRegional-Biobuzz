package org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers;

public class VelocityController {
    private double kP;
    private double kS;
    private double kV1;
    private double kV2;
    private final double speed1;
    private final double speed2;

    public VelocityController(double kP, double kS, double kV1, double kV2, double speed1, double speed2) {
        this.kP = kP;
        this.kS = kS;
        this.kV1 = kV1;
        this.kV2 = kV2;
        this.speed1 = speed1;
        this.speed2 = speed2;
    }

    public void setCoefficients(double kP, double kS, double kV1, double kV2) {
        this.kP = kP;
        this.kS = kS;
        this.kV1 = kV1;
        this.kV2 = kV2;
    }

    public double calculate(double current, double target) {
        double feedforward = (kV1 + (target - speed1) / (speed2 - speed1) * (kV2 - kV1)) * target;
        double feedback = kP * (target - current);


        return feedforward + feedback + kS;
    }
}