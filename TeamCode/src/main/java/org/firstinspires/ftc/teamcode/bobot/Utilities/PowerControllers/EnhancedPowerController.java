package org.firstinspires.ftc.teamcode.bobot.Utilities.PowerControllers;

public class EnhancedPowerController {
    private double kS, kV, kP, kR, kB;
    public EnhancedPowerController(double kS, double kV, double kP, double kR, double kB){
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kR = kR;
        this.kB = kB;
    }

    public void setCoefficients(double kS, double kV, double kP, double kR, double kB){
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kR = kR;
        this.kB = kB;
    }
}
