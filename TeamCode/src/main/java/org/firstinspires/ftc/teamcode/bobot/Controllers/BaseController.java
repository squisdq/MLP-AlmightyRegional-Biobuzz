package org.firstinspires.ftc.teamcode.bobot.Controllers;

import static com.pedropathing.utils.Angle.normalizeSigned;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

@Config
public class BaseController {
    public static double kP = 1.5, kI = 0, kD = 0.08, kF = 0.00, slowModeK = 0.4, tolerance = 0;

    private final Follower follower;

    private final PIDFController pid = new PIDFController(kP, kI, kD, kF);
    private boolean slowMode, fieldCentric, holdHeading = true;
    private double targetHeading;

    public BaseController(HardwareMap hMap, boolean fieldCentric) {
        follower = Constants.create(hMap);
        this.fieldCentric = fieldCentric;
    }

    public void setPose(Pose pose){
        follower.setPose(pose);
    }

    public Follower getFollower() {
        return follower;
    }

    public void update() {
        follower.update();
    }

    public void setFieldCentric(boolean fieldCentric){
        this.fieldCentric = fieldCentric;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public void toggleHeadingHold() {
        holdHeading = !holdHeading;
    }

    public void turnTo(double degrees) {
        holdHeading = true;
        targetHeading = normalizeSigned(Math.toRadians(degrees));
    }

    public Command turnCommand(double degrees) {
        return Command.build()
                .setStart(() -> turnTo(degrees))
                .setExecute(() -> drive(0, 0, 0))
                .setDone(() -> Math.abs(Math.toDegrees(normalizeSigned(targetHeading - follower.pose().heading()))) < tolerance)
                .setEnd(endCondition -> follower.manual(0, 0, 0))
                .requiring(follower);
    }

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return Command.build()
                .setStart(() -> targetHeading = follower.pose().heading())
                .setExecute(() -> drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble()))
                .setDone(() -> false)
                .setEnd(endCondition -> follower.manual(0, 0, 0))
                .requiring(follower);
    }

    public void drive(double forward, double strafe, double turn) {
        pid.setPIDF(kP, kI, kD, kF);

        double heading = follower.pose().heading();
        double k = slowMode ? slowModeK : 1;

        if (!holdHeading || Math.abs(turn) > 0.05) {
            targetHeading = heading;
            turn *= k;
        } else {
            double error = normalizeSigned(targetHeading - heading);
            turn = pid.calculate(follower.twist().omega, targetHeading);
        }

        if (fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward * k, strafe * k, turn, heading));
        else
            follower.manual(forward * k, strafe * k, turn);
    }

    public void debug(MultipleTelemetry telemetry) {
        double heading = follower.pose().heading();

        telemetry.addData("heading", Math.toDegrees(heading));
        telemetry.addData("target", Math.toDegrees(targetHeading));
        telemetry.addData("error", Math.toDegrees(normalizeSigned(targetHeading - heading)));
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.addData("slowMode", slowMode);
        telemetry.addData("holdHeading", holdHeading);
    }
}