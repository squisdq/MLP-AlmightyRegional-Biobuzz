package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Base.OpModeWrapper;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@TeleOp(name = "OffsetTest")
public class LocalizationTest extends OpModeWrapper {
    private Follower follower;
    private double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;

    public void onInit() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        super.init();
    }

    public void onLoop() {
        follower.update();
        Pose pose = follower.pose();

        minX = Math.min(minX, pose.x());
        maxX = Math.max(maxX, pose.x());
        minY = Math.min(minY, pose.y());
        maxY = Math.max(maxY, pose.y());

        if (gamepad1.aWasPressed()) {
            minX = minY = 1e9;
            maxX = maxY = -1e9;
        }

        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading", Math.toDegrees(pose.heading()));
        telemetry.addData("spread", Math.max(maxX - minX, maxY - minY));
        telemetry.update();
        super.loop();
    }
}