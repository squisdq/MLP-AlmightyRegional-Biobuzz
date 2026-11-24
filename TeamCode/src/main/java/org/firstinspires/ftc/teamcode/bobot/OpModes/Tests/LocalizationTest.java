package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Base.OpModeWrapper;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Input.MLPGamepad;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "!1Tests")
public class LocalizationTest extends OpMode {
    private Follower follower;
    private MLPGamepad pad;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72,72, 0));
        follower.update();

        pad= new MLPGamepad(gamepad1);
    }

    @Override
    public void loop() {
        follower.update();
        double forward = -pad.leftY;
        double strafe = -pad.leftX;
        double turn = -pad.rightX;
        follower.manual(forward, strafe, turn);

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        pad.update();
    }
}
