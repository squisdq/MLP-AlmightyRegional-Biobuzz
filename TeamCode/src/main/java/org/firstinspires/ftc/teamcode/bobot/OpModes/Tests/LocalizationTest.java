package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Base.OpModeWrapper;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Input.MLPGamepad;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(group = "!1Tests")
public class LocalizationTest extends OpMode {
    private Follower follower;
    private MLPGamepad pad;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72,72, 0));
        follower.update();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        pad= new MLPGamepad(gamepad1);
    }

    @Override
    public void loop() {
        follower.update();
        double forward = -pad.leftY;
        double strafe = -pad.leftX;
        double turn = -pad.rightX;
        follower.manual(forward, strafe, turn);

        multipleTelemetry.addData("x", follower.pose().x());
        multipleTelemetry.addData("y", follower.pose().y());
        multipleTelemetry.addData("heading", follower.pose().heading());
        multipleTelemetry.update();
        pad.update();
    }
}
