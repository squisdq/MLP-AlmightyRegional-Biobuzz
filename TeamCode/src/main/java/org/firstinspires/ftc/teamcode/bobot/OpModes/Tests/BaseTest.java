package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.BaseController;

@Config
@TeleOp(group = "!1Tests")
public class BaseTest extends LinearOpMode {
    public static double target;
    public static boolean fieldCentric = true;

    MultipleTelemetry multipleTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        BaseController baseController = new BaseController(hardwareMap, fieldCentric);
        baseController.setPose(new Pose(0, 0, 0));
        baseController.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        waitForStart();
        while (opModeIsActive()){
            baseController.turnTo(target);
            baseController.drive(0, 0, 0);
            baseController.update();

            baseController.debug(multipleTelemetry);
            multipleTelemetry.update();
        }
    }
}