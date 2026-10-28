package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.ShooterController;

@Config
@TeleOp(group = "!1Tests")
public class ShooterTest extends LinearOpMode {
    ShooterController sController;
    public static double fTarget, cTarget, bPos = 0.5;

    MultipleTelemetry multipleTelemetry;

    @Override
    public void runOpMode(){
        sController = new ShooterController(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        waitForStart();
        while (opModeIsActive()) {
            sController.setBPos(bPos);
            sController.setFTarget(fTarget);
            sController.setCTarget(cTarget);
            sController.update();
            sController.debug(multipleTelemetry);
            multipleTelemetry.update();
        }
    }
}
