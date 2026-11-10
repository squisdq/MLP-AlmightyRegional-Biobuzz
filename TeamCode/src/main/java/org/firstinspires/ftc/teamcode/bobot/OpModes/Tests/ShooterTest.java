package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.ShooterController;

@Config
@TeleOp(group = "!1Tests")
public class ShooterTest extends LinearOpMode {
    public static double fTarget;

    @Override
    public void runOpMode(){
        ShooterController sController = new ShooterController(hardwareMap);
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            sController.setTarget(fTarget);
            sController.update();
            sController.debug(multipleTelemetry);
            multipleTelemetry.update();
        }
    }
}