package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.Archive.LimelightController;

@Config
@TeleOp(group = "!1Tests")
public class LimelightTest extends LinearOpMode {
    public static LimelightController.Cell cell = LimelightController.Cell.redAudience;

    @Override
    public void runOpMode(){
        LimelightController lController = new LimelightController(hardwareMap);
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            lController.setTarget(cell);

            if(gamepad1.a)
                lController.reset();

            lController.update();

            multipleTelemetry.addData("cell", cell);
            multipleTelemetry.addData("hasTarget", lController.hasTarget());
            multipleTelemetry.addData("angle", lController.getAngle());
            multipleTelemetry.addData("height", lController.getHeight());
            multipleTelemetry.addData("tipped", lController.isTipped());
            multipleTelemetry.update();
        }
    }
}