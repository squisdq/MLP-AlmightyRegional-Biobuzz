package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.BlockerController;

@Config
@TeleOp(group = "!1Tests")
public class BlockerTest extends LinearOpMode {
    public static double pos = 0.5;

    @Override
    public void runOpMode(){
        BlockerController bController = new BlockerController(hardwareMap);
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a)
                bController.setBlockPos();
            else if(gamepad1.b)
                bController.setSupplyPos();
            else
                bController.setPos(pos);

            bController.update();
            multipleTelemetry.addData("pos", pos);
            multipleTelemetry.update();
        }
    }
}