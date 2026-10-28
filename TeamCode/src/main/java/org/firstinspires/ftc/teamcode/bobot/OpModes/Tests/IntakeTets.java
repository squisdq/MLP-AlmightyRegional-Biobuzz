package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;

@Config
@TeleOp(group = "!1Tests")
public class IntakeTets extends OpMode {
    IntakeController iController;
    public static double iPower, tPower, armPos;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        iController = new IntakeController(hardwareMap);

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    }

    @Override
    public void loop() {
        iController.intake(iPower);
        iController.transfer(tPower);
        iController.setPos(armPos);

        iController.update();
        iController.debug(multipleTelemetry);
        multipleTelemetry.update();
    }
}
