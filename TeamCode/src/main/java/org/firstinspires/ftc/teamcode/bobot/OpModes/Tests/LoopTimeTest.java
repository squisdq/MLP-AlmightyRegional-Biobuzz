package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "!1Tests")
public class LoopTimeTest extends OpMode {
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopTime;

    @Override
    public void init() {
        loopTime = 0;
    }

    @Override
    public void loop() {
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("loopTime", loopTime);
        telemetry.addData("loopHz", 1000 / loopTime);
    }
}
