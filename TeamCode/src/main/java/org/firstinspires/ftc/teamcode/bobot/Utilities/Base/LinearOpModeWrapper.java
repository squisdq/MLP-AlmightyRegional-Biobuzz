package org.firstinspires.ftc.teamcode.bobot.Utilities.Base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public abstract class LinearOpModeWrapper extends LinearOpMode {
    private List<LynxModule> hubs;

    protected void setup() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    protected boolean active() {
        telemetry.update();
        for (LynxModule hub : hubs) hub.clearBulkCache();
        return opModeIsActive();
    }
}