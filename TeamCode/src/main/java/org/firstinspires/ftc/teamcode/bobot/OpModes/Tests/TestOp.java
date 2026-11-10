package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(group = "!1Tests", name = "TestPOP")
public class TestOp extends OpMode {
    public static double speed = 1800, kH = 1.7;

    private MasterController masterController;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        masterController = new MasterController(hardwareMap, follower);
        masterController.initServo();
        masterController.toggleManual();
        follower.setPose(PoseStorage.startPose);
        follower.update();

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger >= 0.2)
            masterController.intake();
        else if(gamepad1.left_bumper)
            masterController.outtake();
        else if(gamepad1.right_trigger >= 0.2)
            masterController.shoot();
        else
            masterController.stopIntake();

        if(gamepad1.rightBumperWasPressed())
            masterController.toggleShooter();
        if(gamepad1.startWasPressed())
            masterController.resetPositionRight();
        if(gamepad1.backWasPressed())
            masterController.resetPositionLeft();
        if(gamepad1.xWasPressed())
            masterController.sweepTMode();
        if(gamepad1.bWasPressed())
            masterController.sweepGoal();
        if(gamepad1.yWasPressed())
            masterController.toggleArm();
        if(gamepad1.aWasPressed())
            masterController.setTMode(MasterController.tModes.rCentric);
        if(gamepad1.leftStickButtonWasPressed())
            masterController.toggleManual();
        if(gamepad1.rightStickButtonWasPressed())
            masterController.sweepSlowMode();

        if(masterController.getSOn())
            gamepad1.rumble(100);
        if(masterController.getSOn() && masterController.atTarget())
            gamepad2.rumble(100);

        masterController.setFixF(speed);

        masterController.drive(true, gamepad1, gamepad1.right_stick_button);
        masterController.update();

        masterController.debug(multipleTelemetry);
        follower.update();
        multipleTelemetry.update();
    }
}