package org.firstinspires.ftc.teamcode.bobot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.DaddyController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "!0TeleOp", name = "SoloPOP")
public class SoloOp extends OpMode {
    private DaddyController daddyController;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        daddyController = new DaddyController(hardwareMap, follower);
        daddyController.initServo();
        follower.setPose(PoseStorage.startPose);
        follower.update();

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger >= 0.2)
            daddyController.intake();
        else if(gamepad1.left_bumper)
            daddyController.outtake();
        else if(gamepad1.right_trigger >= 0.2)
            daddyController.shoot();
        else
            daddyController.stopIntake();

        if(gamepad1.rightBumperWasPressed())
            daddyController.toggleShooter();
        if(gamepad1.startWasPressed())
            daddyController.resetPositionRight();
        if(gamepad1.backWasPressed())
            daddyController.resetPositionLeft();
        if(gamepad1.xWasPressed())
            daddyController.sweepTMode();
        if(gamepad1.bWasPressed())
            daddyController.sweepGoal();
        if(gamepad1.yWasPressed())
            daddyController.switchArmState();
        if(gamepad1.aWasPressed())
            daddyController.setTMode(DaddyController.tModes.rCentric);
        if(gamepad1.rightStickButtonWasPressed())
            daddyController.sweepSlowMode();

        if(daddyController.getSOn())
            gamepad1.rumble(100);
        if(daddyController.getSOn() && daddyController.atTarget())
            gamepad2.rumble(100);

        daddyController.drive(true, gamepad1);
        daddyController.update();

        daddyController.debug(multipleTelemetry);
        multipleTelemetry.update();
        follower.update();
    }
}