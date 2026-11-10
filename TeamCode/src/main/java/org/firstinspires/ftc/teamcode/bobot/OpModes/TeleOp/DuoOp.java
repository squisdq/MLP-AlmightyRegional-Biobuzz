package org.firstinspires.ftc.teamcode.bobot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "!0TeleOp", name = "DuoPOP")
public class DuoOp extends OpMode {
    private MasterController masterController;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        masterController = new MasterController(hardwareMap, follower);
        masterController.initServo();
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
        else if(gamepad2.right_trigger >= 0.2)
            masterController.shoot();
        else
            masterController.stopIntake();

        if(gamepad2.rightBumperWasPressed())
            masterController.toggleShooter();
        if(gamepad1.startWasPressed())
            masterController.resetPositionRight();
        if(gamepad1.backWasPressed())
            masterController.resetPositionLeft();
        if(gamepad2.xWasPressed())
            masterController.sweepTMode();
        if(gamepad2.bWasPressed())
            masterController.sweepGoal();
        if(gamepad2.yWasPressed())
            masterController.toggleArm();
        if(gamepad2.aWasPressed())
            masterController.setTMode(MasterController.tModes.rCentric);
        if(gamepad1.rightStickButtonWasPressed())
            masterController.sweepSlowMode();

        if(masterController.getSOn())
            gamepad1.rumble(100);
        if(masterController.getSOn() && masterController.atTarget())
            gamepad2.rumble(100);

        masterController.drive(true, gamepad1, gamepad1.right_stick_button);
        masterController.update();

        masterController.debug(multipleTelemetry);
        multipleTelemetry.update();
        follower.update();
    }
}