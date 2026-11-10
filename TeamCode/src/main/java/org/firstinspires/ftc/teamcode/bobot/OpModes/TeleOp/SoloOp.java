package org.firstinspires.ftc.teamcode.bobot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Input.MLPGamepad;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "!0TeleOp", name = "SoloPOP")
public class SoloOp extends OpMode {
    private MasterController masterController;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;
    private MLPGamepad pad;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        masterController = new MasterController(hardwareMap, follower);
        masterController.initServo();
        follower.setPose(PoseStorage.startPose);
        follower.update();

        pad = new MLPGamepad(gamepad1);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pad.update();

        if(pad.leftTrigger.down)
            masterController.intake();
        else if(pad.leftBumper.down)
            masterController.outtake();
        else if(pad.rightTrigger.down)
            masterController.shoot();
        else
            masterController.stopIntake();

        if(pad.rightBumper.rising)
            masterController.toggleShooter();
        if(pad.start.rising)
            masterController.resetPositionRight();
        if(pad.back.rising)
            masterController.resetPositionLeft();
        if(pad.x.rising)
            masterController.sweepTMode();
        if(pad.b.rising)
            masterController.sweepGoal();
        if(pad.y.rising)
            masterController.toggleArm();
        if(pad.a.rising)
            masterController.setTMode(MasterController.tModes.rCentric);

        if(masterController.getSOn())
            gamepad1.rumble(100);
        if(masterController.getSOn() && masterController.atTarget())
            gamepad2.rumble(100);

        masterController.drive(true, gamepad1, pad.rightStick.down);
        masterController.update();

        masterController.debug(multipleTelemetry);
        multipleTelemetry.update();
        follower.update();
    }
}