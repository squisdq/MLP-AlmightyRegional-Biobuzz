package org.firstinspires.ftc.teamcode.bobot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Input.MLPGamepad;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(group = "!0TeleOp", name = "DuoPOP")
public class DuoOp extends OpMode {
    private MasterController masterController;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;
    private MLPGamepad pad1, pad2;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        masterController = new MasterController(hardwareMap, follower);
        masterController.initServo();
        follower.setPose(PoseStorage.startPose);
        follower.update();

        pad1 = new MLPGamepad(gamepad1);
        pad2 = new MLPGamepad(gamepad2);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pad1.update();
        pad2.update();

        if(pad1.leftTrigger.down)
            masterController.intake();
        else if(pad1.leftBumper.down)
            masterController.outtake();
        else if(pad2.rightTrigger.down)
            masterController.shoot();
        else
            masterController.stopIntake();

        if(pad2.rightBumper.rising)
            masterController.toggleShooter();
        if(pad1.start.rising)
            masterController.resetPositionRight();
        if(pad1.back.rising)
            masterController.resetPositionLeft();
        if(pad2.x.rising)
            masterController.sweepTMode();
        if(pad2.b.rising)
            masterController.sweepGoal();
        if(pad2.y.rising)
            masterController.toggleArm();
        if(pad2.a.rising)
            masterController.setTMode(MasterController.tModes.rCentric);

        if(masterController.getSOn())
            gamepad1.rumble(100);
        if(masterController.getSOn() && masterController.atTarget())
            gamepad2.rumble(100);

        masterController.drive(true, gamepad1, pad1.rightStick.down);
        masterController.update();

        masterController.debug(multipleTelemetry);
        multipleTelemetry.update();
        follower.update();
    }
}