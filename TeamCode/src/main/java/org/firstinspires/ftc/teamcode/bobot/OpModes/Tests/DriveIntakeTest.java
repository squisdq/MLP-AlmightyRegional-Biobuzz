package org.firstinspires.ftc.teamcode.bobot.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.bobot.Controllers.DaddyController.turnK;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveIntakeTest extends OpMode {
    private IntakeController iController;
    private Follower follower;
    private double iPower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseStorage.startPose);
        follower.update();
        iController = new IntakeController(hardwareMap);
        iPower = 0;
    }

    @Override
    public void loop() {
        iPower = gamepad1.right_trigger;
        iController.intake(iPower);

        drive(false, gamepad1);
        iController.update();
        follower.update();
    }

    public void drive(boolean fieldCentric, Gamepad gamepad){
        double forward = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        if(fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        else
            follower.manual(forward, strafe, turn);
    }
}
