package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous
public class HuyAuto extends LinearOpMode {
    Follower follower;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        while (opModeIsActive()){
            follower.manual(0, 0.4, 0);
            follower.update();
        }
    }
}
