package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.Controllers.DaddyController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous
public class PizdaAuto extends LinearOpMode {
    DaddyController daddyController;
    Follower follower;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.update();
        daddyController = new DaddyController(hardwareMap, follower);
        timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        daddyController.setTMode(DaddyController.tModes.rCentric);
        daddyController.setShootPower(0.28);
        while (opModeIsActive()){
            if(timer.milliseconds() > 15000) {
                daddyController.setShooter(true);
                if(timer.milliseconds() > 20000) {
                        if (daddyController.atTarget()) {
                            daddyController.shoot();
                            if(timer.milliseconds() > 28000){
                                daddyController.setShootPower(0.3);
                                daddyController.setShooter(false);
                        }
                    }
                }
            }
            follower.update();
            daddyController.update();
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
        }
    }
}
