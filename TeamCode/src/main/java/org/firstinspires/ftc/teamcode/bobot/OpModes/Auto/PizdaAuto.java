package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous
public class PizdaAuto extends LinearOpMode {
    MasterController masterController;
    Follower follower;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.update();
        masterController = new MasterController(hardwareMap, follower);
        timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        masterController.setTMode(MasterController.tModes.rCentric);
        masterController.setSPower(0.28);
        while (opModeIsActive()){
            if(timer.milliseconds() > 15000) {
                masterController.setShooter(true);
                if(timer.milliseconds() > 20000) {
                        if (masterController.atTarget()) {
                            masterController.shoot();
                            if(timer.milliseconds() > 28000){
                                masterController.setSPower(0.3);
                                masterController.setShooter(false);
                        }
                    }
                }
            }
            follower.update();
            masterController.update();
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
        }
    }
}
