package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.slowFollow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.AutoCommands;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "3 TIP", group = "!0Auto")
public class Auto3 extends LinearOpMode {
    public static boolean isTipped = true;
    private final double intakeMS = 1800, shootMS = 2000;
    private Follower follower;
    private AutoCommands commands;
    private MasterController masterController;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose toIntakeRow = poseFactory.of(9.816, 8.093, 180);
    private final Pose toShootRow = poseFactory.of(44.647, 115.7029, 72.0643);
    private final Pose toIntakeFlower1 = poseFactory.of(45.6687, 125.5994, 90);
    private final Pose toIntakePreload = poseFactory.of(8.5578, 132.0825, 180);
    private final Pose toShootPreload = poseFactory.of(37.5338, 8.1679, 103.1615);
    private final Pose toIntakeFlower2 = poseFactory.of(20.4641, 46.8618, -179.9293);
    private final Pose toIntakeFlower2Control1 = poseFactory.of(41.4459, 46.8877, 0);
    private final Pose toShootFlower2 = poseFactory.of(37.1415, 12.2335, 115.7161);
    private final Pose toPark = poseFactory.of(11.9943, 92.3477, 107.4267);

    public Command autoRoutine() {
        return sequential(
                commands.shooter(true),
                commands.shoot(shootMS),
                commands.intake(),
                slowFollow(follower, toIntakeRow()),
                follow(follower, toShootRow()),
                commands.armFlower(),
                follow(follower, toIntakeFlower1()),
                waitMs(intakeMS),
                commands.shoot(shootMS),
                commands.armUp(),
                slowFollow(follower, toIntakePreload()),
                follow(follower, toShootPreload()),
                commands.stopIntake(),
                commands.shoot(shootMS),
                commands.armFlower(),
                commands.intake(),
                follow(follower, toIntakeFlower2()),
                waitMs(intakeMS),
                commands.armUp(),
                commands.stopIntake(),
                follow(follower, toShootFlower2()),
                commands.shoot(shootMS),
                follow(follower, toPark()),
                stopOpMode()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        masterController = new MasterController(hardwareMap, follower);
        masterController.initServo();
        commands = new AutoCommands(masterController);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            masterController.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.update();
        }
        PoseStorage.startPose = follower.pose();
    }

    public Path toIntakeRow() {
        return Paths.line(start, toIntakeRow).constant(toIntakeRow);
    }

    public Path toShootRow() {
        return Paths.line(toIntakeRow, toShootRow).tangent();
    }

    public Path toIntakeFlower1() {
        return Paths.line(toShootRow, toIntakeFlower1).constant(toIntakeFlower1);
    }

    public Path toIntakePreload() {
        return Paths.line(toIntakeFlower1, toIntakePreload).constant(toIntakePreload);
    }

    public Path toShootPreload() {
        return Paths.line(toIntakePreload, toShootPreload).reverseTangent();
    }

    public Path toShootPreloadConditional(){
        return Paths.line(toIntakePreload, toShootRow).constant(toIntakePreload);
    }

    public Path toIntakeFlower2() {
        return Paths.curve(toShootPreload, toIntakeFlower2Control1, toIntakeFlower2).tangent();
    }

    public Path toShootFlower2() {
        return Paths.line(toIntakeFlower2, toShootFlower2).reverseTangent();
    }

    public Path toPark() {
        return Paths.line(toShootFlower2, toPark).tangent();
    }

    private Command stopOpMode() {
        return instant(this::requestOpModeStop);
    }

    private Path conditionalPath(){if(isTipped) return toShootPreload(); else return toShootPreloadConditional();}
}
