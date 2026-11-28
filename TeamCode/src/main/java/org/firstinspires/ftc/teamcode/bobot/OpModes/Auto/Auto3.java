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
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.safeAndSlow;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.safeFollow;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.slowFollow;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.slowFollowWithT;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.with;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.withT;

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
    private final double intakeMS = 2000, shootMS = 1500;
    private Follower follower;
    private AutoCommands commands;
    private MasterController masterController;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(21, 9.1, 180);
    private final Pose toIntakeRow = poseFactory.of(6.8194, 9.3, 180);
    private final Pose toShootRow = poseFactory.of(40.647, 120.7029, 72.0643);
    private final Pose toIntakeFlower1 = poseFactory.of(45.9656, 128.4112, 90);
    private final Pose toShootFlower1 = poseFactory.of(45.9656, 120.8112, 90);
    private final Pose toIntakePreload = poseFactory.of(8.0578, 73.5825, 180);
    private final Pose toIntakePreloadControl1 = poseFactory.of(10, 110, 180);
    private final Pose toIntakePreloadControl2 = poseFactory.of(56, 69, 180);
    private final Pose toShootPreload = poseFactory.of(38.5338, 26.1679, 103.1615);
    private final Pose toShootPreloadControl1 = poseFactory.of(65, 120.1679, 103.1615);
    private final Pose toShootPreloadControl2 = poseFactory.of(5, 35, 103.1615);
    private final Pose toIntakeFlower2 = poseFactory.of(10.4500, 49.3091, -180);
    private final Pose toIntakeFlower2Control1 = poseFactory.of(41.4459, 49.1091, 0);
    private final Pose toShootFlower2 = poseFactory.of(38.8415, 26.2335, 115.7161);
    private final Pose toPark = poseFactory.of(11.9943, 92.3477, 107.4267);

    public Command autoRoutine() {
        return sequential(
                commands.shooter(true),
                waitMs(700),
                commands.shoot(shootMS),
                commands.intake(),
                safeAndSlow(follower, toIntakeRow(), 0.2),
                commands.shooter(false),
                waitMs(500),
                commands.stopIntake(),
                commands.armFlower(),
                follow(follower, toShootRow()),
                commands.shooter(true),
                commands.shoot(shootMS),
                commands.intake(),
                safeFollow(follower, toIntakeFlower1Cp()),
                safeAndSlow(follower, toIntakeFlower1(), 0.5),
                waitMs(intakeMS),
                follow(follower, toShootFlower1()),
                commands.shoot(shootMS),
                commands.intake(),
                commands.shooter(false),
                slowFollowWithT(follower, toIntakePreload(), 0.1, commands.armUp(), 1),
                waitMs(500),
                follow(follower, toShootPreload()),
                commands.shooter(true),
                commands.stopIntake(),
                commands.shoot(shootMS+500),
                commands.armFlower(),
                commands.intake(),
                commands.shooter(false),
                safeAndSlow(follower, toIntakeFlower2(), 0.5),
                waitMs(intakeMS),
                commands.shooter(true),
                follow(follower, toShootFlower2()),
                commands.shoot(shootMS),
                commands.shooter(false),
                commands.armUp(),
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
        masterController.setSWM(false);
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
        return Paths.line(toShootFlower1, toIntakeFlower1).constant(toIntakeFlower1);
    }

    public Path toIntakeFlower1Cp() {
        return Paths.line(toShootRow, toShootFlower1).constant(toIntakeFlower1);
    }

    public Path toShootFlower1() {
        return Paths.line(toIntakeFlower1, toShootFlower1).linear(toIntakeFlower1, toShootFlower1);
    }

    public Path toIntakePreload() {
        return Paths.curve(toShootFlower1, toIntakePreloadControl1, toIntakePreloadControl2, toIntakePreload).tangent();
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
        return Paths.line(toIntakeFlower2, toShootFlower2).constant(toShootPreload);
    }

    public Path toPark() {
        return Paths.line(toShootFlower2, toPark).tangent();
    }

    private Command stopOpMode() {
        return instant(this::requestOpModeStop);
    }

//    private Path conditionalPath(){if(isTipped) return toShootPreload(); else return toShootPreloadConditional();}
}
