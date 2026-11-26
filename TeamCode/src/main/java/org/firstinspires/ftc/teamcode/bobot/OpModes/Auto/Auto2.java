package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto;

import com.pedropathing.api.Paths;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.FollowCommands.slowFollow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines.AutoCommands;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "2 TIP", group = "!0Auto")
public class Auto2 extends LinearOpMode {

    private final double intakeMS = 1800, shootMS = 2000;
    private Follower follower;
    private AutoCommands commands;
    private MasterController masterController;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(50, 9.1, 180);
    private final Pose toIntakeRow = poseFactory.of(6.8194, 9.7, 180);
    private final Pose toShootRow = poseFactory.of(45.2679, 121.6657, -270);
    private final Pose toShootRowSegment2Start = poseFactory.of(36.2679, 115.6657, -103.8648);
    private final Pose toShootRowSegment2End = poseFactory.of(36.2679, 115.6657, 90);
    private final Pose toIntakeFlower1 = poseFactory.of(45.9656, 129.2112, 90);
    private final Pose toIntakeFlower2 = poseFactory.of(8.725, 51.1091, -180);
    private final Pose toIntakeFlower2Control1 = poseFactory.of(22.8387, 120.0127, 0);
    private final Pose toIntakeFlower2Control2 = poseFactory.of(44.628, 50.2799, 0);
    private final Pose toIntakeFlower2Segment2Start = poseFactory.of(16.525, 46.1091, -94.6735);
    private final Pose toIntakeFlower2Segment2End = poseFactory.of(16.525, 46.1091, 180);
    private final Pose toShootFlower2 = poseFactory.of(40.5854, 123.8225, -106.3661);
    private final Pose toPark = poseFactory.of(10.6496, 100.7262, 0);


    public Command autoRoutine() {
        return sequential(
                waitMs(100),
                commands.shooter(true),
                waitMs(500),
                commands.shoot(shootMS),
                commands.intake(),
                commands.shooter(false),
                slowFollow(follower, toIntakeRow()),
                waitMs(200),
                follow(follower, toShootRow()),
                commands.shooter(true),
                commands.stopIntake(),
                commands.armFlower(),
                waitMs(1000),
                commands.shoot(shootMS),
                follow(follower, toIntakeFlower1()),
                commands.intake(),
                waitMs(1500),
                commands.stopIntake(),
                commands.shoot(shootMS+500),
                commands.intake(),
                commands.shooter(false),
                follow(follower, toIntakeFlower2()),
                waitMs(intakeMS),
                commands.shooter(true),
                follow(follower, toShootFlower2()),
                commands.armUp(),
                commands.stopIntake(),
                waitMs(200),
                commands.shoot(shootMS),
                follow(follower, toPark()),
                waitMs(500),
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
        return Paths.line(toIntakeRow, toShootRow).reverseTangent();
    }

    public Path toIntakeFlower1() {
        return Paths.line(toShootRow, toIntakeFlower1).constant(toIntakeFlower1);
    }

    public Path toIntakeFlower2() {
        return Paths.curve(toIntakeFlower1, toIntakeFlower2Control1, toIntakeFlower2Control2, toIntakeFlower2).linear(toIntakeFlower1, toIntakeFlower2);
    }

    public Path toShootFlower2() {
        return Paths.line(toIntakeFlower2, toShootFlower2).reverseTangent();
    }

    public Path toPark() {
        return Paths.line(toShootFlower2, toPark).constant(toPark);
    }

    private Command stopOpMode(){return instant(this::requestOpModeStop);}
}
