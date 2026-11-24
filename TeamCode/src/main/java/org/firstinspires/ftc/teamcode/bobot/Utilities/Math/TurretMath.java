package org.firstinspires.ftc.teamcode.bobot.Utilities.Math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class TurretMath {
    public static double offsetX = -0.14, offsetY = 0, kLatency = 0.05, kTurn = 0.1;
    public static boolean right = true, far = false;

    private static final BilinearLut timeTravelDevice = BilinearLut.builder()
            .add(0, 0, 0.26).add(50, 0, 0.64).add(100, 0, 1.02)
            .add(0, 40, 0.57).add(50, 40, 0.75).add(100, 40, 1.08)
            .add(0, 80, 0.87).add(50, 80, 0.98).add(100, 80, 1.23)
            .build();

    private static Pose getHive() {
        if (right)
            return far ? PoseStorage.rightHivePoseFar : PoseStorage.rightHivePoseNear;
        return far ? PoseStorage.leftHivePoseFar : PoseStorage.leftHivePoseNear;
    }

    public static double calculateGoalCentricAngle(Follower follower) {
        right = follower.pose().y() < 72;
        Pose hive = getHive();
        Pose futurePose = getFuturePose(follower);
        return Math.toDegrees(Math.atan2(hive.y() - futurePose.y(), hive.x() - futurePose.x()) - futurePose.heading());
    }

    public static double calculateGoalCentricAngleWithoutSOTM(Follower follower){
        right = follower.pose().y() < 72;
        Pose hive = getHive();
        Pose rPose = follower.pose();
        return Math.toDegrees(Math.atan2(hive.y() - rPose.y(), hive.x() - rPose.x()) - rPose.heading());
    }

    public static double calculateFieldCentricAngle(Follower follower) {
        double heading = follower.pose().heading() + follower.velocity().omega * kTurn;
        return (right ? 90 : -90) - Math.toDegrees(heading);
    }

    public static double getRoboCentricAngle() {
        return 0;
    }

    public static double getFutureDistance(Follower follower) {
        Pose hive = getHive();
        Pose shooter = getFuturePose(follower);
        return Math.hypot(hive.x() - shooter.x(), hive.y() - shooter.y());
    }

    public static double getDistance(Follower follower){
        Pose hive = getHive();
        Pose shooter = follower.pose();
        return Math.hypot(hive.x() - shooter.x(), hive.y() - shooter.y());
    }

    public static Pose getFuturePose(Follower follower) {
        Pose hive = getHive();
        Pose pose = follower.pose();
        double vx = follower.velocity().vx;
        double vy = follower.velocity().vy;
        double time = 0;

        for (int i = 0; i < 3; i++)
            time = timeTravelDevice.get(Math.abs(hive.x() - pose.x() - vx * time), Math.abs(hive.y() - pose.y() - vy * time)) + kLatency;

        return new Pose(pose.x() + vx * time, pose.y() + vy * time, pose.heading() + follower.velocity().omega * kTurn);
    }

    public static double dX(Follower follower){
        Pose hive = getHive();
        Pose shooter = getFuturePose(follower);
        return hive.x() - shooter.x();
    }

    public static double dY(Follower follower){
        Pose hive = getHive();
        Pose shooter = getFuturePose(follower);
        return hive.y() - shooter.y();
    }
}