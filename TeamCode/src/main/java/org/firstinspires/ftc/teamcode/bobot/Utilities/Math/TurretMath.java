package org.firstinspires.ftc.teamcode.bobot.Utilities.Math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;

@Config
public class TurretMath {
    public static double offsetX = 2.24311024, offsetY = 0, kLatency = 0, kTurn = 0;
    public static boolean right = true;

//    private static LSQApproximator flightTimeApproximator = LSQApproximator.builder()
//            .degree(1)
//            .add(40, 0.738).add(80, 1.127).add(120, 1.127)
//            .build();

    public static double calculateGoalCentricAngle(Follower follower) {
        right = follower.pose().y() < 72;
        Pose hive = right ? PoseStorage.rightHivePose : PoseStorage.leftHivePose;
        Pose futurePose = getFuturePose(follower);
        return Math.toDegrees(Math.atan2(hive.y() - futurePose.y(), hive.x() - futurePose.x()) - futurePose.heading());
    }

    public static double calculateGoalCentricAngleWithoutSOTM(Follower follower){
        right = follower.pose().y() < 72;
        Pose hive = right ? PoseStorage.rightHivePose : PoseStorage.leftHivePose;
        Pose rPose = follower.pose();
        return Math.toDegrees(Math.atan2(hive.y() - rPose.y(), hive.x() - rPose.x()) - rPose.heading());
    }

    public static double calculateFieldCentricAngle(Follower follower) {
        double heading = follower.pose().heading() + follower.velocity().omega * kTurn;
        return (right ? 90 : -90) - Math.toDegrees(heading);
    }

    public static double getRoboCentricAngle() {
        return 180;
    }

    public static double getFutureDistance(Follower follower) {
        Pose hive = right ? PoseStorage.rightHivePose : PoseStorage.leftHivePose;
        Pose shooter = getFuturePose(follower);
        return Math.hypot(hive.x() - shooter.x(), hive.y() - shooter.y());
    }

    public static Pose getFuturePose(Follower follower) {
        Pose hive = right ? PoseStorage.rightHivePose : PoseStorage.leftHivePose;
        double heading = follower.pose().heading();
        double omega = follower.velocity().omega;
        double rx = offsetX * Math.cos(heading) - offsetY * Math.sin(heading);
        double ry = offsetX * Math.sin(heading) + offsetY * Math.cos(heading);
        double vx = follower.velocity().vx - omega * ry;
        double vy = follower.velocity().vy + omega * rx;
        double x = follower.pose().x() + rx + vx * kLatency;
        double y = follower.pose().y() + ry + vy * kLatency;
        double flightTime = 0;

        for (int i = 0; i < 3; i++)
            flightTime = 0.00486758 * Math.hypot(hive.x() - x - vx * flightTime, hive.y() - y - vy * flightTime) + 0.542966;
//            flightTime = flightTimeApproximator.get(Math.hypot(hive.x() - x - vx * flightTime, hive.y() - y - vy * flightTime));

        return new Pose(x + vx * flightTime, y + vy * flightTime, heading + omega * kTurn);
    }
}