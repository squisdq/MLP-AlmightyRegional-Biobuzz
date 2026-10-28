package org.firstinspires.ftc.teamcode.bobot.Utilities.Data;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config

public class PoseStorage {
    public static Pose startPose = new Pose(0, 0, Math.PI/2);

    public static final Pose rightHivePose = new Pose(60, 60);
    public static final Pose leftHivePose = new Pose(60, 88);
    public static final Pose rightResetPose = new Pose(9.2525, 8.41, 0);
    public static final Pose leftResetPose = new Pose(9.2525, 144 - 8.41, 0);
}