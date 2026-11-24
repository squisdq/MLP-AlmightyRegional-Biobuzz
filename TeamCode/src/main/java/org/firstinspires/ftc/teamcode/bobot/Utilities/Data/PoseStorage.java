package org.firstinspires.ftc.teamcode.bobot.Utilities.Data;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config

public class PoseStorage {
    public static Pose startPose = new Pose(0, 0, 0);

    public static final Pose rightHivePoseNear = new Pose(60, 59);
    public static final Pose leftHivePoseNear = new Pose(58, 86);
    public static final Pose rightHivePoseFar = new Pose(56, 50);
    public static final Pose leftHivePoseFar = new Pose(58, 88);
    public static final Pose rightResetPose = new Pose(6.1, 9.1, 0);
    public static final Pose leftResetPose = new Pose(6.1, 135, 0);
}