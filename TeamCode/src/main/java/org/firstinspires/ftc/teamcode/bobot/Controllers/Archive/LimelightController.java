package org.firstinspires.ftc.teamcode.bobot.Controllers.Archive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class LimelightController {
    public static double cameraHeight = 12, cameraPitch = 0, splitHeight = 40;
    public static int confirmLoops = 3;

    private final Limelight3A limelight;
    private Cell target = Cell.redAudience;
    private boolean wasUp, tipped;
    private int lowLoops;
    private double angle = Double.NaN, height;

    public LimelightController(HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setTarget(Cell target) {
        if (target != this.target) {
            this.target = target;
            reset();
        }
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        double sumX = 0, sumHeight = 0;
        int count = 0;

        if (result != null && result.isValid())
            for (FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < target.firstId || id > target.firstId + 3)
                    continue;
                Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                sumX += tag.getTargetXDegrees();
                sumHeight += cameraHeight + range * Math.sin(Math.toRadians(cameraPitch + tag.getTargetYDegrees()));
                count++;
            }

        if (count == 0) {
            angle = Double.NaN;
            return;
        }

        angle = -sumX / count;
        height = sumHeight / count;

        if (height > splitHeight) {
            wasUp = true;
            lowLoops = 0;
        } else if (wasUp && ++lowLoops >= confirmLoops)
            tipped = true;
    }

    public void reset() {
        wasUp = false;
        tipped = false;
        lowLoops = 0;
    }

    public double getAngle() {
        return angle;
    }

    public boolean hasTarget() {
        return !Double.isNaN(angle);
    }

    public boolean isTipped() {
        return tipped;
    }

    public double getHeight() {
        return height;
    }

    public enum Cell {
        redFar(30), redAudience(34), blueAudience(38), blueFar(42);

        private final int firstId;

        Cell(int firstId) {
            this.firstId = firstId;
        }
    }
}