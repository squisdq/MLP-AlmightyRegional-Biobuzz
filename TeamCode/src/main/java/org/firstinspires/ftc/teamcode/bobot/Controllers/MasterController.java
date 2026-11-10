package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Math.TurretMath;

import java.util.List;

@Config
public class MasterController {
    private final IntakeController iController;
    private final ShooterController sController;
    private final BlockerController bController;
    private final PushBackController pController;
    private final TurretController tController;
    private final Follower follower;
    private final List<LynxModule> hubs;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private tModes tMode;
    private sModes sMode = sModes.fixed;

    private boolean shooterOn, shooting, slowMode, armUp;
    private double distance, loopTime;

    public static double slowModeK = 0.5, turnK = 0.65, shootPower = 1, shootDelay = 300, fixF = 750;

    public MasterController(HardwareMap hMap, Follower follower){
        iController = new IntakeController(hMap);
        sController = new ShooterController(hMap);
        bController = new BlockerController(hMap);
        pController = new PushBackController(hMap);
        tController = new TurretController(hMap);
        this.follower = follower;

        hubs = hMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        TurretMath.right = true;
        setTMode(tModes.gCentric);
    }

    public void initServo(){
        pController.update();
        bController.update();
    }

    public void setSPower(double sPower){
        shootPower = sPower;
    }

    public void toggleArm(){
        armUp = !armUp;

        if(armUp)
            pController.setUpPos();
        else
            pController.setFlowerPos();
    }

    public void resetPP(HardwareMap hMap){
        GoBildaPinpointDriver odo = hMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.resetDeviceConfigurationForOpMode();
    }

    public void intake(){
        iController.intake(1);
    }

    public void outtake(){
        iController.intake(-1);
    }

    public void stopIntake(){
        iController.intake(0);
        afterShoot();
    }

    public void shoot(){
        if(!shooterOn)
            return;

        if(!shooting){
            iController.intake(0);
            bController.setSupplyPos();
            shootTimer.reset();
            shooting = true;
        } else if(shootTimer.milliseconds() >= shootDelay)
            iController.intake(shootPower);
    }

    public void afterShoot(){
        shooting = false;
        bController.setBlockPos();
    }

    public void setArmUp(){
        pController.setUpPos();
    }

    public void setArmFlower(){
        pController.setFlowerPos();
    }

    public void setFixF(double fixF){
        this.fixF = fixF;
    }

    public boolean atTarget(){
        return sController.fAtTarget();
    }

    public void setShooter(boolean isOn){
        shooterOn = isOn;
    }

    public boolean getSOn(){
        return shooterOn;
    }

    public void toggleShooter(){
        shooterOn = !shooterOn;
    }

    public void resetPositionRight(){
        follower.setPose(PoseStorage.rightResetPose);
    }

    public void resetPositionLeft(){
        follower.setPose(PoseStorage.leftResetPose);
    }

    public void setTMode(tModes tMode){
        this.tMode = tMode;

        if(sMode != sModes.manual)
            sMode = tMode == tModes.gCentric ? sModes.lut : sModes.fixed;
    }

    public void setManual(){
        sMode = sModes.manual;
    }

    public void toggleManual(){
        sMode = sMode == sModes.manual ? sModes.fixed : sModes.manual;
        setTMode(tMode);
    }

    public void sweepTMode(){
        setTMode(tMode == tModes.gCentric ? tModes.fCentric : tModes.gCentric);
    }

    public void sweepGoal(){
        TurretMath.right = !TurretMath.right;
    }

    public void sweepSlowMode(){
        slowMode = !slowMode;
    }

    public void drive(boolean fieldCentric, Gamepad gamepad, boolean slow){
        double k = slow ? slowModeK : 1;
        double forward = -gamepad.left_stick_y * k;
        double strafe = -gamepad.left_stick_x * k;
        double turn = -gamepad.right_stick_x * k * turnK;

        if(fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        else
            follower.manual(forward, strafe, turn);
    }

    public void update(){
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        for(LynxModule hub : hubs)
            hub.clearBulkCache();

        double tAngle;

        if(tMode == tModes.gCentric)
            tAngle = TurretMath.calculateGoalCentricAngle(follower);
        else if(tMode == tModes.fCentric)
            tAngle = TurretMath.calculateFieldCentricAngle(follower);
        else
            tAngle = TurretMath.getRoboCentricAngle();

        distance = TurretMath.getFutureDistance(follower);

        double fTarget = 0;

        if(shooterOn)
            fTarget = sMode == sModes.lut ? sController.getSLut(distance) : fixF;

        sController.setTarget(fTarget);
        tController.setAngle(tAngle);

        iController.update();
        if(shooterOn)
            tController.update();
        sController.update();
        bController.update();
        pController.update();
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addData("loopTime", loopTime);
        telemetry.addData("loopHz", 1000 / loopTime);
        telemetry.addData("tMode", tMode);
        telemetry.addData("sMode", sMode);
        telemetry.addData("right", TurretMath.right);
        telemetry.addData("shooterOn", shooterOn);
        telemetry.addData("distance", distance);
        sController.debug(telemetry);
        iController.debug(telemetry);
        tController.debug(telemetry);
    }

    public enum tModes{
        gCentric, fCentric, rCentric
    }

    public enum sModes{
        lut, fixed, manual
    }
}