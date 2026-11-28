package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Base.DebugPrinter;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;
import org.firstinspires.ftc.teamcode.bobot.Utilities.Input.MLPGamepad;
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
//    private final DebugPrinter printer;
    private final List<LynxModule> hubs;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private tModes tMode;
    private sModes sMode = sModes.fixed;

    private boolean shooterOn, shooting, slowMode, armUp, SWMOn = true, speedOk, killed, isFar;
    private double distance, futureDistance, loopTime, holdAngle;

    public static double slowModeK = 0.5, turnK = 0.65, shootPower = 0.8, shootDelay = 300, fixF = 1340, idleF = 900, minSpeed = 12, nearRadius = 15;

    public MasterController(HardwareMap hMap, Follower follower){
        iController = new IntakeController(hMap);
        sController = new ShooterController(hMap);
        bController = new BlockerController(hMap);
        pController = new PushBackController(hMap);
        tController = new TurretController(hMap);
        this.follower = follower;
//        printer = new DebugPrinter(sController);

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

    public void initTurret(){
        tController.update();
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

    public void toggleSWM(){
        SWMOn = !SWMOn;
    }

    public void setSWM(boolean on){
        SWMOn = on;
    }

    public void sweepAim(){
        TurretMath.far = !TurretMath.far;
    }

    public void intake(){
        iController.intake(1);
    }

    public void outtake(){
        iController.intake(-0.4);
    }

    public void stopIntake(){
        iController.intake(0);
        afterShoot();
    }

    public void shoot(){
        if(!shooterOn){
            if(shooting){
                iController.intake(0);
                afterShoot();
            }
            return;
        }

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
        return sController.atTarget();
    }

    public void setShooter(boolean isOn){
        shooterOn = isOn;
        if(isOn)
            killed = false;
    }

    public void toggleShooter(){
        setShooter(!shooterOn);
    }

    public void killShooter(){
        shooterOn = false;
        killed = true;
    }

    public boolean getSOn(){
        return shooterOn;
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

    public void drive(boolean fieldCentric, MLPGamepad pad, boolean slow){
        double k = slow ? slowModeK : 1;
        double forward = -pad.leftY * k;
        double strafe = -pad.leftX * k;
        double turn = -pad.rightX * k * turnK;

        if(fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        else
            follower.manual(forward, strafe, turn);
    }

    public void drive(boolean fieldCentric, MLPGamepad pad){
        double k = pad.rightTrigger.down ? slowModeK : 1;
        double forward = -pad.leftY * k;
        double strafe = -pad.leftX * k;
        double turn = -pad.rightX * k * turnK;

        if(fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        else
            follower.manual(forward, strafe, turn);
    }

    public void driveAnalog(boolean fieldCentric, MLPGamepad pad){
        double k = 1 - (1 - slowModeK) * pad.gamepad.right_trigger;
        double forward = -pad.leftY * k;
        double strafe = -pad.leftX * k;
        double turn = -pad.rightX * k * turnK;

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
            if(SWMOn)
                tAngle = TurretMath.calculateGoalCentricAngle(follower);
            else
                tAngle = TurretMath.calculateGoalCentricAngleWithoutSOTM(follower);
        else if(tMode == tModes.fCentric)
            tAngle = TurretMath.calculateFieldCentricAngle(follower);
        else
            tAngle = TurretMath.getRoboCentricAngle();

        distance = TurretMath.getDistance(follower);
        futureDistance = TurretMath.getFutureDistance(follower);

        if(tMode == tModes.gCentric){
            if(distance < nearRadius)
                tAngle = holdAngle;
            else
                holdAngle = tAngle;
        }

        if(TurretMath.getDistance(follower) >= 55)
            shootPower = 0.6;
        else
            shootPower = 1;
        speedOk = !SWMOn || tMode != tModes.gCentric || distance < nearRadius || Math.hypot(follower.velocity().vx, follower.velocity().vy) >= minSpeed;

        if(shooterOn){
            double fTarget = fixF;

            if(sMode == sModes.lut)
                fTarget = sController.getSLut(SWMOn ? futureDistance : distance);

            sController.setTarget(fTarget);
        } else
            sController.setIdle(killed ? 0 : idleF);

        tController.setAngle(tAngle);

        iController.update();
        if(shooterOn)
            tController.update();
        sController.update();
        bController.update();
        pController.update();
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("Bismillah");
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("loopTime", loopTime);
        telemetry.addData("loopHz", 1000 / loopTime);
//        telemetry.addData("tMode", tMode);
//        telemetry.addData("sMode", sMode);
//        telemetry.addData("SWMOn", SWMOn);
//        telemetry.addData("right", TurretMath.right);
//        telemetry.addData("far", TurretMath.far);
//        telemetry.addData("shooterOn", shooterOn);
//        telemetry.addData("speedOk", speedOk);
        telemetry.addData("distance", distance);
        telemetry.addData("dx", TurretMath.dX(follower));
        telemetry.addData("dy", TurretMath.dY(follower));
        sController.debug(telemetry);
//        iController.debug(telemetry);
//        tController.debug(telemetry);
//        printer.debug(telemetry);
    }

    public enum tModes{
        gCentric, fCentric, rCentric
    }

    public enum sModes{
        lut, fixed, manual
    }
}