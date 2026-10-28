package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.Utilities.Data.PoseStorage;

@Config
public class DaddyController {
    private final IntakeController iController;
    private final ShooterController sController;
    private final TurretController tController;
    private final Follower follower;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final double oX = 2.24311024, oY = 0.00;

    private tModes tMode;
    private sModes sMode = sModes.fixed;
    private tGoal goal;

    private boolean shooterOn, shooting, slowMode;
    private double tAngle, distance, dx, dy, headingDeg;

    public static double slowModeK = 0.5, turnK = 0.65,  kLatency = 0, kVelocity = 0, kFlightBase = 0, kFlightDist = 0, shootPower = 0.3, shootDelay = 100, pulseOn = 120, pulseOff = 80, fixF = 750, kH = 1.7;

    public DaddyController(HardwareMap hMap, Follower follower){
        iController = new IntakeController(hMap);
        sController = new ShooterController(hMap);
        tController = new TurretController(hMap);
        this.follower = follower;

        goal = tGoal.right;
        setTMode(tModes.gCentric);
    }

    public void initServo(){
        iController.initAServo();
        sController.initBServo();
    }

    public void setSPower(double sPower){
        shootPower = sPower;
    }

    public void resetPP(HardwareMap hMap){
        GoBildaPinpointDriver odo = hMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.resetDeviceConfigurationForOpMode();
    }

    public void intake(){
        iController.intake(1);
        iController.transfer(1);
    }

    public void outtake(){
        iController.intake(-1);
        iController.transfer(-1);
    }

    public void stopIntake(){
        iController.intake(0);
        iController.transfer(0);
        afterShoot();
    }

    public void shoot(){
        if(!shooterOn)
            return;

        if(!shooting){
            iController.intake(0);
            iController.transfer(0);
            sController.setState(ShooterController.bStates.supply);
            shootTimer.reset();
            shooting = true;
        } else if(shootTimer.milliseconds() >= shootDelay){
//            iController.transfer((shootTimer.milliseconds() - shootDelay) % (pulseOn + pulseOff) < pulseOn ? shootPower : 0);
            iController.transfer(shootPower);
            iController.intake(0.5);
        }
    }

    public void afterShoot(){
        shooting = false;
        sController.setState(ShooterController.bStates.block);
    }

    public void setKH(double kH){
        this.kH = kH;
    }

    public void setFixF(double fixF){
        this.fixF = fixF;
    }

    public boolean atTarget(){
        return sController.fAtTarget() && sController.cAtTarget();
    }

    public void switchArmState(){
        iController.switchState();
    }

    public void setArmState(IntakeController.rStates armState){
        iController.setState(armState);
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

    public void setShootPower(double power){
        DaddyController.shootPower = power;
    }

    public void toggleManual(){
        sMode = sMode == sModes.manual ? sModes.fixed : sModes.manual;
        setTMode(tMode);
    }

    public void sweepTMode(){
        setTMode(tMode == tModes.gCentric ? tModes.fCentric : tModes.gCentric);
    }

    public void sweepGoal(){
        goal = goal == tGoal.left ? tGoal.right : tGoal.left;
    }

    public double calculateGCentricAngle(){
        return Math.toDegrees(Math.atan2(dy, dx)) - headingDeg;
    }

    public double calculateFCentricAngle(){
        return (goal == tGoal.right ? 90 : -90) - headingDeg;
    }

    public double calculateRCentricAngle(){
        return 180;
    }

    public void sweepSlowMode(){
        slowMode = !slowMode;
    }

    public void drive(boolean fieldCentric, Gamepad gamepad){
        double k = slowMode ? slowModeK : 1;
        double forward = -gamepad.left_stick_y * k;
        double strafe = -gamepad.left_stick_x * k;
        double turn = -gamepad.right_stick_x * k * turnK;

        if(fieldCentric)
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        else
            follower.manual(forward, strafe, turn);
    }

    public void update(){
        boolean gCentric = tMode == tModes.gCentric;

        if(gCentric)
            goal = follower.pose().y() < 72 ? tGoal.right : tGoal.left;

        Pose hive = goal == tGoal.right ? PoseStorage.rightHivePose : PoseStorage.leftHivePose;

        double omega = follower.velocity().omega;
        double heading = follower.pose().heading();
        double rx = oX * Math.cos(heading) - oY * Math.sin(heading);
        double ry = oX * Math.sin(heading) + oY * Math.cos(heading);
        double vx = follower.velocity().vx - omega * ry;
        double vy = follower.velocity().vy + omega * rx;
        double x = follower.pose().x() + rx + vx * kLatency;
        double y = follower.pose().y() + ry + vy * kLatency;
        headingDeg = Math.toDegrees(heading + omega * kLatency);

        dx = hive.x() - x;
        dy = hive.y() - y;

        for(int i = 0; i < 3; i++){
            double flightTime = kFlightBase + kFlightDist * Math.hypot(dx, dy);
            dx = hive.x() - x - kVelocity * vx * flightTime;
            dy = hive.y() - y - kVelocity * vy * flightTime;
        }

        distance = Math.hypot(dx, dy);

        if(gCentric)
            tAngle = calculateGCentricAngle();
        else if(tMode == tModes.fCentric)
            tAngle = calculateFCentricAngle();
        else
            tAngle = calculateRCentricAngle();

        double fTarget = 0, cTarget = 0;

        if(shooterOn){
            boolean lut = sMode == sModes.lut;
            fTarget = lut ? sController.getSLut(distance) : fixF;
            cTarget = lut ? fTarget * sController.getHLut(distance) : fixF * kH;
        }

        sController.setFTarget(fTarget);
        sController.setCTarget(cTarget);
        tController.setAngle(tAngle);

        iController.update();
        if(shooterOn)
            tController.update();
        sController.update();
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addData("tMode", tMode);
        telemetry.addData("sMode", sMode);
        telemetry.addData("goal", goal);
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

    public enum tGoal{
        left, right
    }
}