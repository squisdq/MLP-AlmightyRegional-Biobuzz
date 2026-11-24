package org.firstinspires.ftc.teamcode.bobot.Utilities.AutoRoutines;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;

public class AutoCommands {
    private final MasterController master;

    public AutoCommands(MasterController master){
        this.master = master;
    }

    public Command intake(){
        return instant(master::intake);
    }

    public Command outtake(){
        return instant(master::outtake);
    }

    public Command stopIntake(){
        return instant(master::stopIntake);
    }

    public Command armUp(){
        return instant(master::setArmUp);
    }

    public Command armFlower(){
        return instant(master::setArmFlower);
    }

    public Command shooter(boolean on){
        return instant(() -> master.setShooter(on));
    }

    public Command shoot(double ms){
        return sequential(
                shooter(true),
                waitUntil(master::atTarget),
                race(infinite(master::shoot), waitMs(ms)),
                instant(master::afterShoot)
        );
    }
}