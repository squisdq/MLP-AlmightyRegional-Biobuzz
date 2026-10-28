package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto.Routines;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.BooleanSupplier;

public final class FollowCommands {
    private FollowCommands() {}

    public static Command slowFollow(Follower follower, Path path, double speed) {
        return lazy(() -> follow(follower,
                path.with(Constants.foresightConfig.maxPathSpeed.at(speed))));
    }

    public static Command slowFollow(Follower follower, Path path) {
        return slowFollow(follower, path, 0.4);
    }

    public static Command fastFollow(Follower follower, Path path) {
        return fastFollow(follower, path, 1.0, 1.3);
    }

    public static Command fastFollow(Follower follower, Path path, double speed, double brakeAggression) {
        return lazy(() -> follow(follower,
                path.with(
                        Constants.foresightConfig.maxPathSpeed.at(speed),
                        Constants.foresightConfig.brakeAggression.at(brakeAggression))));
    }

    public static Command tunedFollowing(Follower follower, Path path,
                                         double maxPathSpeed, double decelerationScale, double headingDriveRatio) {
        return lazy(() -> follow(follower,
                path.with(
                        Constants.foresightConfig.maxPathSpeed.at(maxPathSpeed),
                        Constants.foresightConfig.maxDecelerationScale.at(decelerationScale),
                        Constants.foresightConfig.headingDriveRatio.at(headingDriveRatio))));
    }

    public static Command thenHold(Follower follower, Path path) {
        return sequential(follow(follower, path), hold(follower));
    }

    public static Command with(Follower follower, Path path, Command other) {
        return parallel(follow(follower, path), other);
    }

    public static Command withUntilArrived(Follower follower, Path path, Command other) {
        return deadline(follow(follower, path), other);
    }

    public static Command raceWith(Follower follower, Path path, Command other) {
        return race(follow(follower, path), other);
    }

    public static final class At {
        public final double t;
        public final Command command;

        private At(double t, Command command) {
            this.t = t;
            this.command = command;
        }
    }

    public static At at(double t, Command command) {
        return new At(t, command);
    }

    public static Command withT(Follower follower, Path path, At... triggers) {
        Command[] all = new Command[triggers.length + 1];
        all[0] = follow(follower, path);
        for (int i = 0; i < triggers.length; i++) {
            all[i + 1] = onT(follower, triggers[i].t, triggers[i].command);
        }
        return parallel(all);
    }

    public static Command withTUntilArrived(Follower follower, Path path, At... triggers) {
        Command[] all = new Command[triggers.length];
        for (int i = 0; i < triggers.length; i++) {
            all[i] = onT(follower, triggers[i].t, triggers[i].command);
        }
        return deadline(follow(follower, path), all);
    }

    public static Command withTAtSegment(Follower follower, Path path,
                                         int segment, double t, Command command) {
        Command trigger = sequential(
                waitUntil(() -> isRunning(follower)
                        && follower.pathIndex() == segment
                        && follower.completion() >= t),
                command);
        return parallel(follow(follower, path), trigger);
    }

    public static Command onT(Follower follower, double t, Command command) {
        return sequential(
                waitUntil(() -> isRunning(follower) && follower.completion() >= t),
                command);
    }

    public static Command onRemainingDistance(Follower follower, double inches, Command command) {
        return sequential(
                waitUntil(() -> isRunning(follower) && follower.remainingDistance() <= inches),
                command);
    }

    public static Command withRemainingDistance(Follower follower, Path path,
                                                double inches, Command command) {
        return parallel(follow(follower, path), onRemainingDistance(follower, inches, command));
    }

    public static Command until(Follower follower, Path path, BooleanSupplier condition) {
        return sequential(follow(follower, path).until(condition), hold(follower));
    }

    public static Command withTimeout(Follower follower, Path path, long timeoutMs) {
        return sequential(race(follow(follower, path), waitMs(timeoutMs)), hold(follower));
    }

    public static Command followToParametricEnd(Follower follower, Path path) {
        return race(
                follow(follower, path),
                waitUntil(() -> isRunning(follower) && follower.atParametricEnd()));
    }

    public static Command followLenient(Follower follower, Path path, double inches) {
        return race(
                follow(follower, path),
                waitUntil(() -> isRunning(follower)
                        && follower.distanceToEndpoint() < inches
                        && Math.abs(follower.tangentialVelocity())
                        < Constants.foresightConfig.velocityConstraint.get()));
    }

    private static boolean isRunning(Follower follower) {
        return follower.isBusy();
    }
}
