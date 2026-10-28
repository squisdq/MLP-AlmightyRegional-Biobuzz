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

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Конструктор команд следования по пути (Pedro Pathing 3 + Ivy).
 *
 * Пример:
 *
 *   Command pickup = Follow.path(follower, pickupPath)
 *       .slow()                                  // maxPathSpeed = 0.4
 *       .atT(0.3, intakeOn())                    // на 30% пути включить интейк
 *       .atT(0.9, intakeOff())                   // на 90% выключить
 *       .during(spinUpShooter())                 // крутится, пока едем
 *       .timeout(3000)                           // но не дольше 3 сек
 *       .hold()                                  // после - держим позицию
 *       .build();
 *
 *   Command auto = sequential(
 *       Follow.path(follower, toShoot).fast().build(),
 *       shoot(),
 *       pickup
 *   );
 *
 * Импорты Follower / Command / Path / Constants проверь автодополнением.
 */
public final class Follow {

    private final Follower follower;
    private final Path path;

    // --- модификаторы пути (null = не трогаем, берём из Constants) ---
    private Double maxPathSpeed;
    private Double brakeAggression;
    private Double decelScale;
    private Double headingRatio;

    // --- команды рядом с движением ---
    private final List<Command> alongside = new ArrayList<>(); // ждём их завершения
    private final List<Command> during = new ArrayList<>();    // отменяются по прибытии
    private boolean cancelAlongsideOnArrival = false;

    // --- условия завершения ---
    private BooleanSupplier untilCondition;
    private long timeoutMs = -1;
    private boolean endAtParametric = false;
    private double lenientInches = -1;
    private boolean holdAtEnd = false;

    private Follow(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    public static Follow path(Follower follower, Path path) {
        return new Follow(follower, path);
    }

    // ============================================================
    //  СКОРОСТЬ И МОДИФИКАТОРЫ
    // ============================================================

    /** Медленно: maxPathSpeed = 0.4. */
    public Follow slow() {
        return speed(0.4);
    }

    /** Быстро: полная скорость + агрессивнее тормозим в конце. */
    public Follow fast() {
        this.maxPathSpeed = 1.0;
        this.brakeAggression = 1.3;
        return this;
    }

    /** maxPathSpeed: доля от максимальной (0..1). */
    public Follow speed(double maxPathSpeed) {
        this.maxPathSpeed = maxPathSpeed;
        return this;
    }

    public Follow brake(double brakeAggression) {
        this.brakeAggression = brakeAggression;
        return this;
    }

    public Follow decelScale(double decelScale) {
        this.decelScale = decelScale;
        return this;
    }

    public Follow headingRatio(double headingDriveRatio) {
        this.headingRatio = headingDriveRatio;
        return this;
    }

    // ============================================================
    //  ДРУГИЕ КОМАНДЫ ВО ВРЕМЯ ЕЗДЫ
    // ============================================================

    /** Запустить вместе со стартом пути. Конец = когда закончатся и путь, и эта команда. */
    public Follow alongside(Command command) {
        alongside.add(command);
        return this;
    }

    /** Запустить вместе со стартом пути; отменяется, когда путь закончился. */
    public Follow during(Command command) {
        during.add(command);
        return this;
    }

    /** Запустить, когда T (прогресс текущего сегмента 0..1) достиг t. */
    public Follow atT(double t, Command command) {
        alongside.add(sequential(
                waitUntil(() -> running() && follower.completion() >= t),
                command));
        return this;
    }

    /** T-триггер для конкретного сегмента цепочки (по pathIndex). */
    public Follow atT(int segment, double t, Command command) {
        alongside.add(sequential(
                waitUntil(() -> running()
                        && follower.pathIndex() == segment
                        && follower.completion() >= t),
                command));
        return this;
    }

    /** Запустить, когда до конца пути осталось <= inches. */
    public Follow atRemaining(double inches, Command command) {
        alongside.add(sequential(
                waitUntil(() -> running() && follower.remainingDistance() <= inches),
                command));
        return this;
    }

    /** Запустить, когда выполнится произвольное условие. */
    public Follow when(BooleanSupplier condition, Command command) {
        alongside.add(sequential(
                waitUntil(() -> running() && condition.getAsBoolean()),
                command));
        return this;
    }

    /**
     * По прибытии отменять и команды из alongside/atT (по умолчанию ждём, пока они доработают).
     * Осторожно: триггер с t, до которого путь не дошёл, иначе будет висеть вечно.
     */
    public Follow cancelOthersOnArrival() {
        this.cancelAlongsideOnArrival = true;
        return this;
    }

    // ============================================================
    //  УСЛОВИЯ ЗАВЕРШЕНИЯ
    // ============================================================

    /** Прекратить езду, как только условие стало true. */
    public Follow until(BooleanSupplier condition) {
        this.untilCondition = condition;
        return this;
    }

    /** Прекратить езду через timeoutMs миллисекунд, если не доехали. */
    public Follow timeout(long timeoutMs) {
        this.timeoutMs = timeoutMs;
        return this;
    }

    /** Закончить, когда follower.atParametricEnd(), не дожидаясь полной остановки. */
    public Follow endAtParametric() {
        this.endAtParametric = true;
        return this;
    }

    /** Закончить, когда до цели < inches и скорость ниже velocityConstraint. */
    public Follow lenient(double inches) {
        this.lenientInches = inches;
        return this;
    }

    /** После окончания держать позицию (hold). */

    // ============================================================
    //  СБОРКА
    // ============================================================

    public Command build() {
        // 1. сам путь с локальными модификаторами (применяются в момент старта)
        Command main = lazy(() -> follow(follower, applyModifiers()));

        // 2. досрочные завершения
        if (untilCondition != null) {
            main = main.until(untilCondition);
        }
        if (endAtParametric) {
            main = race(main, waitUntil(() -> running() && follower.atParametricEnd()));
        }
        if (lenientInches >= 0) {
            main = race(main, waitUntil(() -> running()
                    && follower.distanceToEndpoint() < lenientInches
                    && Math.abs(follower.tangentialVelocity())
                    < Constants.foresightConfig.velocityConstraint.get()));
        }
        if (timeoutMs >= 0) {
            main = race(main, waitMs(timeoutMs));
        }

        // 3. команды, которые отменяются по прибытии
        List<Command> cancelled = new ArrayList<>(during);
        if (cancelAlongsideOnArrival) {
            cancelled.addAll(alongside);
        }
        if (!cancelled.isEmpty()) {
            main = deadline(main, cancelled.toArray(new Command[0]));
        }

        // 4. команды, которые ждём
        if (!cancelAlongsideOnArrival && !alongside.isEmpty()) {
            Command[] all = new Command[alongside.size() + 1];
            all[0] = main;
            for (int i = 0; i < alongside.size(); i++) {
                all[i + 1] = alongside.get(i);
            }
            main = parallel(all);
        }

        // 5. hold
        if (holdAtEnd) {
            main = sequential(main, hold(follower));
        }
        return main;
    }

    private Path applyModifiers() {
        Path p = path;
        if (maxPathSpeed != null) {
            p = p.with(Constants.foresightConfig.maxPathSpeed.at(maxPathSpeed));
        }
        if (brakeAggression != null) {
            p = p.with(Constants.foresightConfig.brakeAggression.at(brakeAggression));
        }
        if (decelScale != null) {
            p = p.with(Constants.foresightConfig.maxDecelerationScale.at(decelScale));
        }
        if (headingRatio != null) {
            p = p.with(Constants.foresightConfig.headingDriveRatio.at(headingRatio));
        }
        return p;
    }

    /** Пока follower не занят, completion()/atParametricEnd() могут показывать данные прошлого пути. */
    private boolean running() {
        return follower.isBusy();
    }
}
