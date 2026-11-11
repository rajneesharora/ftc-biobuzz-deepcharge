package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Red, flower only: no intake, transport or shooter.
 *
 *   1  from START (same as BiobuzzRedAuto, pressed against the x = 0 wall), drive straight
 *      off the wall to OFF_WALL without turning -- turning right at the wall would swing a
 *      corner into it
 *   2  curve to FLOWER through CURVE_CONTROL, turning TURN_DEG on the way (negative =
 *      clockwise) -- a curve, not a straight line, so it stays clear of the basket floor
 *   3  flower servo to FLOWER_POSITION, then wait FLOWER_WAIT_MS for it to get there
 *   4  drive BACKWARD_IN straight backward (away from the way the robot now faces) and hold
 *
 * The flower servo is not moved before step 3, so it doesn't jump at INIT or PLAY.
 *
 * Same field coordinates as BiobuzzRedAuto: (0,0) is the bottom-right corner, poses are the
 * robot's CENTER, heading 0 = forward points +x, counter-clockwise positive. Every drive has a
 * time limit, so the robot never pushes forever at a point it can't reach.
 *
 * All values are live-editable in Panels (Configurables > BiobuzzRedFlower), read at PLAY.
 */
@Configurable
@Autonomous(name = "Biobuzz Red Flower", group = "Autonomous")
public class BiobuzzRedFlower extends OpMode {

    /** Where the robot is placed (same as BiobuzzRedAuto). */
    public static double START_X = 9, START_Y = 80, START_HEADING_DEG = 0;
    /** Straight off the wall first, keeping the start heading. */
    public static double OFF_WALL_X = 21, OFF_WALL_Y = 80;
    /** Where the curve ends, and the point it bows through (the robot does not drive through it). */
    public static double FLOWER_X = 48, FLOWER_Y = 120;
    public static double CURVE_CONTROL_X = 21, CURVE_CONTROL_Y = 112;
    /** Turn during the curve. -90 = 90 degrees clockwise; +90 would be anti-clockwise. */
    public static double TURN_DEG = -90;

    public static String FLOWER_SERVO_NAME = "flowerGate";
    public static double FLOWER_POSITION = 0.65;
    /** Time for the servo to reach FLOWER_POSITION before driving on. */
    public static double FLOWER_WAIT_MS = 500;

    /** How far to drive straight backward after the flower servo moves. */
    public static double BACKWARD_IN = 12;

    /** Safety limit: give up on a drive after this long and carry on. */
    public static double PATH_TIMEOUT_MS = 5000;

    private final PoseFactory p = PoseFactory.degrees();

    private Follower follower;
    private Dashboard dashboard;
    private Servo flowerServo;   // null if missing from the robot config

    private String step = "waiting for PLAY";
    /** Where the robot really ended up -- stays on screen. */
    private String endedAt = "--";

    private Pose startPose() {
        return p.of(START_X, START_Y, START_HEADING_DEG);
    }

    @Override
    public void init() {
        Scheduler.reset();
        dashboard = new Dashboard(telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        flowerServo = hardwareMap.tryGet(Servo.class, FLOWER_SERVO_NAME);
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(startPose());

        dashboard.addLine("Biobuzz Red Flower -- robot at the start pose, heading " + START_HEADING_DEG + ".");
        dashboard.addData("Start (x, y, heading)", START_X + ", " + START_Y + ", " + START_HEADING_DEG);
        dashboard.addData("Flower servo", flowerServo == null
                ? "MISSING '" + FLOWER_SERVO_NAME + "' in robot config" : "found (not moved until step 2)");
        dashboard.showPose(follower.pose());
        dashboard.update();
    }

    @Override
    public void start() {
        Pose start = startPose();
        Pose offWall = p.of(OFF_WALL_X, OFF_WALL_Y, START_HEADING_DEG);
        double flowerHeading = START_HEADING_DEG + TURN_DEG;
        Pose atFlower = p.of(FLOWER_X, FLOWER_Y, flowerHeading);
        double h = Math.toRadians(flowerHeading);
        Pose behind = p.of(FLOWER_X - BACKWARD_IN * Math.cos(h), FLOWER_Y - BACKWARD_IN * Math.sin(h), flowerHeading);
        follower.setPose(start);

        Path awayFromWall = line(start, offWall).constant(start);
        Path toFlower = curve(offWall, p.of(CURVE_CONTROL_X, CURVE_CONTROL_Y, flowerHeading), atFlower)
                .linear(offWall, atFlower);
        Path backward = line(atFlower, behind).constant(atFlower);

        step = "off the wall";
        schedule(sequential(
                // 1. Straight off the wall.
                driveWithTimeout(awayFromWall),

                // 2. Curve to the flower, turning on the way.
                instant(() -> step = "curving to the flower"),
                driveWithTimeout(toFlower),

                // 3. Flower servo.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_POSITION);
                    }
                    step = "flower servo to " + FLOWER_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),

                // 4. Straight backward, then hold.
                instant(() -> step = "backward " + BACKWARD_IN + " in"),
                driveWithTimeout(backward),
                instant(() -> {
                    follower.hold(follower.pose());
                    endedAt = where();
                    step = "done -- holding position";
                })
        ));
    }

    /** The robot's pose right now, as "x, y, heading" (inches, degrees). */
    private String where() {
        Pose pose = follower.pose();
        return String.format("%.1f, %.1f, %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    /** Follows a path, but moves on after PATH_TIMEOUT_MS even if the end is never reached. */
    private Command driveWithTimeout(Path path) {
        return race(follow(follower, path), waitMs(PATH_TIMEOUT_MS));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        Pose pose = follower.pose();
        PoseStorage.save(pose);

        dashboard.addData("Step", step);
        dashboard.addData("Ended at", endedAt);
        dashboard.showPose(pose);
        dashboard.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        follower.stop();
    }
}
