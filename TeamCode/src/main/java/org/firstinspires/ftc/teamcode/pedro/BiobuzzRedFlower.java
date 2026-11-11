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
 * Red, flower: no intake, transport or shooter.
 *
 *   1  from START (same as BiobuzzRedAuto, pressed against the x = 0 wall), drive straight
 *      off the wall to OFF_WALL without turning -- turning right at the wall would swing a
 *      corner into it
 *   2  curve to FLOWER through CURVE_CONTROL, turning TURN_DEG on the way (negative =
 *      clockwise) -- a curve, not a straight line, so it stays clear of the basket floor
 *   3  flower servo to FLOWER_POSITION, then wait FLOWER_WAIT_MS for it to get there
 *   4  drive BACKWARD_IN straight backward (away from the way the robot now faces)
 *   5  stay there for STAY_MS
 *   6  drive BACKWARD_2_IN further backward
 *   7  drive FORWARD_IN straight forward, still not turning -- it is right by the top wall
 *      and the flower, where turning would swing a corner into them
 *   8  flower servo back to FLOWER_REST_POSITION, wait FLOWER_WAIT_MS
 *   9  curve to SHOOT through SHOOT_CONTROL, turning SHOOT_TURN_DEG more (negative =
 *      clockwise), to the spot where BiobuzzRedAuto shoots its second load
 *  10  drive SHOOT_FORWARD_IN straight forward
 *  11  flower servo down again (FLOWER_POSITION), wait FLOWER_WAIT_MS
 *  12  drive to the second flower spot: SHOOT's x, SHOOT's y + FLOWER_2_DY
 *  13  drive FLOWER_2_FORWARD_IN straight forward
 *  14  flower servo up (FLOWER_REST_POSITION), wait FLOWER_WAIT_MS
 *  15  drive back to SHOOT and hold
 * Steps 10-15 keep the heading the robot had at SHOOT.
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
    public static double FLOWER_X = 54, FLOWER_Y = 112;
    public static double CURVE_CONTROL_X = 19, CURVE_CONTROL_Y = 108;
    /** Turn during the curve. -90 = 90 degrees clockwise; +90 would be anti-clockwise. */
    public static double TURN_DEG = -90;

    public static String FLOWER_SERVO_NAME = "flowerGate";
    public static double FLOWER_POSITION = 0.65;
    /** Where the flower servo goes after the second pull back. */
    public static double FLOWER_REST_POSITION = 0.1;
    /** Time for the servo to reach FLOWER_POSITION before driving on. */
    public static double FLOWER_WAIT_MS = 500;

    /** How far to drive straight backward after the flower servo moves. */
    public static double BACKWARD_IN = 10;
    /** How long to stay there before the second pull back. */
    public static double STAY_MS = 500;
    /** Second pull back, after STAY_MS. */
    public static double BACKWARD_2_IN = 0.1;

    /** Straight forward after the second pull back, before the servo goes back (no turning). */
    public static double FORWARD_IN = 10;
    /** Shooting spot (same as BiobuzzRedAuto's second shot area), and the curve's bend point. */
    public static double SHOOT_X = 134, SHOOT_Y = 69;
    public static double SHOOT_CONTROL_X = 108, SHOOT_CONTROL_Y = 110;
    /** Turn on the way to SHOOT. -90 = another 90 degrees clockwise. */
    public static double SHOOT_TURN_DEG = -90;

    /** After reaching SHOOT: straight forward this far before the flower servo goes down. */
    public static double SHOOT_FORWARD_IN = 10;
    /** Second flower spot: SHOOT's x, and this much more y than SHOOT. */
    public static double FLOWER_2_DY = 10;
    /** From the second flower spot: straight forward this far before the servo goes up. */
    public static double FLOWER_2_FORWARD_IN = 10;

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
        Pose behind2 = p.of(behind.x() - BACKWARD_2_IN * Math.cos(h), behind.y() - BACKWARD_2_IN * Math.sin(h), flowerHeading);
        Pose away = p.of(behind2.x() + FORWARD_IN * Math.cos(h), behind2.y() + FORWARD_IN * Math.sin(h), flowerHeading);
        double shootHeading = flowerHeading + SHOOT_TURN_DEG;
        Pose shoot = p.of(SHOOT_X, SHOOT_Y, shootHeading);
        double hs = Math.toRadians(shootHeading);
        Pose shootAhead = p.of(SHOOT_X + SHOOT_FORWARD_IN * Math.cos(hs), SHOOT_Y + SHOOT_FORWARD_IN * Math.sin(hs), shootHeading);
        Pose flower2 = p.of(SHOOT_X, SHOOT_Y + FLOWER_2_DY, shootHeading);
        Pose flower2Ahead = p.of(SHOOT_X + FLOWER_2_FORWARD_IN * Math.cos(hs),
                SHOOT_Y + FLOWER_2_DY + FLOWER_2_FORWARD_IN * Math.sin(hs), shootHeading);
        follower.setPose(start);

        Path awayFromWall = line(start, offWall).constant(start);
        Path toFlower = curve(offWall, p.of(CURVE_CONTROL_X, CURVE_CONTROL_Y, flowerHeading), atFlower)
                .linear(offWall, atFlower);
        Path backward = line(atFlower, behind).constant(atFlower);
        Path backward2 = line(behind, behind2).constant(atFlower);
        Path offTheWall = line(behind2, away).constant(atFlower);
        Path toShoot = curve(away, p.of(SHOOT_CONTROL_X, SHOOT_CONTROL_Y, flowerHeading), shoot)
                .linear(away, shoot);
        Path shootForward = line(shoot, shootAhead).constant(shoot);
        Path toFlower2 = line(shootAhead, flower2).constant(shoot);
        Path flower2Forward = line(flower2, flower2Ahead).constant(shoot);
        Path backToShoot = line(flower2Ahead, shoot).constant(shoot);

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

                // 4. Straight backward.
                instant(() -> step = "backward " + BACKWARD_IN + " in"),
                driveWithTimeout(backward),

                // 5. Stay.
                instant(() -> {
                    follower.hold(follower.pose());
                    step = "staying " + STAY_MS + " ms";
                }),
                waitMs(STAY_MS),

                // 6. Further backward.
                instant(() -> step = "backward " + BACKWARD_2_IN + " in"),
                driveWithTimeout(backward2),

                // 7. Forward, off the wall and the flower, still not turning.
                instant(() -> step = "forward " + FORWARD_IN + " in"),
                driveWithTimeout(offTheWall),

                // 8. Flower servo back.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_REST_POSITION);
                    }
                    step = "flower servo to " + FLOWER_REST_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),

                // 9. Curve to the shooting spot, turning on the way.
                instant(() -> step = "curving to the shooting spot"),
                driveWithTimeout(toShoot),

                // 10. Forward.
                instant(() -> step = "forward " + SHOOT_FORWARD_IN + " in"),
                driveWithTimeout(shootForward),

                // 11. Flower servo down again.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_POSITION);
                    }
                    step = "flower servo to " + FLOWER_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),

                // 12. To the second flower spot.
                instant(() -> step = "to the second flower spot"),
                driveWithTimeout(toFlower2),

                // 13. Forward.
                instant(() -> step = "forward " + FLOWER_2_FORWARD_IN + " in"),
                driveWithTimeout(flower2Forward),

                // 14. Flower servo up.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_REST_POSITION);
                    }
                    step = "flower servo to " + FLOWER_REST_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),

                // 15. Back to the shooting spot, then hold.
                instant(() -> step = "back to the shooting spot"),
                driveWithTimeout(backToShoot),
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
