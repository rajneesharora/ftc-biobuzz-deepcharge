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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Blue, flower: BiobuzzRedFlower turned half a turn around the field center -- every point
 * (x, y) -> (144 - x, 144 - y), START heading + 180 (the same way BiobuzzBlueAuto is made from
 * BiobuzzRedAuto). Turns and forward/backward moves are relative to the robot, so they stay the
 * same; FLOWER_2_DY is a field y offset, so its sign flips. Kept as its own file so Blue can be
 * tuned on its own -- a change to the steps in Red has to be copied here by hand.
 *
 * Blue, flower: shoot the preloads, pick up balls at two flowers, shoot each load, park.
 *
 * Mechanisms (this robot has no gate): the flywheel runs from PLAY until parking. While
 * picking up, the intake + transportLeft run and transportRight stays still, so the balls
 * wait. Shooting = transportRight on (with the intake + transportLeft) for SHOOT_MS.
 * FEED_ON_ARRIVAL = true starts transportRight when the robot reaches the shooting spot;
 * false starts it as soon as a flower pick-up is done, i.e. while still driving.
 *
 *   0  at START: wait SPIN_UP_MS for the flywheel, shoot the preloads for PRELOAD_SHOOT_MS
 *
 *   1  from START (same as BiobuzzBlueAuto, pressed against the x = 144 wall), drive straight
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
 *   9  straight across to CROSS, away from the hive and still not turning -- through the
 *      middle it hits the hive base -- then curve to SHOOT through SHOOT_CONTROL, turning
 *      SHOOT_TURN_DEG more (negative = clockwise), to the second shooting spot. Coming in this
 *      way keeps it off the second flower at (0, 48).
 *  10  drive SHOOT_FORWARD_IN straight forward
 *  11  slide sideways FLOWER_2_DY in y (still not turning)
 *  12  flower servo down again (FLOWER_POSITION), wait FLOWER_WAIT_MS -- after the slide, so
 *      it can't sweep into anything on the way
 *  13  straight back into the second flower spot (SHOOT's x, SHOOT's y + FLOWER_2_DY) and stay
 *      there for FLOWER_2_STAY_MS (picking up)
 *  14  drive FLOWER_2_FORWARD_IN straight forward
 *  15  flower servo up (FLOWER_REST_POSITION), wait FLOWER_WAIT_MS
 *  16  slide sideways back by FLOWER_2_DY, then straight back to SHOOT -- so steps 10-16 make a
 *      U of straight moves, never a slanted one -- and shoot that load for SHOOT_MS; then the
 *      flywheel, intake and transport go off
 *  17  park at PARK (Red's park, turned), curving through PARK_CONTROL, and hold
 * Steps 10-17 keep the heading the robot had at SHOOT.
 *
 * The flower servo is not moved before step 3, so it doesn't jump at INIT or PLAY.
 *
 * Same field coordinates as the Red programs: (0,0) is the bottom-right corner, poses are the
 * robot's CENTER, heading 0 = forward points +x, counter-clockwise positive. Every drive has a
 * time limit, so the robot never pushes forever at a point it can't reach.
 *
 * All values are live-editable in Panels (Configurables > BiobuzzBlueFlower), read at PLAY.
 */
@Configurable
@Autonomous(name = "Biobuzz Blue Flower", group = "Autonomous")
public class BiobuzzBlueFlower extends OpMode {

    /** Where the robot is placed (same as BiobuzzBlueAuto). */
    public static double START_X = 135, START_Y = 64, START_HEADING_DEG = 180;
    /** Straight off the wall first, keeping the start heading. */
    public static double OFF_WALL_X = 123, OFF_WALL_Y = 64;
    /** Where the curve ends, and the point it bows through (the robot does not drive through it). */
    public static double FLOWER_X = 90, FLOWER_Y = 32;
    public static double CURVE_CONTROL_X = 125, CURVE_CONTROL_Y = 36;
    /** Turn during the curve. -90 = 90 degrees clockwise; +90 would be anti-clockwise. */
    public static double TURN_DEG = -90;

    public static String FLOWER_SERVO_NAME = "flowerGate";
    public static double FLOWER_POSITION = 0.7;
    /** Where the flower servo goes after the second pull back. */
    public static double FLOWER_REST_POSITION = 0.1;
    /** Time for the servo to reach FLOWER_POSITION before driving on. */
    public static double FLOWER_WAIT_MS = 1000;

    /** How far to drive straight backward after the flower servo moves. */
    public static double BACKWARD_IN = 11;
    /** How long to stay there before the second pull back. */
    public static double STAY_MS = 1000;
    /** Second pull back, after STAY_MS. */
    public static double BACKWARD_2_IN = 0.1;

    /** Straight forward after the second pull back, before the servo goes back (no turning). */
    public static double FORWARD_IN = 10;
    /** End of the straight crossing, away from the hive (Red's crossing, turned). */
    public static double CROSS_X = 36, CROSS_Y = 38;
    /** Shooting spot (Red's, turned), and the curve's bend point. */
    public static double SHOOT_X = 10, SHOOT_Y = 74;
    public static double SHOOT_CONTROL_X = 29, SHOOT_CONTROL_Y = 72;
    /** Turn on the way to SHOOT. -90 = another 90 degrees clockwise. */
    public static double SHOOT_TURN_DEG = -90;

    /** After reaching SHOOT: straight forward this far before the flower servo goes down. */
    public static double SHOOT_FORWARD_IN = 12;
    /** Second flower spot: SHOOT's x, and SHOOT's y plus this (negative = lower y; Red's +17 turned). */
    public static double FLOWER_2_DY = -17;
    /** How long to stay at the second flower spot, picking up. */
    public static double FLOWER_2_STAY_MS = 1000;
    /** From the second flower spot: straight forward this far before the servo goes up. */
    public static double FLOWER_2_FORWARD_IN = 12;

    /** Parking spot (Red's, turned), and the point the curve bows through. */
    public static double PARK_X = 48, PARK_Y = 25;
    public static double PARK_CONTROL_X = 44, PARK_CONTROL_Y = 61;

    /** Flywheel spin-up before the first shot (it starts at PLAY and runs until parking). */
    public static double SPIN_UP_MS = 2000;
    /** How long transportRight feeds the preloads into the flywheel. Raise it if a ball is left. */
    public static double PRELOAD_SHOOT_MS = 3000;
    /** How long transportRight feeds the flywheel for each picked-up load. */
    public static double SHOOT_MS = 1500;
    /** true = start feeding when the robot reaches the shooting spot; false = right after a pick-up. */
    public static boolean FEED_ON_ARRIVAL = true;

    /** Safety limit: give up on a drive after this long and carry on. */
    public static double PATH_TIMEOUT_MS = 5000;

    private final PoseFactory p = PoseFactory.degrees();

    private Follower follower;
    private Dashboard dashboard;
    private Servo flowerServo;   // null if missing from the robot config
    private Intake intake;
    private Transport transport;
    private Shooter shooter;
    private final ElapsedTime autoTimer = new ElapsedTime();

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
        intake = new Intake(hardwareMap);
        transport = new Transport(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(startPose());

        dashboard.addLine("Biobuzz Blue Flower -- robot at the start pose, heading " + START_HEADING_DEG + ".");
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
        Pose besideFlower2 = p.of(shootAhead.x(), shootAhead.y() + FLOWER_2_DY, shootHeading);
        Pose flower2 = p.of(SHOOT_X, SHOOT_Y + FLOWER_2_DY, shootHeading);
        Pose flower2Ahead = p.of(SHOOT_X + FLOWER_2_FORWARD_IN * Math.cos(hs),
                SHOOT_Y + FLOWER_2_DY + FLOWER_2_FORWARD_IN * Math.sin(hs), shootHeading);
        Pose besideShoot = p.of(flower2Ahead.x(), flower2Ahead.y() - FLOWER_2_DY, shootHeading);
        follower.setPose(start);

        Path awayFromWall = line(start, offWall).constant(start);
        Path toFlower = curve(offWall, p.of(CURVE_CONTROL_X, CURVE_CONTROL_Y, flowerHeading), atFlower)
                .linear(offWall, atFlower);
        Path backward = line(atFlower, behind).constant(atFlower);
        Path backward2 = line(behind, behind2).constant(atFlower);
        Path offTheWall = line(behind2, away).constant(atFlower);
        Pose cross = p.of(CROSS_X, CROSS_Y, flowerHeading);
        Path across = line(away, cross).constant(away);
        Path toShoot = curve(cross, p.of(SHOOT_CONTROL_X, SHOOT_CONTROL_Y, flowerHeading), shoot)
                .linear(cross, shoot);
        Path shootForward = line(shoot, shootAhead).constant(shoot);
        Path sideToFlower2 = line(shootAhead, besideFlower2).constant(shoot);
        Path intoFlower2 = line(besideFlower2, flower2).constant(shoot);
        Path flower2Forward = line(flower2, flower2Ahead).constant(shoot);
        Path sideBack = line(flower2Ahead, besideShoot).constant(shoot);
        Path backToShoot = line(besideShoot, shoot).constant(shoot);
        Pose park = p.of(PARK_X, PARK_Y, shootHeading);
        Path toPark = curve(shoot, p.of(PARK_CONTROL_X, PARK_CONTROL_Y, shootHeading), park).constant(shoot);

        step = "off the wall";
        autoTimer.reset();
        shooter.setSpinning(true);
        step = "flywheel spinning up";
        schedule(sequential(
                // 0. Shoot the preloads, then go into pick-up mode (right side holds the balls).
                waitMs(SPIN_UP_MS),
                instant(() -> {
                    feed();
                    step = "shooting the preloads";
                }),
                waitMs(PRELOAD_SHOOT_MS),
                instant(this::pickUp),

                // 1. Straight off the wall.
                instant(() -> step = "off the wall"),
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
                instant(this::feedNowIfNotOnArrival),

                // 9. Across at a high y, then curve down to the shooting spot, turning on the way.
                instant(() -> step = "crossing (past the hive)"),
                driveWithTimeout(across),
                instant(() -> step = "curving to the shooting spot"),
                driveWithTimeout(toShoot),
                instant(() -> {
                    follower.hold(follower.pose());
                    feed();
                    step = "shooting load 1";
                }),
                waitMs(SHOOT_MS),
                instant(this::pickUp),

                // 10. Forward.
                instant(() -> step = "forward " + SHOOT_FORWARD_IN + " in"),
                driveWithTimeout(shootForward),

                // 11. Sideways, level with the second flower.
                instant(() -> step = "sideways " + FLOWER_2_DY + " in (y)"),
                driveWithTimeout(sideToFlower2),

                // 12. Flower servo down again.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_POSITION);
                    }
                    step = "flower servo to " + FLOWER_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),

                // 13. Straight back into the second flower spot.
                instant(() -> step = "back into the second flower spot"),
                driveWithTimeout(intoFlower2),
                instant(() -> {
                    follower.hold(follower.pose());
                    step = "staying at flower 2, " + FLOWER_2_STAY_MS + " ms";
                }),
                waitMs(FLOWER_2_STAY_MS),

                // 14. Forward.
                instant(() -> step = "forward " + FLOWER_2_FORWARD_IN + " in"),
                driveWithTimeout(flower2Forward),

                // 15. Flower servo up.
                instant(() -> {
                    follower.hold(follower.pose());
                    if (flowerServo != null) {
                        flowerServo.setPosition(FLOWER_REST_POSITION);
                    }
                    step = "flower servo to " + FLOWER_REST_POSITION;
                }),
                waitMs(FLOWER_WAIT_MS),
                instant(this::feedNowIfNotOnArrival),

                // 16. Sideways back, straight back to the shooting spot and shoot; then
                //     everything off for parking.
                instant(() -> step = "sideways back " + FLOWER_2_DY + " in (y)"),
                driveWithTimeout(sideBack),
                instant(() -> step = "back to the shooting spot"),
                driveWithTimeout(backToShoot),
                instant(() -> {
                    follower.hold(follower.pose());
                    feed();
                    step = "shooting load 2";
                }),
                waitMs(SHOOT_MS),
                instant(() -> {
                    shooter.setSpinning(false);
                    intake.setState(Intake.State.OFF);
                    transport.setState(Transport.State.OFF);
                }),

                // 17. Park, then hold.
                instant(() -> step = "parking"),
                driveWithTimeout(toPark),
                instant(() -> {
                    follower.hold(follower.pose());
                    endedAt = where();
                    step = "done -- parked at " + Math.round(autoTimer.seconds() * 10) / 10.0 + " s";
                })
        ));
    }

    /** Pick-up mode: intake + transportLeft pull balls in; transportRight still, so they wait. */
    private void pickUp() {
        intake.setState(Intake.State.INTAKE);
        transport.setSides(Transport.State.FEED, Transport.State.OFF);
    }

    /** Shoot: transportRight joins in and carries the balls into the (spinning) flywheel. */
    private void feed() {
        intake.setState(Intake.State.INTAKE);
        transport.setSides(Transport.State.FEED, Transport.State.FEED);
    }

    /** FEED_ON_ARRIVAL = false: start feeding as soon as the pick-up is done. */
    private void feedNowIfNotOnArrival() {
        if (!FEED_ON_ARRIVAL) {
            feed();
        }
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

        intake.update();
        transport.update();
        shooter.update();

        Pose pose = follower.pose();
        PoseStorage.save(pose);

        dashboard.addData("Step", step);
        dashboard.addData("Time (s)", Math.round(autoTimer.seconds() * 10) / 10.0 + "  (auto is 30 s)");
        dashboard.addData("Ended at", endedAt);
        dashboard.showPose(pose);
        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        dashboard.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        follower.stop();
        intake.stop();
        transport.stop();
        shooter.stop();
    }
}
