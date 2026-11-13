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
 * Red, flower: shoot the preloads, pick up balls at two flowers, shoot each load, park.
 *
 * Mechanisms (this robot has no gate): the flywheel runs from PLAY until parking. While
 * picking up, the intake + transportLeft run and transportRight stays still, so the balls
 * wait. Shooting = transportRight on (with the intake + transportLeft) for SHOOT_MS.
 * FEED_ON_ARRIVAL = true starts transportRight when the robot reaches the shooting spot;
 * false starts it as soon as a flower pick-up is done, i.e. while still driving.
 *
 *   0  at START: wait SPIN_UP_MS for the flywheel, shoot the preloads for SHOOT_MS
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
 *   9  straight across at a high y to CROSS, still not turning -- low through the middle it
 *      hits the hive base -- then curve down to SHOOT through SHOOT_CONTROL, turning
 *      SHOOT_TURN_DEG more (negative = clockwise), to where BiobuzzRedAuto shoots its second
 *      load. Coming in low keeps it off the second flower at (144, 96).
 *  10  drive SHOOT_FORWARD_IN straight forward
 *  11  flower servo down again (FLOWER_POSITION), wait FLOWER_WAIT_MS
 *  12  drive to the second flower spot: SHOOT's x, SHOOT's y + FLOWER_2_DY
 *  13  drive FLOWER_2_FORWARD_IN straight forward
 *  14  flower servo up (FLOWER_REST_POSITION), wait FLOWER_WAIT_MS
 *  15  drive back to SHOOT and shoot that load for SHOOT_MS; flywheel, intake, transport off
 *  16  park at PARK (same as BiobuzzRedAuto's park), curving through PARK_CONTROL, and hold
 * Steps 10-16 keep the heading the robot had at SHOOT.
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
    /** End of the straight crossing at a high y (same as BiobuzzRedAuto's crossing). */
    public static double CROSS_X = 108, CROSS_Y = 106;
    /** Shooting spot (same as BiobuzzRedAuto's second shot area), and the curve's bend point. */
    public static double SHOOT_X = 134, SHOOT_Y = 69;
    public static double SHOOT_CONTROL_X = 115, SHOOT_CONTROL_Y = 72;
    /** Turn on the way to SHOOT. -90 = another 90 degrees clockwise. */
    public static double SHOOT_TURN_DEG = -90;

    /** After reaching SHOOT: straight forward this far before the flower servo goes down. */
    public static double SHOOT_FORWARD_IN = 10;
    /** Second flower spot: SHOOT's x, and this much more y than SHOOT. */
    public static double FLOWER_2_DY = 10;
    /** From the second flower spot: straight forward this far before the servo goes up. */
    public static double FLOWER_2_FORWARD_IN = 10;

    /** Parking spot (same as BiobuzzRedAuto), and the point the curve bows through. */
    public static double PARK_X = 90, PARK_Y = 117;
    public static double PARK_CONTROL_X = 100, PARK_CONTROL_Y = 78;

    /** Flywheel spin-up before the first shot (it starts at PLAY and runs until parking). */
    public static double SPIN_UP_MS = 2000;
    /** How long transportRight feeds the flywheel for each load. Raise it if a ball is left. */
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
        Pose cross = p.of(CROSS_X, CROSS_Y, flowerHeading);
        Path across = line(away, cross).constant(away);
        Path toShoot = curve(cross, p.of(SHOOT_CONTROL_X, SHOOT_CONTROL_Y, flowerHeading), shoot)
                .linear(cross, shoot);
        Path shootForward = line(shoot, shootAhead).constant(shoot);
        Path toFlower2 = line(shootAhead, flower2).constant(shoot);
        Path flower2Forward = line(flower2, flower2Ahead).constant(shoot);
        Path backToShoot = line(flower2Ahead, shoot).constant(shoot);
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
                waitMs(SHOOT_MS),
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
                instant(() -> step = "crossing (high, past the hive)"),
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
                instant(this::feedNowIfNotOnArrival),

                // 15. Back to the shooting spot and shoot; then everything off for parking.
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

                // 16. Park, then hold.
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
