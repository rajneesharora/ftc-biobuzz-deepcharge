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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.lib.Battery;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Blue: BiobuzzRedAuto turned half a turn around the field center -- every point
 * (x, y) -> (144 - x, 144 - y), every heading + 180 (the robot faces the audience). Left and
 * right stay the same as in Red, and the 180-degree turn goes the same way. Kept as its own
 * file so Blue can be tuned on its own -- a change to the steps in Red has to be copied here
 * by hand.
 *
 *   1  at START: flywheel spins up, then intake + transport + gate run -> shoot
 *   2  flywheel and gate stop; intake + transport keep running (balls wait at the gate)
 *   3  curve (bowing away from the wall) to APPROACH, just before the balls
 *   4  creep slowly along the row to INTAKE_END, intake first, eating the balls one after
 *      another -- then wait INTAKE_WAIT_MS
 *   5  intake + transport stop; curve to CROSS_START
 *   6  straight across to CROSS_END
 *   7  straight to TURN, turning 180 degrees on the way
 *   8  straight to SHOOT_2; flywheel spins up, then intake + transport + gate -> shoot again
 *   9  curve to PARK
 *
 * The heading is HEADING_DEG up to CROSS_END and HEADING_DEG + 180 from TURN on. The creep
 * drives in a straight line from APPROACH to INTAKE_END in any direction (it works out
 * forward/sideways from the heading), so moving the balls only needs new numbers.
 *
 * Field, same coordinates as Red: (0,0) is the bottom-right corner. Blue starts against the
 * x = 144 wall, and its balls lie along that wall at low y. Heading 0 = the robot's forward
 * points +x; 180 = forward points -x (away from the x = 144 wall). Poses are the robot's
 * CENTER (robot 18" x 18", so the center is 9" from a wall it touches). Targets stay 2-3" off
 * the walls, and every drive has a time limit, so the robot never pushes forever at a point
 * it can't reach.
 *
 * All values are live-editable in Panels (Configurables > BiobuzzBlueAuto), read at PLAY.
 * Needs this robot's Pedro tuning finished first.
 */
@Configurable
@Autonomous(name = "Biobuzz Blue Shoot Auto", group = "Autonomous")
public class BiobuzzBlueAuto extends OpMode {

    public static double HEADING_DEG = 180;

    /** Where the robot is placed: pushed against the x = 144 wall, so x = 135 (136 is inside it). */
    public static double START_X = 135, START_Y = 64;

    /** End of the curve, clear of the balls (they lie from (144,0) to (144,6)). */
    public static double APPROACH_X = 129, APPROACH_Y = 16;
    /** Bends the curve down and away from the wall. The robot does not drive through it. */
    public static double CONTROL_X = 108, CONTROL_Y = 40;
    /** End of the creep, where the robot sits after taking the balls. */
    public static double INTAKE_END_X = 133, INTAKE_END_Y = 16;

    /** Start of the crossing, reached by a curve from INTAKE_END that bows through CROSS_CONTROL. */
    public static double CROSS_START_X = 96, CROSS_START_Y = 34;
    public static double CROSS_CONTROL_X = 114, CROSS_CONTROL_Y = 26;
    /** End of the straight crossing to the far side. Kept high to clear the flower at (94, 4). */
    public static double CROSS_END_X = 36, CROSS_END_Y = 38;
    /** The robot turns 180 degrees while driving from CROSS_END to here. */
    public static double TURN_X = 24, TURN_Y = 76;
    /**
     * Second shooting spot, turned from Red's field-tuned one. The robot backs in (heading 360,
     * i.e. 0) and the surgical tube sticks ~3" out of its back, so it must stay well off the
     * x = 0 wall (x = 11 would put the tube in the wall); high y keeps it clear of the flower
     * at (0, 48).
     */
    public static double SHOOT_2_X = 18, SHOOT_2_Y = 77;
    /** Parking spot, reached by a curve that bows through PARK_CONTROL -- so the robot pulls
     *  away from the (0, 48) flower before going down. */
    public static double PARK_X = 54, PARK_Y = 27;
    public static double PARK_CONTROL_X = 44, PARK_CONTROL_Y = 66;

    /** Power while creeping along the balls (0-1). Lower = slower, gentler intake. */
    public static double CREEP_POWER = 0.3;

    public static double SPIN_UP_1_MS = 3000;
    public static double SPIN_UP_2_MS = 2000;
    /** Long enough for ALL balls to go through. Raise it if a ball is left behind. */
    public static double SHOOT_MS = 2000;
    /** Extra time at the end of the row so the last ball gets pulled in. */
    public static double INTAKE_WAIT_MS = 1000;
    /** Safety limits: give up on a drive after this long and carry on. */
    public static double PATH_TIMEOUT_MS = 5000;
    public static double CREEP_TIMEOUT_MS = 4000;

    private final PoseFactory p = PoseFactory.degrees();
    private final ElapsedTime creepTimer = new ElapsedTime();

    private Follower follower;
    private Dashboard dashboard;
    private Intake intake;
    private Transport transport;
    private Shooter shooter;

    private String step = "waiting for PLAY";

    private MatchRecorder recorder;
    private String recordedStep = "";
    /** Where the robot really was at the second shot and at the end -- stays on screen. */
    private String shot2At = "--";
    private String parkedAt = "--";

    private Pose pose(double x, double y) {
        return p.of(x, y, HEADING_DEG);
    }

    /** Same, but facing the other way (for the far side). */
    private Pose turnedPose(double x, double y) {
        return p.of(x, y, HEADING_DEG + 180);
    }

    @Override
    public void init() {
        Scheduler.reset();
        dashboard = new Dashboard(telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(pose(START_X, START_Y));

        intake = new Intake(hardwareMap);
        transport = new Transport(hardwareMap);
        shooter = new Shooter(hardwareMap);
        recorder = new MatchRecorder("auto_blue", new Battery(hardwareMap));
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(pose(START_X, START_Y));

        dashboard.addLine("Biobuzz Blue Shoot Auto -- robot at the start pose, heading " + HEADING_DEG + ".");
        dashboard.addData("Start (x, y, heading)", START_X + ", " + START_Y + ", " + HEADING_DEG);
        dashboard.showPose(follower.pose());
        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        dashboard.update();
        record();
    }

    @Override
    public void start() {
        recorder.setPhase("RUN");
        Pose start = pose(START_X, START_Y);
        Pose approach = pose(APPROACH_X, APPROACH_Y);
        Pose intakeEnd = pose(INTAKE_END_X, INTAKE_END_Y);
        Pose crossStart = pose(CROSS_START_X, CROSS_START_Y);
        Pose crossEnd = pose(CROSS_END_X, CROSS_END_Y);
        Pose turn = turnedPose(TURN_X, TURN_Y);
        Pose shootSpot2 = turnedPose(SHOOT_2_X, SHOOT_2_Y);
        Pose park = turnedPose(PARK_X, PARK_Y);
        follower.setPose(start);

        Path toBalls = curve(start, pose(CONTROL_X, CONTROL_Y), approach).constant(start);
        Path toCross = curve(intakeEnd, pose(CROSS_CONTROL_X, CROSS_CONTROL_Y), crossStart).constant(start);
        Path across = line(crossStart, crossEnd).constant(start);
        Path turnAround = line(crossEnd, turn).linear(crossEnd, turn);
        Path toShoot2 = line(turn, shootSpot2).constant(turn);
        Path toPark = curve(shootSpot2, turnedPose(PARK_CONTROL_X, PARK_CONTROL_Y), park).constant(turn);

        shooter.setSpinning(true);
        step = "spinning up (1)";

        schedule(sequential(
                // 1. Shoot the preloads.
                waitMs(SPIN_UP_1_MS),
                instant(() -> {
                    intake.setState(Intake.State.INTAKE);
                    transport.setState(Transport.State.FEED);
                    shooter.setGateTestOpen(true);
                    step = "shooting (1)";
                }),
                waitMs(SHOOT_MS),

                // 2. Shooter off; intake + transport keep running, balls wait at the gate.
                instant(() -> {
                    shooter.setGateTestOpen(false);
                    shooter.setSpinning(false);
                    step = "curving to the balls";
                }),

                // 3. Curve around to just right of the balls.
                driveWithTimeout(toBalls),

                // 4. Creep backwards along the row, then let the last ball in.
                instant(() -> {
                    creepTimer.reset();
                    // Field direction to the end of the row, turned into the robot's own
                    // forward / left, since manual() drives relative to the robot.
                    double[] dir = creepDirection();
                    double h = Math.toRadians(HEADING_DEG);
                    double forward = dir[0] * Math.cos(h) + dir[1] * Math.sin(h);
                    double left = -dir[0] * Math.sin(h) + dir[1] * Math.cos(h);
                    follower.manual(CREEP_POWER * forward, CREEP_POWER * left, 0);
                    step = "intaking along the wall";
                }),
                waitUntil(() -> pastIntakeEnd(follower.pose())
                        || creepTimer.milliseconds() > CREEP_TIMEOUT_MS),
                instant(() -> follower.hold(follower.pose())),
                waitMs(INTAKE_WAIT_MS),

                // 5-7. Cross to the far side, turning around on the last leg.
                instant(() -> {
                    intake.setState(Intake.State.OFF);
                    transport.setState(Transport.State.OFF);
                    step = "crossing to the far side";
                }),
                driveWithTimeout(toCross),
                driveWithTimeout(across),
                instant(() -> step = "turning around"),
                driveWithTimeout(turnAround),

                // 8. Up to the far wall, then spin up and shoot.
                instant(() -> step = "driving to shoot (2)"),
                driveWithTimeout(toShoot2),
                instant(() -> {
                    follower.hold(follower.pose());
                    shooter.setSpinning(true);
                    shot2At = where();
                    step = "spinning up (2)";
                }),
                waitMs(SPIN_UP_2_MS),
                instant(() -> {
                    intake.setState(Intake.State.INTAKE);
                    transport.setState(Transport.State.FEED);
                    shooter.setGateTestOpen(true);
                    step = "shooting (2)";
                }),
                waitMs(SHOOT_MS),

                // 9. Park.
                instant(() -> {
                    shooter.setGateTestOpen(false);
                    shooter.setSpinning(false);
                    intake.setState(Intake.State.OFF);
                    transport.setState(Transport.State.OFF);
                    step = "parking";
                }),
                driveWithTimeout(toPark),
                instant(() -> {
                    follower.hold(follower.pose());
                    parkedAt = where();
                    step = "done -- parked";
                })
        ));
    }

    /** Unit vector (x, y) on the field from APPROACH to INTAKE_END. */
    private static double[] creepDirection() {
        double dx = INTAKE_END_X - APPROACH_X;
        double dy = INTAKE_END_Y - APPROACH_Y;
        double length = Math.hypot(dx, dy);
        return length == 0 ? new double[] {0, 0} : new double[] {dx / length, dy / length};
    }

    /** True once the robot has reached (or passed) INTAKE_END along the creep direction. */
    private static boolean pastIntakeEnd(Pose pose) {
        double[] dir = creepDirection();
        return (pose.x() - INTAKE_END_X) * dir[0] + (pose.y() - INTAKE_END_Y) * dir[1] >= 0;
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
        dashboard.addData("Shot 2 from", shot2At);
        dashboard.addData("Parked at", parkedAt);
        dashboard.showPose(pose);
        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        dashboard.update();
        record();
    }

    /**
     * One row of the match recording (/sdcard/FIRST/analysis/auto_blue_*.csv): the step, the
     * pose and every mechanism. Each step change is also written as an event, so the file shows
     * how long every step really took.
     */
    private void record() {
        Pose pose = follower.pose();
        recorder.put("step", step);
        recorder.put("x", pose.x());
        recorder.put("y", pose.y());
        recorder.put("heading_deg", Math.toDegrees(pose.heading()));
        recorder.put("pedro_mode", PedroDrive.modeName(follower));
        intake.record(recorder);
        transport.record(recorder);
        shooter.record(recorder);
        if (!step.equals(recordedStep)) {
            recorder.event("step: " + step);
            recordedStep = step;
        }
        recorder.endLoop();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        follower.stop();
        intake.stop();
        transport.stop();
        shooter.stop();
        recorder.setPhase("STOP");
        recorder.endLoop();
        recorder.close();
    }
}
