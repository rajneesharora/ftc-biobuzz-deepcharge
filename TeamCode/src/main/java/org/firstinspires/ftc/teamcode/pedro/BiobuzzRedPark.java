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

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Red, shoot late and park. Stays at START the whole time it shoots:
 *
 *   1  flywheel spins up at PLAY, then waits WAIT_MS (e.g. for the alliance partner)
 *   2  intake + transport + gate run -> shoot, for SHOOT_MS
 *   3  everything stops; park at PARK along the same curve BiobuzzRedAuto uses after its
 *      second shot (bowing through PARK_CONTROL)
 *
 * The heading stays 180 throughout: the robot's back (and the surgical tube) toward the far
 * x = 144 wall.
 *
 * Same field coordinates as BiobuzzRedAuto: (0,0) is the bottom-right corner, poses are the
 * robot's CENTER, heading 0 = forward points +x. The drive has a time limit, so the robot never
 * pushes forever at a point it can't reach.
 *
 * All values are live-editable in Panels (Configurables > BiobuzzRedPark), read at PLAY.
 * Needs this robot's Pedro tuning finished first.
 */
@Configurable
@Autonomous(name = "Biobuzz Red Park", group = "Autonomous")
public class BiobuzzRedPark extends OpMode {

    public static double HEADING_DEG = 180;

    /** Where the robot is placed. */
    public static double START_X = 122, START_Y = 84;
    /** Parking spot (tuned on the field), and the point the curve bows through. */
    public static double PARK_X = 108, PARK_Y = 130;
    public static double PARK_CONTROL_X = 100, PARK_CONTROL_Y = 78;

    /** Time from PLAY (flywheel already spinning) until the balls are fed in. */
    public static double WAIT_MS = 8000;
    /** Long enough for ALL balls to go through. Raise it if a ball is left behind. */
    public static double SHOOT_MS = 2000;
    /** Intake power during this auto (TeleOp uses Intake.INTAKE_POWER). */
    public static double INTAKE_POWER = 0.6;
    /** Safety limit: give up on the drive after this long. */
    public static double PATH_TIMEOUT_MS = 5000;

    private final PoseFactory p = PoseFactory.degrees();

    private Follower follower;
    private Dashboard dashboard;
    private Intake intake;
    private Transport transport;
    private Shooter shooter;

    private String step = "waiting for PLAY";
    /** Where the robot really ended up -- stays on screen. */
    private String parkedAt = "--";

    private Pose pose(double x, double y) {
        return p.of(x, y, HEADING_DEG);
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
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(pose(START_X, START_Y));

        dashboard.addLine("Biobuzz Red Park -- robot at the start pose, heading " + HEADING_DEG + ".");
        dashboard.addData("Start (x, y, heading)", START_X + ", " + START_Y + ", " + HEADING_DEG);
        dashboard.showPose(follower.pose());
        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        dashboard.update();
    }

    @Override
    public void start() {
        Pose start = pose(START_X, START_Y);
        Pose park = pose(PARK_X, PARK_Y);
        follower.setPose(start);

        Path toPark = curve(start, pose(PARK_CONTROL_X, PARK_CONTROL_Y), park).constant(start);

        intake.setIntakePower(INTAKE_POWER);
        shooter.setSpinning(true);
        step = "flywheel on, waiting";

        schedule(sequential(
                // 1. Wait with the flywheel spinning.
                waitMs(WAIT_MS),

                // 2. Shoot.
                instant(() -> {
                    intake.setState(Intake.State.INTAKE);
                    transport.setState(Transport.State.FEED);
                    shooter.setGateTestOpen(true);
                    step = "shooting";
                }),
                waitMs(SHOOT_MS),

                // 3. Park.
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
        dashboard.addData("Parked at", parkedAt);
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
