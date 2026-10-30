package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.lib.Battery;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

/**
 * Blue, far side: shoot the preloads, drive straight to the flower, turn around.
 *
 *   0 s      intake, transportLeft and flywheel start; transportRight held still
 *   3 s      transportRight and the gate start too -> balls go into the shooter
 *   10 s     all balls shot: everything stops; drive straight toward the audience
 *   end      turn 180 in place and hold
 *
 * Field (seen from the audience): x = 0 audience wall, x = 144 far wall, y = 0 right wall.
 * Heading 0 = facing the far wall, 180 = facing the audience. Poses are the robot's CENTER.
 * Robot is 18" x 18": START_X 134 puts its back ~1" from the far wall; END_X 19 puts its
 * front 10" from the audience wall.
 *
 * Everything below is live-editable in Panels; it is read when PLAY is pressed (the start
 * pose also during INIT).
 *
 * Needs this robot's Pedro tuning finished (Localization Check + AutoTune) first.
 */
@Configurable
@Autonomous(name = "Blue Far Shoot + Park", group = "Autonomous")
public class BlueFarShootAuto extends OpMode {

    public static double START_X = 134;
    public static double START_Y = 60;
    public static double START_HEADING_DEG = 180;

    public static double END_X = 19;
    public static double END_Y = 60;

    /** Flywheel spin-up time, with transportRight held still. */
    public static double SPIN_UP_MS = 3000;
    /** How long transportRight feeds the shooter before driving off. Long enough for ALL balls. */
    public static double SHOOT_MS = 7000;

    private final PoseFactory p = PoseFactory.degrees();

    private Follower follower;
    private Dashboard dashboard;
    private Intake intake;
    private Transport transport;
    private Shooter shooter;

    private String step = "waiting for PLAY";

    private MatchRecorder recorder;
    private String recordedStep = "";

    private Pose startPose() {
        return p.of(START_X, START_Y, START_HEADING_DEG);
    }

    @Override
    public void init() {
        Scheduler.reset();
        dashboard = new Dashboard(telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());

        intake = new Intake(hardwareMap);
        transport = new Transport(hardwareMap);
        shooter = new Shooter(hardwareMap);
        recorder = new MatchRecorder("auto_blue_far", new Battery(hardwareMap));
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(startPose());

        dashboard.addLine("Blue Far Shoot + Park -- place the robot at the start, facing the audience.");
        dashboard.addData("Start (x, y, heading)", START_X + ", " + START_Y + ", " + START_HEADING_DEG);
        dashboard.addData("End (x, y)", END_X + ", " + END_Y);
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
        Pose start = startPose();
        follower.setPose(start);

        // Drive keeping the start heading, then turn around in place at the end.
        Pose end = p.of(END_X, END_Y, START_HEADING_DEG);
        Pose endTurned = p.of(END_X, END_Y, START_HEADING_DEG - 180);
        Path toFlower = line(start, end).constant(start);

        // Spin up first: balls wait in the transport because the right side is held.
        intake.setState(Intake.State.INTAKE);
        transport.setState(Transport.State.FEED);
        transport.setRightStopped(true);
        shooter.setSpinning(true);
        step = "spinning up";

        schedule(sequential(
                waitMs(SPIN_UP_MS),
                instant(() -> {
                    transport.setRightStopped(false);
                    shooter.setGateTestOpen(true);   // timed: feed whatever the RPM reads
                    step = "shooting";
                }),
                waitMs(SHOOT_MS),
                instant(() -> {
                    intake.setState(Intake.State.OFF);
                    transport.setState(Transport.State.OFF);
                    shooter.setGateTestOpen(false);
                    shooter.setSpinning(false);
                    step = "driving to the flower";
                }),
                follow(follower, toFlower),
                instant(() -> step = "turning 180 and holding"),
                hold(follower, endTurned)
        ));
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
        dashboard.showPose(pose);
        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        dashboard.update();
        record();
    }

    /**
     * One row of the match recording (/sdcard/FIRST/analysis/auto_blue_far_*.csv): the step, the
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
        intake.stop();
        transport.stop();
        shooter.stop();
        recorder.setPhase("STOP");
        recorder.endLoop();
        recorder.close();
    }
}
