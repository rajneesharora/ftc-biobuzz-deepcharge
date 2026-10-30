package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Normal teleop drive: Pedro's Follower in manual mode. Motor names/directions come from
 * pedro/Constants.java, heading from the Pinpoint.
 *
 * Independent of autonomous by default: it starts at START_X/Y/HEADING, whatever ran before.
 * Turn USE_AUTO_POSE on once the auto is finished, to continue from where auto stopped.
 * (Field-centric driving works either way -- it uses the heading at PLAY, not the pose.)
 *
 * follower.update() sends the motor powers, so update() MUST run every loop -- if it is
 * skipped the motors keep their last power.
 */
@Configurable
public class PedroDrive implements DriveTrain {

    /** false = ignore auto completely. true = start where the last auto ended (if one ran). */
    public static boolean USE_AUTO_POSE = false;
    /** Where teleop thinks the robot starts when not using the auto pose. Read once at init. */
    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING_DEG = 0;

    private final Follower follower;
    private final String startSource;

    public PedroDrive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        if (USE_AUTO_POSE && PoseStorage.hasPose()) {
            follower.setPose(PoseStorage.load());
            startSource = "from last auto";
        } else {
            follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
            startSource = USE_AUTO_POSE ? "fixed (no auto pose saved)" : "fixed (auto pose off)";
        }
        PoseStorage.clear();
    }

    @Override
    public void drive(double forward, double left, double turn) {
        follower.manual(forward, left, turn);
    }

    /** Pedro holds the pose from the moment the button went down, and pushes back if bumped. */
    @Override
    public void lock(boolean justPressed) {
        if (justPressed) {
            follower.hold(follower.pose());
        }
    }

    @Override
    public double heading() {
        return follower.pose().heading();
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.addData("Start pose", startSource);
        dashboard.showPose(follower.pose());
    }

    @Override
    public void record(MatchRecorder recorder) {
        Pose pose = follower.pose();
        recorder.put("x", pose.x());
        recorder.put("y", pose.y());
        recorder.put("heading_deg", Math.toDegrees(pose.heading()));
        recorder.put("pedro_mode", modeName(follower));
    }

    /** What the Follower is doing: MANUAL, HOLD, FOLLOW or IDLE. For the dashboard and recordings. */
    public static String modeName(Follower follower) {
        return follower.manual() ? "MANUAL" : follower.holding() ? "HOLD"
                : follower.following() ? "FOLLOW" : "IDLE";
    }

    @Override
    public String name() {
        return "PEDRO";
    }
}
