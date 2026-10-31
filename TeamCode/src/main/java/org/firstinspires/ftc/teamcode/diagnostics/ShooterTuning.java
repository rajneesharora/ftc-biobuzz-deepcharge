package org.firstinspires.ftc.teamcode.diagnostics;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TagCamera;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Logs every shot -- distance, angle, speed and result -- for building the distance -> RPM
 * model. Change the RPM as often as you like; each shot is saved with the values it had.
 * One person, gamepad 1:
 *
 *   left stick / right stick X   drive slowly (robot-centric) between test spots
 *   D-pad Up / Down              flywheel on / off
 *   D-pad Right / Left           target RPM +/- RPM_STEP        (big steps)
 *   right / left bumper          target RPM +/- RPM_FINE_STEP   (fine steps)
 *   right trigger                toggle intake + transport (press on / press off)
 *   X (hold)                     gate: shoot. Tap it to let ONE ball go while tuning.
 *   A / B / Y / BACK             result of the last shot: IN / SHORT / LONG / WIDE (left or
 *                                right). Press another one to correct it.
 *
 * Shot detection: a ball passing through takes speed out of the flywheel, so a drop of more
 * than DIP_RPM below the speed just before it, while the gate is (or just was) open, counts as
 * one shot. Each dip is one ball; shots less than BURST_GAP_MS apart are numbered 1, 2, 3 in
 * a burst (later balls in a burst fly slower). If X is pressed and released with no dip, the
 * attempt is still logged, marked "no dip".
 *
 * The goal can be a single tag or a cluster (SDK 12 reports several tags read together as one
 * cluster, with a name but no id). GOAL_TAG empty = the nearest one in view; otherwise only a
 * detection whose id or name contains GOAL_TAG counts, e.g. "20" or "BlueAudience". Range is
 * averaged over the last RANGE_SAMPLES frames. A shot taken with no goal tag in view is still
 * logged, with blank distance and angles.
 *
 * Each run saves to its own file, /sdcard/FIRST/shooter_shots_<date_time>.csv, rewritten after
 * every shot and result, so nothing is lost if the program stops. Fetch them with:
 *   adb pull /sdcard/FIRST/
 *
 * Needs the camera ("Webcam 1") and this robot's Pedro tuning (for driving).
 */
@Configurable
@TeleOp(name = "Shooter Tuning", group = "Diagnostics")
public class ShooterTuning extends LinearOpMode {

    /** Empty = nearest tag or cluster in view; otherwise its id or name must contain this. */
    public static String GOAL_TAG = "";
    public static double RPM_STEP = 50;
    public static double RPM_FINE_STEP = 10;
    public static double DRIVE_SPEED = 0.4;
    public static int RANGE_SAMPLES = 10;
    /** Stop trusting the averaged range after the tag has been out of view this long. */
    public static double TAG_LOST_MS = 300;

    /** How far the flywheel must drop below its speed just before, to count as a ball. */
    public static double DIP_RPM = 150;
    /** A dip still counts this long after X is let go (the ball may still be on its way). */
    public static double AFTER_GATE_MS = 400;
    /** Shots closer together than this are one burst (numbered 1, 2, 3...). */
    public static double BURST_GAP_MS = 1000;
    /** How many recent shots to list on screen. */
    public static int SHOTS_SHOWN = 8;

    /** How far back "the speed just before the ball" looks. */
    private static final double PRE_SHOT_WINDOW_MS = 250;

    /** One logged shot. */
    private static class Shot {
        int number, inBurst;
        String goal;
        double rangeIn = Double.NaN, bearingDeg = Double.NaN, yawDeg = Double.NaN;
        double xIn = Double.NaN, yIn = Double.NaN;
        double targetRpm, launchRpm, lowestRpm, batteryV;
        boolean dipSeen;
        String result = "";
    }

    private final Button.Group buttons = new Button.Group();
    private final Button shooterOn  = buttons.add(() -> gamepad1.dpad_up);
    private final Button shooterOff = buttons.add(() -> gamepad1.dpad_down);
    private final Button rpmUp      = buttons.add(() -> gamepad1.dpad_right);
    private final Button rpmDown    = buttons.add(() -> gamepad1.dpad_left);
    private final Button rpmFineUp  = buttons.add(() -> gamepad1.right_bumper);
    private final Button rpmFineDn  = buttons.add(() -> gamepad1.left_bumper);
    private final Button intakeOn   = buttons.toggle(() -> gamepad1.right_trigger > 0.5);
    private final Button shoot      = buttons.add(() -> gamepad1.x);
    private final Button resultIn    = buttons.add(() -> gamepad1.a);
    private final Button resultShort = buttons.add(() -> gamepad1.b);
    private final Button resultLong  = buttons.add(() -> gamepad1.y);
    private final Button resultWide  = buttons.add(() -> gamepad1.back);

    // Goal tag, as of the latest frame that saw it.
    private final ArrayDeque<Double> ranges = new ArrayDeque<>();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private String goalId = "";
    private double goalBearing, goalYaw, goalX, goalY;
    private String cameraSees = "--";
    private String tagsSeen = "--";

    // Shot detection.
    private final ElapsedTime clock = new ElapsedTime();
    private final ArrayDeque<double[]> rpmHistory = new ArrayDeque<>();   // {time ms, rpm}
    private double gateClosedAtMs = -1e9;
    private double lastShotAtMs = -1e9;
    private boolean armed = true;          // ready to count the next dip
    private boolean dipThisPress = false;
    private double noDipCheckAtMs = -1;   // >= 0: log a "no dip" attempt at this time
    private Shot dipping = null;           // shot whose dip is still going on

    private final List<Shot> shots = new ArrayList<>();
    private File logFile;
    private String saveStatus = "no shots yet";

    @Override
    public void runOpMode() {
        Dashboard dashboard = new Dashboard(telemetry);
        DriveTrain drive = new PedroDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transport transport = new Transport(hardwareMap);
        TagCamera camera = new TagCamera(hardwareMap);
        // TARGET_RPM is shared with every OpMode until the robot restarts, so put it back at
        // the end -- otherwise the competition TeleOp would shoot at the last RPM tried here.
        double originalRpm = Shooter.TARGET_RPM;
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        logFile = new File(AppUtil.FIRST_FOLDER, "shooter_shots_" + stamp + ".csv");

        while (opModeInInit()) {
            camera.update();
            readGoalTag(camera);
            dashboard.addLine("Shooter Tuning -- aim at the goal. Press PLAY.");
            showGoal(dashboard);
            camera.report(dashboard);
            dashboard.update();
        }

        while (opModeIsActive()) {
            buttons.update();
            camera.update();
            readGoalTag(camera);

            // Driving: slow and robot-centric, just to move between test spots.
            drive.drive(-gamepad1.left_stick_y * DRIVE_SPEED, -gamepad1.left_stick_x * DRIVE_SPEED,
                    -gamepad1.right_stick_x * DRIVE_SPEED);
            drive.update();

            // RPM and flywheel.
            if (rpmUp.pressed())     Shooter.TARGET_RPM += RPM_STEP;
            if (rpmDown.pressed())   Shooter.TARGET_RPM -= RPM_STEP;
            if (rpmFineUp.pressed()) Shooter.TARGET_RPM += RPM_FINE_STEP;
            if (rpmFineDn.pressed()) Shooter.TARGET_RPM -= RPM_FINE_STEP;
            Shooter.TARGET_RPM = Math.max(0, Shooter.TARGET_RPM);
            if (shooterOn.pressed())  shooter.setSpinning(true);
            if (shooterOff.pressed()) shooter.setSpinning(false);

            // Intake + transport bring balls to the closed gate; X opens the gate to shoot.
            intake.setState(intakeOn.isOn() ? Intake.State.INTAKE : Intake.State.OFF);
            transport.setState(intakeOn.isOn() ? Transport.State.FEED : Transport.State.OFF);
            shooter.setGateTestOpen(shoot.down());

            intake.update();
            transport.update();
            shooter.update();

            detectShots(shooter);

            // Result of the last shot (pressing another button corrects it).
            if (resultIn.pressed())    setResult("IN");
            if (resultShort.pressed()) setResult("SHORT");
            if (resultLong.pressed())  setResult("LONG");
            if (resultWide.pressed())  setResult("WIDE");

            showGoal(dashboard);
            dashboard.addData("RPM target / actual", Math.round(Shooter.TARGET_RPM) + " / "
                    + Math.round(shooter.getRPM()) + (shooter.atSpeed() ? "  READY" : ""));
            dashboard.addData("Flywheel", shooter.isSpinning() ? "ON" : "off");
            dashboard.addData("Intake + transport", intakeOn.isOn() ? "ON" : "off");
            showShots(dashboard);
            camera.report(dashboard);
            dashboard.update();
        }

        drive.stop();
        shooter.stop();
        intake.stop();
        transport.stop();
        camera.stop();
        Shooter.TARGET_RPM = originalRpm;
    }

    /** Counts a ball each time the flywheel dips while the gate is (or was just) open. */
    private void detectShots(Shooter shooter) {
        double now = clock.milliseconds();
        double rpm = shooter.getRPM();

        // Speed just before now = the highest reading in the last PRE_SHOT_WINDOW_MS.
        rpmHistory.addLast(new double[] {now, rpm});
        while (rpmHistory.peekFirst()[0] < now - PRE_SHOT_WINDOW_MS) {
            rpmHistory.removeFirst();
        }
        double before = 0;
        for (double[] sample : rpmHistory) {
            before = Math.max(before, sample[1]);
        }

        if (shoot.pressed()) {
            dipThisPress = false;
            noDipCheckAtMs = -1;
        }
        if (shoot.down()) {
            gateClosedAtMs = now;
        }
        boolean gateWindow = shoot.down() || now - gateClosedAtMs < AFTER_GATE_MS;

        // Flywheel off: any dip in progress is over.
        if (!shooter.isSpinning()) {
            dipping = null;
            armed = true;
        }

        // Follow the dip of the latest shot down to its lowest point; re-arm once it recovers.
        if (dipping != null) {
            dipping.lowestRpm = Math.min(dipping.lowestRpm, rpm);
            if (rpm > dipping.launchRpm - DIP_RPM / 2) {
                dipping = null;
                armed = true;
                save();
            }
        }

        if (armed && gateWindow && shooter.isSpinning() && rpm < before - DIP_RPM) {
            Shot shot = newShot(shooter, before);
            shot.dipSeen = true;
            shot.lowestRpm = rpm;
            dipping = shot;
            armed = false;
            dipThisPress = true;
        }

        // X tapped and let go without any dip: once the ball would have arrived, log the attempt.
        if (shoot.released() && !dipThisPress) {
            noDipCheckAtMs = now + AFTER_GATE_MS;
        }
        if (noDipCheckAtMs >= 0 && dipThisPress) {
            noDipCheckAtMs = -1;
        }
        if (noDipCheckAtMs >= 0 && now >= noDipCheckAtMs) {
            noDipCheckAtMs = -1;
            Shot shot = newShot(shooter, rpm);
            shot.lowestRpm = rpm;
            save();
        }
    }

    /** Logs a shot with everything as it is right now. */
    private Shot newShot(Shooter shooter, double launchRpm) {
        double now = clock.milliseconds();
        Shot shot = new Shot();
        Shot last = shots.isEmpty() ? null : shots.get(shots.size() - 1);
        shot.number = shots.size() + 1;
        shot.inBurst = last != null && now - lastShotAtMs < BURST_GAP_MS ? last.inBurst + 1 : 1;
        lastShotAtMs = now;
        if (tagInView()) {
            shot.goal = goalId;
            shot.rangeIn = averageRange();
            shot.bearingDeg = goalBearing;
            shot.yawDeg = goalYaw;
            shot.xIn = goalX;
            shot.yIn = goalY;
        } else {
            shot.goal = "no tag";
        }
        shot.targetRpm = Shooter.TARGET_RPM;
        shot.launchRpm = launchRpm;
        shot.batteryV = batteryVoltage();
        shots.add(shot);
        return shot;
    }

    private void setResult(String result) {
        if (shots.isEmpty()) {
            saveStatus = "no shot to mark yet";
            return;
        }
        shots.get(shots.size() - 1).result = result;
        save();
    }

    private double batteryVoltage() {
        double volts = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            volts = Math.max(volts, sensor.getVoltage());
        }
        return volts;
    }

    /** Finds the goal (single tag or cluster) in this frame and adds its range to the average. */
    private void readGoalTag(TagCamera camera) {
        AprilTagDetection goal = null;
        String goalLabel = "";
        int singles = 0, clusters = 0, unknown = 0, noPose = 0, filtered = 0;
        StringBuilder seen = new StringBuilder();
        for (AprilTagDetection tag : camera.tags()) {
            String label;
            if (tag instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                if (single.metadata == null) {
                    unknown++;   // not a game tag: its size isn't known, so no range
                    seen.append(seen.length() > 0 ? ", " : "").append(single.id).append(" (unknown)");
                    continue;
                }
                singles++;
                label = single.id + " " + single.metadata.name;
            } else if (tag instanceof AprilTagClusterDetection) {
                clusters++;
                label = "cluster " + ((AprilTagClusterDetection) tag).metadata.name;
            } else {
                continue;
            }
            seen.append(seen.length() > 0 ? ", " : "").append(label);
            if (tag.ftcPose == null) {
                noPose++;
                continue;
            }
            if (!GOAL_TAG.isEmpty() && !label.toLowerCase().contains(GOAL_TAG.toLowerCase())) {
                filtered++;
                continue;
            }
            if (goal == null || tag.ftcPose.range < goal.ftcPose.range) {
                goal = tag;
                goalLabel = label;
            }
        }
        cameraSees = singles + " tag(s), " + clusters + " cluster(s)"
                + (unknown > 0 ? ", " + unknown + " unknown" : "")
                + (noPose > 0 ? ", " + noPose + " without range" : "")
                + (filtered > 0 ? ", " + filtered + " skipped (not '" + GOAL_TAG + "')" : "");
        tagsSeen = seen.length() > 0 ? seen.toString() : "none";
        if (goal == null) {
            return;
        }
        if (!goalLabel.equals(goalId)) {
            ranges.clear();   // a different tag: don't average its range with the old one's
        }
        goalId = goalLabel;
        goalBearing = goal.ftcPose.bearing;
        goalYaw = goal.ftcPose.yaw;
        goalX = goal.ftcPose.x;
        goalY = goal.ftcPose.y;
        ranges.addLast(goal.ftcPose.range);
        while (ranges.size() > Math.max(1, RANGE_SAMPLES)) {
            ranges.removeFirst();
        }
        sinceTagSeen.reset();
    }

    private boolean tagInView() {
        return !ranges.isEmpty() && sinceTagSeen.milliseconds() < TAG_LOST_MS;
    }

    private double averageRange() {
        double sum = 0;
        for (double r : ranges) {
            sum += r;
        }
        return sum / ranges.size();
    }

    private void showGoal(Dashboard dashboard) {
        dashboard.section("GOAL TAG" + (GOAL_TAG.isEmpty() ? " (nearest)" : " ('" + GOAL_TAG + "' only)"));
        dashboard.addData("Camera sees", cameraSees);
        dashboard.addData("Tags in view (id name)", tagsSeen);
        if (tagInView()) {
            dashboard.addData("Goal", goalId);
            dashboard.addData("Range (avg, in)", round1(averageRange()));
            dashboard.addData("Bearing / yaw (deg)", round1(goalBearing) + " / " + round1(goalYaw)
                    + "  (bearing + = goal to the left)");
        } else {
            dashboard.addData("Goal tag", "NOT in view");
        }
    }

    private void showShots(Dashboard dashboard) {
        dashboard.section("SHOTS -- " + saveStatus);
        dashboard.addLine("#  burst  range  RPM set/launch/low  result");
        for (int i = Math.max(0, shots.size() - SHOTS_SHOWN); i < shots.size(); i++) {
            Shot s = shots.get(i);
            dashboard.addLine(s.number + "  " + s.inBurst + "  " + (Double.isNaN(s.rangeIn) ? "no tag" : round1(s.rangeIn))
                    + "  " + Math.round(s.targetRpm) + "/" + Math.round(s.launchRpm) + "/" + Math.round(s.lowestRpm)
                    + "  " + (s.result.isEmpty() ? "?" : s.result) + (s.dipSeen ? "" : "  (no dip)"));
        }
    }

    /** Rewrites this run's file with every shot so far. */
    private void save() {
        try (FileWriter out = new FileWriter(logFile)) {
            out.write("shot,in_burst,goal,range_in,bearing_deg,yaw_deg,x_in,y_in,"
                    + "target_rpm,launch_rpm,lowest_rpm,dip_seen,battery_v,result\n");
            for (Shot s : shots) {
                out.write(s.number + "," + s.inBurst + "," + s.goal + "," + csv(s.rangeIn) + ","
                        + csv(s.bearingDeg) + "," + csv(s.yawDeg) + "," + csv(s.xIn) + "," + csv(s.yIn) + ","
                        + Math.round(s.targetRpm) + "," + Math.round(s.launchRpm) + ","
                        + Math.round(s.lowestRpm) + "," + s.dipSeen + "," + round1(s.batteryV) + ","
                        + s.result + "\n");
            }
            saveStatus = shots.size() + " saved to " + logFile.getName();
        } catch (IOException e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
        }
    }

    private static String csv(double value) {
        return Double.isNaN(value) ? "" : String.valueOf(round1(value));
    }

    private static double round1(double value) {
        return Math.round(value * 10) / 10.0;
    }
}
