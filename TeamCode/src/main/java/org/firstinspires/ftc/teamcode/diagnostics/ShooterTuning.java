package org.firstinspires.ftc.teamcode.diagnostics;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds which shooter RPM works at which distance from the goal, for the distance -> RPM table.
 * One person, gamepad 1:
 *
 *   left stick / right stick X   drive slowly (robot-centric) between distances
 *   D-pad Up / Down              flywheel on / off
 *   D-pad Right / Left           target RPM +/- RPM_STEP        (big steps)
 *   right / left bumper          target RPM +/- RPM_FINE_STEP   (fine steps)
 *   right trigger                toggle intake + transport (press on / press off)
 *   X (hold)                     gate: shoot
 *   A                            record: "this RPM works at this distance"
 *   B                            undo the last recorded point
 *
 * Range is the camera -> goal tag distance, averaged over the last RANGE_SAMPLES frames so it
 * doesn't jump. A point can only be recorded while the goal tag is in view.
 *
 * The goal can be a single tag or a cluster (since SDK 12, several tags read together come back
 * as one cluster detection, with a name but no id). GOAL_TAG empty = the nearest one in view;
 * otherwise only a detection whose id or name contains GOAL_TAG counts, e.g. "20" or "Blue".
 * "Camera sees" shows what is detected and what was skipped.
 *
 * Recorded points stay on screen and are saved to /sdcard/FIRST/shooter_table.csv on every
 * change, so nothing is lost if the program stops. Copy the list from the screen, or fetch the
 * file with:  adb pull /sdcard/FIRST/shooter_table.csv
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

    private static final File TABLE_FILE = new File(AppUtil.FIRST_FOLDER, "shooter_table.csv");

    /** One "this works" point. */
    private static class Point implements Comparable<Point> {
        final double rangeIn, targetRpm, actualRpm;
        final String tagId;

        Point(double rangeIn, double targetRpm, double actualRpm, String tagId) {
            this.rangeIn = rangeIn;
            this.targetRpm = targetRpm;
            this.actualRpm = actualRpm;
            this.tagId = tagId;
        }

        @Override
        public int compareTo(Point other) {
            return Double.compare(rangeIn, other.rangeIn);
        }
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
    private final Button record     = buttons.add(() -> gamepad1.a);
    private final Button undo       = buttons.add(() -> gamepad1.b);

    private final ArrayDeque<Double> ranges = new ArrayDeque<>();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final List<Point> points = new ArrayList<>();
    private String goalId = "";
    private double goalBearing;
    private String cameraSees = "--";
    private String saveStatus = "nothing recorded yet";

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

        while (opModeInInit()) {
            camera.update();
            readGoalTag(camera);
            dashboard.addLine("Shooter Tuning -- aim at the Blue goal. Press PLAY.");
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

            // Record / undo.
            if (record.pressed()) {
                if (tagInView()) {
                    points.add(new Point(averageRange(), Shooter.TARGET_RPM, shooter.getRPM(), goalId));
                    save();
                } else {
                    saveStatus = "NOT recorded -- goal tag not in view";
                }
            }
            if (undo.pressed() && !points.isEmpty()) {
                points.remove(points.size() - 1);
                save();
            }

            showGoal(dashboard);
            dashboard.addData("RPM target / actual", Math.round(Shooter.TARGET_RPM) + " / "
                    + Math.round(shooter.getRPM()) + (shooter.atSpeed() ? "  READY" : ""));
            dashboard.addData("Flywheel", shooter.isSpinning() ? "ON" : "off");
            dashboard.addData("Intake + transport", intakeOn.isOn() ? "ON" : "off");
            showTable(dashboard);
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

    /** Finds the goal (single tag or cluster) in this frame and adds its range to the average. */
    private void readGoalTag(TagCamera camera) {
        AprilTagDetection goal = null;
        String goalLabel = "";
        int singles = 0, clusters = 0, unknown = 0, noPose = 0, filtered = 0;
        for (AprilTagDetection tag : camera.tags()) {
            String label;
            if (tag instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                if (single.metadata == null) {
                    unknown++;   // not a game tag: its size isn't known, so no range
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
        if (goal == null) {
            return;
        }
        if (!goalLabel.equals(goalId)) {
            ranges.clear();   // a different tag: don't average its range with the old one's
        }
        goalId = goalLabel;
        goalBearing = goal.ftcPose.bearing;
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
        if (tagInView()) {
            dashboard.addData("Goal", goalId);
            dashboard.addData("Range (avg, in)", round1(averageRange()));
            dashboard.addData("Bearing (deg)", round1(goalBearing) + "  (+ = goal is to the left)");
        } else {
            dashboard.addData("Goal tag", "NOT in view");
        }
    }

    private void showTable(Dashboard dashboard) {
        dashboard.section("RECORDED (sorted by range) -- " + saveStatus);
        List<Point> sorted = new ArrayList<>(points);
        Collections.sort(sorted);
        for (Point point : sorted) {
            dashboard.addLine(round1(point.rangeIn) + " in  ->  " + Math.round(point.targetRpm)
                    + " RPM   (actual " + Math.round(point.actualRpm) + ", " + point.tagId + ")");
        }
    }

    /** Rewrites the whole table file, sorted by range. */
    private void save() {
        List<Point> sorted = new ArrayList<>(points);
        Collections.sort(sorted);
        try (FileWriter out = new FileWriter(TABLE_FILE)) {
            out.write("range_in,target_rpm,actual_rpm,tag\n");
            for (Point point : sorted) {
                out.write(round1(point.rangeIn) + "," + Math.round(point.targetRpm) + ","
                        + Math.round(point.actualRpm) + "," + point.tagId + "\n");
            }
            saveStatus = points.size() + " saved to " + TABLE_FILE.getName();
        } catch (IOException e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
        }
    }

    private static double round1(double value) {
        return Math.round(value * 10) / 10.0;
    }
}
