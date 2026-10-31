package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.TagCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Shows everything the camera reads from each AprilTag in view. Motors never run.
 *
 *   Range     straight-line distance camera -> tag, in inches   (what shooter speed will use)
 *   Bearing   left/right angle to the tag; + = tag is to the LEFT, 0 = dead ahead (for aiming)
 *   Yaw       how much the tag is turned away from facing the camera
 *   x / y / z tag position from the camera: right+, forward+, up+ (inches)
 *   Confidence  decision margin -- higher is better; very low values are probably misreads
 *
 * Check with a tape measure that Range matches the real camera-to-tag distance (within an
 * inch or two). Live picture with tag outlines: Driver Station > menu > Camera Stream, during
 * INIT. Works during INIT as well as after PLAY.
 *
 * Only tags of this season's game have a known size; others show their id but no range.
 */
@TeleOp(name = "AprilTag Check", group = "Diagnostics")
public class AprilTagCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Dashboard dashboard = new Dashboard(telemetry);
        TagCamera camera = new TagCamera(hardwareMap);

        while (!isStopRequested()) {
            camera.update();
            List<AprilTagDetection> tags = camera.tags();

            dashboard.addLine("Point the camera at a tag. Motors are off.");
            camera.report(dashboard);
            dashboard.addData("Tags seen", tags.size());

            for (AprilTagDetection tag : tags) {
                // Since SDK 12 a detection is either one tag (AprilTagSingleDetection, which has
                // the id) or a group of tags read together (a cluster). ftcPose is on both.
                if (tag instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                    if (single.metadata == null) {
                        dashboard.section("Tag " + single.id + " (unknown -- not a game tag, no range)");
                        continue;
                    }
                    dashboard.section("Tag " + single.id + " (" + single.metadata.name + ")");
                    dashboard.addData("Confidence", round1(single.decisionMargin));
                } else if (tag instanceof AprilTagClusterDetection) {
                    dashboard.section("Tag cluster (" + ((AprilTagClusterDetection) tag).metadata.name + ")");
                } else {
                    dashboard.section("Tag cluster");
                }
                if (tag.ftcPose == null) {
                    dashboard.addData("Range (in)", "--");
                    continue;
                }
                dashboard.addData("Range (in)", round1(tag.ftcPose.range));
                dashboard.addData("Bearing (deg)", round1(tag.ftcPose.bearing));
                dashboard.addData("Yaw (deg)", round1(tag.ftcPose.yaw));
                dashboard.addData("x / y / z (in)", round1(tag.ftcPose.x) + " / "
                        + round1(tag.ftcPose.y) + " / " + round1(tag.ftcPose.z));
            }
            dashboard.update();
        }

        camera.stop();
    }

    private static double round1(double value) {
        return Math.round(value * 10) / 10.0;
    }
}
