package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTag camera: the goBILDA global shutter USB camera (3122-0004-0001), facing the same way
 * as the shooter. Configure it on the Driver Station as a webcam named "Webcam 1".
 *
 * Runs at 640x480 MJPEG (most of this camera's modes are MJPEG-only) with goBILDA's published
 * lens calibration for that size, so ranges come out in real inches. Changing the resolution
 * means changing the four LENS_ numbers too -- goBILDA's user guide has them for each size.
 *
 * Exposure: auto by default. For sharper tags while moving, set MANUAL_EXPOSURE = true and
 * lower EXPOSURE_MS (raise GAIN if the picture gets too dark). Live-editable in Panels; applied
 * as soon as the camera is streaming.
 *
 * Missing camera is reported on the dashboard, never a crash.
 */
@Configurable
public class TagCamera implements Subsystem {

    public static boolean ENABLED = true;
    public static String NAME = "Webcam 1";

    // Lens calibration for 640x480, from goBILDA's user guide for 3122-0004-0001.
    public static double LENS_FX = 545.584, LENS_FY = 545.584;
    public static double LENS_CX = 350.588, LENS_CY = 223.150;

    // Live-editable in Panels.
    public static boolean MANUAL_EXPOSURE = false;
    public static double EXPOSURE_MS = 6;
    public static int GAIN = 150;

    private final VisionPortal portal;          // null if missing
    private final AprilTagProcessor aprilTag;   // null if missing
    private final String status;

    // What was last sent to the camera, so it's only sent again when a setting changes.
    private boolean appliedManual;
    private double appliedExposureMs = -1;
    private int appliedGain = -1;
    private boolean controlsApplied = false;

    public TagCamera(HardwareMap hardwareMap) {
        WebcamName webcam = ENABLED ? hardwareMap.tryGet(WebcamName.class, NAME) : null;
        if (webcam == null) {
            portal = null;
            aprilTag = null;
            status = ENABLED ? "MISSING '" + NAME + "' in robot config" : "disabled";
            return;
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
        status = "active";
    }

    /** Every tag seen in the latest frame. Empty (never null) if none, or no camera. */
    public List<AprilTagDetection> tags() {
        return aprilTag == null ? new ArrayList<>() : aprilTag.getDetections();
    }

    /**
     * The single tag with this id if it's in view (and its size is known), otherwise null.
     * Since SDK 12, id and metadata live on AprilTagSingleDetection, not AprilTagDetection.
     */
    public AprilTagSingleDetection find(int id) {
        for (AprilTagDetection tag : tags()) {
            if (tag instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                if (single.id == id && single.metadata != null) {
                    return single;
                }
            }
        }
        return null;
    }

    public boolean isStreaming() {
        return portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Sends exposure/gain to the camera once it's streaming, and again whenever they change. */
    @Override
    public void update() {
        if (!isStreaming()) {
            return;
        }
        if (controlsApplied && appliedManual == MANUAL_EXPOSURE
                && appliedExposureMs == EXPOSURE_MS && appliedGain == GAIN) {
            return;
        }
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (MANUAL_EXPOSURE) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            gain.setGain(GAIN);
        } else {
            exposure.setMode(ExposureControl.Mode.Auto);
        }
        appliedManual = MANUAL_EXPOSURE;
        appliedExposureMs = EXPOSURE_MS;
        appliedGain = GAIN;
        controlsApplied = true;
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("CAMERA");
        if (portal == null) {
            dashboard.addData("Camera", status);
            return;
        }
        dashboard.addData("Camera", portal.getCameraState() + "  [" + status + "]");
        dashboard.addData("Camera FPS", Math.round(portal.getFps()));
        dashboard.addData("Exposure", MANUAL_EXPOSURE ? EXPOSURE_MS + " ms, gain " + GAIN : "auto");
    }

    /** Turns the camera off. Call once, at the end of the OpMode. */
    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }
}
