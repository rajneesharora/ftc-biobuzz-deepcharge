package org.firstinspires.ftc.teamcode.lib;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * All the Panels wiring in one place: telemetry, the field drawing and the browser gamepad.
 *
 * Everything sent here also reaches the Driver Station, so an OpMode using this needs no
 * telemetry calls of its own. It only shows things -- it never moves the robot.
 *
 * Panels: http://192.168.43.1:8001 on robot Wi-Fi, or over USB:
 *   adb forward tcp:8001 tcp:8001 && adb forward tcp:8002 tcp:8002
 */
public class Dashboard {

    /** Radius of the robot circle on the Panels field, in inches. */
    private static final double ROBOT_RADIUS = 9;

    private final Telemetry driverStation;
    private final TelemetryManager panels;
    private final FieldManager field;

    public Dashboard(Telemetry driverStation) {
        this.driverStation = driverStation;
        this.panels = PanelsTelemetry.INSTANCE.getTelemetry();
        this.field = PanelsField.INSTANCE.getField();
        // Pedro's 0-144 field coordinates, so poses can be drawn as-is.
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
    }

    /**
     * Combines a gamepad plugged into the Panels browser tab with the real one. Buttons from
     * either count, but any stick or trigger the browser reports as non-zero REPLACES the real
     * one -- so the browser controller wins whenever it's touched (or drifting).
     */
    public Gamepad mergeGamepad(Gamepad driverStationGamepad) {
        return PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(driverStationGamepad);
    }

    public void addLine(String line) {
        panels.addLine(line);
    }

    public void addData(String label, Object value) {
        panels.addData(label, value);
    }

    public void section(String title) {
        panels.addLine("");
        panels.addLine("--- " + title + " ---");
    }

    /** Shows x / y / heading and draws the robot on the Panels field. */
    public void showPose(Pose pose) {
        addData("X (in)", round1(pose.x()));
        addData("Y (in)", round1(pose.y()));
        addData("Heading (deg)", round1(Math.toDegrees(pose.heading())));

        field.setStyle(PanelsField.INSTANCE.getBLUE(), PanelsField.INSTANCE.getWHITE(), 1.0);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);
        // A small dot on the front edge so you can see which way the robot faces.
        field.moveCursor(pose.x() + ROBOT_RADIUS * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS * Math.sin(pose.heading()));
        field.circle(1.5);
    }

    /** Call once per loop, last. Sends everything to Panels and the Driver Station. */
    public void update() {
        field.update();
        panels.update(driverStation);
    }

    private static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }
}
