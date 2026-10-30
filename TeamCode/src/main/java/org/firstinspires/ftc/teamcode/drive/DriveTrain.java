package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * What teleop needs from a drivetrain. Two versions exist:
 *
 *   PedroDrive          -- normal. Drives through Pedro's Follower, heading from the Pinpoint.
 *   DirectMecanumDrive  -- backup. Sets motor powers itself, heading from the Control Hub
 *                          IMU. Does not touch Pedro or the Pinpoint at all, so it still
 *                          works if either is broken.
 *
 * Sign convention for both (Pedro's): +forward, +left, +turn = counter-clockwise.
 */
public interface DriveTrain {

    /** Robot-centric drive powers, -1 to 1, already scaled by the speed mode. */
    void drive(double forward, double left, double turn);

    /** Called every loop while the lock button is held. justPressed is true on the first loop. */
    void lock(boolean justPressed);

    /** Field heading in radians, counter-clockwise positive, as of the last update(). */
    double heading();

    /** Call once per loop, after drive()/lock(). Reads sensors and sends the command. */
    void update();

    /** Optional extras. A drivetrain that can't do one just ignores it. */
    default void toggleBrakeMode() { }
    default void toggleVelocityMode() { }

    void stop();

    void report(Dashboard dashboard);

    /** Adds this drivetrain's columns to the match recording. Put the same columns every loop. */
    void record(MatchRecorder recorder);

    /** Shown on the dashboard so the drivers always know which TeleOp is running. */
    String name();
}
