package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * One part of the robot (intake, transport, shooter...). Teleop and auto both talk to these,
 * so a mechanism is written once and used everywhere.
 *
 * The one rule: update() must never wait. No sleep(), no while(busy). Remember a state and
 * check it again next loop instead.
 */
public interface Subsystem {

    /** Called once per loop. Sends the current command to the hardware. */
    void update();

    /** Adds this subsystem's lines to the dashboard. */
    void report(Dashboard dashboard);

    /** Adds this subsystem's columns to the match recording. Put the same columns every loop. */
    default void record(MatchRecorder recorder) { }

    /** Turns everything off. Safe to call any time. */
    void stop();
}
