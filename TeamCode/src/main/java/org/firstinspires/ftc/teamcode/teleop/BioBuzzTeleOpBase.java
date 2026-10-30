package org.firstinspires.ftc.teamcode.teleop;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Battery;
import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;
import org.firstinspires.ftc.teamcode.subsystems.FlowerGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

/**
 * Everything both BioBuzz TeleOps share: controls, field-centric math, mechanisms, dashboard.
 * The two OpModes differ only in which DriveTrain createDrive() returns. A control change made
 * here reaches both, so the backup never drifts out of date.
 *
 * Not an OpMode on its own (it's abstract and has no @TeleOp), so it never shows on the DS.
 * The loop never waits for anything.
 *
 * DRIVER -- gamepad 1 (or a controller plugged into the Panels browser tab):
 *   left stick        drive (field-centric by default)
 *   right stick X     turn
 *   left trigger      hold for slow mode
 *   right trigger     hold for turbo
 *   A                 make the way the robot faces now the new "forward"
 *   BACK              switch field-centric / robot-centric
 *   B                 hold to lock in place (Pedro: holds position; backup: brakes)
 *   Y                 backup only: switch velocity / raw power (needs drive encoders!)
 *
 * OPERATOR -- gamepad 2:
 *   right bumper      toggle intake + transport (keeps running until pressed again)
 *   left bumper       hold to unjam (intake and transport backwards)
 *   D-pad Up          start shooter flywheel
 *   D-pad Down        stop shooter flywheel
 *   X (hold)          shooter gate only (no speed check). Keep right bumper on to feed more
 *                     balls up while shooting.
 *   Y (hold)          rotate the flower gate servo
 *
 * Brake/coast is START_IN_BRAKE_MODE in Panels (DirectMecanumDrive); X is the gate now.
 *
 * Every loop, from INIT to STOP, is written to /sdcard/FIRST/analysis/teleop_*.csv by
 * MatchRecorder: both gamepads, the drive command, pose, and every mechanism.
 */
public abstract class BioBuzzTeleOpBase extends LinearOpMode {

    /** Which drivetrain this TeleOp uses. Called once, during init. */
    protected abstract DriveTrain createDrive(HardwareMap hardwareMap);

    private DriveTrain drive;
    private Dashboard dashboard;
    private Gamepad pad;        // driver, gamepad 1
    private Gamepad operator;   // operator, gamepad 2

    private Intake intake;
    private Transport transport;
    private Shooter shooter;
    private FlowerGate flowerGate;

    private MatchRecorder recorder;
    private int lastGamepadId;
    private int lastOperatorId;
    // What updateDriving() sent to the drivetrain this loop, for the recording.
    private double cmdForward, cmdLeft, cmdTurn;

    /** Field heading that counts as "away from the driver". Set at start and by button A. */
    private double driverForwardHeading;

    // Must be ABOVE the buttons -- they register themselves while being created.
    private final Button.Group buttons = new Button.Group();

    private final Button slowMode       = buttons.add(() -> pad.left_trigger > 0.5);
    private final Button turboMode      = buttons.add(() -> pad.right_trigger > 0.5);
    private final Button resetForward   = buttons.add(() -> pad.a);
    private final Button toggleCentric  = buttons.add(() -> pad.back);
    private final Button lockPosition   = buttons.add(() -> pad.b);
    private final Button velocityMode   = buttons.add(() -> pad.y);

    private final Button intakeOn       = buttons.toggle(() -> operator.right_bumper);
    private final Button eject          = buttons.add(() -> operator.left_bumper);
    private final Button shooterStart   = buttons.add(() -> operator.dpad_up);
    private final Button shooterStop    = buttons.add(() -> operator.dpad_down);
    private final Button shoot          = buttons.add(() -> operator.x);
    private final Button flower         = buttons.add(() -> operator.y);

    @Override
    public void runOpMode() {
        dashboard = new Dashboard(telemetry);
        drive = createDrive(hardwareMap);

        intake = new Intake(hardwareMap);
        transport = new Transport(hardwareMap);
        shooter = new Shooter(hardwareMap);
        flowerGate = new FlowerGate(hardwareMap);

        recorder = new MatchRecorder("teleop_" + drive.name().toLowerCase().replaceAll("[^a-z]+", "_")
                .replaceAll("_$", ""), new Battery(hardwareMap));
        lastGamepadId = gamepad1.getGamepadId();
        lastOperatorId = gamepad2.getGamepadId();
        try {
            runRecorded();
        } catch (RuntimeException e) {
            // Get the crash into the file before the OpMode dies.
            recorder.event("CRASH " + e);
            recorder.endLoop();
            throw e;
        } finally {
            recorder.close();
        }
    }

    private void runRecorded() {
        // INIT: keep the sensors running so heading/pose are live before PLAY.
        while (opModeInInit()) {
            drive.update();
            dashboard.addLine("BioBuzz TeleOp [" + drive.name() + "] ready. Press PLAY.");
            drive.report(dashboard);
            dashboard.update();
            recordLoop();
        }
        if (isStopRequested()) {
            return;
        }

        recorder.setPhase("RUN");
        driverForwardHeading = drive.heading();

        while (opModeIsActive()) {
            readGamepad();
            updateDriving();
            drive.update();
            updateMechanisms();
            updateDashboard();
            recordLoop();
        }

        drive.stop();
        intake.stop();
        transport.stop();
        shooter.stop();
        flowerGate.stop();
        recorder.setPhase("STOP");
        recorder.endLoop();
    }

    /** Reads the gamepads once, so everything below sees the same thing. */
    private void readGamepad() {
        pad = dashboard.mergeGamepad(gamepad1);
        operator = gamepad2;
        buttons.update();
    }

    private void updateDriving() {
        if (toggleCentric.pressed()) {
            DriverSettings.FIELD_CENTRIC = !DriverSettings.FIELD_CENTRIC;   // same switch Panels shows
        }
        if (resetForward.pressed()) {
            driverForwardHeading = drive.heading();
        }
        if (velocityMode.pressed()) {
            drive.toggleVelocityMode();
        }

        if (lockPosition.down()) {
            drive.lock(lockPosition.pressed());
            cmdForward = 0;
            cmdLeft = 0;
            cmdTurn = 0;
            return;
        }

        // Pedro's frame: +forward, +left, +turn = counter-clockwise. The gamepad's Y axis is
        // negative when pushed forward, and its X axes are positive to the right, so all
        // three are flipped.
        double forward = -deadzone(pad.left_stick_y);
        double left = -deadzone(pad.left_stick_x);
        double turn = -deadzone(pad.right_stick_x) * DriverSettings.TURN_SCALE;

        if (DriverSettings.FIELD_CENTRIC) {
            // Rotate the stick (which is in the driver's frame) into the robot's frame.
            double angle = drive.heading() - driverForwardHeading;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double robotForward = forward * cos + left * sin;
            double robotLeft = -forward * sin + left * cos;
            forward = robotForward;
            left = robotLeft;
        }

        double speed = DriverSettings.NORMAL_SPEED;
        if (slowMode.down()) {
            speed = DriverSettings.SLOW_SPEED;
        } else if (turboMode.down()) {
            speed = DriverSettings.TURBO_SPEED;
        }

        cmdForward = forward * speed;
        cmdLeft = left * speed;
        cmdTurn = turn * speed;
        drive.drive(cmdForward, cmdLeft, cmdTurn);
    }

    private void updateMechanisms() {
        // Ejecting wins over the toggle and lasts only while held. The toggle is left alone,
        // so letting go puts the intake back to what it was doing.
        if (eject.down()) {
            intake.setState(Intake.State.EJECT);
            transport.setState(Transport.State.REVERSE);
        } else {
            intake.setState(intakeOn.isOn() ? Intake.State.INTAKE : Intake.State.OFF);
            // With a gate (RUN_WITH_INTAKE) the transport runs with the intake toggle only, so
            // balls are carried up to the stopped gate and X opens just the gate. Without one,
            // the transport is what feeds the wheel, so it runs while X is held.
            boolean feed = Transport.RUN_WITH_INTAKE ? intakeOn.isOn() : shoot.down();
            transport.setState(feed ? Transport.State.FEED : Transport.State.OFF);
        }

        if (shooterStart.pressed()) {
            shooter.setSpinning(true);
        } else if (shooterStop.pressed()) {
            shooter.setSpinning(false);
        }
        // Held X spins the gate whatever the flywheel speed (the driver's choice).
        shooter.setGateTestOpen(shoot.down());
        flowerGate.setRotating(flower.down());

        intake.update();
        transport.update();
        shooter.update();
        flowerGate.update();
    }

    private void updateDashboard() {
        dashboard.section("DRIVE: " + drive.name());
        dashboard.addData("Mode", lockPosition.down() ? "LOCKED"
                : DriverSettings.FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        dashboard.addData("Speed", slowMode.down() ? "SLOW" : turboMode.down() ? "TURBO" : "NORMAL");
        drive.report(dashboard);

        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        flowerGate.report(dashboard);

        dashboard.update();
    }

    /**
     * One row of the match recording: what the driver did, what the code asked the drivetrain
     * for, and what every part of the robot was doing.
     *
     * Reading it: raw_* is the driver's gamepad straight from the Driver Station. If raw_* moves
     * but cmd_* doesn't, the problem is in the code. If raw_* freezes or gamepad_id changes, it's
     * the gamepad or the DS link. If merged_* differs from raw_*, a browser gamepad in Panels is
     * overriding the driver. operator_* is gamepad 2, which runs the mechanisms. A big loop_ms
     * means the loop stalled while the motors kept their last power.
     */
    private void recordLoop() {
        recorder.put("uptime_ms", SystemClock.uptimeMillis());
        recorder.put("gamepad_id", gamepad1.getGamepadId());
        recorder.put("gamepad_timestamp", gamepad1.timestamp);
        recordGamepad("raw", gamepad1);
        recordGamepad("merged", pad);   // null until PLAY, when readGamepad() first runs
        recorder.put("operator_id", gamepad2.getGamepadId());
        recorder.put("operator_timestamp", gamepad2.timestamp);
        recordGamepad("operator", gamepad2);

        boolean running = pad != null;
        recorder.put("cmd_forward", running ? cmdForward : null);
        recorder.put("cmd_left", running ? cmdLeft : null);
        recorder.put("cmd_turn", running ? cmdTurn : null);
        recorder.put("drive_mode", !running ? null : lockPosition.down() ? "LOCKED"
                : DriverSettings.FIELD_CENTRIC ? "FIELD" : "ROBOT");
        recorder.put("speed_mode", !running ? null
                : slowMode.down() ? "SLOW" : turboMode.down() ? "TURBO" : "NORMAL");
        drive.record(recorder);
        intake.record(recorder);
        transport.record(recorder);
        shooter.record(recorder);
        flowerGate.record(recorder);

        int gamepadId = gamepad1.getGamepadId();
        if (gamepadId != lastGamepadId) {
            recorder.event("gamepad1 id " + lastGamepadId + " -> " + gamepadId);
            lastGamepadId = gamepadId;
        }
        int operatorId = gamepad2.getGamepadId();
        if (operatorId != lastOperatorId) {
            recorder.event("gamepad2 id " + lastOperatorId + " -> " + operatorId);
            lastOperatorId = operatorId;
        }
        if (running) {
            eventIf(intakeOn.pressed(), intakeOn.isOn() ? "intake on" : "intake off");
            eventIf(eject.pressed(), "eject start");
            eventIf(eject.released(), "eject end");
            eventIf(shooterStart.pressed(), "flywheel start");
            eventIf(shooterStop.pressed(), "flywheel stop");
            eventIf(shoot.pressed(), "shoot start");
            eventIf(shoot.released(), "shoot end");
            eventIf(lockPosition.pressed(), "lock");
            eventIf(lockPosition.released(), "unlock");
            eventIf(resetForward.pressed(), "reset forward");
            eventIf(toggleCentric.pressed(), DriverSettings.FIELD_CENTRIC ? "field-centric" : "robot-centric");
            eventIf(velocityMode.pressed(), "velocity mode toggled");
            eventIf(flower.pressed(), "flower gate start");
            eventIf(flower.released(), "flower gate stop");
        }
        recorder.endLoop();
    }

    private void recordGamepad(String prefix, Gamepad g) {
        recorder.put(prefix + "_lx", g == null ? null : g.left_stick_x);
        recorder.put(prefix + "_ly", g == null ? null : g.left_stick_y);
        recorder.put(prefix + "_rx", g == null ? null : g.right_stick_x);
        recorder.put(prefix + "_lt", g == null ? null : g.left_trigger);
        recorder.put(prefix + "_rt", g == null ? null : g.right_trigger);
        recorder.put(prefix + "_buttons", g == null ? null : buttonsDown(g));
    }

    /** The buttons held right now, e.g. "A RB DU". Empty when none are. */
    private static String buttonsDown(Gamepad g) {
        StringBuilder held = new StringBuilder();
        if (g.a) held.append("A ");
        if (g.b) held.append("B ");
        if (g.x) held.append("X ");
        if (g.y) held.append("Y ");
        if (g.left_bumper) held.append("LB ");
        if (g.right_bumper) held.append("RB ");
        if (g.dpad_up) held.append("DU ");
        if (g.dpad_down) held.append("DD ");
        if (g.dpad_left) held.append("DL ");
        if (g.dpad_right) held.append("DR ");
        if (g.back) held.append("BACK ");
        if (g.start) held.append("START ");
        return held.toString().trim();
    }

    private void eventIf(boolean happened, String text) {
        if (happened) {
            recorder.event(text);
        }
    }

    private static double deadzone(double stick) {
        return Math.abs(stick) < DriverSettings.STICK_DEADZONE ? 0.0 : stick;
    }
}
