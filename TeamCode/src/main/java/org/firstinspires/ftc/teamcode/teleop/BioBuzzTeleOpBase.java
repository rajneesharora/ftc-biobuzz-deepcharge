package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
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
 * DRIVER -- gamepad 1 (and a controller in the Panels browser tab, only if
 *           DriverSettings.ALLOW_BROWSER_GAMEPAD is on -- keep it off for matches):
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

        // INIT: keep the sensors running so heading/pose are live before PLAY.
        while (opModeInInit()) {
            drive.update();
            dashboard.addLine("BioBuzz TeleOp [" + drive.name() + "] ready. Press PLAY.");
            drive.report(dashboard);
            dashboard.update();
        }
        if (isStopRequested()) {
            return;
        }

        driverForwardHeading = drive.heading();

        while (opModeIsActive()) {
            readGamepad();
            updateDriving();
            drive.update();
            updateMechanisms();
            updateDashboard();
        }

        drive.stop();
        intake.stop();
        transport.stop();
        shooter.stop();
        flowerGate.stop();
    }

    /** Reads the gamepads once, so everything below sees the same thing. */
    private void readGamepad() {
        pad = DriverSettings.ALLOW_BROWSER_GAMEPAD ? dashboard.mergeGamepad(gamepad1) : gamepad1;
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

        drive.drive(forward * speed, left * speed, turn * speed);
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
        if (DriverSettings.ALLOW_BROWSER_GAMEPAD) {
            dashboard.addData("Browser gamepad", "ON -- turn off for matches");
        }
        drive.report(dashboard);

        intake.report(dashboard);
        transport.report(dashboard);
        shooter.report(dashboard);
        flowerGate.report(dashboard);

        dashboard.update();
    }

    private static double deadzone(double stick) {
        return Math.abs(stick) < DriverSettings.STICK_DEADZONE ? 0.0 : stick;
    }
}
