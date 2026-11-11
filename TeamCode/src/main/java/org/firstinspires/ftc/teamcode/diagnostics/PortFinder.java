package org.firstinspires.ftc.teamcode.diagnostics;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Dashboard;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/**
 * Finds out what is plugged into which port. Lists every motor, CR servo and servo in the
 * ACTIVE robot config with its hub and port, and runs only the one you select, so you can
 * watch what moves. Uses no device names from the code, so it works with any config.
 *
 *   D-pad Up / Down   select the previous / next port (the old one stops)
 *   A (hold)          run forward      (servo: move position up)
 *   B (hold)          run backward     (servo: move position down)
 *   D-pad Right/Left  servo only: position +/- SERVO_STEP per click (0.1)
 *   right/left bumper servo only: position +/- SERVO_FINE_STEP per click (0.01)
 *
 * Encoders: every motor line shows its encoder count. Turn a wheel by hand and see which
 * line changes -- that is the port its encoder cable is in. It must match its motor's port.
 *
 * Only ports that are in the config can be tested. To check every port, make a temporary
 * config with a motor on every motor port and a servo on every servo port, then switch
 * back to the real one.
 *
 * Robot on blocks: a selected drive motor really spins.
 */
@Configurable
@TeleOp(name = "Port Finder", group = "Diagnostics")
public class PortFinder extends LinearOpMode {

    public static double MOTOR_POWER = 0.3;
    public static double CR_SERVO_POWER = 0.5;
    /** How fast a positional servo moves while A/B is held, in position units per second. */
    public static double SERVO_SPEED = 0.25;
    /** Positional servo: how far one D-pad Right/Left click moves it. */
    public static double SERVO_STEP = 0.1;
    /** Positional servo: how far one bumper click moves it. */
    public static double SERVO_FINE_STEP = 0.01;

    private enum Kind {
        MOTOR("motor"), CR_SERVO("CR servo"), SERVO("servo");

        final String label;

        Kind(String label) {
            this.label = label;
        }
    }

    private static class Port {
        final Kind kind;
        final String name;
        final String hub;
        final int number;
        final HardwareDevice device;
        double servoPosition = 0.5;   // a positional servo starts from the middle

        Port(Kind kind, String name, String hub, int number, HardwareDevice device) {
            this.kind = kind;
            this.name = name;
            this.hub = hub;
            this.number = number;
            this.device = device;
        }

        String label() {
            return hub + "  " + kind.label + " port " + number + "  = '" + name + "'";
        }
    }

    private final List<Port> ports = new ArrayList<>();

    @Override
    public void runOpMode() {
        Dashboard dashboard = new Dashboard(telemetry);
        findPorts();

        Button.Group buttons = new Button.Group();
        Button previous = buttons.add(() -> gamepad1.dpad_up);
        Button next = buttons.add(() -> gamepad1.dpad_down);
        Button stepUp = buttons.add(() -> gamepad1.dpad_right);
        Button stepDown = buttons.add(() -> gamepad1.dpad_left);
        Button fineUp = buttons.add(() -> gamepad1.right_bumper);
        Button fineDown = buttons.add(() -> gamepad1.left_bumper);

        while (opModeInInit()) {
            dashboard.addLine("Port Finder -- " + ports.size() + " devices in the active config.");
            dashboard.addLine("Robot on blocks! Press PLAY, then D-pad to select, hold A/B to run.");
            listPorts(dashboard, -1);
            dashboard.update();
        }

        int selected = 0;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            buttons.update();

            if (ports.isEmpty()) {
                dashboard.addLine("No motors or servos in the active config.");
                dashboard.update();
                continue;
            }

            if (next.pressed() || previous.pressed()) {
                stop(ports.get(selected));
                int step = next.pressed() ? 1 : -1;
                selected = (selected + step + ports.size()) % ports.size();
            }

            Port port = ports.get(selected);
            double direction = gamepad1.a ? 1 : gamepad1.b ? -1 : 0;
            run(port, direction, dt);
            if (port.kind == Kind.SERVO) {
                double step = (stepUp.pressed() ? SERVO_STEP : 0) - (stepDown.pressed() ? SERVO_STEP : 0)
                        + (fineUp.pressed() ? SERVO_FINE_STEP : 0) - (fineDown.pressed() ? SERVO_FINE_STEP : 0);
                if (step != 0) {
                    port.servoPosition = Range.clip(port.servoPosition + step, 0, 1);
                    ((Servo) port.device).setPosition(port.servoPosition);
                }
            }

            dashboard.addLine("D-pad Up/Down select   A forward   B backward");
            if (port.kind == Kind.SERVO) {
                dashboard.addLine("Servo: D-pad Right/Left +/-" + SERVO_STEP + "   bumpers +/-" + SERVO_FINE_STEP);
            }
            dashboard.section("SELECTED (" + (selected + 1) + " of " + ports.size() + ")");
            dashboard.addLine(port.label());
            if (port.kind == Kind.SERVO) {
                dashboard.addData("Servo position", Math.round(port.servoPosition * 1000) / 1000.0);
            }
            listPorts(dashboard, selected);
            dashboard.update();
        }

        for (Port port : ports) {
            stop(port);
        }
    }

    /** Reads every motor and servo from the config, sorted by hub, then type, then port. */
    private void findPorts() {
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            // Power control only; the encoder still counts in this mode.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            ports.add(new Port(Kind.MOTOR, nameOf(motor), hubOf(motor.getController()),
                    motor.getPortNumber(), motor));
        }
        for (CRServo servo : hardwareMap.getAll(CRServo.class)) {
            servo.setPower(0);
            ports.add(new Port(Kind.CR_SERVO, nameOf(servo), hubOf(servo.getController()),
                    servo.getPortNumber(), servo));
        }
        // Positional servos are not commanded until A/B is pressed, so nothing moves at init.
        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            ports.add(new Port(Kind.SERVO, nameOf(servo), hubOf(servo.getController()),
                    servo.getPortNumber(), servo));
        }
        ports.sort(Comparator.comparing((Port p) -> p.hub)
                .thenComparing(p -> p.kind)
                .thenComparingInt(p -> p.number));
    }

    private void run(Port port, double direction, double dt) {
        switch (port.kind) {
            case MOTOR:
                ((DcMotor) port.device).setPower(direction * MOTOR_POWER);
                break;
            case CR_SERVO:
                ((CRServo) port.device).setPower(direction * CR_SERVO_POWER);
                break;
            case SERVO:
                if (direction != 0) {
                    port.servoPosition = Range.clip(port.servoPosition + direction * SERVO_SPEED * dt, 0, 1);
                    ((Servo) port.device).setPosition(port.servoPosition);
                }
                break;
        }
    }

    /** Motors and CR servos stop. A positional servo just stays where it is. */
    private void stop(Port port) {
        if (port.kind == Kind.MOTOR) {
            ((DcMotor) port.device).setPower(0);
        } else if (port.kind == Kind.CR_SERVO) {
            ((CRServo) port.device).setPower(0);
        }
    }

    private void listPorts(Dashboard dashboard, int selected) {
        dashboard.section("ALL PORTS");
        for (int i = 0; i < ports.size(); i++) {
            Port port = ports.get(i);
            String line = (i == selected ? ">> " : "   ") + port.label();
            if (port.kind == Kind.MOTOR) {
                line += "  enc " + ((DcMotor) port.device).getCurrentPosition();
            }
            dashboard.addLine(line);
        }
    }

    private String nameOf(HardwareDevice device) {
        Set<String> names = hardwareMap.getNamesOf(device);
        return names.isEmpty() ? "?" : names.iterator().next();
    }

    /** The hub's name from the config, e.g. "Control Hub" or "Expansion Hub 2". */
    private String hubOf(HardwareDevice controller) {
        Set<String> names = hardwareMap.getNamesOf(controller);
        return names.isEmpty() ? controller.getConnectionInfo() : names.iterator().next();
    }
}
