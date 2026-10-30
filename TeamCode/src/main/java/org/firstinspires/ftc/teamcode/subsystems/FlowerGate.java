package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * Flower gate: one servo (Control Hub servo port 3, "flowerGate").
 *
 * Works with either servo type, whichever the Driver Station config says:
 *   Continuous Rotation Servo -- spins at ROTATE_POWER while rotating, stops otherwise.
 *   Servo                     -- goes to ROTATED_POSITION while rotating, back to
 *                                HOME_POSITION otherwise.
 * If it turns the wrong way, flip REVERSED.
 *
 * The servo is only commanded after PLAY (in update()), so it doesn't move during INIT.
 * Missing servo is reported on the dashboard, never a crash.
 */
@Configurable
public class FlowerGate implements Subsystem {

    public static boolean ENABLED = true;
    public static String NAME = "flowerGate";
    public static boolean REVERSED = false;

    // Live-editable in Panels.
    /** Continuous rotation servo: speed while rotating, 0 to 1. */
    public static double ROTATE_POWER = 1.0;
    /** Regular servo: position at rest and while rotating, 0 to 1. */
    public static double HOME_POSITION = 0.0;
    public static double ROTATED_POSITION = 0.5;

    private final CRServo crServo;   // null if missing or configured as a regular servo
    private final Servo servo;       // null if missing or configured as continuous rotation
    private final String status;
    private boolean rotating = false;

    public FlowerGate(HardwareMap hardwareMap) {
        crServo = ENABLED ? hardwareMap.tryGet(CRServo.class, NAME) : null;
        servo = ENABLED && crServo == null ? hardwareMap.tryGet(Servo.class, NAME) : null;
        if (crServo != null) {
            crServo.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
        if (servo != null) {
            servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }

        if (!ENABLED) {
            status = "disabled";
        } else if (crServo != null) {
            status = "active (continuous rotation)";
        } else if (servo != null) {
            status = "active (position servo)";
        } else {
            status = "MISSING '" + NAME + "' in robot config";
        }
    }

    public void setRotating(boolean rotating) {
        this.rotating = rotating;
    }

    public boolean isRotating() {
        return rotating;
    }

    @Override
    public void update() {
        if (crServo != null) {
            crServo.setPower(rotating ? ROTATE_POWER : 0);
        }
        if (servo != null) {
            servo.setPosition(rotating ? ROTATED_POSITION : HOME_POSITION);
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("FLOWER GATE");
        dashboard.addData("Flower gate", (rotating ? "ROTATING" : "STOPPED") + "  [" + status + "]");
    }

    @Override
    public void record(MatchRecorder recorder) {
        recorder.put("flower_gate_rotating", rotating);
    }

    @Override
    public void stop() {
        rotating = false;
        if (crServo != null) {
            crServo.setPower(0);
        }
    }
}
