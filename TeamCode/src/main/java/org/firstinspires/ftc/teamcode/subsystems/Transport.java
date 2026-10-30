package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * Transport: two continuous-rotation servos that carry pieces from the intake into the
 * flywheel. Configure them on the Driver Station as "Continuous Rotation Servo", NOT "Servo".
 *
 * Set each side so both push balls toward the shooter (checked on this robot: both REVERSED).
 * If pieces get pushed the wrong way by one side, flip that side's REVERSED flag.
 *
 * Missing servos are reported on the dashboard, never a crash.
 */
@Configurable
public class Transport implements Subsystem {

    public static boolean ENABLED = true;
    public static String LEFT_NAME = "transportLeft";
    public static String RIGHT_NAME = "transportRight";
    public static boolean LEFT_REVERSED = true;
    public static boolean RIGHT_REVERSED = true;

    public static double FEED_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;
    /**
     * true = also run while the intake is on, carrying balls up to the closed shooter gate,
     * where they wait. Right for a robot WITH a gate.
     * false = only run while firing (and ejecting). Use this if there is NO gate, otherwise
     * balls are shot as soon as they're picked up.
     */
    public static boolean RUN_WITH_INTAKE = true;

    public enum State { OFF, FEED, REVERSE }

    private final CRServo left;
    private final CRServo right;
    private final String status;
    private State state = State.OFF;
    private boolean rightStopped = false;

    public Transport(HardwareMap hardwareMap) {
        left = ENABLED ? hardwareMap.tryGet(CRServo.class, LEFT_NAME) : null;
        right = ENABLED ? hardwareMap.tryGet(CRServo.class, RIGHT_NAME) : null;
        if (left != null) {
            left.setDirection(LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
        if (right != null) {
            right.setDirection(RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        if (!ENABLED) {
            status = "disabled";
        } else if (left == null || right == null) {
            status = "MISSING" + (left == null ? " '" + LEFT_NAME + "'" : "")
                    + (right == null ? " '" + RIGHT_NAME + "'" : "") + " in robot config";
        } else {
            status = "active";
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    /**
     * true = the right servo stays still whatever the state, so balls wait in the transport
     * (used by auto while the flywheel spins up). TeleOp never sets this.
     */
    public void setRightStopped(boolean stopped) {
        this.rightStopped = stopped;
    }

    @Override
    public void update() {
        double power;
        switch (state) {
            case FEED:    power = FEED_POWER;    break;
            case REVERSE: power = REVERSE_POWER; break;
            default:      power = 0;             break;
        }
        if (left != null) {
            left.setPower(power);
        }
        if (right != null) {
            right.setPower(rightStopped ? 0 : power);
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("TRANSPORT");
        dashboard.addData("Transport", state + (rightStopped ? " (right held)" : "") + "  [" + status + "]");
    }

    @Override
    public void record(MatchRecorder recorder) {
        recorder.put("transport", state);
        recorder.put("transport_right_held", rightStopped);
    }

    @Override
    public void stop() {
        state = State.OFF;
        rightStopped = false;
        update();
    }
}
