package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Dashboard;

/**
 * Transport: two continuous-rotation servos that carry pieces from the intake into the
 * flywheel. Configure them on the Driver Station as "Continuous Rotation Servo", NOT "Servo".
 *
 * Set each side so both push balls toward the shooter (new robot: neither reversed; the old
 * robot had both REVERSED).
 * If pieces get pushed the wrong way by one side, flip that side's REVERSED flag.
 *
 * Missing servos are reported on the dashboard, never a crash.
 */
@Configurable
public class Transport implements Subsystem {

    public static boolean ENABLED = true;
    public static String LEFT_NAME = "transportLeft";
    public static String RIGHT_NAME = "transportRight";
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = false;

    public static double FEED_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;
    /**
     * true = also run while the intake is on, carrying balls up to the closed shooter gate,
     * where they wait. Right for a robot WITH a gate.
     * false = only run while firing (and ejecting). Use this if there is NO gate, otherwise
     * balls are shot as soon as they're picked up.
     */
    public static boolean RUN_WITH_INTAKE = false;   // this robot has no gate

    public enum State { OFF, FEED, REVERSE }

    private final CRServo left;
    private final CRServo right;
    private final String status;
    private State state = State.OFF;        // left side (and both, when set with setState)
    private State rightState = State.OFF;
    private boolean rightStopped = false;
    private final boolean leftReversedUsed = LEFT_REVERSED;    // what INIT actually used
    private final boolean rightReversedUsed = RIGHT_REVERSED;

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

    /** Both sides the same. */
    public void setState(State state) {
        this.state = state;
        this.rightState = state;
    }

    /** Each side on its own, e.g. left carrying balls in while right only feeds the shooter. */
    public void setSides(State left, State right) {
        this.state = left;
        this.rightState = right;
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
        if (left != null) {
            left.setPower(powerFor(state));
        }
        if (right != null) {
            right.setPower(rightStopped ? 0 : powerFor(rightState));
        }
    }

    private static double powerFor(State state) {
        switch (state) {
            case FEED:    return FEED_POWER;
            case REVERSE: return REVERSE_POWER;
            default:      return 0;
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("TRANSPORT");
        dashboard.addData("Transport L / R", state + " / " + (rightStopped ? "held" : rightState) + "  [" + status + "]");
        dashboard.addData("LEFT / RIGHT_REVERSED (used at INIT)", leftReversedUsed + " / " + rightReversedUsed);
    }

    @Override
    public void stop() {
        state = State.OFF;
        rightState = State.OFF;
        rightStopped = false;
        update();
    }
}
