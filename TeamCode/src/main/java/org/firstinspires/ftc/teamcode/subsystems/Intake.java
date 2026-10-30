package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * Intake: one motor, plus an optional second one (e.g. a top and bottom roller).
 *
 * Devices are looked up with tryGet(), so a robot without an intake still runs -- the
 * dashboard shows MISSING instead of crashing. That lets the same code run on every robot.
 * The second motor is optional: if "intake2" isn't in the config it is simply not used.
 *
 * Names must match the Driver Station robot configuration exactly (case-sensitive).
 */
@Configurable
public class Intake implements Subsystem {

    /** Read once at init. false = never touch the hardware (pure placeholder). */
    public static boolean ENABLED = true;
    public static String MOTOR_NAME = "intake";
    public static String MOTOR_2_NAME = "intake2";
    public static boolean REVERSED = false;
    public static boolean REVERSED_2 = false;

    // Live-editable in Panels.
    public static double INTAKE_POWER = 0.8;
    public static double EJECT_POWER = -1.0;

    public enum State { OFF, INTAKE, EJECT }

    private final DcMotorEx motor;    // null if missing
    private final DcMotorEx motor2;   // null if missing or not fitted
    private final String status;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        motor = ENABLED ? hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME) : null;
        motor2 = ENABLED ? hardwareMap.tryGet(DcMotorEx.class, MOTOR_2_NAME) : null;
        setUp(motor, REVERSED);
        setUp(motor2, REVERSED_2);

        if (!ENABLED) {
            status = "disabled";
        } else if (motor == null) {
            status = "MISSING '" + MOTOR_NAME + "' in robot config";
        } else {
            status = motor2 == null ? "active (1 motor)" : "active (2 motors)";
        }
    }

    private static void setUp(DcMotorEx m, boolean reversed) {
        if (m == null) {
            return;
        }
        m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    @Override
    public void update() {
        double power;
        switch (state) {
            case INTAKE: power = INTAKE_POWER; break;
            case EJECT:  power = EJECT_POWER;  break;
            default:     power = 0;            break;
        }
        if (motor != null) {
            motor.setPower(power);
        }
        if (motor2 != null) {
            motor2.setPower(power);
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("INTAKE");
        dashboard.addData("Intake", state + "  [" + status + "]");
    }

    @Override
    public void record(MatchRecorder recorder) {
        recorder.put("intake", state);
    }

    @Override
    public void stop() {
        state = State.OFF;
        update();
    }
}
