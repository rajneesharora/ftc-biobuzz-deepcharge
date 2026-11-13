package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Dashboard;

/**
 * Shooter: two flywheel motors ("shooterTop", "shooterBottom") geared together with same-size
 * gears, so they always turn at the same speed. An optional gate servo can hold balls back.
 *
 * Both motors get the SAME power from one speed controller that reads shooterTop's encoder:
 *
 *   power = (KV * targetRPM + KS) * NOMINAL_VOLTS / battery  +  KP * (targetRPM - actualRPM)
 *
 * Same power means they can never fight over the speed. BOTTOM_REVERSED makes shooterBottom
 * push the gears the same way as shooterTop. Port Finder (raw power, one motor at a time)
 * showed shooterBottom spins the flywheels the opposite way to shooterTop, so it is reversed.
 * (An encoder-sign check was misleading here: the two encoder cables appear to be swapped.)
 * Each motor's ENCODER cable must be in the SAME port number as its power cable -- check that
 * by eye; with geared motors, turning the wheel by hand moves both encoders, so it can't tell.
 * NEVER change BOTTOM_REVERSED without re-checking: if it's wrong, the two motors fight through
 * the gears -- they stall, buzz and get hot, and can strip the gears or burn out.
 *
 * BOTTOM_ENABLED = false leaves shooterBottom unpowered (it just turns along through the
 * gears) -- use that to check shooterTop's direction and RPM on its own first.
 *
 * shooterTop's ENCODER cable must be plugged in, in shooterTop's port. If the flywheel reads
 * ~0 RPM while being driven hard, or reads BACKWARDS (negative) while being driven forward,
 * the controller stops and shows ENCODER FAULT instead of pushing full power forever; turn the
 * flywheel off and on again after fixing it.
 *
 * Tuning (Panels > Configurables > Shooter, flywheel spinning, no balls):
 *   1. KP = 0. Adjust KV until actual RPM is close to TARGET_RPM (raise KV if it's slow).
 *   2. Raise KP until it recovers quickly after a shot, without the RPM bouncing.
 *   3. Send the numbers to be saved -- Panels edits are not kept.
 */
@Configurable
public class Shooter implements Subsystem {

    public static boolean ENABLED = true;
    public static String TOP_NAME = "shooterTop";
    public static String BOTTOM_NAME = "shooterBottom";
    /** Flip if the flywheels spin backwards (balls not shot out). Flip BOTH directions together. */
    public static boolean TOP_REVERSED = false;
    public static boolean BOTTOM_REVERSED = true;
    /** false = only shooterTop is powered (for re-checking directions). Checked: both work together. */
    public static boolean BOTTOM_ENABLED = true;
    /** Encoder ticks per motor output revolution. goBILDA 5202/5203 bare (6000 RPM) = 28. */
    public static double TICKS_PER_REV = 28;

    // Live-editable in Panels.
    public static double TARGET_RPM = 2550;
    /** Power per RPM at NOMINAL_VOLTS. ~1 / free speed: 6000 RPM motor -> about 0.00017. */
    public static double KV = 0.00017;
    /** Extra power to overcome friction. */
    public static double KS = 0.02;
    /** Power per RPM of error -- how hard it pushes to catch up after a shot. */
    public static double KP = 0.0004;
    public static double NOMINAL_VOLTS = 12.0;
    public static double MAX_POWER = 1.0;
    /** How close counts as "at speed". */
    public static double RPM_TOLERANCE = 100;

    /** false = no gate fitted (the transport alone feeds the wheels). */
    public static boolean USE_GATE = false;
    public static String GATE_NAME = "shooterGate";
    public static boolean GATE_REVERSED = true;
    public static double GATE_FEED_POWER = 1.0;

    private final DcMotorEx top;      // null if missing
    private final DcMotorEx bottom;   // null if missing
    private final CRServo gate;       // null if missing or USE_GATE is false
    private final VoltageSensor battery;
    private final String status;

    private boolean spinning = false;
    private boolean fireRequested = false;
    private boolean gateTestOpen = false;

    private double power = 0;
    private double volts = NOMINAL_VOLTS;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime stalledTimer = new ElapsedTime();
    private boolean encoderFault = false;
    private String faultReason = "";
    private final boolean topReversedUsed = TOP_REVERSED;      // what INIT actually used
    private final boolean bottomReversedUsed = BOTTOM_REVERSED;

    public Shooter(HardwareMap hardwareMap) {
        top = ENABLED ? hardwareMap.tryGet(DcMotorEx.class, TOP_NAME) : null;
        bottom = ENABLED ? hardwareMap.tryGet(DcMotorEx.class, BOTTOM_NAME) : null;
        setUp(top, TOP_REVERSED);
        setUp(bottom, BOTTOM_REVERSED);

        gate = ENABLED && USE_GATE ? hardwareMap.tryGet(CRServo.class, GATE_NAME) : null;
        if (gate != null) {
            gate.setDirection(GATE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        VoltageSensor found = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            found = sensor;
            break;
        }
        battery = found;

        if (!ENABLED) {
            status = "disabled";
        } else if (top == null || bottom == null || (USE_GATE && gate == null)) {
            status = "MISSING" + (top == null ? " '" + TOP_NAME + "'" : "")
                    + (bottom == null ? " '" + BOTTOM_NAME + "'" : "")
                    + (USE_GATE && gate == null ? " '" + GATE_NAME + "'" : "") + " in robot config";
        } else {
            status = "active" + (USE_GATE ? " (gate)" : " (fed by transport)");
        }
    }

    private static void setUp(DcMotorEx motor, boolean reversed) {
        if (motor == null) {
            return;
        }
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Power is computed here, not by the hub; the encoder still counts in this mode.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpinning(boolean spinning) {
        if (spinning && !this.spinning) {
            encoderFault = false;   // turning it on again retries after a fault
            stalledTimer.reset();
        }
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    /** Hold true to shoot. Ignored until the flywheel is at speed. */
    public void setFire(boolean fire) {
        this.fireRequested = fire;
    }

    /** Measured flywheel speed (shooterTop's encoder), or 0 if it's missing. */
    public double getRPM() {
        return top == null ? 0 : top.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /** shooterBottom's encoder, if its cable is plugged in -- should match getRPM(). */
    public double getBottomRPM() {
        return bottom == null ? 0 : bottom.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public boolean atSpeed() {
        if (top == null) {
            return spinning;   // no wheel to measure: pretend, so the controls stay testable
        }
        return spinning && !encoderFault && Math.abs(getRPM() - TARGET_RPM) < RPM_TOLERANCE;
    }

    /** True while a piece should go into the wheel: gate open, transport feeding. */
    public boolean isFiring() {
        return fireRequested && spinning && atSpeed();
    }

    /** Run the gate regardless of flywheel speed (TeleOp X, Mechanism Check, auto's timed shooting). */
    public void setGateTestOpen(boolean open) {
        this.gateTestOpen = open;
    }

    public boolean isGateOpen() {
        return isFiring() || gateTestOpen;
    }

    @Override
    public void update() {
        if (battery != null && voltageTimer.milliseconds() > 250) {
            volts = Math.max(8, battery.getVoltage());   // ignore glitchy low readings
            voltageTimer.reset();
        }

        double rpm = getRPM();
        if (!spinning || TARGET_RPM <= 0 || top == null || encoderFault) {
            power = 0;
        } else {
            double feedforward = (KV * TARGET_RPM + KS) * NOMINAL_VOLTS / volts;
            power = Range.clip(feedforward + KP * (TARGET_RPM - rpm), 0, MAX_POWER);

            // Driven forward but reading backwards: wrong encoder cable/direction. Stop at once,
            // or the controller would push harder and harder to "catch up".
            if (rpm < -100) {
                encoderFault = true;
                faultReason = "'" + TOP_NAME + "' reads BACKWARDS (" + Math.round(rpm)
                        + " RPM) -- encoder cable in the wrong port, or swapped with " + BOTTOM_NAME;
                power = 0;
            }
            // Driven hard but not turning: the encoder cable is probably out. Stop, don't run away.
            if (power < 0.5 || Math.abs(rpm) > 100) {
                stalledTimer.reset();
            } else if (stalledTimer.seconds() > 1.5) {
                encoderFault = true;
                faultReason = "'" + TOP_NAME + "' reads ~0 RPM -- check its encoder cable";
                power = 0;
            }
        }

        if (top != null) {
            top.setPower(power);
        }
        if (bottom != null) {
            bottom.setPower(BOTTOM_ENABLED ? power : 0);
        }
        if (gate != null) {
            gate.setPower(isGateOpen() ? GATE_FEED_POWER : 0);
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("SHOOTER");
        dashboard.addData("Shooter", (spinning ? "SPINNING" : "OFF") + "  [" + status + "]");
        if (encoderFault) {
            dashboard.addData("ENCODER FAULT", faultReason + "; flywheel stopped. Fix, then turn it off and on");
        }
        dashboard.addData("RPM (actual / target)", Math.round(getRPM()) + " / " + Math.round(TARGET_RPM));
        dashboard.addData("Bottom encoder RPM", Math.round(getBottomRPM()) + "  (0 = cable not plugged in)");
        dashboard.addData("Power / battery", Math.round(power * 100) / 100.0 + " / " + Math.round(volts * 10) / 10.0 + " V");
        dashboard.addData("Bottom motor", BOTTOM_ENABLED ? "powered" : "OFF (BOTTOM_ENABLED = false)");
        dashboard.addData("TOP / BOTTOM_REVERSED (used at INIT)", topReversedUsed + " / " + bottomReversedUsed);
        dashboard.addData("Ready to fire", atSpeed());
        if (USE_GATE) {
            dashboard.addData("Gate", gate == null ? "--" : isGateOpen() ? "FEEDING" : "STOPPED");
        }
    }

    @Override
    public void stop() {
        spinning = false;
        fireRequested = false;
        gateTestOpen = false;
        update();
    }
}
