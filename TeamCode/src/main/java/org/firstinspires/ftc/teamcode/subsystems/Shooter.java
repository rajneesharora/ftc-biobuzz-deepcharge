package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;
import org.firstinspires.ftc.teamcode.lib.VelocityMotor;

/**
 * Shooter: one velocity-controlled flywheel, plus a gate that holds balls back from it.
 * The gate is a CONTINUOUS ROTATION servo (configure it as "Continuous Rotation Servo"):
 * stopped, it blocks the balls; spinning at GATE_FEED_POWER, it passes them to the wheel.
 *
 * isFiring() is true only while fire is held AND the wheel is at speed, so the gate never
 * feeds a slow wheel. After each shot the RPM dips; the gate stops until it recovers,
 * which spaces the shots out so each one leaves at the same speed.
 *
 * The gate is only commanded after PLAY (in update()), so it doesn't move during INIT.
 *
 * The flywheel's ENCODER cable must be plugged in (same port number, encoder connector).
 * Without it the hub thinks the wheel is stopped and drives it at full power.
 *
 * Tuning (Panels > Configurables, flywheel spinning, robot on blocks):
 *   1. VEL_P = 0. Raise VEL_F until actual RPM is close to TARGET_RPM.
 *   2. Raise VEL_P until it recovers quickly after a shot without oscillating.
 *   3. Copy the numbers back into this file -- Panels edits are not saved.
 */
@Configurable
public class Shooter implements Subsystem {

    public static boolean ENABLED = true;
    public static String FLYWHEEL_NAME = "shooter";
    public static boolean FLYWHEEL_REVERSED = true;
    /**
     * Encoder ticks per FLYWHEEL MOTOR output revolution. goBILDA 5202/5203 bare 1:1
     * (6000 RPM) = 28; 3.7:1 (1620 RPM) = 103.8; 5.2:1 (1150 RPM) = 145.1. Check yours.
     */
    public static double FLYWHEEL_TICKS_PER_REV = 28;

    /** false = no gate fitted (the transport alone feeds the wheel). */
    public static boolean USE_GATE = true;
    public static String GATE_NAME = "shooterGate";
    /** Flip if the gate spins balls back toward the transport instead of into the wheel. */
    public static boolean GATE_REVERSED = true;
    /** Gate speed while feeding, 0 to 1. Lower it if balls go in too close together. */
    public static double GATE_FEED_POWER = 1.0;

    // Live-editable in Panels.
    public static double TARGET_RPM = 2550;
    public static double VEL_P = 20;
    public static double VEL_I = 0;
    public static double VEL_D = 0;
    public static double VEL_F = 12;

    private final VelocityMotor flywheel;   // null if missing
    private final CRServo gate;             // null if missing or USE_GATE is false
    private final String status;

    private boolean spinning = false;
    private boolean fireRequested = false;
    private boolean gateTestOpen = false;

    // Read once per update(), so the dashboard and the recording don't each ask the hub again.
    private double measuredRpm = 0;
    private boolean gateOpen = false;

    public Shooter(HardwareMap hardwareMap) {
        DcMotorEx motor = ENABLED ? hardwareMap.tryGet(DcMotorEx.class, FLYWHEEL_NAME) : null;
        flywheel = motor == null ? null : new VelocityMotor(motor,
                FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD,
                FLYWHEEL_TICKS_PER_REV);

        gate = ENABLED && USE_GATE ? hardwareMap.tryGet(CRServo.class, GATE_NAME) : null;
        if (gate != null) {
            gate.setDirection(GATE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        if (!ENABLED) {
            status = "disabled";
        } else if (flywheel == null || (USE_GATE && gate == null)) {
            status = "MISSING" + (flywheel == null ? " '" + FLYWHEEL_NAME + "'" : "")
                    + (USE_GATE && gate == null ? " '" + GATE_NAME + "'" : "") + " in robot config";
        } else {
            status = USE_GATE ? "active (gate)" : "active (fed by transport)";
        }
    }

    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    /** Hold true to shoot. Ignored until the flywheel is at speed. */
    public void setFire(boolean fire) {
        this.fireRequested = fire;
    }

    public boolean atSpeed() {
        if (flywheel == null) {
            return spinning;   // no wheel to measure: pretend, so the controls stay testable
        }
        return flywheel.atSpeed();
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
        if (flywheel != null) {
            flywheel.setPIDF(VEL_P, VEL_I, VEL_D, VEL_F);
            flywheel.setTargetRPM(spinning ? TARGET_RPM : 0);
            measuredRpm = flywheel.getRPM();
        }
        gateOpen = isGateOpen();
        if (gate != null) {
            gate.setPower(gateOpen ? GATE_FEED_POWER : 0);
        }
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.section("SHOOTER");
        dashboard.addData("Shooter", (spinning ? "SPINNING" : "OFF") + "  [" + status + "]");
        dashboard.addData("RPM (actual / target)", flywheel == null ? "-- / " + Math.round(TARGET_RPM)
                : Math.round(measuredRpm) + " / " + Math.round(flywheel.getTargetRPM()));
        dashboard.addData("Ready to fire", atSpeed());
        dashboard.addData("Gate", gate == null ? "--" : isGateOpen() ? "FEEDING" : "STOPPED");
        dashboard.addData("Firing", isFiring());
    }

    /**
     * Each shot shows up as a dip in shooter_rpm. How deep it goes and how long it takes to get
     * back within RPM_TOLERANCE is what limits how fast the robot can shoot.
     */
    @Override
    public void record(MatchRecorder recorder) {
        recorder.put("shooter_spinning", spinning);
        recorder.put("shooter_target_rpm", flywheel == null ? null : flywheel.getTargetRPM());
        recorder.put("shooter_rpm", flywheel == null ? null : measuredRpm);
        // Same test as VelocityMotor.atSpeed(), on this loop's reading -- no extra hub read.
        recorder.put("shooter_at_speed", flywheel != null && flywheel.getTargetRPM() > 0
                && Math.abs(measuredRpm - flywheel.getTargetRPM()) < VelocityMotor.RPM_TOLERANCE);
        recorder.put("shooter_fire_requested", fireRequested);
        recorder.put("gate_override", gateTestOpen);
        recorder.put("gate_open", gateOpen);
    }

    @Override
    public void stop() {
        spinning = false;
        fireRequested = false;
        gateTestOpen = false;
        update();
    }
}
