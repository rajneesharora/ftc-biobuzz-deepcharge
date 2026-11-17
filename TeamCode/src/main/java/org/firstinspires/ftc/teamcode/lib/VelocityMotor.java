package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A motor that holds a target speed using the hub's velocity control. Good for flywheels and
 * intakes: the speed stays the same as the battery drains, which raw power does not.
 */
public class VelocityMotor {

    /** How close counts as "at speed", in RPM. */
    public static double RPM_TOLERANCE = 10;

    private final DcMotorEx motor;
    private final double ticksPerRevolution;
    private double targetRPM = 0;
    private double p, i, d, f;

    /**
     * @param ticksPerRevolution encoder ticks per OUTPUT shaft turn, including the gearbox.
     *                           goBILDA 5202/5203 bare motor (6000 RPM) = 28, 19.2:1 = 537.7,
     *                           5.2:1 = 145.1
     */
    public VelocityMotor(DcMotorEx motor, DcMotorSimple.Direction direction, double ticksPerRevolution) {
        if (!Double.isFinite(ticksPerRevolution) || ticksPerRevolution <= 0) {
            throw new IllegalArgumentException("Ticks per revolution must be positive and finite");
        }
        this.motor = motor;
        this.ticksPerRevolution = ticksPerRevolution;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Sends velocity PIDF to the motor, but only when a number actually changed. */
    public void setPIDF(double p, double i, double d, double f) {
        if (p == this.p && i == this.i && d == this.d && f == this.f) {
            return;
        }
        motor.setVelocityPIDFCoefficients(p, i, d, f);
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    /** Spin to this speed. 0 = off. */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        motor.setVelocity(rpm * ticksPerRevolution / 60.0);
    }

    public void stop() {
        setTargetRPM(0);
    }

    /** How fast it is actually spinning. */
    public double getRPM() {
        return motor.getVelocity() * 60.0 / ticksPerRevolution;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /** Close enough to the target speed? */
    public boolean atSpeed() {
        return targetRPM > 0 && Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }
}
