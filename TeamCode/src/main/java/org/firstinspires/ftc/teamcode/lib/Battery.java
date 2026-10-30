package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Iterator;

/**
 * Reads the robot battery voltage, and can scale motor power to match it.
 *
 * Motor specs (speed, torque) are rated at a nominal voltage. As the battery sags during a
 * match, the same motor power produces less speed and torque, so a fixed power command
 * drifts slower. compensate() scales a power command up by NOMINAL_VOLTAGE / actual voltage
 * so the robot responds the same way however charged the battery is.
 *
 * The voltage sensor is read at most once per REFRESH_INTERVAL_SECONDS and cached in
 * between, since battery voltage changes slowly and sensor reads are not free.
 */
public class Battery {
    /** Voltage the motor specs are rated at. Power is scaled relative to this. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    private static final double REFRESH_INTERVAL_SECONDS = 0.25;

    private final VoltageSensor sensor;   // null if the hub reports none
    private final ElapsedTime sinceRefresh = new ElapsedTime();
    private double voltage;

    public Battery(HardwareMap hardwareMap) {
        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        sensor = sensors.hasNext() ? sensors.next() : null;
        voltage = sensor == null ? 0 : sensor.getVoltage();
        sinceRefresh.reset();
    }

    /** Battery voltage in volts, refreshed at most every REFRESH_INTERVAL_SECONDS. 0 = no sensor. */
    public double getVoltage() {
        if (sensor != null && sinceRefresh.seconds() >= REFRESH_INTERVAL_SECONDS) {
            voltage = sensor.getVoltage();
            sinceRefresh.reset();
        }
        return voltage;
    }

    /**
     * Scales a motor power command so it produces the same output at the current battery
     * voltage as it would at NOMINAL_VOLTAGE. The result is not clamped, so it can exceed
     * [-1, 1] at low voltage. The caller is responsible for clamping before setPower().
     */
    public double compensate(double power) {
        double currentVoltage = getVoltage();
        if (currentVoltage <= 0) {
            // No usable reading (sensor missing or not yet reporting). Pass power through unchanged.
            return power;
        }
        return power * NOMINAL_VOLTAGE / currentVoltage;
    }
}
