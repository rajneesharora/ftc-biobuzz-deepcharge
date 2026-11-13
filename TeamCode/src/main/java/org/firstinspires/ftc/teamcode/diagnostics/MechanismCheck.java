package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.FlowerGate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

/**
 * Tests each mechanism on its own. The drivetrain never moves. Use it after any wiring or
 * config change, to check every device is found and spins the right way.
 *
 *   Y (hold)   intake in          A (hold)   intake out
 *   X (hold)   transport feed     B (hold)   transport reverse
 *   D-pad Up   flywheel on/off    (watch actual vs target RPM)
 *   right bumper (hold)  spin the shooter gate (CR servo) -- check it feeds INTO the wheel
 *   left bumper (hold)   flower gate to ROTATED_POSITION (down); let go = HOME_POSITION (up)
 *
 * Shooter: check shooterTop alone first (Shooter.BOTTOM_ENABLED = false). Once the flywheels
 * spin the shooting way and actual RPM follows the target, set BOTTOM_ENABLED = true and
 * listen -- any buzzing or grinding means the two motors are fighting: turn it off at once.
 *
 * Anything showing MISSING has a name that doesn't match the robot config. The INIT screen
 * also lists every name in the active config by type, so you can compare them directly.
 */
@TeleOp(name = "Mechanism Check", group = "Diagnostics")
public class MechanismCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Dashboard dashboard = new Dashboard(telemetry);
        Intake intake = new Intake(hardwareMap);
        Transport transport = new Transport(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        FlowerGate flowerGate = new FlowerGate(hardwareMap);

        // What the ACTIVE robot config actually contains, by type. A transport servo must be
        // and the gate must be under CR servos, with the exact names.
        String motors = String.valueOf(hardwareMap.getAllNames(DcMotor.class));
        String crServos = String.valueOf(hardwareMap.getAllNames(CRServo.class));
        String servos = String.valueOf(hardwareMap.getAllNames(Servo.class));

        while (opModeInInit()) {
            dashboard.addLine("Mechanism Check -- drivetrain stays still. Press PLAY.");
            dashboard.section("ACTIVE CONFIG");
            dashboard.addData("Motors", motors);
            dashboard.addData("CR servos (transport, gate)", crServos);
            dashboard.addData("Servos (positional)", servos);
            intake.report(dashboard);
            transport.report(dashboard);
            shooter.report(dashboard);
            flowerGate.report(dashboard);
            dashboard.update();
        }

        boolean flywheelOn = false;
        boolean dpadWasDown = false;

        while (opModeIsActive()) {
            intake.setState(gamepad1.y ? Intake.State.INTAKE
                    : gamepad1.a ? Intake.State.EJECT : Intake.State.OFF);
            transport.setState(gamepad1.x ? Transport.State.FEED
                    : gamepad1.b ? Transport.State.REVERSE : Transport.State.OFF);

            if (gamepad1.dpad_up && !dpadWasDown) {
                flywheelOn = !flywheelOn;
            }
            dpadWasDown = gamepad1.dpad_up;
            shooter.setSpinning(flywheelOn);
            shooter.setGateTestOpen(gamepad1.right_bumper);
            flowerGate.setRotating(gamepad1.left_bumper);

            intake.update();
            transport.update();
            shooter.update();
            flowerGate.update();

            dashboard.addLine("Y/A intake   X/B transport   D-pad Up flywheel   RB gate   LB flower");
            intake.report(dashboard);
            transport.report(dashboard);
            shooter.report(dashboard);
            flowerGate.report(dashboard);
            dashboard.update();
        }

        intake.stop();
        transport.stop();
        shooter.stop();
        flowerGate.stop();
    }
}
