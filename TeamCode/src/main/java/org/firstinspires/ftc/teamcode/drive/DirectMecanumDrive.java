package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.MatchRecorder;

/**
 * BACKUP drive: sets the four motor powers directly, like the other team's MecanumDrive.
 * Uses nothing from Pedro and not the Pinpoint, so it still drives when those are broken.
 *
 * !! Fill in the names, directions and hub orientation below. They are NOT read from
 * pedro/Constants.java on purpose -- a broken Constants must not break the backup too. Copy
 * the motor names/directions from the drivetrain block in Constants once, by hand.
 */
@Configurable
public class DirectMecanumDrive implements DriveTrain {

    // ---- Must match the robot config (read once at init) ---------------------
    public static String FRONT_LEFT = "frontLeft";
    public static String FRONT_RIGHT = "frontRight";
    public static String BACK_LEFT = "backLeft";
    public static String BACK_RIGHT = "backRight";
    public static DcMotorSimple.Direction FRONT_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction BACK_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

    /** How the Control Hub is mounted. Wrong values = field-centric drives the wrong way. */
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---- Start-up behaviour ---------------------------------------------------
    public static boolean START_IN_BRAKE_MODE = true;
    /**
     * Velocity mode needs an encoder cable on EVERY drive motor. With no encoder the hub thinks
     * the wheel is stopped and drives it at full power. Leave false unless they are plugged in.
     */
    public static boolean START_IN_VELOCITY_MODE = false;

    // Hub velocity PIDF, used only in velocity mode. Live-editable when TUNE_PIDF is true.
    public static boolean TUNE_PIDF = false;
    public static double VEL_P = 10.0;
    public static double VEL_I = 3.0;
    public static double VEL_D = 0.0;
    public static double VEL_F = 12.0;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DcMotorEx[] motors;
    private final IMU imu;

    private boolean brakeMode = START_IN_BRAKE_MODE;
    private boolean velocityMode = START_IN_VELOCITY_MODE;
    private double heading;
    private boolean locked;
    private double motorP = Double.NaN, motorI = Double.NaN, motorD = Double.NaN, motorF = Double.NaN;

    public DirectMecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);
        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        frontLeft.setDirection(FRONT_LEFT_DIR);
        frontRight.setDirection(FRONT_RIGHT_DIR);
        backLeft.setDirection(BACK_LEFT_DIR);
        backRight.setDirection(BACK_RIGHT_DIR);
        applyRunMode();
        applyZeroPowerBehavior();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();
    }

    @Override
    public void drive(double forward, double left, double turn) {
        if (locked) {
            locked = false;
            applyZeroPowerBehavior();   // back to the driver's BRAKE/COAST choice
        }
        setPowers(forward, left, turn);
    }

    private void setPowers(double forward, double left, double turn) {
        // Same mixing as Pedro's mecanum, so both drives feel the same.
        double fl = forward - left - turn;
        double fr = forward + left + turn;
        double bl = forward + left - turn;
        double br = forward - left + turn;

        // Scale all four down together instead of clipping, so diagonals keep their direction.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /** No odometry here, so "lock" just stops hard with BRAKE. It won't push back if hit. */
    @Override
    public void lock(boolean justPressed) {
        if (justPressed) {
            locked = true;
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        setPowers(0, 0, 0);
    }

    @Override
    public double heading() {
        return heading;
    }

    @Override
    public void update() {
        // One IMU read per loop -- it is a real I2C read, so don't ask twice.
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        applyPidfTuning();
    }

    @Override
    public void toggleBrakeMode() {
        brakeMode = !brakeMode;
        applyZeroPowerBehavior();
    }

    @Override
    public void toggleVelocityMode() {
        velocityMode = !velocityMode;
        applyRunMode();
    }

    @Override
    public void stop() {
        setPowers(0, 0, 0);
    }

    @Override
    public void report(Dashboard dashboard) {
        dashboard.addData("Stopping", brakeMode ? "BRAKE" : "COAST");
        dashboard.addData("Motor mode", velocityMode ? "VELOCITY" : "RAW POWER");
        dashboard.addData("Heading (IMU deg)", Math.round(Math.toDegrees(heading)));
        dashboard.addLine("No odometry in backup mode -- position not tracked.");
    }

    @Override
    public void record(MatchRecorder recorder) {
        recorder.put("heading_deg", Math.toDegrees(heading));
        recorder.put("brake_mode", brakeMode);
        recorder.put("velocity_mode", velocityMode);
        recorder.put("locked", locked);
    }

    @Override
    public String name() {
        return "DIRECT (backup)";
    }

    private void applyPidfTuning() {
        if (!TUNE_PIDF || (VEL_P == motorP && VEL_I == motorI && VEL_D == motorD && VEL_F == motorF)) {
            return;
        }
        for (DcMotorEx motor : motors) {
            motor.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
        }
        motorP = VEL_P;
        motorI = VEL_I;
        motorD = VEL_D;
        motorF = VEL_F;
    }

    private void applyRunMode() {
        for (DcMotorEx motor : motors) {
            motor.setMode(velocityMode
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void applyZeroPowerBehavior() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(brakeMode
                    ? DcMotor.ZeroPowerBehavior.BRAKE
                    : DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
