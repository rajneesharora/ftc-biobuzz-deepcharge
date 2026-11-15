# BioBuzz TeleOp drop-in (Pedro Pathing 3.x)

Copy these files into your existing Pedro 3 project (the one where `testpedro` already runs).
Your `pedro/Constants.java` is **not** included. Keep yours: everything here calls
`Constants.create(hardwareMap)`.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
  teleop/BioBuzzTeleOpBase.java   <- shared controls + mechanisms (not on the DS menu)
  teleop/BioBuzzTeleOp.java       <- "BioBuzz TeleOp"          drives through Pedro (normal)
  teleop/BioBuzzTeleOpBackup.java <- "BioBuzz TeleOp (Backup)" direct motors + Control Hub IMU
  teleop/DriverSettings.java      <- speeds/deadzone, live-editable in Panels
  drive/DriveTrain.java           <- interface both drives implement
  drive/PedroDrive.java, drive/DirectMecanumDrive.java
  subsystems/Subsystem.java       <- interface every mechanism implements
  subsystems/Intake.java          <- PLACEHOLDER (ENABLED = false)
  subsystems/Transport.java       <- PLACEHOLDER (ENABLED = false)
  subsystems/Shooter.java         <- PLACEHOLDER (ENABLED = false)
  lib/Button.java                 <- edge detection + opt-in toggles
  lib/Dashboard.java              <- Panels telemetry + field drawing + browser gamepad
  lib/PoseStorage.java            <- hands the end-of-auto pose to teleop
  lib/VelocityMotor.java          <- RPM-controlled motor for flywheel/intake
  diagnostics/LocalizationCheck.java  <- push-by-hand odometry check, motors off
  pedro/testpedro.java            <- your auto, with the sequencing bug fixed (replaces yours)
build.dependencies.gradle         <- reference; compare with yours
```

## Gradle
1. `build.dependencies.gradle`: make sure you have `revhub:3.0.1`, `tuning:1.0.1`, Ivy, and
   `com.bylazar:fullpanels` plus the `mymaven.bylazar.com` repository. If your Ivy lines
   already work, keep them.
2. `build.common.gradle`: `compileSdk 34` (the Pedro 3 Quickstart already does this).
3. Sync Gradle, then `./gradlew :TeamCode:assembleDebug`.

## Which TeleOp runs?
The Driver Station lists every class with `@TeleOp(name=...)` in its TeleOp menu, and the
driver picks one. The base class has no annotation, so it never appears. Add `@Disabled`
to hide one.

TeleOp and auto are independent: the auto doesn't reference TeleOp, and TeleOp starts from
its own fixed pose (`PedroDrive.START_X/Y/HEADING_DEG`). When the auto is finished, set
`PedroDrive.USE_AUTO_POSE = true` so TeleOp continues from where auto ended. Optionally add
`preselectTeleOp = "BioBuzz TeleOp"` to the auto's `@Autonomous` too.

All files in TeamCode build together, so a compile error in a half-written auto blocks
deploying TeleOp too. Keep the auto compiling, or comment out the broken part before
deploying.

Before first use of the backup: set motor names, directions and `HUB_LOGO`/`HUB_USB` in
`DirectMecanumDrive`. Leave `START_IN_VELOCITY_MODE = false` unless every drive motor has an
encoder cable. Without one, velocity mode runs that wheel at full power.

## Panels
* Robot Wi-Fi: http://192.168.43.1:8001
* USB: `adb forward tcp:8001 tcp:8001 && adb forward tcp:8002 tcp:8002`, then http://localhost:8001
  (re-run after every install; 8002 is the live-data socket).
* **Configurables** tab: every `public static` field on a `@Configurable` class (speeds,
  deadzone, shooter RPM/PIDF...) can be edited while the OpMode runs. **Edits are not saved.**
  Copy the values you like back into the source.

## Adding a real mechanism later
1. Add the device to the robot config on the Driver Station, using the name in the
   subsystem (`intake`, `transport`, `shooter`, `shooterGate`) or change `HARDWARE_NAME`.
2. Set `ENABLED = true` in that subsystem and pick the direction.
3. If the name is wrong, the dashboard shows `MISSING '<name>'`. It won't crash.

After a config change, pull a copy into git so the repo matches the robot:
`adb pull /sdcard/FIRST/<YourConfig>.xml robot-config/`

## Bring-up order on the chassis
1. **Localization Check**: push by hand and confirm x/y/heading directions. Push exactly 48".
2. **BioBuzz TeleOp**: confirm field-centric drive. A re-zeros "forward", B locks in place.
3. **Pedro Long Straight 180 Auto**: test on its own. TeleOp doesn't need it.

## Match recordings
Every TeleOp and auto writes one CSV row per loop to `/sdcard/FIRST/analysis/` on the robot
(`lib/MatchRecorder`): loop time, battery, both gamepads, drive command, pose, auto step, and
every mechanism (including flywheel target/actual RPM and the gate). Nothing to switch on.

After a practice or a match, with the laptop on robot Wi-Fi or USB:
```
adb pull /sdcard/FIRST/analysis/ recordings/
python3 tools/match_report.py recordings/
```
The report gives, per file: loop stalls, battery sag, how long each auto step took, flywheel
spin-up time, and per volley the shots, RPM dip, recovery and gap between shots, with a verdict
on whether the flywheel or the feed limits the shot rate. It also flags signs the driver lost
control (gamepad dropping out, the Panels browser gamepad overriding the sticks, a recording that
ends mid-match). Each match is about 1-2 MB; clear the folder now and then.
