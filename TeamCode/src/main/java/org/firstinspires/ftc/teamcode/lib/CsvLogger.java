package org.firstinspires.ftc.teamcode.lib;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Appends CSV rows to a file under /sdcard/FIRST/analysis/ on the Control Hub, so data can be
 * pulled off after a match (adb pull /sdcard/FIRST/analysis/) and opened in a spreadsheet --
 * unlike telemetry, which is gone the moment the OpMode stops.
 *
 * One file per run (timestamped name). Every row is flushed straight away, so the rows written
 * before a crash or a frozen loop are still in the file.
 *
 * Never crashes the OpMode: if the file can't be opened or written, logging just stops and the
 * reason goes to the robot log (adb logcat | grep CsvLogger).
 */
public class CsvLogger {
    private static final String TAG = "CsvLogger";
    private static final File DIRECTORY = new File(Environment.getExternalStorageDirectory(), "FIRST/analysis");

    private FileWriter writer;   // null once logging has stopped

    public CsvLogger(String fileNamePrefix, String... headerColumns) {
        File file = new File(DIRECTORY, fileNamePrefix + "_" + System.currentTimeMillis() + ".csv");
        try {
            DIRECTORY.mkdirs();
            writer = new FileWriter(file, false);
        } catch (IOException e) {
            RobotLog.ee(TAG, "Could not create %s: %s", file, e.getMessage());
            writer = null;
        }
        writeRow((Object[]) headerColumns);
    }

    /** False if the file couldn't be opened, or a write failed and logging stopped. */
    public boolean isLogging() {
        return writer != null;
    }

    public void writeRow(Object... values) {
        if (writer == null) {
            return;
        }
        StringBuilder line = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                line.append(',');
            }
            line.append(values[i]);
        }
        line.append('\n');
        try {
            writer.write(line.toString());
            writer.flush();
        } catch (IOException e) {
            RobotLog.ee(TAG, "Write failed, logging stopped: %s", e.getMessage());
            close();
        }
    }

    public void close() {
        if (writer == null) {
            return;
        }
        try {
            writer.close();
        } catch (IOException e) {
            RobotLog.ee(TAG, "Close failed: %s", e.getMessage());
        }
        writer = null;
    }
}
