package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Records one CSV row per loop, so after a match you can work out what the robot was doing:
 * why it stopped answering the sticks, what limits how fast it shoots, whether the battery sagged.
 *
 * Every row starts with the same four columns:
 *   t_ms       time since the OpMode was initialised
 *   phase      INIT or RUN
 *   loop_ms    time since the previous row -- a big number means the loop stalled
 *   battery_v  battery voltage
 * then every column put() during that loop, then "events": things that happened once, like a
 * button press or an auto step starting.
 *
 * Use it once per loop, like the dashboard:
 *   recorder.put("intake", intake.getState());   // as many as you like
 *   recorder.event("shooter on");                // optional
 *   recorder.endLoop();                          // writes the row
 *
 * The first row fixes the columns, so put() the same columns every loop, even when nothing
 * changed. A column first put() after the first row is left out of the file.
 *
 * While in INIT, rows are written at most every INIT_ROW_INTERVAL_MS (plus any loop with an
 * event), since a robot can sit in INIT for minutes before a match starts.
 *
 * Files land in /sdcard/FIRST/analysis/ ("adb pull /sdcard/FIRST/analysis/"). A 2:30 match at
 * ~50 loops a second is around 1-2 MB, so clear the folder out now and then.
 * tools/match_report.py turns a file into a summary: stalls, battery, shots, step times.
 */
public class MatchRecorder {

    /** INIT rows are thinned to one per this many ms; RUN writes every loop. */
    public static double INIT_ROW_INTERVAL_MS = 100;

    private final String fileNamePrefix;
    private final Battery battery;
    private final ElapsedTime sinceStart = new ElapsedTime();
    private final Map<String, Object> row = new LinkedHashMap<>();
    private final StringBuilder events = new StringBuilder();

    private CsvLogger csv;          // opened by the first endLoop(), once the columns are known
    private List<String> columns;
    private String phase = "INIT";
    private double lastRowMs = 0;

    public MatchRecorder(String fileNamePrefix, Battery battery) {
        this.fileNamePrefix = fileNamePrefix;
        this.battery = battery;
    }

    /** INIT, RUN... The change is also written as an event. */
    public void setPhase(String phase) {
        if (!phase.equals(this.phase)) {
            event("phase " + phase);
        }
        this.phase = phase;
    }

    public void put(String column, Object value) {
        row.put(column, value);
    }

    /** Something that happened once this loop. Shows in the "events" column. */
    public void event(String text) {
        if (events.length() > 0) {
            events.append(" | ");
        }
        events.append(text);
    }

    /** Writes this loop's row and starts the next. Call once per loop, last. */
    public void endLoop() {
        double nowMs = sinceStart.milliseconds();
        if (columns != null && phase.equals("INIT") && events.length() == 0
                && nowMs - lastRowMs < INIT_ROW_INTERVAL_MS) {
            row.clear();
            return;
        }
        if (columns == null) {
            columns = new ArrayList<>(row.keySet());
            List<String> header = new ArrayList<>();
            header.add("t_ms");
            header.add("phase");
            header.add("loop_ms");
            header.add("battery_v");
            header.addAll(columns);
            header.add("events");
            csv = new CsvLogger(fileNamePrefix, header.toArray(new String[0]));
        }

        Object[] values = new Object[columns.size() + 5];
        values[0] = round(nowMs);
        values[1] = phase;
        values[2] = round(nowMs - lastRowMs);
        values[3] = round(battery.getVoltage());
        for (int i = 0; i < columns.size(); i++) {
            values[4 + i] = clean(row.get(columns.get(i)));
        }
        values[values.length - 1] = clean(events.toString());
        csv.writeRow(values);

        row.clear();
        events.setLength(0);
        lastRowMs = nowMs;
    }

    /** Closes the file. Safe to call more than once. */
    public void close() {
        if (csv != null) {
            csv.close();
        }
    }

    /** Keeps the file readable: short numbers, and no commas or line breaks inside a cell. */
    private static Object clean(Object value) {
        if (value == null) {
            return "";
        }
        if (value instanceof Double || value instanceof Float) {
            return round(((Number) value).doubleValue());
        }
        return value.toString().replace(',', ';').replace('\n', ' ');
    }

    private static double round(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return value;   // Math.round would turn NaN into a believable-looking 0
        }
        return Math.round(value * 1000) / 1000.0;
    }
}
