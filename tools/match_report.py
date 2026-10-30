#!/usr/bin/env python3
"""Summarise MatchRecorder CSV files (TeleOp and autos) into numbers worth acting on.

Get the files off the robot first, over USB or robot Wi-Fi:
    adb pull /sdcard/FIRST/analysis/ recordings/

Then:
    python3 tools/match_report.py recordings/              # every CSV in the folder
    python3 tools/match_report.py recordings/teleop_pedro_1730000000000.csv

For each file it reports:
  Loop      -- loop time while running, and every stall (motors keep their last power during one)
  Battery   -- voltage at the start, lowest point, and end
  Auto      -- how long each step really took
  Shooter   -- spin-up time, and per volley: shots, dip, recovery time and gap between shots,
               with a hint at whether the flywheel or the feed is what limits the shot rate
  Inputs    -- signs the driver lost control: gamepad id changes, browser gamepad overriding the
               real one, sticks pushed with no drive command, crashes

Standard library only. Shots are found from flywheel RPM dips, so tune --dip-rpm if it
miscounts (a real shot dips well below target; noise doesn't).
"""

import argparse
import csv
import statistics
import sys
from pathlib import Path


SHOT_WINDOW_S = 0.15     # a shot's RPM drop happens within this long
MIN_SHOT_GAP_S = 0.15    # two drops closer than this are the same shot
VOLLEY_BREAK_S = 3.0     # this long without a shot starts a new volley


def num(value):
    """A CSV cell as a float, or None if it's empty / not a number."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return None if result != result else result   # NaN -> None


def flag(value):
    return str(value).strip().lower() == "true"


def pct(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100 * (len(ordered) - 1))))]


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def seconds(row):
    return num(row["t_ms"]) / 1000.0


def section(title):
    print(f"\n  {title}")


def report_loop(rows, stall_ms):
    running = [r for r in rows if r["phase"] == "RUN"]
    if len(running) < 2:
        section("Loop: no RUN rows (OpMode never started?)")
        return
    loops = [num(r["loop_ms"]) for r in running[1:] if num(r["loop_ms"]) is not None]
    duration = seconds(running[-1]) - seconds(running[0])
    section("Loop")
    print(f"    ran {duration:.1f} s, {len(running)} loops ({len(running) / max(duration, 1e-6):.0f}/s)")
    print(f"    loop ms: median {statistics.median(loops):.1f}, p95 {pct(loops, 95):.1f}, max {max(loops):.1f}")
    stalls = [r for r in running[1:] if (num(r["loop_ms"]) or 0) >= stall_ms]
    if stalls:
        print(f"    STALLS >= {stall_ms:.0f} ms: {len(stalls)}")
        for r in stalls[:10]:
            print(f"      at {seconds(r):7.2f} s: {num(r['loop_ms']):.0f} ms  {r.get('step') or r.get('events') or ''}")
    # A recording that just stops mid-RUN is what a frozen loop or a dead app looks like.
    if rows[-1]["phase"] == "RUN":
        print("    FILE ENDS IN RUN -- no STOP row: the OpMode froze, crashed hard, or lost power")


def report_battery(rows):
    volts = [(seconds(r), num(r["battery_v"])) for r in rows if num(r["battery_v"])]
    if not volts:
        return
    low_t, low_v = min(volts, key=lambda tv: tv[1])
    section("Battery")
    print(f"    start {volts[0][1]:.2f} V, lowest {low_v:.2f} V at {low_t:.1f} s, end {volts[-1][1]:.2f} V")
    if volts[0][1] < 12.8:
        print("    started below 12.8 V -- swap to a fresher battery before matches")


def report_steps(rows):
    starts = []
    for r in rows:
        for event in (r.get("events") or "").split(" | "):
            if event.startswith("step: "):
                starts.append((seconds(r), event[len("step: "):]))
    if not starts:
        return
    end = seconds(rows[-1])
    run_start = next((seconds(r) for r in rows if r["phase"] == "RUN"), starts[0][0])
    section(f"Auto steps (from PLAY; whole auto {end - run_start:.1f} s)")
    for i, (t, name) in enumerate(starts):
        until = starts[i + 1][0] if i + 1 < len(starts) else end
        print(f"    {t - run_start:6.2f} s  {until - t:5.2f} s  {name}")


def report_shooter(rows, dip_rpm, tolerance_rpm):
    if "shooter_rpm" not in rows[0]:
        return
    samples = [(seconds(r), num(r["shooter_rpm"]), num(r["shooter_target_rpm"]),
                flag(r["shooter_spinning"]), flag(r.get("gate_open")))
               for r in rows if num(r["shooter_rpm"]) is not None]
    if not any(s[3] for s in samples):
        return
    section("Shooter")

    # Spin-up: spinning switches on -> first reading within tolerance of target.
    was_spinning = False
    spin_start = None
    for t, rpm, target, spinning, _ in samples:
        if spinning and not was_spinning:
            spin_start = t
        if spin_start is not None and target and abs(rpm - target) < tolerance_rpm:
            print(f"    spin-up at {spin_start:6.2f} s took {t - spin_start:.2f} s to reach {target:.0f} RPM")
            spin_start = None
        was_spinning = spinning

    # Shots: a sharp drop of at least dip_rpm within SHOT_WINDOW_S, once the wheel has reached
    # speed since it was switched on (so spin-up isn't a shot). A shot has recovered when the
    # wheel is back within tolerance. If the next ball hits first, that shot never recovered --
    # the sign the flywheel, not the feed, limits the shot rate.
    shots = []          # [time, lowest rpm, recovery seconds or None, target]
    current = None
    armed = False
    window = []
    for t, rpm, target, spinning, _ in samples:
        if not spinning or not target:
            if current:
                shots.append(current)
                current = None
            armed, window = False, []
            continue
        if not armed:
            armed = abs(rpm - target) < tolerance_rpm
            window = [(t, rpm)]
            continue
        window = [(wt, wr) for wt, wr in window if t - wt <= SHOT_WINDOW_S]
        peak = max((wr for _, wr in window), default=rpm)
        window.append((t, rpm))
        last_shot_t = current[0] if current else (shots[-1][0] if shots else -1e9)
        if peak - rpm >= dip_rpm and t - last_shot_t >= MIN_SHOT_GAP_S:
            if current:
                shots.append(current)            # never recovered before this ball
            current = [t, rpm, None, target]
            window = [(t, rpm)]
            continue
        if current:
            current[1] = min(current[1], rpm)
            if abs(rpm - target) < tolerance_rpm:
                current[2] = t - current[0]
                shots.append(current)
                current = None
    if current:
        shots.append(current)
    if not shots:
        print(f"    no shots found (no drop of {dip_rpm:.0f} RPM within {SHOT_WINDOW_S} s) -- try a smaller --dip-rpm")
        return

    volleys, group = [], [shots[0]]
    for shot in shots[1:]:
        if shot[0] - group[-1][0] > VOLLEY_BREAK_S:
            volleys.append(group)
            group = []
        group.append(shot)
    volleys.append(group)

    for n, volley in enumerate(volleys, 1):
        recoveries = [s[2] for s in volley if s[2] is not None]
        unrecovered = sum(1 for s in volley[:-1] if s[2] is None)
        gaps = [b[0] - a[0] for a, b in zip(volley, volley[1:])]
        line = (f"    volley {n} at {volley[0][0]:6.2f} s: {len(volley)} shots in "
                f"{volley[-1][0] - volley[0][0]:.2f} s, lowest {min(s[1] for s in volley):.0f} RPM "
                f"(target {volley[0][3]:.0f})")
        if gaps:
            line += f", gap avg {statistics.mean(gaps):.2f} s"
        if recoveries:
            line += f", recovery avg {statistics.mean(recoveries):.2f} s"
        print(line)
        if unrecovered:
            print(f"      {unrecovered} of {len(volley) - 1} shots hit before the wheel recovered: "
                  f"FLYWHEEL-limited -- raise VEL_P, check VEL_F, or space the balls out")
        elif gaps and recoveries:
            spare = statistics.mean(gaps) - statistics.mean(recoveries)
            if spare > 0.25:
                print(f"      wheel was ready {spare:.2f} s before the next ball: FEED-limited "
                      f"(gate/transport speed, or the operator's pace)")
            else:
                print("      next ball arrived just as the wheel recovered: balanced")
        if volley[-1][2] is None:
            print("      last shot never recovered (wheel switched off or recording ended first)")


def report_inputs(rows):
    notes = []
    for r in rows:
        for event in (r.get("events") or "").split(" | "):
            if event.startswith(("gamepad", "CRASH")):
                notes.append(f"{seconds(r):7.2f} s  {event}")
    if "raw_lx" in rows[0]:
        overridden = stuck = 0
        for r in rows:
            if r["phase"] != "RUN":
                continue
            raw = [num(r[k]) or 0 for k in ("raw_lx", "raw_ly", "raw_rx")]
            merged = [num(r[k]) or 0 for k in ("merged_lx", "merged_ly", "merged_rx")]
            cmd = [num(r[k]) or 0 for k in ("cmd_forward", "cmd_left", "cmd_turn")]
            if any(abs(a - b) > 0.05 for a, b in zip(raw, merged)):
                overridden += 1
            if max(abs(v) for v in raw) > 0.3 and max(abs(v) for v in cmd) < 0.01 and r.get("drive_mode") != "LOCKED":
                stuck += 1
        if overridden:
            notes.append(f"{overridden} loops where the Panels browser gamepad overrode the driver's sticks")
        if stuck:
            notes.append(f"{stuck} loops with a stick pushed but no drive command (not locked)")
    if notes:
        section("Inputs / errors")
        for note in notes:
            print(f"    {note}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+", type=Path, help="CSV files or folders of them")
    parser.add_argument("--stall-ms", type=float, default=100, help="loop time that counts as a stall (default 100)")
    parser.add_argument("--dip-rpm", type=float, default=150, help="RPM below target that counts as a shot (default 150)")
    parser.add_argument("--tolerance-rpm", type=float, default=100,
                        help="back within this of target = recovered; match VelocityMotor.RPM_TOLERANCE (default 100)")
    args = parser.parse_args()

    files = []
    for path in args.paths:
        files.extend(sorted(path.glob("*.csv")) if path.is_dir() else [path])
    if not files:
        sys.exit("no CSV files found")

    for path in files:
        rows = load(path)
        print(f"\n=== {path.name}  ({len(rows)} rows)")
        if not rows:
            continue
        report_loop(rows, args.stall_ms)
        report_battery(rows)
        report_steps(rows)
        report_shooter(rows, args.dip_rpm, args.tolerance_rpm)
        report_inputs(rows)


if __name__ == "__main__":
    main()
