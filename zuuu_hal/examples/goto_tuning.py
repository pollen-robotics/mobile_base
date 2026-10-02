"""Measure the go-to precision of the mobile base on your floor, and its friction breakaway.

Runs from any computer that reaches the robot, with the Python SDK installed (pip install reachy2-sdk).
Keep at least 1 m of free space around the robot.

    python goto_tuning.py --host <robot ip> odom
    python goto_tuning.py --host <robot ip> bench --pattern square --dist-tol 0.01 --angle-tol 1
    python goto_tuning.py --host <robot ip> breakaway

odom:      prints the odometry pose (what the go-to controls), 5 times per second.
bench:     resets the odometry, then runs a sequence of goto(..., wait=True) commands like an application would.
           Prints, for each goal, how long it took and the error when goto() returned and a few seconds later.
           The odometry is logged at 20 Hz to a CSV file. Change the zuuu_hal goto_* parameters between two runs
           (ros2 param set /zuuu_hal goto_precision_p_xy 12.0) to compare tunings on the same floor.
breakaway: sends small, increasing speed commands along x, y and in rotation, and reports the smallest one that
           actually moves the base: the static friction of your floor and payload, in command units. The go-to
           must command at least that much to start a correction from rest.
"""

import argparse
import csv
import math
import time

from reachy2_sdk import ReachySDK

PATTERNS = {
    # (x, y, theta_deg) in the odometry frame, reset at the start of the run
    "square": [(0.5, 0.0, 90.0), (0.5, 0.5, 180.0), (0.0, 0.5, 270.0), (0.0, 0.0, 360.0)],
    "line": [(0.8, 0.0, 0.0), (0.0, 0.0, 0.0)],
    "side_steps": [(0.0, 0.03, 0.0), (0.0, 0.0, 0.0), (0.03, 0.0, 0.0), (0.0, 0.0, 0.0)],
    "turns": [(0.0, 0.0, 10.0), (0.0, 0.0, 0.0), (0.0, 0.0, 90.0), (0.0, 0.0, 0.0)],
}


def odom(reachy):
    o = reachy.mobile_base.odometry  # x, y in m, theta in degrees, vx, vy in m/s, vtheta in deg/s
    return o["x"], o["y"], o["theta"], o["vx"], o["vy"], o["vtheta"]


def errors(pose, goal):
    return 1000 * math.hypot(pose[0] - goal[0], pose[1] - goal[1]), pose[2] - goal[2]


def cmd_odom(reachy, args):
    while True:
        x, y, th, vx, vy, vth = odom(reachy)
        print(f"x={x:+.4f} m  y={y:+.4f} m  theta={th:+.2f} deg   vx={vx:+.3f} vy={vy:+.3f} vtheta={vth:+.1f}")
        time.sleep(0.2)


def cmd_bench(reachy, args):
    goals = PATTERNS[args.pattern]
    reachy.mobile_base.reset_odometry()
    time.sleep(0.5)
    rows, summary = [], []
    t0 = time.time()
    for i, goal in enumerate(goals):
        t_goal = time.time()
        goto_id = reachy.mobile_base.goto(
            *goal, distance_tolerance=args.dist_tol, angle_tolerance=args.angle_tol, timeout=args.timeout
        )
        returned = None
        while True:
            pose = odom(reachy)
            now = time.time()
            rows.append([round(now - t0, 3), i, *pose])
            if returned is None and reachy.is_goto_finished(goto_id):
                returned = (now - t_goal, pose)
                if not args.settle:
                    break
            if returned is not None and now - t_goal - returned[0] > args.settle:
                break
            time.sleep(0.05)
        d_ret, a_ret = errors(returned[1], goal)
        d_end, a_end = errors(pose, goal)
        summary.append((i, goal, returned[0], d_ret, a_ret, d_end, a_end))

    print(f"\npattern {args.pattern}, tolerances {args.dist_tol * 1000:.0f} mm / {args.angle_tol} deg")
    print(f"{'goal':>4} {'target':>20} {'time':>7} | {'when goto() returned':>22} | {f'{args.settle:.1f} s later':>22}")
    for i, goal, dur, d_ret, a_ret, d_end, a_end in summary:
        target = f"({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.0f})"
        print(f"{i:>4} {target:>20} {dur:6.1f}s | {d_ret:7.1f} mm {a_ret:+7.2f} deg | {d_end:7.1f} mm {a_end:+7.2f} deg")
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "goal", "x", "y", "theta_deg", "vx", "vy", "vtheta_deg_s"])
        w.writerows(rows)
    print(f"odometry log: {args.csv}")


def cmd_breakaway(reachy, args):
    # (name, unit, levels, function building (vx, vy, vtheta_deg_s) from a level, displacement measure)
    axes = [
        ("x", "m/s", [0.01 * k for k in range(1, 16)], lambda v: (v, 0.0, 0.0)),
        ("y", "m/s", [0.01 * k for k in range(1, 16)], lambda v: (0.0, v, 0.0)),
        ("theta", "deg/s", [3.0 * k for k in range(1, 16)], lambda v: (0.0, 0.0, v)),
    ]
    results = {}
    for name, unit, levels, to_cmd in axes:
        found = None
        for k, level in enumerate(levels):
            sign = 1 if k % 2 == 0 else -1  # alternate directions so that the robot stays in place
            start = odom(reachy)
            t_end = time.time() + args.duration
            while time.time() < t_end:
                reachy.mobile_base.set_goal_speed(*to_cmd(sign * level))
                reachy.mobile_base.send_speed_command()
                time.sleep(0.1)
            reachy.mobile_base.set_goal_speed(0.0, 0.0, 0.0)
            reachy.mobile_base.send_speed_command()
            time.sleep(0.5)
            end = odom(reachy)
            moved_mm = 1000 * math.hypot(end[0] - start[0], end[1] - start[1])
            turned_deg = abs(end[2] - start[2])
            print(f"{name:>5} command {sign * level:+6.2f} {unit}: moved {moved_mm:5.1f} mm, turned {turned_deg:5.2f} deg")
            if (name == "theta" and turned_deg > 0.5) or (name != "theta" and moved_mm > 3.0):
                found = level
                break
        results[name] = (found, unit)
    print("\nSmallest command that moves the base from rest:")
    for name, (found, unit) in results.items():
        print(f"  {name:>5}: {found if found is not None else 'not found'} {unit}")
    print("To start a correction of e from rest, the go-to must command at least this much: with the staged go-to,")
    print("goto_precision_p_xy * e (m) or goto_precision_p_theta * e (rad) must reach it, or the breakaway term")
    print("(goto_breakaway_rate_*) will ramp up to it after a short wait.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("odom")
    b = sub.add_parser("bench")
    b.add_argument("--pattern", choices=PATTERNS, default="square")
    b.add_argument("--dist-tol", type=float, default=0.01, help="m (the SDK default is 0.05)")
    b.add_argument("--angle-tol", type=float, default=1.0, help="deg (the SDK default is 5)")
    b.add_argument("--timeout", type=float, default=20.0)
    b.add_argument("--settle", type=float, default=3.0, help="s of logging after goto() returned")
    b.add_argument("--csv", default="goto_bench.csv")
    k = sub.add_parser("breakaway")
    k.add_argument("--duration", type=float, default=1.0, help="s per command level")
    args = ap.parse_args()

    reachy = ReachySDK(host=args.host)
    if reachy.mobile_base is None:
        raise SystemExit("No mobile base found on this robot")
    reachy.mobile_base.turn_on()
    {"odom": cmd_odom, "bench": cmd_bench, "breakaway": cmd_breakaway}[args.cmd](reachy, args)


if __name__ == "__main__":
    main()
