#!/usr/bin/env python3
"""Collect planned arm and end-effector data during teleoperation."""

import argparse
import csv
import datetime
import json
import os
import sys
import time

try:
    from carm import CArmDualBot, CArmSingleCol
except ModuleNotFoundError:
    sdk_python = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "arm_control_sdk", "carm_python")
    )
    if not os.path.isdir(os.path.join(sdk_python, "carm")):
        raise
    sys.path.insert(0, sdk_python)
    from carm import CArmDualBot, CArmSingleCol


DATA_FIELDS = (
    "plan_joint_pos",
    "plan_joint_vel",
    "plan_joint_tau",
    "plan_eeff_pos",
    "plan_eeff_vel",
    "plan_eeff_tau",
)


def default_output_path():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return "collect_data_{}.csv".format(timestamp)


def collect_plan_data(arm, method_prefix=""):
    """Read one complete planned-data sample from one SDK arm view."""
    method_name = lambda field: method_prefix + field if method_prefix else "get_" + field
    return {
        field: getattr(arm, method_name(field))()
        for field in DATA_FIELDS
    }


def build_parser():
    parser = argparse.ArgumentParser(
        description="采集遥操过程中的机械臂和末端计划值"
    )
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--single-a3", action="store_true", help="六轴单臂")
    modes.add_argument("--dual-a3", action="store_true", help="六轴双臂")
    modes.add_argument("--dual-d3", action="store_true", help="七轴人形双臂")

    parser.add_argument("--ip", help="单臂或七轴双臂的控制器 IP")
    parser.add_argument("--left-ip", help="六轴双臂左臂控制器 IP")
    parser.add_argument("--right-ip", help="六轴双臂右臂控制器 IP")
    parser.add_argument("--left-index", type=int, default=0, help="七轴双臂左臂索引")
    parser.add_argument("--right-index", type=int, default=1, help="七轴双臂右臂索引")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--output",
        default=default_output_path(),
        help="CSV 输出文件，默认是当前执行目录下带时间戳的文件",
    )
    parser.add_argument(
        "--interval-ms",
        type=float,
        default=100.0,
        help="采集间隔，单位毫秒，默认 100 ms",
    )
    parser.add_argument("--duration", type=float, help="采集时长，单位秒")
    parser.add_argument("--count", type=int, help="采集帧数")
    return parser


def validate_arguments(args, parser):
    if args.interval_ms <= 0:
        parser.error("--interval-ms 采集间隔必须大于 0 毫秒")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration 必须大于 0")
    if args.count is not None and args.count <= 0:
        parser.error("--count 必须大于 0")

    if args.single_a3 or args.dual_d3:
        if not args.ip:
            parser.error("--single-a3 和 --dual-d3 必须指定 --ip")
        if args.left_ip or args.right_ip:
            parser.error("--left-ip/--right-ip 只适用于 --dual-a3")
    elif args.dual_a3:
        if not args.left_ip or not args.right_ip:
            parser.error("--dual-a3 必须同时指定 --left-ip 和 --right-ip")
        if args.ip:
            parser.error("--dual-a3 不使用 --ip")


def create_arms(args):
    """Create SDK objects and return (objects, arm views)."""
    if args.single_a3:
        arm = CArmSingleCol(args.ip, args.port, args.timeout)
        return [arm], [(arm, "single")]

    if args.dual_a3:
        left = CArmSingleCol(args.left_ip, args.port, args.timeout)
        try:
            right = CArmSingleCol(args.right_ip, args.port, args.timeout)
        except Exception:
            left.disconnect()
            raise
        return [left, right], [(left, "left"), (right, "right")]

    dual = CArmDualBot(
        args.ip,
        args.port,
        args.timeout,
        left_index=args.left_index,
        right_index=args.right_index,
    )
    return [dual], [
        (dual, "left", "get_left_"),
        (dual, "right", "get_right_"),
    ]


def validate_arms(views):
    """Require every selected arm view to be connected and type-compatible."""
    checked = set()
    for view in views:
        arm = view[0]
        if id(arm) in checked:
            continue
        checked.add(id(arm))
        if not arm.is_connected():
            raise RuntimeError("机械臂连接失败: {}".format(view[1]))
        if not arm._is_specified_arm():
            raise RuntimeError("机械臂类型校验失败: {}".format(view[1]))


def write_sample(writer, timestamp, label, arm, method_prefix=""):
    sample = collect_plan_data(arm, method_prefix)
    row = {
        "timestamp": "{:.9f}".format(timestamp),
        "arm": label,
    }
    row.update({field: json.dumps(sample[field], ensure_ascii=False, separators=(",", ":"))
                for field in DATA_FIELDS})
    writer.writerow(row)


def run(args):
    objects = []
    frame_count = 0
    interval = args.interval_ms / 1000.0

    try:
        objects, views = create_arms(args)
        validate_arms(views)
        started = time.monotonic()
        next_sample = started

        with open(args.output, "w", newline="", encoding="utf-8") as output_file:
            fieldnames = ["timestamp", "arm"] + list(DATA_FIELDS)
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                now = time.monotonic()
                if args.duration is not None and now - started >= args.duration:
                    break
                if args.count is not None and frame_count >= args.count:
                    break

                if now < next_sample:
                    time.sleep(next_sample - now)
                timestamp = time.time()
                for view in views:
                    write_sample(writer, timestamp, view[1], view[0],
                                 view[2] if len(view) > 2 else "")
                output_file.flush()
                frame_count += 1
                next_sample += interval
                if next_sample < time.monotonic():
                    next_sample = time.monotonic()
    finally:
        for arm in reversed(objects):
            try:
                arm.disconnect()
            except Exception as exc:
                print("断开机械臂失败: {}".format(exc), file=sys.stderr)


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_arguments(args, parser)
    try:
        run(args)
    except KeyboardInterrupt:
        print("已停止采集")
    except Exception as exc:
        print("采集失败: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
