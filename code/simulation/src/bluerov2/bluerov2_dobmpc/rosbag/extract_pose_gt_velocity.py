#!/usr/bin/env python3

import argparse
import csv
import os
import sys
from typing import Optional

try:
    import rosbag
except ImportError as exc:
    sys.stderr.write("Failed to import rosbag. Ensure ROS (e.g., Noetic) is installed and sourced.\n")
    raise


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract vx, vy, vz, yaw (angular.z) from /bluerov2/pose_gt in a ROS bag for the full duration")
    parser.add_argument("bag_path", type=str, help="Absolute path to the .bag file")
    parser.add_argument("--topic", type=str, default="/bluerov2/pose_gt", help="Odometry topic to read (default: /bluerov2/pose_gt)")
    parser.add_argument("--output", type=str, default=None, help="Output CSV path (default: <bag_dir>/all_vlocity.csv)")
    parser.add_argument("--with-time", dest="with_time", action="store_true", help="Include ROS time (secs.nanosecs) as the first column")
    return parser.parse_args()


def ensure_output_path(bag_path: str, output_path: Optional[str]) -> str:
    if output_path is not None:
        return output_path
    bag_dir = os.path.dirname(os.path.abspath(bag_path))
    return os.path.join(bag_dir, "all_vlocity.csv")


def extract_velocities(bag_path: str, topic: str, output_csv_path: str, include_time: bool) -> None:
    total_messages = 0
    written_messages = 0
    max_vx = float("-inf")
    max_vy = float("-inf")
    max_vz = float("-inf")
    max_yaw = float("-inf")
    with rosbag.Bag(bag_path, "r") as bag, open(output_csv_path, "w", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)
        if include_time:
            csv_writer.writerow(["time", "vx", "vy", "vz", "yaw"])  # yaw == angular.z
        else:
            csv_writer.writerow(["vx", "vy", "vz", "yaw"])  # yaw == angular.z

        for _, message, timestamp in bag.read_messages(topics=[topic]):
            total_messages += 1
            try:
                vx = float(message.twist.twist.linear.x)
                vy = float(message.twist.twist.linear.y)
                vz = float(message.twist.twist.linear.z)
                yaw_rate = float(message.twist.twist.angular.z)
                if include_time:
                    csv_writer.writerow([f"{timestamp.secs}.{timestamp.nsecs:09d}", vx, vy, vz, yaw_rate])
                else:
                    csv_writer.writerow([vx, vy, vz, yaw_rate])
                written_messages += 1
                if vx > max_vx:
                    max_vx = vx
                if vy > max_vy:
                    max_vy = vy
                if vz > max_vz:
                    max_vz = vz
                if yaw_rate > max_yaw:
                    max_yaw = yaw_rate
            except Exception as exc:
                # Skip malformed messages but continue processing
                continue

    sys.stdout.write(f"Wrote {written_messages} rows to {output_csv_path} (from {total_messages} messages).\n")
    sys.stdout.write(f"max_vx={max_vx}, max_vy={max_vy}, max_vz={max_vz}, max_yaw={max_yaw}\n")


def main() -> None:
    args = parse_arguments()
    output_csv_path = ensure_output_path(args.bag_path, args.output)
    extract_velocities(args.bag_path, args.topic, output_csv_path, args.with_time)


if __name__ == "__main__":
    main() 