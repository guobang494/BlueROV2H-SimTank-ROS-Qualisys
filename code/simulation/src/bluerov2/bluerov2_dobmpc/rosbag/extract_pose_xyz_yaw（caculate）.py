#!/usr/bin/env python3
import os
import sys
import argparse
import csv
import math

try:
    import rosbag  # type: ignore
except Exception as e:
    print("[ERROR] Failed to import rosbag. Please source your ROS environment (e.g., 'source /opt/ros/noetic/setup.bash').")
    raise


def parse_args():
    parser = argparse.ArgumentParser(description="从 rosbag 提取 pose_gt 的 x, y, z, yaw 随时间变化到 CSV。")
    parser.add_argument('bag_path', help='rosbag 文件的绝对路径（.bag）')
    parser.add_argument('--pose-topic', default='/bluerov2/pose_gt', help='Odometry 话题（默认：/bluerov2/pose_gt）')
    parser.add_argument('--out-dir', default=None, help='输出目录（默认：与 bag 同目录）')
    return parser.parse_args()


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # yaw (Z) from quaternion
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def read_pose_xyz_yaw(bag_path: str, pose_topic: str):
    samples = []
    with rosbag.Bag(bag_path, 'r') as bag:
        type_info = bag.get_type_and_topic_info()[1]
        available = set(type_info.keys())
        if pose_topic not in available:
            print(f"[ERROR] 话题未在 bag 中找到：{pose_topic}")
            return []
        for topic, msg, t in bag.read_messages(topics=[pose_topic]):
            try:
                ts = msg.header.stamp.to_sec()
            except Exception:
                ts = t.to_sec()
            try:
                x = float(msg.pose.pose.position.x)
                y = float(msg.pose.pose.position.y)
                z = float(msg.pose.pose.position.z)
                qx = float(msg.pose.pose.orientation.x)
                qy = float(msg.pose.pose.orientation.y)
                qz = float(msg.pose.pose.orientation.z)
                qw = float(msg.pose.pose.orientation.w)
                yaw = quat_to_yaw(qx, qy, qz, qw)
            except Exception:
                continue
            samples.append((ts, x, y, z, yaw))
    samples.sort(key=lambda s: s[0])
    return samples


def write_csv(out_path: str, samples):
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time_sec', 'x', 'y', 'z', 'yaw_rad'])
        for ts, x, y, z, yaw in samples:
            w.writerow([f"{ts:.6f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{yaw:.6f}"])
    print(f"[OK] Wrote CSV: {out_path}")


def main():
    args = parse_args()
    bag_path = os.path.abspath(args.bag_path)
    if not os.path.isfile(bag_path):
        print(f"[ERROR] Bag not found: {bag_path}")
        sys.exit(1)

    out_dir = args.out_dir or os.path.dirname(bag_path)
    os.makedirs(out_dir, exist_ok=True)

    print('[INFO] Reading bag for pose_gt pose...')
    samples = read_pose_xyz_yaw(bag_path, args.pose_topic)
    if not samples:
        print('[WARN] 未读取到任何位姿样本。')
    out_csv = os.path.join(out_dir, 'pose_gt_xyz_yaw.csv')
    write_csv(out_csv, samples)
    print('[DONE] Extraction complete.')


if __name__ == '__main__':
    main() 