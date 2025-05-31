#!/usr/bin/env python

import rospy
import tf2_ros
import os
import pandas as pd
from geometry_msgs.msg import TransformStamped
import math

class TransformCollector:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot", 'default_robot')
        self.parent_frame = "map"
        self.child_frames = [f"{self.robot_name}_tf/base_footprint"]

        # Speicherpfad vorbereiten
        base_directory       = rospy.get_param('~log_base_directory', os.path.expanduser('~/.ros/logs'))
        if not os.path.exists(base_directory):
            os.makedirs(base_directory)

        self.csv_path = os.path.join(base_directory, f"{self.robot_name}_transform_data.csv")

        # Spaltennamen definieren
        self.columns = ["timestamp"]
        for child in self.child_frames:
            prefix = child.split("/")[-1]
            self.columns += [
                f"{prefix}_x", f"{prefix}_y", f"{prefix}_z", f"{prefix}_roll", f"{prefix}_pitch", f"{prefix}_yaw"
            ]

        # CSV-Datei vorbereiten
        if not os.path.exists(self.csv_path):
            df = pd.DataFrame(columns=self.columns)
            df.to_csv(self.csv_path, index=False)

        # TF2 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10)

    def get_rpy(self, quat):

        # Extract values
        x, y, z, w = quat.x, quat.y, quat.z, quat.w

        # Roll (x-Achse)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-Achse)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Gimbal lock
        else:
            pitch = math.asin(sinp)

        # Yaw (z-Achse)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def run(self):
        while not rospy.is_shutdown():
            timestamp = rospy.Time.now().to_sec()
            row = {"timestamp": timestamp}

            for child in self.child_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(child, self.parent_frame,rospy.Time(0), rospy.Duration(1.0))
                    t = transform.transform.translation
                    r = transform.transform.rotation
                    roll, pitch, yaw = self.get_rpy(r)
                    prefix = child.split("/")[-1]
                    row[f"{prefix}_x"] = t.x
                    row[f"{prefix}_y"] = t.y
                    row[f"{prefix}_z"] = t.z
                    row[f"{prefix}_roll"] = roll
                    row[f"{prefix}_pitch"] = pitch
                    row[f"{prefix}_yaw"] = yaw
                except Exception as e:
                    rospy.logwarn(f"Transform {child} -> {self.parent_frame} fehlgeschlagen: {e}")
                    prefix = child.split("/")[-1]
                    row[f"{prefix}_x"] = None
                    row[f"{prefix}_y"] = None
                    row[f"{prefix}_z"] = None
                    row[f"{prefix}_roll"] = None
                    row[f"{prefix}_pitch"] = None
                    row[f"{prefix}_yaw"] = None

            pd.DataFrame([row]).to_csv(self.csv_path, mode='a', header=False, index=False)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("transform_collector")
    collector = TransformCollector()
    collector.run()
