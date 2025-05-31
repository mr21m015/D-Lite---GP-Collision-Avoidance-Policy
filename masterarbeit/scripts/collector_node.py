#!/usr/bin/env python

import rospy
import pandas as pd
import os

from masterarbeit.msg import CollisionAvoidance, CollisionDetection


class DataCollector:
    def __init__(self):
        # Robotername aus dem Parameter-Server
        robot_name = rospy.get_param('~robot', 'default_robot')

        # Pfad zu den CSV-Dateien
        base_directory = rospy.get_param('~log_base_directory', os.path.expanduser('~/.ros/logs'))
        os.makedirs(base_directory, exist_ok=True)
        self.base_footprint_csv = os.path.join(base_directory, f'{robot_name}_base_footprint_data.csv')
        self.collision_csv       = os.path.join(base_directory, f'{robot_name}_collision_data.csv')

        # Spaltenköpfe
        self.base_footprint_columns = [
            'timestamp',
            'base_footprint_plan_x',
            'base_footprint_plan_y',
            'base_footprint_alternative_plan_x',
            'base_footprint_alternative_plan_y',
            'goal_x',
            'goal_y',
            'path_choice',
            'intersect_radius',
            'intersect_centers_x',
            'intersect_centers_y',
            'time_to_collision',
            'obstacle_x',
            'obstacle_y',
            'velocity_x',
            'velocity_y',
            'odom_vel',
            'main_path_length',
            'alternative_path_length',
            'direction',
            'realtime_duration'
        ]
        self.collision_columns = [
            'timestamp',
            'collision'
        ]

        # CSV-Dateien mit Headern anlegen, falls sie nicht existieren
        if not os.path.exists(self.base_footprint_csv):
            pd.DataFrame(columns=self.base_footprint_columns).to_csv(self.base_footprint_csv, index=False)
        if not os.path.exists(self.collision_csv):
            pd.DataFrame(columns=self.collision_columns).to_csv(self.collision_csv, index=False)

        # Topics abonnieren
        base_topic      = f'/{robot_name}/base_footprint_plan_data'
        collision_topic = f'/{robot_name}/collision_detector'
        self.base_sub  = rospy.Subscriber(base_topic,      CollisionAvoidance,    self.base_footprint_callback)
        self.coll_sub  = rospy.Subscriber(collision_topic, CollisionDetection,    self.collision_callback)

        rospy.loginfo(f"Subscribed to {base_topic} and {collision_topic}")

    def safe_list(self, maybe_list):
        """Immer eine Liste zurückliefern."""
        if hasattr(maybe_list, '__len__') and not isinstance(maybe_list, str):
            return list(maybe_list)
        return []

    def safe_float(self, maybe_val, default=0.0):
        """Versuche, float zu casten oder default zurückliefern."""
        try:
            return float(maybe_val)
        except Exception:
            return default

    def base_footprint_callback(self, msg):
        # Intersect-Zentren
        centers_x = self.safe_list(getattr(msg.intersect, 'center_x', []))
        centers_y = self.safe_list(getattr(msg.intersect, 'center_y', []))
        intersect_centers = list(zip(centers_x, centers_y))

        # Hauptpfad
        plan_x, plan_y = [], []
        for p in getattr(msg, 'base_footprint_plan', []):
            # falls es noch ein Point ist, p.x vorhanden, sonst PoseStamped
            plan_x.append(p.pose.position.x)
            plan_y.append(p.pose.position.y)

        # Alternativpfad
        alt_x, alt_y = [], []
        for p in getattr(msg, 'base_footprint_alternative_plan', []):
            alt_x.append(p.pose.position.x)
            alt_y.append(p.pose.position.y)

        # Goal
        goal_msg = getattr(msg, 'goal_base_footprint', None)
        if goal_msg and hasattr(goal_msg, 'pose'):
            goal_x = getattr(goal_msg.pose.position, 'x', 0.0)
            goal_y = getattr(goal_msg.pose.position, 'y', 0.0)
        else:
            goal_x = goal_y = 0.0

        # Kreise und deren Geschwindigkeiten
        circles = getattr(msg.intersect, 'circles', [])
        circle_x = [getattr(c.center, 'x', 0.0)      for c in circles]
        circle_y = [getattr(c.center, 'y', 0.0)      for c in circles]
        vel_x    = [getattr(c.velocity, 'x', 0.0)    for c in circles]
        vel_y    = [getattr(c.velocity, 'y', 0.0)    for c in circles]

        # Sonstige Felder
        path_choice = getattr(msg, 'path_choice', -1)
        odom_vel    = getattr(msg, 'odom_vel',    0.0)
        main_len    = self.safe_float(getattr(msg, 'main_path_length',        None))
        alt_len     = self.safe_float(getattr(msg, 'alternative_path_length', None))
        radii = getattr(msg.intersect, 'radius', [])
        intersect_radius = self.safe_float(radii[0]) if len(radii) > 0 else 0.0
        times = getattr(msg.intersect, 'time', [])
        time_to_collision = self.safe_float(times[0]) if len(times) > 0 else 0.0

        row = {
            'timestamp':                 msg.header.stamp.to_sec(),
            'base_footprint_plan_x':     plan_x,
            'base_footprint_plan_y':     plan_y,
            'base_footprint_alternative_plan_x': alt_x,
            'base_footprint_alternative_plan_y': alt_y,
            'goal_x':                    goal_x,
            'goal_y':                    goal_y,
            'path_choice':               path_choice,
            'intersect_radius':          intersect_radius,
            'intersect_centers_x':       centers_x,
            'intersect_centers_y':       centers_y,
            'time_to_collision':         time_to_collision,
            'obstacle_x':                circle_x,
            'obstacle_y':                circle_y,
            'velocity_x':                vel_x,
            'velocity_y':                vel_y,
            'odom_vel':                  odom_vel,
            'main_path_length':          main_len,
            'alternative_path_length':   alt_len,
            'direction':                 msg.direction,
            'realtime_duration':         msg.realtime_duration
        }

        pd.DataFrame([row]).to_csv(self.base_footprint_csv, mode='a', header=False, index=False)

    def collision_callback(self, msg):
        row = {
            'timestamp': msg.header.stamp.to_sec(),
            'collision': getattr(msg, 'collision_', False)
        }
        pd.DataFrame([row]).to_csv(self.collision_csv, mode='a', header=False, index=False)


if __name__ == '__main__':
    rospy.init_node('data_collector', anonymous=True)
    collector = DataCollector()
    rospy.spin()
