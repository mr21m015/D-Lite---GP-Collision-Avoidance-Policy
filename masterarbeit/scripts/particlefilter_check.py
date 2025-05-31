#!/usr/bin/env python

import rospy
import pandas as pd
import os
import tf
from math import sin, cos
from datetime import datetime
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles
import message_filters
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class ParticleDataCollector:
    def __init__(self):
        # Robotername aus dem Parameter-Server abrufen
        robot_itself_name = rospy.get_param('~robot', '')
        robot_name = rospy.get_param('~robot_other', '')

        # Transformations-Buffer und Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Dynamischen Pfad zur CSV-Datei festlegen
        base_directory = rospy.get_param('~log_base_directory', os.path.expanduser('~/.ros/logs'))
        if not os.path.exists(base_directory):
            os.makedirs(base_directory)
        
        self.particle_check_csv = os.path.join(base_directory, f'{robot_itself_name}_simulation_data.csv')

        # Initialisieren der DataFrame-Spalten mit ID und time als erste Spalten
        self.particle_check_columns = [
            'ID',
            'time',
            'timestamp',
            'real_position_x',
            'real_position_y',
            'estimated_position_x',
            'estimated_position_y',
            'delta_position_x',
            'delta_position_y',
            'real_velocity_x',
            'real_velocity_y',
            'estimated_velocity_x',
            'estimated_velocity_y',
            'delta_velocity_x',
            'delta_velocity_y'
        ]

        # Wenn die Datei nicht existiert, erstelle sie und füge die Überschriften hinzu
        if not os.path.exists(self.particle_check_csv):
            df = pd.DataFrame(columns=self.particle_check_columns)
            df.to_csv(self.particle_check_csv, mode='w', header=True, index=False)

        # Dynamischer odom-Frame
        odom_frame = f"{robot_name}_tf/odom"

        # Topics mit Roboternamen erstellen
        robot_odom_topic = f'/{robot_name}/odom'
        obstacle_topic = f'/{robot_itself_name}/obstacles'

        # Abonnieren der Topics mit Synchronisation
        odom_sub = message_filters.Subscriber(robot_odom_topic, Odometry)
        obstacle_sub = message_filters.Subscriber(obstacle_topic, Obstacles)

        # Synchronisation
        ts = message_filters.ApproximateTimeSynchronizer([odom_sub, obstacle_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.particle_check_callback)
        
        # Speichere den odom_frame für spätere Verwendung
        self.odom_frame = odom_frame

    def particle_check_callback(self, odom_msg, obstacle_msg):
        # Prüfen, ob Hindernisse erkannt wurden
        if not obstacle_msg.circles:
            rospy.loginfo("Keine Hindernisse erkannt, Daten werden nicht gespeichert.")
            return

        try:
            # Transformiere Odometry-Position in den map-Frame
            pose = PoseStamped()
            pose.header = odom_msg.header
            pose.pose = odom_msg.pose.pose
            transform = self.tf_buffer.lookup_transform('map', self.odom_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            # Position des Roboters in map-Koordinaten
            real_position_x = transformed_pose.pose.position.x
            real_position_y = transformed_pose.pose.position.y

            # Orientierung des Roboters (Quaternion in map-Koordinaten)
            orientation = transformed_pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)  # Extrahiere Yaw-Winkel

        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform konnte nicht gefunden werden: {e}")
            return
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn(f"Transform Fehler: {e}")
            return

        # Geschwindigkeit aus Odometry transformieren
        linear_velocity_x = odom_msg.twist.twist.linear.x
        linear_velocity_y = odom_msg.twist.twist.linear.y
        real_velocity_x = linear_velocity_x * cos(yaw) - linear_velocity_y * sin(yaw)
        real_velocity_y = linear_velocity_x * sin(yaw) + linear_velocity_y * cos(yaw)

        # Schleife für alle Hindernisse
        for circle in obstacle_msg.circles:
            estimated_position_x = circle.center.x
            estimated_position_y = circle.center.y
            estimated_velocity_x = circle.velocity.x
            estimated_velocity_y = circle.velocity.y

            # Differenzen berechnen und als Absolutwerte darstellen
            delta_position_x = abs(real_position_x - estimated_position_x)
            delta_position_y = abs(real_position_y - estimated_position_y)
            delta_velocity_x = abs(real_velocity_x - estimated_velocity_x)
            delta_velocity_y = abs(real_velocity_y - estimated_velocity_y)

            # Bedingung: Speichern nur, wenn mindestens eine Positionsdifferenz < 1.5
            if delta_position_x < 1.5 and delta_position_y < 1.5:
                # Daten speichern, "ID" und "time" bleiben leer (wird später hinzugefügt)
                row = {
                    'ID': '',
                    'time': '',
                    'timestamp': odom_msg.header.stamp.to_sec(),
                    'real_position_x': real_position_x,
                    'real_position_y': real_position_y,
                    'estimated_position_x': estimated_position_x,
                    'estimated_position_y': estimated_position_y,
                    'delta_position_x': delta_position_x,
                    'delta_position_y': delta_position_y,
                    'real_velocity_x': real_velocity_x,
                    'real_velocity_y': real_velocity_y,
                    'estimated_velocity_x': estimated_velocity_x,
                    'estimated_velocity_y': estimated_velocity_y,
                    'delta_velocity_x': delta_velocity_x,
                    'delta_velocity_y': delta_velocity_y
                }

                df_row = pd.DataFrame([row])
                df_row.to_csv(self.particle_check_csv, mode='a', header=False, index=False)

                rospy.loginfo(f"Hindernis erkannt und gespeichert: delta_x={delta_position_x}, delta_y={delta_position_y}")
            else:
                rospy.loginfo(f"Hindernis ignoriert: delta_x={delta_position_x}, delta_y={delta_position_y}")


if __name__ == '__main__':
    rospy.init_node('particle_data_collector', anonymous=True)
    collector = ParticleDataCollector()
    rospy.spin()
