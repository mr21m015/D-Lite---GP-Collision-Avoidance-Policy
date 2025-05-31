#!/usr/bin/env python3

import rospy
import pandas as pd
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult

class PerGoalDataLogger:
    def __init__(self):
        # params
        self.robot      = rospy.get_param('~robot_name', 'robot1')
        base_dir       = rospy.get_param('~log_base_directory', os.path.expanduser('~/.ros/logs'))
        os.makedirs(base_dir, exist_ok=True)

        # CSV-Dateien
        logs_dir = os.path.join(base_dir, 'robot_state_logs')
        os.makedirs(logs_dir, exist_ok=True)
        self.goal_csv = os.path.join(logs_dir, f'{self.robot}_goals.csv')
        self.odom_csv = os.path.join(logs_dir, f'{self.robot}_odom.csv')


        # Header initialisieren
        if not os.path.exists(self.goal_csv):
            pd.DataFrame(columns=[
                'goal_id','t_start_s','goal_x','goal_y',
                't_end_s','duration_s','status_code','status_text'
            ]).to_csv(self.goal_csv, index=False)
        if not os.path.exists(self.odom_csv):
            pd.DataFrame(columns=[
                'goal_id','timestamp_s',
                'pos_x','pos_y','pos_z',
                'lin_x','lin_y','lin_z',
                'ang_x','ang_y','ang_z'
            ]).to_csv(self.odom_csv, index=False)

        # state
        self.pending = {}
        self.current = None  

        # subs
        rospy.Subscriber(f'/{self.robot}/move_base/goal',
                         MoveBaseActionGoal,   self.goal_cb)
        rospy.Subscriber(f'/{self.robot}/move_base/result',
                         MoveBaseActionResult, self.result_cb)
        rospy.Subscriber(f'/{self.robot}/odom',
                         Odometry,             self.odom_cb)

        rospy.loginfo(f"[{self.robot}] DataLogger gestartet")

    def goal_cb(self, msg):
        gid = msg.goal_id.id
        ts  = msg.header.stamp.to_sec()
        x   = msg.goal.target_pose.pose.position.x
        y   = msg.goal.target_pose.pose.position.y
        self.pending[gid] = {'t_start': ts, 'x': x, 'y': y}
        self.current = gid

    def result_cb(self, msg):
        gid = msg.status.goal_id.id
        if gid not in self.pending:
            return
        start = self.pending[gid]['t_start']
        end   = msg.header.stamp.to_sec()
        dur   = end - start
        row = {
            'goal_id': gid,
            't_start_s': start,
            'goal_x': self.pending[gid]['x'],
            'goal_y': self.pending[gid]['y'],
            't_end_s': end,
            'duration_s': dur,
            'status_code': msg.status.status,
            'status_text': msg.status.text
        }
        pd.DataFrame([row]).to_csv(self.goal_csv, mode='a', header=False, index=False)
        del self.pending[gid]
        if self.current == gid:
            self.current = None

    def odom_cb(self, msg):
        if not self.current:
            return
        p   = msg.pose.pose.position
        v   = msg.twist.twist
        row = {
            'goal_id':    self.current,
            'timestamp_s': msg.header.stamp.to_sec(),
            'pos_x':      p.x,
            'pos_y':      p.y,
            'pos_z':      p.z,
            'lin_x':      v.linear.x,
            'lin_y':      v.linear.y,
            'lin_z':      v.linear.z,
            'ang_x':      v.angular.x,
            'ang_y':      v.angular.y,
            'ang_z':      v.angular.z
        }
        pd.DataFrame([row]).to_csv(self.odom_csv, mode='a', header=False, index=False)

if __name__=='__main__':
    rospy.init_node('per_goal_data_logger', anonymous=True)
    PerGoalDataLogger()
    rospy.spin()
