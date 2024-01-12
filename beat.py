#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from pkg_servo.msg import Num
import pathlib
import json
import toml
import pandas as pd
from collections import OrderedDict

with open(pathlib.Path.home() / '/workspaces/ros2/servo/src/pkg_servo/src/1_wayne_0_1_8.json') as f:
  action_file = json.load(f)

with open(pathlib.Path.home() / '/workspaces/ros2/servo/src/pkg_servo/src/config_beat.toml') as n:
  joints_toml = toml.load(n)  

# motor_ids=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,262,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51]
names = action_file["names"]
weights_data = [frame["weights"] for frame in action_file["frames"]]


df = pd.DataFrame(weights_data, columns=names)
# print(df)   

def linear(val, min_val, max_val):
    slope = (max_val - min_val)
    intercept = min_val
    pos = int(slope * val + intercept)
    return pos

# Apply linear transformation to each column in the DataFrame
for key, val in joints_toml.items():
    df[key] = df[key].apply(linear, args=(val['min'], val['max']))

def publish_brow_values():
    rclpy.init()
    node = rclpy.create_node('pub')
    publisher = node.create_publisher(Num, 'motor_pos', 10)
    for index, row in df.iterrows():
        # Convert each column value to the required format
        row_data = [(int(joints_toml[key]['motor_id']), int(row[key])) for key in joints_toml]
        message = Num()
        message.data = [str(row_data)]
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        publish_brow_values()
    except Exception as e:
        print(f"Error: {e}")

   