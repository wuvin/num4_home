#!/bin/bash

ros2 topic list
echo "Running publisher..."
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from Laptop'" --rate 1
