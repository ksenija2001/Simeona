#!/bin/sh

ros2 action send_goal /move_distance robotic_interfaces/action/MoveDistance "{goal_distance: 0.3, goal_theta: 1.57}" --feedback && sleep 0.5 && 
ros2 action send_goal /move_distance robotic_interfaces/action/MoveDistance "{goal_distance: 0.3, goal_theta: 3.14}" --feedback && sleep 0.5 &&
ros2 action send_goal /move_distance robotic_interfaces/action/MoveDistance "{goal_distance: 0.3, goal_theta: -1.57}" --feedback && sleep 0.5 &&
ros2 action send_goal /move_distance robotic_interfaces/action/MoveDistance "{goal_distance: 0.3, goal_theta: 0.0}" --feedback


