#!/usr/bin/env bash

# 确保ROS环境已经加载
source /opt/ros/melodic/setup.bash
source $(catkin locate --devel)/setup.bash

# 配置每段路径的等待时间（秒）
WAIT_TIME_1_2=5  # 起点到第一个点的等待时间
WAIT_TIME_2_3=7  # 第一个点到第二个点的等待时间
WAIT_TIME_3_4=15  # 第二个点到第三个点的等待时间
WAIT_TIME_4_5=15  # 第三个点到第四个点的等待时间
WAIT_TIME_5_6=10  # 第四个点到第五个点的等待时间

# 第一个点：从当前位置稍微向左前方移动
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: -2.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"

sleep $WAIT_TIME_1_2

# 第二个点：向外移动，调整姿态
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: 1.0
    y: 6.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"

sleep $WAIT_TIME_2_3

# 下一个目标点：到达左侧空地中心
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: -6.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
    
sleep $WAIT_TIME_3_4
   
# 下一个目标点：到达下方空地中心
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: 0.0
    y: -10.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.707
    w: 0.707"

sleep $WAIT_TIME_4_5

# 下一个目标点：到达右侧空地中心
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: 11.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"
  
sleep $WAIT_TIME_5_6
  
# 最后一个目标点：到达上侧空地中心
rostopic pub -1 /car1/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car1/map'
pose:
  position:
    x: 0.0
    y: 13.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"
  
