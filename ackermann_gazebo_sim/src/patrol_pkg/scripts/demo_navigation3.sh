#!/usr/bin/env bash

# 确保ROS环境已经加载
source /opt/ros/melodic/setup.bash
WS_DEVEL=$(cd "$(dirname "$0")/../../../../.." && pwd)/devel
[ -f "$WS_DEVEL/setup.bash" ] && source "$WS_DEVEL/setup.bash"

# 配置每段路径的等待时间（秒）
WAIT_TIME_1_2=5  # 起点到第一个点的等待时间
WAIT_TIME_2_3=7  # 第一个点到第二个点的等待时间
WAIT_TIME_3_4=15  # 第二个点到第三个点的等待时间
WAIT_TIME_4_5=15  # 第三个点到第四个点的等待时间
WAIT_TIME_5_6=10  # 第四个点到第五个点的等待时间

# 第一个点：从当前位置稍微向左前方移动
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: 2.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"

sleep $WAIT_TIME_1_2

# 第二个点：向外移动，调整姿态
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: 3.0
    y: 8.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"

sleep $WAIT_TIME_2_3

# 下一个目标点：到达左侧空地中心
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: -10.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

sleep $WAIT_TIME_3_4

# 下一个目标点：到达下方空地中心
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: 0.0
    y: -14.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.707
    w: 0.707"

sleep $WAIT_TIME_4_5

# 下一个目标点：到达右侧空地中心
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: 15.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"

sleep $WAIT_TIME_5_6

# 最后一个目标点：到达上侧空地中心
rostopic pub -1 /car3/move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'car3/map'
pose:
  position:
    x: 0.0
    y: 17.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"
  
