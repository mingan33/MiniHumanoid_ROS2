sudo ./can.sh //使能四路CAN通道

ros2 launch MotCtrl_Can MotCtrl.launch.py  //开启电机CAN通信控制节点

ros2 run robot_control robot_control //开启机器人控制节点
