本文介绍 ROS2 的内置包 robot_state_publisher。robot_state_publisher 是 ROS2 官方包 robot_state_publisher 提供的通用节点，几乎是所有带 URDF 的 ROS2 系统都会启动的第一个节点，可以直接在 launch 文件中创建启动：

```python
def generate_launch_description():
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )
    return LaunchDescription(
        [rsp_node]
    )
```


robot_state_publisher 包中的 robot_state_publisher 做 4 件事情：\
- 发布 /robot_description 话题：把 robot_description 参数（URDF XML 字符串、文件路径）以 std_msgs/String 发布到 /robot_description 话题，方便其他节点（RViz、MoveIt、TF 工具）动态获取模型；
- 发布 /tf_static 话题：将 URDF 中的固定关节变换发到 /tf_static 话题，只发一次；
- 订阅 /joint_states 话题：这是它唯一的输入来源（sensor_msgs/JointState）。rsp 本身不做状态估计，也不控制关节——关节值从哪来（真机驱动、Gazebo、Isaac、自己的合并节点）它不关心。
- 发布 /tf 话题：结合收到的关节位姿，用正运动学计算每个子 link 相对父 link 的变换，发布到 /tf 话题；

