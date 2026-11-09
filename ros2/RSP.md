robot_state_publisher 用它表示运动学树

urdf::Model model;
model.initString(urdf_xml);                 // 解析 URDF
KDL::Tree tree;
kdl_parser::treeFromUrdfModel(model, tree); // URDF → KDL 树
kdl_parser 负责把 URDF 的 link/joint 转成 KDL 的 Segment/Chain。
每个关节变成一个 KDL::Segment，里面含 <origin> 和关节类型/轴。
RSP 随后调用 segment.pose(q) 算每条边的变换（见图 1）。
注意：RSP 只用 KDL 的“树 + 段”表示，不做 IK。


robot_state_publisher (rsp) 节点
robot_state_publisher 是 ROS2 官方包 robot_state_publisher 提供的通用节点（不是 MoveIt 或 UR 专用），几乎是所有带 URDF 的 ROS2 系统都会启动的第一个节点。

它做三件事
1. 发布 /robot_description 话题
把 robot_description 参数（URDF XML 字符串）以 std_msgs/String 发布到 /robot_description，方便其他节点（RViz、MoveIt、TF 工具）动态获取模型。参数也可以是文件路径（较新版本支持）。

2. 把 URDF 转成 TF 树

解析 URDF 里的 <link> / <joint>，建立运动学树
固定关节 (fixed joint)：不依赖任何输入，只发一次（/tf_static，latched）
可动关节 (revolute/prismatic/continuous)：每收到一帧关节位置，就用正运动学算出每条子 link 相对父 link 的变换，以 /tf 发布
3. 订阅并同步 /joint_states
这是它唯一的输入来源（sensor_msgs/JointState）。rsp 本身不做状态估计，也不控制关节——关节值从哪来（真机驱动、Gazebo、Isaac、自己的合并节点）它不关心。