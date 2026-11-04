TF（Transform）本质是 ROS 维护的一张“坐标系关系图”：

- 每个 frame（坐标系） 是图里的一个节点；
- 两个 frame 之间的刚体变换（平移 + 旋转，SE(3)）是图里的一条边；
- 结构必须是树：只有一个根，每个 frame 只能有一个父节点（不能有环、不能有多个父）。
一旦有了这张树，任意两个 frame 之间的变换都可以通过沿路径逐段相乘得到：
T_world_to_wrist3 = T(world→base) · T(base→shoulder) · ... · T(wrist_2→wrist_3)

```
world
├─(fixed, xyz=0 0.5 0)─ ur5e_left_base_link
│   ├─(revolute, q1)─ ur5e_left_shoulder_link
│   │   ├─(revolute, q2)─ ur5e_left_upper_arm_link
│   │   │   ├─(revolute, q3)─ ur5e_left_forearm_link
│   │   │   │   ├─(revolute, q4)─ ur5e_left_wrist_1_link
│   │   │   │   │   ├─(revolute, q5)─ ur5e_left_wrist_2_link
│   │   │   │   │   │   ├─(revolute, q6)─ ur5e_left_wrist_3_link
│   │   │   │   │   │   │   └─(fixed)─ ur5e_left_Gripper
└─(fixed)─ ur5e_right_base_link
    └─ ... 右臂同构
```
4 条固定边：world→base（左右各一）、wrist_3→Gripper（左右各一）
12 条可动边：左右各 6 个转动关节，位移量来自 /joint_states

TF 树的关键性质
性质	说明
边存的是相对变换	不是每个 frame 的全局位姿，tf2 查询时才逐段复合
边带时间戳	动态边随时间更新，tf2 缓冲区保留历史，可查询过去时刻并插值
静态边不变	只发一次，不需要反复广播
查询 API	lookupTransform(target_frame, source_frame, time)
消费者	RViz、MoveIt、tf2_ros::Buffer、你自己写的脚本
TF 不是“机器人专用”：相机、激光雷达、地图、世界坐标都可以挂进同一棵树。