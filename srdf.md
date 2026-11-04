本文介绍机器人领域的 SRDF 文件。


<group_state> 是 SRDF 里给某个规划组保存的一组"命名关节值"——可以理解成这个组的预设位姿 / 预设关节配置。它只描述"角度是多少"，本身不会让机器人动。

1. 语法

<group_state name="状态名" group="组名">
    <joint name="关节名" value="值"/>   <!-- 可多个 -->
</group_state>
name：这个预设的名字，在该组内唯一。
group：必须是在同一 SRDF 里用 <group> 定义过的组。
joint：关节名必须是 URDF 里存在的、且属于该组的关节。
value：revolute/continuous 关节是弧度，prismatic 关节是米。
解析它的是 ROS 2 的 srdfdom 包，结构类似：


struct GroupState {
  std::string group_;
  std::string name_;
  struct Joint { std::string name_; double value_; };
  std::vector<Joint> joint_values_;
};
2. SRDF → MoveIt 的加载链路

URDF (robot_description)          SRDF (robot_description_semantic)
   link/joint 树        +          <group> / <group_state> / <disable_collisions>
        │                                  │
        └───────────────┬──────────────────┘
                        ▼
             moveit::core::RobotModel
                        │ 为每个 <group_state> 构造一个 RobotStatePtr
                        ▼
             moveit::core::JointModelGroup
                        └─ named_states_: "left_home" → RobotStatePtr
也就是说，<group_state> 最终被实例化成 RobotState，挂在对应 JointModelGroup 的 named_states_ 表里，可通过 JointModelGroup::getNamedState("left_home") 或 getNamedStates() 取到。

3. 它在运行时怎么被使用
RViz
MotionPlanning 面板 → Planning → Goal State 下拉框，会列出这些名字。选中后点 Plan 即可。

C++（MoveIt 2）

moveit::core::RobotState state(robot_model_);
state.setToDefaultValues(robot_model_->getJointModelGroup("left_arm"), "left_home");
// 或者
const moveit::core::RobotStatePtr named = jmg->getNamedState("left_home");
Python（moveit_commander）

group = moveit_commander.MoveGroupCommander("left_arm")
group.set_named_target("left_home")   # 只设目标，不执行
group.plan()
group.execute()
注意
只是"预设值"，不会自动下发、不会触发运动。
规划器（OMPL 等）本身不依赖它；它是给人用的便捷入口（默认起点、回 home、测试）。
与 config/initial_positions.yaml 完全是两回事：那个是 ros2_control 假硬件启动时的初始关节值（本仓库里全是 0），group_state 是 MoveIt 语义层的预设。



```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--This does not replace URDF, and is not an extension of URDF.
    This is a format for representing semantic information about the robot structure.
    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
-->
<robot name="dual_ur5e">
    <!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->
    <!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included-->
    <!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included-->
    <!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group-->
    <!--SUBGROUPS: Groups can also be formed by referencing to already defined group names-->
    <group name="left_arm">
        <chain base_link="ur5e_left_base_link" tip_link="ur5e_left_Gripper"/>
    </group>
    <group name="right_arm">
        <chain base_link="ur5e_right_base_link" tip_link="ur5e_right_Gripper"/>
    </group>
    <group name="dual_arms">
        <group name="left_arm"/>
        <group name="right_arm"/>
    </group>
    <!--GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'-->
    <group_state name="left_home" group="left_arm">
        <joint name="ur5e_left_elbow_joint" value="1.749"/>
        <joint name="ur5e_left_shoulder_lift_joint" value="-2.072"/>
        <joint name="ur5e_left_shoulder_pan_joint" value="0"/>
        <joint name="ur5e_left_wrist_1_joint" value="-1.208"/>
        <joint name="ur5e_left_wrist_2_joint" value="-1.46"/>
        <joint name="ur5e_left_wrist_3_joint" value="0.034"/>
    </group_state>
    <group_state name="right_home" group="right_arm">
        <joint name="ur5e_right_elbow_joint" value="-1.769"/>
        <joint name="ur5e_right_shoulder_lift_joint" value="-1.174"/>
        <joint name="ur5e_right_shoulder_pan_joint" value="-3.1429"/>
        <joint name="ur5e_right_wrist_1_joint" value="-1.936"/>
        <joint name="ur5e_right_wrist_2_joint" value="1.585"/>
        <joint name="ur5e_right_wrist_3_joint" value="-3.1589"/>
    </group_state>
    <group_state name="dual_home" group="dual_arms">
        <joint name="ur5e_left_elbow_joint" value="1.749"/>
        <joint name="ur5e_left_shoulder_lift_joint" value="-2.072"/>
        <joint name="ur5e_left_shoulder_pan_joint" value="0"/>
        <joint name="ur5e_left_wrist_1_joint" value="-1.208"/>
        <joint name="ur5e_left_wrist_2_joint" value="-1.46"/>
        <joint name="ur5e_left_wrist_3_joint" value="0.034"/>
        <joint name="ur5e_right_elbow_joint" value="-1.769"/>
        <joint name="ur5e_right_shoulder_lift_joint" value="-1.174"/>
        <joint name="ur5e_right_shoulder_pan_joint" value="-3.143"/>
        <joint name="ur5e_right_wrist_1_joint" value="-1.936"/>
        <joint name="ur5e_right_wrist_2_joint" value="1.585"/>
        <joint name="ur5e_right_wrist_3_joint" value="-3.1589"/>
    </group_state>
    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->
    <disable_collisions link1="ur5e_left_Gripper" link2="ur5e_left_wrist_1_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_Gripper" link2="ur5e_left_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_Gripper" link2="ur5e_left_wrist_3_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_left_shoulder_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_base_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_forearm_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_upper_arm_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_wrist_1_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_base_link" link2="ur5e_right_wrist_3_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_forearm_link" link2="ur5e_left_upper_arm_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_forearm_link" link2="ur5e_left_wrist_1_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_forearm_link" link2="ur5e_left_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_forearm_link" link2="ur5e_right_base_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_forearm_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_left_upper_arm_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_base_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_forearm_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_upper_arm_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_wrist_1_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_shoulder_link" link2="ur5e_right_wrist_3_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_upper_arm_link" link2="ur5e_right_base_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_upper_arm_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_upper_arm_link" link2="ur5e_right_upper_arm_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_1_link" link2="ur5e_left_wrist_2_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_wrist_1_link" link2="ur5e_left_wrist_3_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_1_link" link2="ur5e_right_base_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_1_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_2_link" link2="ur5e_left_wrist_3_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_left_wrist_2_link" link2="ur5e_right_base_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_3_link" link2="ur5e_right_shoulder_link" reason="Never"/>
    <disable_collisions link1="ur5e_left_wrist_3_link" link2="ur5e_right_wrist_1_link" reason="Never"/>
    <disable_collisions link1="ur5e_right_Gripper" link2="ur5e_right_wrist_1_link" reason="Never"/>
    <disable_collisions link1="ur5e_right_Gripper" link2="ur5e_right_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_right_Gripper" link2="ur5e_right_wrist_3_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_base_link" link2="ur5e_right_shoulder_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_forearm_link" link2="ur5e_right_upper_arm_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_forearm_link" link2="ur5e_right_wrist_1_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_forearm_link" link2="ur5e_right_wrist_2_link" reason="Never"/>
    <disable_collisions link1="ur5e_right_shoulder_link" link2="ur5e_right_upper_arm_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_wrist_1_link" link2="ur5e_right_wrist_2_link" reason="Adjacent"/>
    <disable_collisions link1="ur5e_right_wrist_1_link" link2="ur5e_right_wrist_3_link" reason="Never"/>
    <disable_collisions link1="ur5e_right_wrist_2_link" link2="ur5e_right_wrist_3_link" reason="Adjacent"/>
</robot>

```