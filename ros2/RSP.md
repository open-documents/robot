robot_state_publisher 用它表示运动学树

urdf::Model model;
model.initString(urdf_xml);                 // 解析 URDF
KDL::Tree tree;
kdl_parser::treeFromUrdfModel(model, tree); // URDF → KDL 树
kdl_parser 负责把 URDF 的 link/joint 转成 KDL 的 Segment/Chain。
每个关节变成一个 KDL::Segment，里面含 <origin> 和关节类型/轴。
RSP 随后调用 segment.pose(q) 算每条边的变换（见图 1）。
注意：RSP 只用 KDL 的“树 + 段”表示，不做 IK。