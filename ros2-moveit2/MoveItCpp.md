# MoveItCpp

MoveItCpp 是 MoveIt 2 在 C++ 中暴露的门面之一：它把「场景维护 → 算法规划 → 轨迹执行」三层装配起来，对外提供 `getRobotModel()`、`getPlanningSceneMonitor()`、`getPlanningPipelines()`、`plan()`、`execute()` 等高层接口。

```cpp
moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
```

---

## 0. MoveItCpp 的内部组成

```
MoveItCpp（门面对象：装配 + 高层 API）
│
├── PlanningSceneMonitor                 职责：维护世界模型
│   ├── RobotModelLoader → RobotModel    读 robot_description / robot_description_semantic
│   ├── PlanningScene                    机器人状态 + 障碍物 + 碰撞矩阵
│   ├── CurrentStateMonitor              /joint_states → RobotState
│   └── 订阅/服务/发布                    /planning_scene、/collision_object、
│                                       /monitored_planning_scene、get_planning_scene
│
├── PlanningPipeline(s)                  算法层（OMPL / CHOMP / Pilz）
│                                        输入 PlanningScene + MotionPlanRequest，输出轨迹
│
└── TrajectoryExecutionManager           执行层
                                         轨迹 → 控制器的 follow_joint_trajectory
```

### MoveItCpp 构造函数内部做了 4 件事

| 步骤 | 做什么 |
|---|---|
| 1 | `loadPlanningSceneMonitor()`：新建 `PlanningSceneMonitor`。它的构造过程会用 `RobotModelLoader` 读 `robot_description` / `robot_description_semantic`，建出 `RobotModel` |
| 2 | 启动四个 monitor：`startStateMonitor()`、`startPublishingPlanningScene()`、`startSceneMonitor()`、`startWorldGeometryMonitor()` |
| 3 | `loadPlanningPipelines()`：读 `planning_pipelines.pipeline_names` 及各 pipeline 参数，建出 `PlanningPipeline` |
| 4 | 创建 `TrajectoryExecutionManager`：读控制器配置，供 `execute()` 使用 |

> 注意：`providePlanningSceneService()` **不在**构造函数里，需要自己调（见第 2 节）。

---

## 1. RobotModel

### 1.1 整体解析链路

```
URDF (XML)                                SRDF (XML)
┌──────────────────┐                  ┌────────────────────────┐
│ link / joint     │                  │ group / group_state    │
│ axis / limit     │                  │ end_effector / ...     │
│ <mimic>          │                  │                        │
└────────┬─────────┘                  └───────────┬────────────┘
         │ urdfdom                                │ srdfdom
         ▼                                        ▼
  urdf::Model (Link/Joint)                srdf::Model (Group/...)
         │                                        │
         └──────────────────┬─────────────────────┘
                            ▼
                 moveit::core::RobotModel
   ┌───────────────────────┼──────────────────────────┐
   ▼                       ▼                          ▼
LinkModel ×17        JointModel ×17              JointModelGroup ×3
· 1 个 parent joint   · joint_index               · vector<const JointModel*>（引用，不拷贝）
· 树结构              · first_variable_index      · variable_names_ / variable_index_list_
                      · variable_names_[0|1|3|7]  · active / mimic / fixed 三个桶
                      · variable_bounds_
                      · 其中 1 个是自动补的
                        ASSUMED_FIXED_ROOT_JOINT
                            │
                            │ 全局汇总（buildJointInfo）
                            ▼
             variable_names_[12] · variable_count_ = 12
             joint_variables_index_map_ : 名 → 全局下标
             joints_of_variable_         : 变量下标 → 关节
                            │
                            │ 运行时按此长度分配
                            ▼
                       RobotState
        · 持有 RobotModel 的引用（不拥有结构）
        · position_[12] / velocity_[12] / acceleration_[12]
        · 关节/连杆变换缓存 + 脏标志（FK 结果）
        · 附着的运行时物体
```

读图注意：

- `LinkModel ×17` 是 17 个 link 各有一个 parent joint，但**根 link `world` 例外**，它没有父关节；
- `JointModel ×17` 里 16 个来自 URDF，第 17 个是 SRDF 没写 `<virtual_joint>` 时自动补的 `ASSUMED_FIXED_ROOT_JOINT`（fixed，占 0 个变量）；
- `variable_count_ = 12` 只统计 active + mimic，fixed 关节不占格；
- `acceleration_` 与 `effort_` 共用同一块内存，图里只示意长度。

### 1.2 来源

```cpp
moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());  // L62
robot_model_ = moveit_cpp_->getRobotModel();                                // L66
```

调用链：

```
MoveItCpp 构造
  → PlanningSceneMonitor（MoveItCpp 的成员）
      → RobotModelLoader
          → RDFLoader
              → urdfdom  解析 URDF  → urdf::Model
              → srdfdom  解析 SRDF  → srdf::Model
          → moveit::core::RobotModel(urdf, srdf)
```

- `RobotModel` 由 `PlanningSceneMonitor` 内的 `RobotModelLoader` 创建；
- `MoveItCpp::getRobotModel()` 直接返回 `PlanningSceneMonitor` 已经建好的那个模型（Humble 里 `MoveItCpp` 在构造函数中把它缓存到成员 `robot_model_`）；
- `PlanningSceneMonitor` 不是“门面”，而是 MoveItCpp 的核心成员。

### 1.3 SRDF 的解析规则（`RobotModel::addJointModelGroup`）

| SRDF 写法 | 处理方式 |
|---|---|
| `<chain base_link="A" tip_link="B"/>` | 从 `B` 沿父关节往上走，依次把经过 link 的**父关节**加入组；走到 `A` 就停，**不加入 `A` 的父关节** |
| `<link name="L"/>` | 把 `L` 的**父关节**加入组（不是把 link 加进去） |
| `<joint name="J"/>` | 直接把关节 `J` 加入组 |
| 子组（本工程写作嵌套 `<group name="..."/>`，规范写法也支持 `<subgroup name="..."/>`） | 把子组已收集的 **active + fixed + mimic** 三类关节全部并入父组 |

注意事项：

1. **不要**把根 link（`world`）写进 `<link>`，因为它没有父关节；
2. 名字写错（link/joint 不存在）会被**静默跳过**，不会报错，只会导致组里少东西；
3. 组内关节会按 `joint_index`（URDF 深度优先顺序）重新排序；
4. **组至少要有一个有效关节**，否则 `addJointModelGroup` 打一条警告并返回失败，该组不会被创建：

   ```
   Group 'xxx' must have at least one valid joint
   ```

   此时 `getJointModelGroup("xxx")` 返回 `nullptr`。

5. 组内的关节会按「变量数 + 是否 mimic」分成三类：
   - 变量数 > 0 且无 mimic → **active**（可驱动）
   - 变量数 > 0 且有 mimic → **mimic**
   - 变量数 = 0 → **fixed**

   注意分类依据是**变量数和 mimic 指针**，不是关节类型本身。

### 1.4 RobotModel 内部对象

**LinkModel：描述连杆**

- 名称、`link_index`；
- 父关节（根 link 除外）、父 link、子关节列表 → 构成树；
- 关节 `<origin>` 对应的常量变换；
- 碰撞几何（shapes）及其局部变换；
- 可视网格的文件名、偏移、缩放。

**JointModel：描述关节**

- `joint_index`：DFS 序号；
- `first_variable_index`：本关节第一个变量在**整机**状态数组里的下标，配合 `getVariableCount()` 得到占用区间；
- `variable_names_[0|1|3|7]`：变量名；
- `variable_bounds_`：每个变量的位置/速度/加速度/加加速度边界；
- `mimic_ / mimic_factor_ / mimic_offset_`：从动关系；
- `variable_index_map_`：**关节内部表**（变量名 → 关节内第几个）；
- 其中 1 个是自动补的 `ASSUMED_FIXED_ROOT_JOINT`（SRDF 无 `<virtual_joint>` 时）。

**JointModelGroup：描述规划组**

- `vector<const JointModel*>`：**引用**，不拷贝关节对象；
- `variable_names_` / `variable_index_list_`：组内变量名与「组内下标 → 整机下标」映射；
- `joint_variables_index_map_`：**组内表**（名字 → 组内下标）；
- active / mimic / fixed 三个桶。

### 1.5 全局汇总（`buildJointInfo`）

| 成员 | 含义 |
|---|---|
| `variable_names_` | 整机所有变量的名字（**含 mimic，不含 fixed**） |
| `variable_count_` | 整机变量总数 |
| `joint_variables_index_map_` | 变量名 → 整机变量下标  &&  关节名 → 该关节首变量的整机变量下标。因此，`joint_variables_index_map_` 有两种 key：变量名和关节名，1-DOF 关节时两者相同，多 DOF 关节时关节名指向首变量。 |
| `joints_of_variable_` | 变量下标 → 所属关节（与 `variable_names_` 平行）。数组，下标 = 整机变量下标，元素 = const JointModel*，指向 joint_model_vector_ 里那个关节对象 |


### 1.6 RobotState

一个 `RobotModel` 可以对应任意多个 `RobotState`。

```cpp
moveit::core::RobotModelConstPtr robot_model = ...;   // 全进程共享
auto s1 = std::make_shared<moveit::core::RobotState>(robot_model);
auto s2 = std::make_shared<moveit::core::RobotState>(robot_model);
```

`RobotState` 包含：

- 对 `RobotModel` 的引用（**不拥有**结构）；
- `position_` / `velocity_` / `acceleration_` / `effort_`：按**变量**存储的运行时数据；
- 关节变换、连杆变换、碰撞体变换缓存 + 脏标志（FK 结果）；
- 附着的运行时物体（`AttachedBody`）。

补充：`acceleration_` 与 `effort_` **共用同一块内存**，不会同时有值；`velocity_` 等也只在设置过后才有效（有 `has_velocity_` 等标志）。

```
auto current_state = scene->getCurrentState();
   std::vector<double> cv;
    current_state.copyJointGroupPositions("dual_arms", cv);

mutable moveit::core::RobotState scratch_;
Eigen::Ref<const Eigen::VectorXd>& q
scratch_.setJointGroupPositions(combined_, q);
```



### 1.7 变量与关节

MoveIt 需要把「关节结构」和「运行时数值」分开存放。数值统一放在一个 `double` 数组里，数组的每一格就叫一个**变量（variable）**。

| 关节类型 | 变量数 | 变量名举例 |
|:---:|:---:|---|
| revolute | 1 | `ur5e_left_elbow_joint`（变量名 = 关节名） |
| continuous | 1 | 同上 |
| prismatic | 1 | 同上 |
| planar | 3 | `my_joint/x`、`my_joint/y`、`my_joint/theta` |
| floating | 7 | `my_joint/trans_x`、`trans_y`、`trans_z`、`rot_x`、`rot_y`、`rot_z`、`rot_w` |
| fixed | 0 | 无 |

要点：

- **变量数 ≠ 自由度**：floating 有 6 个机械自由度，但用四元数表示姿态需要 4 个数，所以占 7 格；
- **fixed 占 0 格**，但仍占一个 `joint_index`；
- **mimic 占格但不是独立自由度**，值由主关节派生：`q_mimic = f * q_master + o`；
- 1-DOF 关节的变量名复用关节名，这是约定，不是恒等式。

### 1.8 URDF / SRDF 标签落点总表

| 文件 | 标签 | RobotModel 中的落点 | 相关方法 |
|---|---|---|---|
| URDF | `<link name="...">` | `LinkModel` | `getLinkModel()`、`getName()`、`getLinkIndex()`、`getChildJointModels()` |
| URDF | `<joint name="..." type="...">` | `JointModel` 子类 | `getJointModel()`、`getJointModelNames()`、`getType()`、`getTypeName()` |
| URDF | `<parent link>` / `<child link>` | `JointModel` 的父/子 link 指针；`LinkModel` 的父关节 | `getParentLinkModel()`、`getChildLinkModel()`、`getParentJointModel()` |
| URDF | `<origin xyz rpy>` | `LinkModel` 的关节原点变换（常量） | `getJointOriginTransform()` |
| URDF | `<axis xyz>` | revolute/prismatic 关节的轴（floating/fixed 忽略） | `RevoluteJointModel::getAxis()` 等 |
| URDF | `<limit lower upper velocity effort>` | `JointModel::variable_bounds_` | `getVariableBounds()` |
| URDF | `<safety_controller>` | 与 `<limit>` 取交集后写入 `variable_bounds_` | `getVariableBounds()` |
| URDF | `<mimic joint multiplier offset>` | `JointModel::mimic_ / mimic_factor_ / mimic_offset_` | `getMimic()`、`getMimicFactor()`、`getMimicOffset()` |
| URDF | `<visual>` | `LinkModel` 的可视网格信息 | `getVisualMeshFilename()`、`getVisualMeshOrigin()`、`getVisualMeshScale()` |
| URDF | `<collision>` | `LinkModel::shapes_` 与碰撞体局部变换 | `getShapes()`、`getCollisionOriginTransforms()` |
| URDF | `<inertial>` | **不进入** `RobotModel` | — |
| URDF | `<dynamics>` | **不进入** `RobotModel` | — |
| SRDF | `<group>`（chain / link / joint / 子组） | `JointModelGroup` | `getActiveJointModelNames()`（可驱动）、`getJointModelNames()`（全部）、`getFixedJointModels()`、`getMimicJointModels()`、`getVariableNames()`、`getVariableIndexList()` |
| SRDF | `<group_state>` | `JointModelGroup::default_states_` | `getDefaultStateNames()`、`getVariableDefaultPositions(name, ...)` |
| SRDF | `<end_effector>` | `RobotModel` 的末端执行器组 | `hasEndEffector()`、`getEndEffector()`、`getEndEffectors()` |
| SRDF | `<virtual_joint>` | 根关节（fixed / planar / floating） | `getRootJoint()` |
| SRDF | `<passive_joint>` | `JointModel::passive_` 标志 | `isPassive()` |
| SRDF | `<disable_collisions>` | **不进入** `RobotModel`；由 `PlanningScene` 的 `AllowedCollisionMatrix` 使用 | `getAllowedCollisionMatrix()`（模型侧可通过 `getSRDF()` 取原始数据） |

说明：

- `getActiveJointModelNames()` 的顺序是 `joint_index` 升序，即 **URDF 深度优先顺序**；
- `<group_state>` 在 Humble 中存为 `default_states_`，通过 `getDefaultStateNames()` / `getVariableDefaultPositions()` 访问；
- `<disable_collisions>` 的落点是碰撞矩阵，不是机器人模型。

---

## 2. PlanningSceneMonitor

### 2.1 它负责的 5 件事

| 方法 | 作用 |
|---|---|
| `startStateMonitor()` | 创建 `CurrentStateMonitor`，订阅 `/joint_states` |
| `startSceneMonitor()` | 订阅外部场景增量 `/planning_scene` |
| `startWorldGeometryMonitor()` | 订阅 `/collision_object`、`/planning_scene_world`、octomap（可选） |
| `startPublishingPlanningScene()` | 发布完整场景到 `/monitored_planning_scene` |
| `providePlanningSceneService()` | 提供 `get_planning_scene` 服务 |

### 2.2 PlannerNode 中的实际调用

```cpp
moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());               // L62
moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();           // L63
moveit_cpp_->getPlanningSceneMonitorNonConst()->startSceneMonitor();                     // L64
robot_model_ = moveit_cpp_->getRobotModel();                                             // L66
```

**L62：构造 MoveItCpp**

内部依次完成：建 `PlanningSceneMonitor`（含 `RobotModelLoader` → `RobotModel`）、启动 state/publishing/scene/world 四个 monitor、加载 planning pipeline、创建 `TrajectoryExecutionManager`。

**L63：`providePlanningSceneService()` — 必要**

对外提供 `get_planning_scene` 服务（`moveit_msgs/srv/GetPlanningScene`），返回完整场景：机器人状态 + 世界碰撞对象 + 附着物体。

- 构造函数**没有**调用它，所以这一行不能省；
- 不调用时 `/monitored_planning_scene` 照常有，但服务不存在，客户端会报 `service not available`；
- 如果同机还有 `move_group`，两者会重复 advertise 同名服务，可能冲突。本工程的 launch 没有 `move_group`，无影响。

**L64：`startSceneMonitor()` — 冗余**

MoveItCpp 构造函数内部已经调用过 `startSceneMonitor()`。这里再调一次会先 `stopSceneMonitor()` 再重新订阅：

```cpp
void PlanningSceneMonitor::startSceneMonitor(const std::string& scene_topic)
{
  stopSceneMonitor();          // 先停
  ...
  planning_scene_subscriber_ = create_subscription<...>(...);   // 再订阅
}
```

所以**不会泄漏、不会报错**，只是重复建了一次订阅。可以删掉。

**L66：`getRobotModel()`**

返回 `PlanningSceneMonitor` 已经建好的 `RobotModelConstPtr`，不是新建。

### 2.3 为什么放在 `init()` 而不是构造函数里

`MoveItCpp` 的构造函数接收 `rclcpp::Node::SharedPtr`，调用处传的是 `shared_from_this()`。

`shared_from_this()` 要求对象**已经被 `shared_ptr` 持有**。如果写在 `PlannerNode` 的构造函数里，此时 `main()` 中的 `std::make_shared<PlannerNode>()` 还没返回，内部 weak_ptr 尚未就绪，会抛 `std::bad_weak_ptr`。

因此拆成两步（`planner_node.cpp:545-552`）：

```cpp
auto node = std::make_shared<PlannerNode>();   // L548
node->init();                                  // L549
```

### 2.4 `getPlanningSceneMonitor` 的两个版本

Humble 中的实际签名：

| 方法 | 返回 | 说明 |
|---|---|---|
| `getPlanningSceneMonitor() const` | `const PlanningSceneMonitorPtr&` | 返回成员的 const 引用，不增加引用计数 |
| `getPlanningSceneMonitorNonConst()` | `PlanningSceneMonitorPtr` | 返回 shared_ptr 的**拷贝**（引用计数 +1），指向同一个对象 |

需要澄清两点：

1. **两者都不是 `ConstPtr`**，指向的都是可修改的 `PlanningSceneMonitor`。区别只在方法是 const、返回的是引用还是拷贝。
2. 返回的是 **shared_ptr 的拷贝，不是 monitor 对象的拷贝**。对象生命周期仍由 `MoveItCpp` 持有；你手上的只是同一对象的另一个句柄。

本工程用 `NonConst()` 版只是为了语义上明确「要调用非 const 方法」，并非语法强制（通过 `const shared_ptr<T>&` 的 `operator->` 仍能调用 `T` 的非 const 方法）。

这个对象是场景系统的中枢：持有 `PlanningScene`（世界模型）、`CurrentStateMonitor`（关节状态）、碰撞对象订阅、`get_planning_scene` 服务、`/monitored_planning_scene` 发布器。

---

## 3. 一句话速查

| 概念 | 一句话 |
|---|---|
| `MoveItCpp` | 装配场景、算法、执行三层的门面 |
| `PlanningSceneMonitor` | 场景中枢：模型 + 状态 + 障碍 + 服务 |
| `RobotModel` | 结构定义（只读、共享） |
| `JointModelGroup` | SRDF 定义的关节集合（引用关节对象） |
| `RobotState` | 某一时刻的变量取值 + FK 缓存（多份、独立） |
| 变量 | 状态数组里的一格；数量由关节类型决定 |
| `first_variable_index` | 关节第一个变量在整机数组中的起点 |
| `joint_variables_index_map_` | 名字 → 下标（整机版 / 组内版；关节内版叫 `variable_index_map_`） |
