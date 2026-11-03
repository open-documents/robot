URDF 提供的是运动学拓扑 + 每个关节的固定安装位姿
URDF <joint type="..."> 的作用
type 决定 父子连杆之间允许哪种相对运动，以及该关节有哪些子标签（axis/limit/dynamics）。
URDF 规范共 6 种：
type	自由度	运动	常见场景	本工程
fixed	0	完全刚性连接	传感器安装、工具法兰、底盘挂载	4 个
revolute	1	绕单轴转动，有角度上下限	机械臂关节、舵机	12 个
continuous	1	绕单轴转动，无角度限制（可无限转）	车轮、转台	0
prismatic	1	沿单轴平移，有位移上下限	滑轨、升降台	0
planar	3	平面内平移 + 绕平面法线转动	少见	0
floating	6	任意平移 + 旋转	自由飞行物体	0
相关子标签
<joint name="ur5e_left_shoulder_pan_joint" type="revolute">
  <origin xyz="0 0 0.16249999" rpy="0 0 -3.14159257"/>
  <parent link="ur5e_left_base_link"/>
  <child link="ur5e_left_shoulder_link"/>
  <axis xyz="0 0 1"/>                <!-- 转动/平移轴，在关节坐标系下 -->
  <limit effort="150.0" lower="-6.28318531" upper="6.28318531" velocity="3.14159265"/>
  <dynamics friction="0" damping="0"/>
</joint>
子标签	作用	哪些 type 需要
<origin>	父 link → 关节坐标系的固定变换；也是 q=0 时子 link 的位姿	全部
<parent>/<child>	树结构	全部
<axis>	运动轴；缺省 (1,0,0)	revolute / continuous / prismatic / planar
<limit>	lower/upper（角度或米）、effort、velocity	revolute / prismatic 必需；continuous 只需 effort/velocity
<dynamics>	摩擦、阻尼	可选
<mimic>	跟随另一个关节（如夹爪双指联动）	可选
<calibration>	零点标定	可选



<joint> 的 <origin> 语义是：在关节零位时，child 坐标系相对 parent 坐标系的位姿。用齐次矩阵写就是：
        ┌           ┐
T_p_c = │  R(rpy)  t(xyz) │
        │    0      1     │
        └           ┘


        t = [x, y, z]ᵀ 是平移，表达在 parent 坐标系里
R(rpy) 是旋转，URDF 规定是绕固定轴 X→Y→Z 依次转（extrinsic），等价于矩阵乘法顺序：

R = Rz(yaw) · Ry(pitch) · Rx(roll)



左臂安装位姿为例：


xyz = (-0.128064, 0.237478, -0.746870)
rpy = (0.007602, 0.023780, 0.809016)
计算流程（rsp 内部是 KDL 做的，原理一致）：

由 rpy 构造旋转矩阵 R = Rz(0.809016)·Ry(0.023780)·Rx(0.007602)
由 R 转四元数，常用公式（迹法）：

tr = r11 + r22 + r33
qw = 0.5·√(1 + tr)
qx = (r32 - r23) / (4·qw)
qy = (r13 - r31) / (4·qw)
qz = (r21 - r12) / (4·qw)
这个 yaw≈46.36° 的位姿，四元数大约 (0.0039, 0.0116, 0.3934, 0.9193)。

3. 平移直接取 xyz，不做旋转（t 本来就在 parent 系里）。

注意这里的转换顺序 先旋转再平移：如果写成 T = R·T(t)（先平移后旋转），结果就错了——平移会被 R 跟着转。正确的组合是 T = T(t)·R
1. 
其中，URDF 的 rpy 是 extrinsic X-Y-Z：先绕世界固定的 X 轴转 roll，再绕固定的 Y 轴转 pitch，最后绕固定的 Z 轴转 yaw。写成矩阵就是右乘（因为每次左乘都会叠加在“当前”坐标系上）：


R = Rz(yaw) · Ry(pitch) · Rx(roll)
```
          1    0       0    
Rx(r) =   0  cos r   -sin r       绕 X 轴转 r (roll)
          0  sin r    cos r 

          cos p   0   sin p 
Ry(p) =     0     1     0         绕 Y 轴转 p (pitch)
         -sin p   0   cos p ┘

        ┌ cos y  -sin y   0 ┐
Rz(y) = │ sin y   cos y   0 │      绕 Z 轴转 y (yaw)
        └   0       0     1 ┘
