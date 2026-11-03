
URDF 提供的是运动学拓扑 + 每个关节的固定安装位姿

# 左乘、右乘
定义符号如下：

| 符号 | 含义 |
| :---: | :---: |
| $R$、$R_{F \leftarrow body}$ | 映射：将 body 系映射到 F 系 |
| $a$ | body 系中的某坐标轴 a |
| $R \cdot a$ | F 系中的某坐标轴 a（a 经过 R 变换） |
| $Rot\_body(a, \theta)$ | 在 body 系，绕轴 a 转 θ 的矩阵 |
| $Rot\_F(R \cdot a, \theta)$ | 在 F 系，绕轴 $R \cdot a$ 转 θ 的矩阵 |
|  |  |

# 一、关节
关节用于连接父子连杆。URDF 文件中通过 <joint> 标签定义关节，本章介绍关节标签的属性和子标签。
```xml
<joint name="ur5e_left_shoulder_pan_joint" type="revolute">
  <origin xyz="0 0 0.16249999" rpy="0 0 -3.14159257"/>
  <parent link="ur5e_left_base_link"/>
  <child link="ur5e_left_shoulder_link"/>
  <axis xyz="0 0 1"/>
  <limit effort="150.0" lower="-6.28318531" upper="6.28318531" velocity="3.14159265"/>
  <dynamics friction="0" damping="0"/>
</joint>
```
## 1、关节类型（type 属性）
关节类型决定父子连杆间相对运动的方式，通过关节类型结合关节数据，确定子连杆到父连杆的变换关系。URDF 文件中通过 <joint> 标签的 type 属性定义关节类型，如下所示：
```xml
<joint type="...">
```

在 URDF 规范中，共 6 种关节类型：

| 关节类型 type | 自由度 | 运动 |
| :---: | :---: | :---: |
| fixed | 0 | 完全刚性连接 |
| revolute | 1 | 绕单轴转动，有角度上下限 |
| continuous | 1 | 绕单轴转动，无角度限制 |
| prismatic | 1 | 沿单轴平移，有位移上下限 |
| planar | 3 | 平面内平移 + 绕平面法线转动 |
| floating | 6 | 任意平移 + 旋转 |

## 2、父子连杆变换关系（origin、axis 标签）
父子连杆变换关系通过 <origin>、<axis> 标签内容配合实时关节值共同确定：
- <origin>：定义关节在零位时，关节坐标系（即子连杆坐标系）在父连杆坐标系中的位姿。描述子连杆相对于父连杆的固定变换；
- <axis>：定义关节坐标系中关节的运动轴。该标签只对可动关节（revolute、continuous、prismatic）有意义，指明关节绕哪根轴转或沿哪根轴移；

基于此，子连杆坐标系下的坐标到父连杆坐标系的变换矩阵如下：
$$
T_{parent←child}(q) = T_{origin}(xyz, rpy) · T_{joint}(axis, q)
$$
其中：
$$$$
- $T_{joint}(axis, q)$：负责描述关节实际运动带来的变换。axis 定义了运动的方向（比如绕 X/Y/Z 轴转动），q 是关节变量（旋转关节是角度，移动关节是距离）。
- $T_{origin}(xyz, rpy)$：负责描述关节安装位置的固定偏移。它通过平移 xyz 和旋转 rpy，确立了关节坐标系相对于父连杆坐标系的原点位置。



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

其中


R = Rz(yaw) · Ry(pitch) · Rx(roll)

          1    0       0    
Rx(r) =   0  cos r   -sin r       绕 X 轴转 r (roll)
          0  sin r    cos r 

          cos p   0   sin p 
Ry(p) =     0     1     0         绕 Y 轴转 p (pitch)
         -sin p   0   cos p ┘

        ┌ cos y  -sin y   0 ┐
Rz(y) = │ sin y   cos y   0 │      绕 Z 轴转 y (yaw)
        └   0       0     1 ┘



# 附录
$$T_{origin}(xyz, rpy)=
\begin{bmatrix}
R(rpy) & t(xyz) \\
0 & 1  \\
\end{bmatrix}$$

## 1、

$$
Rx(a) = 
\begin{bmatrix}
1 & 0      & 0 \\
0 & cosa & -sina  \\
0 & sina  & cosa
\end{bmatrix}
\quad
Ry(b) = 
\begin{bmatrix}
cosb & 0   & sinb \\
0       & 1  & 0  \\
-sinb & 0  & cosb
\end{bmatrix}
\quad
Rz(c) = 
\begin{bmatrix}
cosc & -sinc  & 0 \\
sinc  & cosc  & 0  \\
0      & 0       & 1
\end{bmatrix}
$$
