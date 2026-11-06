本文介绍机器人领域的 URDF 文件。URDF 文件描述机器人拓扑、运动学参数（link/joint/origin/axis/limit/collision）。

# 左乘、右乘
定义符号如下：

| 符号 | 含义 |
| :---: | :---: |
| $R$、$R_{F \leftarrow body}$ | 映射：将 body 系映射到 F 系 |
| $a$ | body 系中的某坐标轴 a |
| $R \cdot a$ | F 系中的某坐标轴 a（a 经过 R 变换） |
| $Rot\_body(a, \theta)$ | 在 body 系，绕轴 a 转 θ 的矩阵 |
| $Rot\_F(R \cdot a, \theta)$ | 在 F 系，绕轴 $R \cdot a$ 转 θ 的矩阵 |

已知坐标系 body 下绕轴 a 旋转 θ 的变换矩阵为 $Rot_{body}(a, θ)$，那么在坐标系 F 下绕 a 旋转 θ 的变换矩阵为：$Rot_F(R·a, θ) = R · Rot_{body}(a, θ) · R⁻¹$
证明：对 F 系里任意向量 v，做下面操作：
- $R⁻¹ · v$：把 $v$ 从 F 坐标系映射到 body 坐标系；
- $Rot_{body}(a,θ) · (R⁻¹ v)$：在 body 坐标系中绕 a 转 θ；
- $R · (...)$：将坐标映射回 F 坐标系；

# 一、关节
关节用于连接父子连杆。URDF 文件中通过 <joint> 标签定义关节，本章介绍关节标签的属性和子标签。
```xml
<joint name="..." type="revolute|continuous|prismatic|planar|floating|fixed">
  <origin xyz="..." rpy="..."/>
  <parent link="..."/>
  <child link="..."/>
  <axis xyz="..."/>
  <calibration .../>
  <dynamics damping="..." friction="..."/>
  <limit lower="..." upper="..." effort="..." velocity="..."/>
  <safety_controller .../>
  <mimic joint="..." multiplier="..." offset="..."/>
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



## prismatic —— 沿单轴平移，有限位

定义：沿 <axis> 方向直线滑动，位移被 \[lower, upper\] 限制（单位：米）。

<joint name="lift_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="1.2" velocity="0.5" effort="500.0"/>
</joint>


变换：

$$T(q) = \mathrm{Trans}(\hat a \cdot q)$$


## planar —— 平面内 3 自由度

定义：在垂直于某轴（URDF 约定为 z）的平面内做二维平移 + 绕该轴旋转，共 3 DOF。
```xml
<joint name="base_planar_joint" type="planar">
  <parent link="world"/>
  <child link="base_link"/>
  <axis xyz="0 0 1"/>
</joint>
```


变换：

$$
T(q) = \mathrm{Trans}(x, y, 0) \cdot \mathrm{Rot}(z, \theta), \quad q = [x, y, \theta]
$$
## 2、父子连杆变换关系（origin、axis 标签）
父子连杆变换关系通过 <origin>、<axis> 标签内容配合实时关节值共同确定：
- <origin>：定义关节在零位时，关节坐标系（即子连杆坐标系）在父连杆坐标系中的位姿。描述子连杆相对于父连杆的固定变换；
- <axis>：定义关节坐标系中关节的运动轴。该标签只对可动关节（revolute、continuous、prismatic）有意义，指明关节绕哪根轴转或沿哪根轴移；

基于此，子连杆坐标系下的坐标到父连杆坐标系的变换矩阵如下：
$$
T_{parent←child}(q) = T_{origin}(xyz, rpy) · T_{joint}(axis, q)
$$
其中（具体公式定义见附录）：
- $T_{origin}(xyz, rpy)$：负责描述关节安装位置的固定偏移。它通过平移 xyz 和旋转 rpy，确立了关节坐标系相对于父连杆坐标系的原点位置；
- $T_{joint}(axis, q)$：负责描述关节实际运动带来的变换。axis 定义了运动的方向（比如绕 X/Y/Z 轴转动），q 是关节变量（旋转关节是角度，移动关节是距离）；









# 附录
## 1、固定变化
在关节零位时，子连杆到父连杆的固定变换 $T_{origin}(xyz, rpy)$，如下：
$$
T_{origin}(xyz, rpy)=
\begin{bmatrix}
R(rpy) & \mathbf{t(xyz)} \\
\mathbf{0}^T & 1  \\
\end{bmatrix}
$$
1）$R(rpy)$ 
$$
R(rpy) = R_z(yaw) \cdot R_y(pitch) \cdot R_x(roll)
$$
$$
R_x(a) = 
\begin{bmatrix}
1 & 0      & 0 \\
0 & cosa & -sina  \\
0 & sina  & cosa
\end{bmatrix}
\quad

R_y(b) = 
\begin{bmatrix}
cosb & 0   & sinb \\
0       & 1  & 0  \\
-sinb & 0  & cosb
\end{bmatrix}
\quad

R_z(c) = 
\begin{bmatrix}
cosc & -sinc  & 0 \\
sinc  & cosc  & 0  \\
0      & 0       & 1
\end{bmatrix}
\\

$$

记为：
$$
R(rpy) = 
\begin{bmatrix}
r_{11} & r_{12}  & r_{13}  \\
r_{21} & r_{22}  & r_{23}  \\
r_{31} & r_{32}  & r_{33}
\end{bmatrix}
$$
2）$t(xyz) = [x, y, z]^T$

## 2、关节变换
关节变换 $T_{joint}(axis, q)$，定义如下：
$$
T_{joint}(axis, q) = \begin{bmatrix}
R(axis, q) & \mathbf{0} \\
\mathbf{0}^T & 1
\end{bmatrix}
$$
1）绕标准轴 x 旋转的矩阵
$$
T_{joint}(X, q) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & \cos q & -\sin q & 0 \\
0 & \sin q & \cos q & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
2）绕标准轴 y 旋转的矩阵
$$
T_{joint}(Y, q) = \begin{bmatrix}
\cos q & 0 & \sin q & 0 \\
0 & 1 & 0 & 0 \\
-\sin q & 0 & \cos q & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
3）绕标准轴 z 旋转的矩阵
$$
T_{joint}(Z, q) = \begin{bmatrix}
\cos q & -\sin q & 0 & 0 \\
\sin q & \cos q & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

4）任意单位向量轴的旋转矩阵（罗德里格斯公式）
$$
R(axis, q) = \begin{bmatrix}
n_x^2(1-\cos q)+\cos q & n_x n_y(1-\cos q)-n_z\sin q & n_x n_z(1-\cos q)+n_y\sin q \\
n_y n_x(1-\cos q)+n_z\sin q & n_y^2(1-\cos q)+\cos q & n_y n_z(1-\cos q)-n_x\sin q \\
n_z n_x(1-\cos q)-n_y\sin q & n_z n_y(1-\cos q)+n_x\sin q & n_z^2(1-\cos q)+\cos q
\end{bmatrix}
$$

