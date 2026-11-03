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



把“右乘/左乘”这件事拆开讲
先纠正我上一条里一句含糊的话：“左乘会叠加在当前坐标系上”说反了。准确的规则是：

左乘 = 绕固定（世界）轴转；右乘 = 绕自身（当前）轴转。

下面从头推一遍，你就能看出 R = Rz·Ry·Rx 是怎么来的。

一、先定好约定
用列向量，R 把“子坐标系里的坐标”映射到“父坐标系”：


v_parent = R · v_child
复合旋转时，最右边的矩阵最先作用（矩阵乘法结合律不改变这个顺序）：


Rz · Ry · Rx · v   ≡   Rz · (Ry · (Rx · v))
                             ↑    ↑    ↑
                           最后  中间  最先
所以 R = Rz·Ry·Rx 读成：先执行 Rx，再执行 Ry，最后执行 Rz。

二、固定轴旋转为什么是“左乘”
假设当前总旋转是 R（一开始是单位阵 I），现在要绕世界坐标系里那根固定的轴 a 转 θ 度。

关键点：轴 a 是用世界坐标写死的，它不随物体转动而改变。所以无论物体此刻转到哪，这次旋转在世界坐标里都表示为同一个矩阵 Rot(a, θ)。把它作用在已经转好的姿态上，就是直接左乘：


R_new = Rot(a, θ) · R
于是 URDF 的 fixed-axis XYZ 顺序：


初始:          R = I
绕世界 X 转 roll:   R ← Rx(roll) · I        = Rx
绕世界 Y 转 pitch:  R ← Ry(pitch) · Rx       = Ry·Rx
绕世界 Z 转 yaw:    R ← Rz(yaw) · Ry · Rx    = Rz·Ry·Rx
每一步都是左乘，所以后转的写在左边，最终是 Rz·Ry·Rx。

这就是 URDF 里 <origin rpy="roll pitch yaw"> 的含义，也是 tf2 Quaternion.setRPY() 的定义：“roll 绕固定 X，pitch 绕固定 Y，yaw 绕固定 Z”。

三、绕自身轴转为什么变成“右乘”
假设物体当前已经有姿态 R，现在要绕物体自己的轴 a（比如它自己的 Y 轴）转 θ。

这个轴在世界坐标里已经不是 a 了，而是被 R 转过的 R·a。绕世界轴 R·a 转 θ 的矩阵是：


Rot(R·a, θ) = R · Rot(a, θ) · R⁻¹
把它作用在当前姿态上：


R_new = Rot(R·a, θ) · R
      = R · Rot(a, θ) · R⁻¹ · R
      = R · Rot(a, θ)
R⁻¹·R 抵消，变成了右乘。所以 intrinsic X→Y→Z：


绕自身 X:  R = Rx
绕自身 Y:  R = Rx·Ry
绕自身 Z:  R = Rx·Ry·Rz
得到的是 Rx·Ry·Rz，和固定轴的 Rz·Ry·Rx 不是一回事。

四、两者差多少：数字说话
取 roll = 90°, pitch = 0, yaw = 90°，作用在同一个向量 e_z = (0,0,1) 上：

顺序	计算	结果
Rz·Ry·Rx（固定轴，URDF）	先绕世界 X 转 90°，再绕世界 Z 转 90°	(1, 0, 0)
Rx·Ry·Rz（自身轴）	先绕自身 Z 转 90°，再绕自身 X 转 90°	(0, -1, 0)
完全是两个方向。

再用你前面那个真实 rpy (0.007602, 0.023780, 0.809016)，对 e_z 作用：

顺序	结果
Rz·Ry·Rx	(0.021912, 0.011958, 0.999688)
Rx·Ry·Rz	(0.023778, -0.007600, 0.999688)
因为 roll/pitch 很小，差异不大，但确实不同——顺序写反不会报错，只会悄悄算错，这是四元数/旋转矩阵 bug 的常见来源。

五、一个有用的等价关系
同一组角度下：


extrinsic X-Y-Z  ≡  intrinsic Z-Y-X
Rz·Ry·Rx        ≡  Rz·Ry·Rx
也就是说，R = Rz·Ry·Rx 可以有两种读法，矩阵完全一样：

固定轴读法：先绕世界 X 转 roll，再绕世界 Y 转 pitch，再绕世界 Z 转 yaw
自身轴读法：先绕自身 Z 转 yaw，再绕新的 Y 转 pitch，最后绕最新的 X 转 roll
这是欧拉角表示的一个经典恒等式，可以帮你检查自己的理解：如果某本书写“intrinsic XYZ = Rx·Ry·Rz”，那它对应的 extrinsic 就是 Z-Y-X，不是 X-Y-Z。

六、记忆口诀
旋转方式	矩阵写法	口诀
绕固定轴，执行顺序 X→Y→Z	Rz·Ry·Rx	后转的写在左边
绕自身轴，执行顺序 X→Y→Z	Rx·Ry·Rz	先转的写在左边
URDF 的 rpy 属于第一种，所以必须是 Rz(yaw)·Ry(pitch)·Rx(roll)。

一句话总结：R = Rz·Ry·Rx 不是因为“左乘叠加在当前系”，恰恰相反——因为这三根轴始终是指向世界坐标的固定轴，每次旋转都不需要变换轴，直接左乘上去，于是最后转的跑到了最左边。绕自身轴才需要把轴用当前姿态 R 变换一次，那个 R⁻¹·R 的抵消把乘法推到了右边。