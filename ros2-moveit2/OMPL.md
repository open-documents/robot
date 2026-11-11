ompl::base::RealVectorStateSpace 是 OMPL 里表示 $R^n$（n 维实数向量）的状态空间类。下面是创建 n 维状态空间实例：

```
std::make_shared<ompl::base::RealVectorStateSpace>(n);
```


不设会怎样
RealVectorStateSpace 的构造函数会创建 bounds_(dim)，而 RealVectorBounds(dim) 的内部 resize 把每一维的 low/high 都初始化成 0.0（不是 ±∞）



约束

```
ompl::base::ConstraintPtr cptr(std::make_shared<MoveItDualArmConstraint>(constraint));
```


ProjectedStateSpace 是 OMPL「解耦式约束规划」的两种流形状态空间之一

```
auto css  = std::make_shared<ompl::base::ProjectedStateSpace>(ambient, cptr);
```



css -> csi

```
auto csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(css);
```





ConstrainedSpaceInformation 构造时多做的两件事
源码（ConstrainedSpaceInformation.h）：


ConstrainedSpaceInformation(StateSpacePtr space) : SpaceInformation(std::move(space))
{
  stateSpace_->as<ConstrainedStateSpace>()->setSpaceInformation(this);   // ①
  setValidStateSamplerAllocator([](const SpaceInformation *si) {         // ②
    return std::make_shared<ConstrainedValidStateSampler>(si);
  });
}
① 反向关联 si

ConstrainedStateSpace 内部有个 si_ 裸指针，discreteGeodesic() 沿流形走每一步时要做碰撞检查：


auto &&svc = si_->getStateValidityChecker();
...
if (!(interpolate || svc->isValid(scratch))) break;
没有 setSpaceInformation(this)，si_ 就是空，css->setup() 直接抛异常：


Must associate a SpaceInformation object to the ConstrainedStateSpace via setStateInformation() before use.
这正是你 L68-69 注释写的情况。

② 换成约束专用采样器

普通 ValidStateSampler 只查 si_->isValid()；ConstrainedValidStateSampler 额外查约束：


sampler_->sampleUniform(state);
while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < attempts_);
注意这里调用的是 constraint_->isSatisfied() → 默认实现 → 你的 function()，所以又绕回了约束残差。



Constraint

```
Constraint 对外暴露的虚函数
├── function(x, out)      ← 必须重写（没有默认实现）
├── jacobian(x, out)      ← 可选，基类有数值差分默认版
├── project(x)            ← 可选，基类默认 = 牛顿法
├── distance(x)           ← 可选，基类默认 = ‖F(x)‖
└── isSatisfied(x)        ← 可选，基类默认 = ‖F(x)‖² ≤ tol²
```