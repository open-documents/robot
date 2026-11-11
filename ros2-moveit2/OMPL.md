ompl::base::RealVectorStateSpace 是 OMPL 里表示 $R^n$（n 维实数向量）的状态空间类。下面是创建 n 维状态空间实例：

```
std::make_shared<ompl::base::RealVectorStateSpace>(n);
```


不设会怎样
RealVectorStateSpace 的构造函数会创建 bounds_(dim)，而 RealVectorBounds(dim) 的内部 resize 把每一维的 low/high 都初始化成 0.0（不是 ±∞）


ProjectedStateSpace 是 OMPL「解耦式约束规划」的两种流形状态空间之一



