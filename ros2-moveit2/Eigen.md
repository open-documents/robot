

```
Eigen::Ref
```



```
Eigen::VectorXd
Eigen::Matrix4d
      arm.T_wrist3_tool.linear() = M.topLeftCorner<3, 3>();
      arm.T_wrist3_tool.translation() = M.topRightCorner<3, 1>();  
```