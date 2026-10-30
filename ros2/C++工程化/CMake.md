1、CMake三个阶段
- configure：执行 CMakeLists.txt。find_package 在这里跑，产物是内存里的 imported target
- generate：计算每个 target 的最终命令行，写进 flags.make / link.txt
- build：g++ 编译（要 -isystem），ld 链接（要 .so）

g++ 解析 #include <rclcpp/rclcpp.hpp> 时的搜索顺序是:-isystem 列表 → -I 列表 → 系统默认目录(/usr/include、gcc 内置目录)

# 常见命令
1、find_package()


## 、install()
功能：构建（build）完后，将文件拷贝到指定位置；

|     |     |     |
| --- | --- | --- |
|     |     |     |
|     |     |     |



**参数**
TARGETS

## 、add_library()



# 功能对照

