# 智能小车RK3399端ROS节点
## 项目说明
1. 该项目实现千乘智能小车MC110的RK3399端代码ROS化
2. 主分支实现接入uwb数据和motion capture数据
3. 主分支控制算法使用纯跟踪算法
4. 整体流程为接收传感器数据，使用[robot localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)库对位姿进行估计。根据估计位姿使用[纯跟踪算法](https://github.com/Mrhonor/Pure_Pursuit)求解控制向量，通过串口通信下发至STM32控制车辆运动。
5. 需要先将[Eigen库](https://gitlab.com/libeigen/eigen)和[json库](https://github.com/nlohmann/json)克隆下来。新建External文件夹，将Eigen和json库移至External文件夹下。

## 节点说明
### **smart_car_main** 
1. 该节点实现小车控制和与STM32端串口通信功能，核心通信节点
2. 运行方式
```
roslaunch smart_car_mc110 core.launch
```
3. 状态向量通过 **/EKF/State**发布，控制向量接收自 **/MPCC/Control**

### **smart_car_uwb** 
1. 该节点实现uwb传感器ROS消息的解包
2. 运行方式
```
roslaunch smart_car_mc110 uwb.launch
```

### **smart_car_motion_capture** 
1. 该节点实现motion_capture数据的解包
2. 打开方式
```
roslaunch smart_car_mc110 motion.launch
```

### 可以通过all.launch一起运行上述三个节点
```
roslaunch smart_car_mc110 all.launch
```

## 启动说明
1. 先启动传感器ROS节点
2. 启动robot localization节点
3. 启动纯跟踪算法节点
4. 运行all.launch
