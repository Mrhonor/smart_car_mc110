# 智能小车RK3399端ROS节点
## 项目说明
1. 该项目实现千乘智能小车MC110的RK3399端代码ROS化
2. 实现单目摄像头（小车读取摄像头测试通过）、realsense（未实现）、激光雷达（未实现）和uart通信（测试通过）ROS节点化
3. 去除原有的WIFI通信控制功能
4. 预计用roslaunch打包运行所有节点
5. 实现motion capture输出至ekf输入节点，目前用gazebo仿真数据。


## 节点说明

### **smart_car_main** 
1. 该节点实现小车控制和与STM32端串口通信功能
2. 通过***/dev/ttyS4***与STM32端通信，目前已控制小车通信通过
3. 打开方式
```
rosrun smart_car_mc110 smart_car_main
```

### **smart_car_single_eye** 
1. 该节点实现单目摄像头进行视觉检测车道线功能
2. 读取摄像头测试通过，运行车道线检测结果通过
3. 打开方式
```
rosrun smart_car_mc110 smart_car_single_eye
```
3. 可以通过订阅 ***/smart_car_mc110/single_eye/img*** 获取摄像头的视频输入
4. 可以通过订阅 ***/smart_car_mc110/single_eye/ctanSlop*** 获取计算出的曲率数据

### **smart_car_motion_capture** 
1. 该节点实现motion_capture和robot_pose_ekf通信功能
2. 目前处于gazebo仿真阶段
3. 打开方式
```
rosrun smart_car_mc110 smart_car_motion_capture
```