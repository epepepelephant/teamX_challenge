# 思想道德与法治 校内赛

## 技术报告
[语雀报告]https://www.yuque.com/yuqueyonghu0v3vd9/yq33pp/egm8l67b2pydo5he?singleDoc#

## 结构

### 功能代码
[功能包目录](./src/teamx_challenge)
```
src
└── teamx_challenge
    ├── codewhilecompeting #比赛时候的代码的目录
    ├── CMakeLists.txt  
    ├── include
    │   └── teamx_challenge
    ├── launch
        └── vision.launch.py 
        └── shooter.launch.py  #集成两个shooter和vision两个节点
    ├── numbers  # 存放模板
    │   ├── 1.jpg
    │   ├── 2.jpg
    │   ├── 3.jpg
    │   ├── 4.jpg
    │   └── 5.jpg
    ├── package.xml
    └── src
        ├── shooter_node.cpp
        └── vision_node.cpp
```

### 误差结果
[裁判系统运行结果](./results/)

## 算法原理

### vision_node.cpp
* 通过订阅相机的图像，检测了装甲板，球体，长方体，完成了识别数字和帧间匹配（跟踪目标）
* 创建了装甲板、球体、长方体等类，用于存储相关信息，并在TestNode中用armor_list等vector数组来保存了每一帧的图像信息，同时也在每一帧的开始清空
* 所有函数都集成在了TestNode这个节点当中（先声明后完成），通过这些函数实现不同的功能
* 在回调函数最后利用armor_list等vector数组中信息构建消息并发布给裁判系统评分

#### 目标检测
* 颜色筛选 + 形态学处理 + 筛选每一个形状对应的特征 + 记录在vector数组里面 + 最后的消息输出

#### 装甲板识别
* 筛选出装甲板两边的红色灯条 + 进行灯条匹配构造装甲板 + 记录灯条外侧点作为发送给裁判系统的点

#### 数字识别
* 总体思路为模板匹配
* 选择模板匹配的理由：仿真环境下只要能获得数字图像就比较容易实现模板匹配，没有必要采用机器学习加大工程量
* 通过灯条的位置相对扩充，获得装甲板图像 + 仿射变换 + 图像预处理获得白色的区域（数字图像）+模板匹配（归一法）

#### 动态跟踪（帧间匹配）
* 对比上一帧和现在帧的图像距离和面积等特点，确定某一装甲板是不是同一个目标


### shooter_node.cpp
* 订阅 /vision/target 获取视觉节点的信息
* 接受 /referee/hit_arror 的请求
* 利用图像信息，摄像头内参，请求中的装甲板数据解pnp，获得到装甲板相对camera_robot的距离，通过一定处理，获得装甲板的三维坐标
* 计算欧拉角 pitch = atan2(y/x)
* yaw通过抛体运动进行计算，即二次方程求根公式，若有低抛解，则选取低抛


## 运行指令
[常用指令](./常用指令.txt)

## 依赖项
ROS 2 Humble
Gazebo Ignition Fortress
OpenCV
C++17 

## 在docker上运行
[vision_node要替换的相关行](/src/teamx_challenge/src/vision_node.cpp#L705-L708)然后运行docker-compose111.yml即可
string template_path = "/home/va/templates/";
  for (int i = 1; i <= 5; i++)
  {
    string pathi = template_path + to_string(i) + ".jpg";