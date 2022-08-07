代码中的header.name应该对应无人机1号与2号

## 1. 用到的Msgs:

### 1.1 ROS自带的Sensor消息类型

Ros sensor_msgs/image: 

> http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Image.html

Ros sensor_msgs/Camera_info

> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html



### 1.2 Unity 所传输的传感器消息:

```c++
enum UnityMessageType : uint32_t {
  UNITY_STATE = 0,
  UNITY_CAMERA,
  UNITY_IMU,
  UNITY_DEPTH,
  UNITY_FISHEYE,
  UNITY_DETECTIONS,
  MESSAGE_TYPE_COUNT
};
```

这些消息统一由类UnityStreamParser 以 TCPIP stream的形式发送



### 1.3 ROS 接受无人机消息用的LCM消息传递系统

LCM: Lightweight Communications and Marshalling

>https://www.guyuehome.com/35912

## 2.用到的类

### 2.1 用于Unity与ROS通信的TCPIP类：

TCPStreamReader: 定义 Host以及端口port， 以及提供消息serverstream的server， 提供函数可阅读传输的各个数据类型消息以及判断连接稳定性。

> TCPStreamReader.h 与TCPStreamReader.cpp

TCPImageServer: 通过TCPIP传输机制接受来自Unity的图片，包含图片的长宽，接收时间戳，也可根据flag m_flap翻转/不翻转接收到的图片

> TCPImageServer.cpp和TCPImageServer.h



libsocket相关头文件（不需要修改）

### 2.2 接受Unity的传感器消息并发布到对应无人机底下相关话题的类

#### 2.2.1 RGBCameraParser(rgb_camera_parser.h)

从TCPIP消息流中读取 其中一个无人机的RGB消息， 包含timestamp, img 和时间间隔（不确定有什么用） time_offset， 同时可以获得 field of view (GetFieldOfView函数) 和Max Perception Range (GetMaxRange) 

节点中的image_publisher 和CameraInfoPublisher 同样是map类型的publisher(无人机编号+对应ros::publisher)

- image_publisher:

  将TCPIP消息中的图片相关消息转化成ros可读懂的图片消息类型，并获取图片的长宽高，时间戳等（相见ros image消息类型Ros sensor_msgs/image:  ），并发布到对应无人机底下的 “image_raw话题”

- Camera info publisher:

  将TCPIP消息的包含的相机参数消息转化为Ros sensor_msgs/Camera_info， 会用到的相机内参： 焦距fx.fy， 图片中心 cx, cy

#### 2.2.2 IMU_Parser(imu_parser.h)

获取无人机的角速度（gx,gy,gz）以及线加速度(ax,ay,az), 由于Unity采用左手坐标系所以对xy方向的数据进行了交换

#### 2.2.3 fisheye_camera_parse(fisheye_camera_parse.h)

不知道有什么用

#### 2.2.4 depth_camera_parser (depth_camera_parser.h)

继承RGB Camera sensor，保持相机内参不变，将得到的RGB图片处理为深度图片 	



 ## 3. 节点

### 3.1 unity_ros(Unity_ros.cpp)

unity_ros： Unity -> ros:  IP地址： 127.0.0.1, port : 9998

ros 计算得出的命令到unity -> ip地址 127.0.0.1 port: 9999

依靠 magic == 0xDEADC0DE 判断消息流的稳定性

用一个指针shared_ptr向量保存各个类型的Unity传感器消息，



### 3.2 TrueStateParser(true_state_parser.h):

从Unity传出的消息流中读取

- px py pz 无人机位置
- qx, qy, qz, qw 无人机位姿(quternion)
- vx, vy, vz  无人机速度
- rx, ry, rz 无人机的旋转角速度

这些数据都是连续的float消息，每个占4个byte的带宽

这些接受的到的Unity中的无人机真实位置消息会用transformstamped 复制给true_body相对world坐标的位姿。





node 里面定义了两个publisher（map类型(unity stream header 与 ros::publisher)

- pose_publisher将接受到的无人机相对坐标body的 posemsg(x,y, z, qx,qz,qy,qw) advertise给无人机的pose话题
- twist_publisher将接收到的无人机相对body坐标的速度角速度传给无人机的twsit话题



### 3.3 StateEstimateCorruptorNode (state_estimate_corruptor_node.cpp)

Node Publisher:   发布噪声处理过的状态

- Pose_pub : to topic: “/pose_est”
- velocity_pub: to topic: "/twist_est"
- state_pub: to topic: "/current_state_est"

Node Subscriber:

- subscribe pose and twist from "ture body" 

在subscriber 的callback function中：

1. 对于true body的x,y,z 基于伯努里概率赋予currupted_pose(of "body") 漂移位移（相对上一次受到的无人机pose的位移）和白噪声（随机产生的符合高斯分布的随机数），处理的结果为body的x,y,z

2. 对于true body的角速度不做处理赋给body，线速度基于漂移量赋予一定白噪声

3. 将corrupted twist和pose传给body 作为其state(相对世界坐标world，消息类型: nav_msgs::Odometry) 

### 3.4 w_to_unity(w_to_unity.cpp)

subscribe topic: pose_topic (相对ture_body坐标)

将接受到的mav_msg::Actuator 消息类型（包含对无人机四个螺旋桨的转速控制(Unit: rad/s)）通过127.0.0.1 的12346端口传回给unity





## 4. Controller Node

### 4.1 ControllerNode: 

1. ros::subscriber: desired_state: 

   - xd: desired position in the world frame (vector3d)

   - vd: desired velocity in world frame (vector3d)

   - ad: desired acceleration in world frame (vector3d)

   - 监听话题： desired_stage

2. ros::subscriber current_state

   - x current position (vector3d)

   - v current velocity (vector3d)

   - R current orientation (Matrix3d)

   - omega curretn angular velocity (vector 3d)

   - 监听话题: current_state_est



3. ros::publisher prop_speed

   - 根据期望值与实际值的差值(ex, ev)计算无人机的body frame (x, y, z 轴)
   - 计算body frame的朝向与实际朝向的无误差
   - 根据前面所得的误差项计算无人机的torque和wrench，并球的螺旋桨转速(rad/s)

   - 将螺旋桨转速发布到话题 rotor_speed_cmds



### 4.2 trajectory planner

目前的期望轨迹是个圆，发布到话题desired_state上
