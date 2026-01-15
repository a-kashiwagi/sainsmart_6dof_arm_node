# Sainsmart 6dof Arm Node for ROS1
A ros1 node of six dof arm for sainsmart

## 概要

ROS1 用に作成した Sainsmart 6dof ARM Node です。この Sainsmart の ARM は有名ですが、なぜか ROS 用の Node が見当たらなかったので作成しました。ROS1 用 Work space に Package を作成して使用するように組んでいます。

![Sainsmart 6dof Arm](https://github.com/wedesoft/arduino-sainsmart/raw/master/6axis-size.jpg)

## 位置制御
Joint は、サーボモータで駆動しているため、位置制御となりますが、サーボに搭載されているポテンショメータ（ロータリーエンコーダー）以外に、Joint 角度を取得するセンサーは搭載していない為、/joint_states を subscribe して、joint を動かす push 型制御をしております。

```cpp
                                                   // Subscribe to /joint_states topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 10, jointStateCallback);
```

## Elbow Joint について
実機では、Elbow（肘）Joint は、直接サーボモータが付いておらず、別 Link を介して駆動する為、コード上で角度を調整しています。

```cpp
                                       // Adjustment (degree)
double adjust[6] = {
     0,                                // For joint 1 
     0,                                // For joint 2
  -110,                                // For joint 3
     0,                                // For joint 4
    30,                                // For joint 5
     2                                 // For joint 6
};
```
```cpp
        }else if( strcmp(buffer,"joint3") == 0){
                                       // for joint3
            joint[2] = (RadToDeg(msg->position[i]) + adjust[2]) - joint[1];
```

## Shoulder Joint について
同じように、Shoulder（肩）Joint は、回転方向を反転しています。

```cpp
        }else if( strcmp(buffer,"joint2") == 0){
                                       // for joint2
            joint[1] = (RadToDeg(msg->position[i]) + adjust[1]) * INVERT;
```

## 通信速度
Arduino UNO との通信速度は、115200[bps] まで上げると、安定した動作に支障を来す為、19200[bps] をお勧めします。

```cpp
                                       // Set serial speed
    cfsetospeed(&tty, SERIAL_SPEED);
    cfsetispeed(&tty, SERIAL_SPEED);
```


**以上**
