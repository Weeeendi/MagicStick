## 家居控制魔棒



### 功能描述

* 可以通过动作进行家居控制
* 配合beacon定位模组实现，实现指定设备控制（待实现）



### 需求组件

- MPU 9250

- 蓝牙或wifi模组连接 tuya App

  

### 实现原理

#### 1.姿态算法

根据九轴运动传感器实现魔棒的姿态解算



#### 2.运动轨迹算法
基础功能：判断上下左右动作

高级功能：需要对运动轨迹进行分析是否符合某种图形，一种图形对应一个设备  △，□，○等

难点：高级功能需要研究下是否有好的AI框架（设备上传轨迹  手机来分析，考虑到芯片计算能力问题，不建议设备本身来算）



#### 3.联动控制

连接到物联网平台，并且该设备应该是有网关权限的，因为他可以联动控制其他智能家居，所以需要拿到家庭下的其他家居的列表，并且可以自由编译图形所对应的家居类型

当实现功能 2 后，对平台的要求是需要家庭空间的管理能力，来根据空间相对位置控制指定的家居产品







