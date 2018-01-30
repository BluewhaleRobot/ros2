# ROS2和不同的DDS程序
ROS2是建立在DDS程序的基础上的。DDS程序被用来发现节点，序列化和传递信息。
[这篇文章](http://design.ros2.org/articles/ros_on_dds.html)详细介绍了DDS程序的开发动机。
总而言之，DDS程序提供了ROS系统所需的一些功能，比如分布式发现节点(并不是像ROS1那样中心化)，控制传输中的不同的"通信质量（Quality of Service）"选项。

DDS是一个被很多公司实现的工业标准。比如RTI的实现[Connext]((https://www.rti.com/products/)和eProsima的实现[Fast RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps)。
ROS2 支持多种实现方式。因为没有必要“一个尺码的鞋给所有人穿”。用户有选择的自由。
在选择DDS实现的时候你要考虑很多方面：比如法律上你要考虑他们的协议，技术上要考虑是否支持跨平台。
不同的公司也许会为了适应不同的场景提出不止一种的DDS实现方式。
比如RTI为了不同的目标就有很多种他们的Connext的变种。从小到微处理器到需要特殊资质的应用程序（我们支持标准的桌面版）。

为了能够在ROS2中使用一个DDS实现，需要一个ROS中间件(RMW软件包), 这个包需要利用DDS程序提供的API和工具实现ROS中间件的接口。
为了在ROS2中使用一个DDS实现，有大量的工作需要做。但是为了防止ROS2的代码过于绑定某种DDS程序必须支持至少几种DDS程序。因为用户可能会根据他们的项目需求选择不同的DDS程序。

### 支持的RMW实现

| 名称 | 协议 | RMW 实现 | 状态 |
| ------------- | ------------- | ----- | ---- |
| eProsima _Fast RTPS_ | Apache 2 | `rmw_fastrtps_cpp` | 完全支持. 默认的RMW. 已经打包在发布的文件中. |
| RTI _Connext_ | commercial, research | `rmw_connext_cpp` | 完全支持. 需要从源码编译支持. |
| RTI _Connext_ (dynamic implementation) | commercial, research | `rmw_connext_dynamic_cpp` | 停止支持. alpha 8.* 之前版本完全支持|
| PrismTech _Opensplice_ | LGPL (only v6.4), commercial | `rmw_opensplice_cpp` | 停止支持. alpha 8.* 之前版本完全支持|
| OSRF _FreeRTPS_ | Apache 2 | -- | 部分支持. 开发暂停. |

_*暂停支持意味着从 ROS2 alpha 8 版本以后新的添加进ROS2的功能还没有添加到这些中间件的实现中来。
这些中间件的实现也许以后会会有也许以后也不会有。_

对于如何同时使用多个RMW实现的方法，可以看[[使用多个RMW实现]]页面
