# 使用多个ROS2中间件的实现
这篇文章详细说明了默认的ROS2中间件实现和如何选择其他的中间件。

## 开始之间
你需要已经读过[[DDS和ROS中间件]]这篇文章。

## 多个RMW实现
当前的ROS二进制包发布版本已经支持了一种RMW实现（FastRTPS）由于其他支持的RMW无法随意的发布。

尽管ROS的二进制发布版本只支持一种RMW实现。一个从源码编译的工作空间，可以用来同时编译安装多个RMW实现。
当ROS2的核心代码编译的时候，如果有任何的RMW已经在电脑上安装配置完成，那么对应的RMW也会被同时编译进去。
例如如果[Fast RTPS的RMW 软件包](https://github.com/ros2/rmw_fastrtps_cpp)在工作空间中，当发现
已经安装过Fast RTPS时，这个软件包也会被同时编译。

在很多情况下你会发现使用不同的RMW实现的节点是能够通信的。然而并不是在所有情况下都成立。
将来会列出能够在不同RWM实现件通信的配置列表。

## 默认的RMW实现

如果一个ROS2的工作空间有多个RMW实现，那么如果里面有Fast RTPS，那么它就会被作为默认实现。
如果没有Fast RTPS，那就会按照字母顺序选择默认的RMW实现。
这个字母顺序是按照提供RMW实现的ROS包的名字来的。
比如如果同时安装有`rmw_opensplice_cpp`和`rmw_connext_cpp` ROS软件包。`rmw_connext_cpp`会被选择为默认的RMW实现。
如果同时安装有`rmw_fastrtps_cpp`，那么它就会被作为默认实现。
下面会介绍在运行ROS2例子的时候如何选择RMW实现。

注意：对于ROS2 alpha版本直到 alpha 8版本 在选择默认版本中，只考虑字母顺序原则。Fast RTPS 相对于其他的实现并没有什么优先级。

## 指定RMW实现
---
为了能够同时使用多种实现，你必须要从源码安装ROS
---

### C++
ROS 2 C++ [示例程序](https://github.com/ros2/demos/tree/master/demo_nodes_cpp/src)会编译生成一组可执行文件比如'talker'
默认情况下这些程序会使用默认的RMW实现。
想要使用不同的RMW实现，可译设置环境变量`RMW_IMPLEMENTATION`。比如
- `RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker`.这样talker会使用Connext作为默认的RMW实现。


#### beta 1 和之前版本
beta1和之前版本并不支持`RMW_IMPLEMENTATION`环境变量。
ROS 2 C++ 例子程序会针对每一个安装好的RMW实现编译一个版本的可执行文件。比如会编译生成下面的可执行文件。

- 一个叫做`talker__rmw_fastrtps_cpp`的可执行文件，此文件使用Fast RTPS作为RMW实现。
- 一个叫做`talker__rmw_connext_cpp`的可执行文件，此文件使用Connext作为RMW实现。
- 一个叫做`talker`的可执行文件，此文件使用默认的RMW实现。

### Python

ROS2的Python[例子](https://github.com/ros2/demos/tree/master/demo_nodes_py)默认情况下使用默认的RMW实现。
例如假设你已经安装配置完成你的ROS2工作空间。下面的命令会执行talker发布者，其使用默认的RMW实现。

```bash
ros2 run demo_nodes_py talker
```

和C++一样，你也可以通过设置`RMW_IMPLEMENTATION`环境变量来更改RMW实现。
```bash
export RMW_IMPLEMENTATION=rmw_connext_cpp
ros2 run demo_nodes_py talker
```

#### beta 1和之前版本

在beta1和之前版本，同样也不支持`RMW_IMPLEMENTATION`环境变量。
然而你可以通过设置 `RCLPY_IMPLEMENTATION`环境变量来指定RMW实现。
例如想要指定`rmw_fastrtps_cpp`作为RMW实现，在Linux上你可以执行：


```bash
RCLPY_IMPLEMENTATION=rmw_fastrtps_cpp talker_py
```

## 在你的工作空间添加RMW实现

假设你在编译你的工作空间的时候只安装使用了Fast RTPS 作为中间件。
那么其他的RMW实现的ROS包，比如`rmw_connext_cpp`,就很有可能无法找到相关的DDS实现的安装。
即使你之后又安装了其他的DDS实现，比如说Connext，你必须在Connext RMW实现编译的时候重新触发再次检查Connext的安装位置。
你可以在你下次编译时设置`--force-cmake-configure`标记来触发。然后你就能看到新的编译结果已经能够支持新的加入的DDS实现了。

当你在使用`--force-cmake-configure`选项进行编译的时候，你很有可能会遇到问题说默认的RMW发生了改变。为了解决这个问题。你可以通过设置`RMW_IMPLEMENTATION`环境变量，把RMW设置成和原来的RMW一样。也可以直接删除报错这个错误的软件包的build文件夹，然后再添加上`--start-with <软件包名称>`参数后重新编译。

## 常见问题

### 确保使用特定的RMW实现

#### ROS2 ardent 和之后版本

如果`RMW_IMPLEMENTATION`环境变量设置为一个尚未支持或安装的RMW版本。你会看类似于下面的错误信息
```
Expected RMW implementation identifier of 'rmw_connext_cpp' but instead found 'rmw_fastrtps_cpp', exiting with 102.
```

如果你安装了多个RMW实现，但是你设置了一个并未安装的实现，那么你会看到类似下面的错误信息。

```
Error getting RMW implementation identifier / RMW implementation not installed (expected identifier of 'rmw_connext_cpp'), exiting with 1.
```

如果发生了以上的情况，请再次确认你时候已经安装了对应的RMW实现。

#### ROS 2 beta 2和之后版本

在ROS 2 beta 2/ beta 3, 设置一个无效的RMW实现并不会产生任何错误。
如果你想确认时候已经使用了否个版本RMW实现，你可以设置`RCL_ASSERT_RMW_ID_MATCHES`环境变量。这个环境变量会让节点只允许使用指定的RMW实现。

```
RCL_ASSERT_RMW_ID_MATCHES=rmw_connext_cpp RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker
```
