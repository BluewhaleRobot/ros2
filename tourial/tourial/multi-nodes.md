# 在同一个进程中使用多个节点

## ROS 1 - Node 和 Nodelets

在ROS1中你可以写一个[节点](http://wiki.ros.org/Nodes)也可以写一个[小节点](http://wiki.ros.org/nodelet)(Nodelet)。
ROS 1 的节点会被编译成一个可执行文件。
ROS 1的小节点会被编译成一个动态链接库。当程序运行的时候会被动态的加载到容器进程里面。

## ROS 2 - 统一API

在ROS2里面，推荐编写小节点——我们称之为组件`Component`。
这样我们就更容易为已经存在的代码添加一些通用的概念，比如[生命周期](http://design.ros2.org/articles/node_lifecycle.html).
使用不同的API所带来的麻烦完全被ROS2给避免了。节点进和小节点在ROS2中完全使用相同的API。

> 你也可以继续使用节点的风格的主函数，但是一般是并不推荐的

通过把进程的结构变成一个部署是的选项，用户可以自由的在下面的模式进行选择
* 在不同的进程中运行多个节点。这样可以使不同的进程独立开。一个崩溃其他可以正常运行。也更方便调试各个节点。
* 在同一个进程中运行多个节点。这样可以使得通信更加高效。

在未来的roslaunch版本中，会支持配置进程的结构。

## 编写一个组件

由于一个组件会被编译生成一个共享链接库。所以它并没有一个主函数入口。(see [Talker source code](https://github.com/ros2/demos/blob/master/composition/src/talker_component.cpp))
组件继承自`rclcpp::Node`类
由于它并不由一个线程直接控制，所以不要在构造函数里面执行很长时间的任务，甚至是阻塞的任务。
你可以用定时器来实现周期性的提示。
另外它可以创建发布者，订阅者，服务提供者和服务客户端。

为了使这样一个类能够成为一个组件，很重要的一点是使用`class_loader`软件包注册自己（可以查看最后一行的源代码）。
这样组件就能够在库文件被加载进进程时被发现。

## 使用组件
[composition](https://github.com/ros2/demos/tree/master/composition)软件包包含了几种不同的使用组件的方式。
最常见的是下面几种

1. 你启动了一个通用的容器进程([1](https://github.com/ros2/demos/blob/master/composition/src/api_composition.cpp)) 然后调用由此容器提供的ROS服务[load_node](https://github.com/ros2/demos/blob/master/composition/srv/LoadNode.srv)。
  这个ROS服务接着会根据传入的软件包名称和组件名称载入对应的组件开始执行。
  除了使用程序来调用ROS服务外，你还可以使用[命令行工具](https://github.com/ros2/demos/blob/master/composition/src/api_composition_cli.cpp)去传入参数触发ROS服务。
2. 你可以创建一个[自定义的可执行文件](https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp)。这个文件中包含多个节点。
  这种方法要求每个组件都有一个头文件。（第一个方法并不需要这样）

## 运行例子程序

来自[composition](https://github.com/ros2/demos/tree/master/composition)软件包的程序可以通过下面的指令执行

### 使用ROS服务的运行时组件（第一种方式）。此组件有发布者和订阅者。

在第一终端执行：

  ros2 run composition api_composition

在第二个终端执行：

  ros2 run composition api_composition_cli composition composition::Talker

现在第一个终端会显示已经翟茹一个组件，同时会不停的显示已经发送了一个消息。

在刚才第二个终端中执行

  ros2 run composition api_composition_cli composition composition::Listener

现在第一个终端应该会不停的显示收到的消息。

> 这个例子在代码中使用了固定的主题名称，所以你不可以运行两遍`api_composition`
> 总的来说你也可以分别运行两个容器进程，然后分别载入发布者和订阅者程序。你会发现他们也是可以相互通信的。

### 使用ROS服务的运行时组件（第一种方式）。此组建有服务提供者和服务客户端

这个例子和上面的例子非常相似。

首先在第一个终端中输入

  ros2 run composition api_composition

在第二个终端中输入

  ros2 run composition api_composition_cli composition composition::Server
  ros2 run composition api_composition_cli composition composition::Client

在这个例子中客户端发送请求至服务端，服务端处理请求然后返回响应。客户端将收到的响应打印出来。

### 使用ROS服务的编译时组件（第二种方式）

这个例子展示了相同的共享链接库可以重用然后编译出一个使用多个组件的可执行程序。这个程序包含了以上的四个组件：发布者，订阅者，服务提供者，服务客户端

在终端执行

  ros2 run composition manual_composition  

终端会重复显示来自两对程序的信息。

### 使用dlopen的运行时组件

这个例子展示了第一种的替代方式。启动一个通用的容器进程，然后不通过ROS接口直接传递给它要载入的库。
这个进程会载入每一个库，然后创建对应的"rclcpp::Node"类示例。

**Linux** 在终端中执行

  ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

**OSX** 在终端中执行

  ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.dylib `ros2 pkg prefix composition`/lib/liblistener_component.dylib

**Windows** 在命令行中执行

  ros2 pkg prefix composition

来获取composition软件包的安装位置，然后执行

  ros2 run composition dlopen_composition <path_to_composition_install>\bin\talker_component.dll <path_to_composition_install>\bin\listener_component.dll

现在在终端会重复输出每个发送和接收的消息。
