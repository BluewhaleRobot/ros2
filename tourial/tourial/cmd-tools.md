# 简介

ROS 2 包含了一些列的命令行工具去方便测试开发ROS 2的应用程序。

## 使用方法
这些工具的主要入口是 `ros2`指令。这个指令包含了大量的用来测试开发的子命令。这些命令可以方便的操作节点，主题，服务和其他。

想要看到所有的子命令可以执行

```
ros2 --help
```

```
ros2 run demo_nodes_cpp talker -t chatter2
```

## 例子

为了能够使用命令行工具实现经典的发送者订阅者例子，我们要使用topic子命令。它可以用来向某个主题发送或接受信息。

在一个终端发布消息
```
$ ros2 topic pub /chatter std_msgs/String "data: Hello world"
publisher: beginning loop
publishing std_msgs.msg.String(data='Hello world')

publishing std_msgs.msg.String(data='Hello world')
```
在一个终端显示接收到的消息
```
$ ros2 topic echo /chatter
data: Hello world

data: Hello world
```

## 实现过程

ROS2 使用了分布式发现进程去自动的连接不同节点。
这个过程故意不适用一个中心化的发现机制（就好像ROS1里面的主节点一样）。发现ROS网络中的其他节点可能会需要一些时间。
正是因为如此，在后台有一个长期运行的程序，它保存了ROS网络中的信息。这样就可以更加快速的处理请求和提供快速响应。例如节点的名称列表。

这个后台程序在你第一次运行命令行工具的时候就会开始执行。
你可以运行`ros2 daemon --help`来查看更多的和后台程序交互的选项。

## 代码实现

ros2命令的源代码在这里[ros2](https://github.com/ros2/ros2cli)

ros2工具实现了一个框架，可以通过插件去扩展功能。
例如，[`sros2` 软件包](https://github.com/ros2/sros2)提供了“security” 子命da令。当这个软件包安装之后ros2指令会自动的检测到这个程序并添加对应的子命令。
