# 自定义接口 （消息、服务）

**未完成**

尽管我们鼓励重用已有的标准消息和服务定义。然而在很多情况下你还是要自己定义消息或者服务。
自定义消息和服务第一步时创建`.msg`或`.srv`文件。定义方式可以参照[[ROS接口]]

为了方便起见，`.msg`文件放置于软件包文件夹下的msg文件夹内。`.srv`文件放置于srv文件夹内。

在写完你的`.msg`或`.srv`文件后，你需要在你的CmakeLists.txt文件内添加一些代码。使得代码生成程序能够处理你的定义文件。先要了解更加详细的教程可以参照[pendulum_msgs package](https://github.com/ros2/demos/tree/master/pendulum_msgs)，作为一个例子。你可以在这个包的[CMakeLists.txt](https://github.com/ros2/demos/blob/master/pendulum_msgs/CMakeLists.txt)文件中看到相关的CMake的调用。
