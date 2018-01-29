# Linux下的安装

## 通过Debian包安装

在Beta2版本我们生成了ROS2的deb软件包（用于Ubuntu 16.04版本）。这些文件在一个为了测试用的临时软件源内。下面的链接和指令用于安装最新的ROS2版本-当前版本ardent。

相关资源信息

* [Jenkins Instance](http://build.ros2.org/)
* [Repositories](http://repo.ros2.org)
* Status Pages \( [amd64 ](http://repo.ros2.org/status_page/ros_ardent_default.html)[arm64](http://repo.ros2.org/status_page/ros_ardent_uxv8.html)\)

## 设置软件源

想要安装deb软件包，你需要现在自己的apt软件源列表内添加我们的软件源。

首先你要像下面一样添加gpg密钥到自己的电脑中

```bash
sudo apt update && sudo apt install curl
curl http://repo.ros2.org/repos.key | sudo apt-key add -
```

然后执行下面的指令添加软件源到自己的apt源列表中

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'
```



## 安装ROS2软件包

下面的指令会安装所有的**ros-ardent-\* **软件包除了**ros-ardent-ros1-bridge** 和 **ros-ardent-turtlebot2-\***。因为这两个软件包依赖于ROS 1的软件包。

按照下面的指令安装对应的软件包

    sudo apt update
    sudo apt install `apt list "ros-ardent-*" 2> /dev/null | grep "/" | awk -F/ '{print $1}' | grep -v -e ros-ardent-ros1-bridge -e ros-ardent-turtlebot2- | tr "\n" " "`

## 环境变量设置

```
source /opt/ros/ardent/setup.bash
```

如果你安装了Python的软件包 argcomplete\( 0.8.5 或以上版本\)你可以通过执行下面的指令来为ROS2的命令行工具添加自动补全功能。

```
source /opt/ros/ardent/share/ros2cli/environment/ros2-argcomplete.bash
```

argcomplete可以通过

```
sudo pip install argcomplete
```

来安装。

## 选择 RMW 实现

默认的RMW实现是 FastRTPS。

通过设置环境变量 RMW\_IMPLEMENTATION=rmw\_opensplice\_cpp。你可以切换成OpenSplice。



## 额外的依赖于ROS1的软件包

ros1\_bridge和TurtleBot的例子程序一样都是依赖于ROS1的软件包。

为了能够安装这些软件包，请安装[这里](http://wiki.ros.org/Installation/Ubuntu?distro=kinetic)的说明添加ROS1的软件源。



如果你在使用docker，你可以设置docker image为ros:kinetic或osrf/ros:kinetic-desktop。由于软件已经集成在镜像中这样就省去了配置安装的麻烦。

安装完成之后，现在你可以开始安装下面剩余的软件包了。

```
sudo apt update
sudo apt install ros-ardent-ros1-bridge ros-ardent-turtlebot2-*
```

当然如果你对这些软件包没有兴趣，也可以不用安装。

# Windows下的安装

## 系统要求

在beta-1版本我们支持Windows8.1和Windows10. 在beta-2版本我们只支持Windows10。

## 安装依赖

### 安装Chocolatey

Chocolatey 是一个windows下的软件包管理程序。可以通过他们的安装说明安装。

[https://chocolatey.org/](https://chocolatey.org/)

之后你会用Chocolatey来安装其他开发工具

### 安装Python

打开一个命令行工具。同时按下Win + R在弹出的窗口中输入cmd。在命令行工具中输入下面的指令通过Chocolatey来安装Python。

```
choco install -y python
```

安装OpenSSL

从[这个页面](https://slproweb.com/download/Win64OpenSSL-1_0_2n.exe)中下载OpenSSL安装包。

按照默认设置安装这个软件。接着添加环境变量\(下面的指令默认你是按照默认参数安装的\)

* `setx -m OPENSSL_CONF C:\OpenSSL-Win64\bin\openssl.cfg`
* 添加 C:\OpenSSL-Win64\bin\ 到你的PATH环境变量中

### 安装Visual Studio Community 2015

Microsoft 提供了一个Visual Studio的免费版本，叫做community。我们可以用它来编译ROS2的应用程序。

建议选择英文版安装，因为出错时的错误信息是英语，这样搜索的时候更容易搜到。

[https://www.visualstudio.com/vs/older-downloads/](https://www.visualstudio.com/vs/older-downloads/)

确保在安装时选择了Visual C++的功能。首先选择Custom installlation

![](/assets/import.png)

接着选上Visual C++

![](/assets/install_windows_1.png)

再次确认已经选择上了对应的功能

![](/assets/install_windows_2.png)

### 安装一个DDS程序

二进制软件包中已经默认绑定了 eProsima FastRTPS 和 Adlink OpenSplice 作为中间件。如果你想用其他的DDS软件。那么你需要使用[源代码安装](https://github.com/ros2/ros2/wiki/Windows-Development-Setup)。

#### eProsima FastRTPS & Boost\(只能在beta-1或之后的版本中使用\)

FastRTPS











































