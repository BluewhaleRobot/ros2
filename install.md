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
注意如果安装失败，比如我就遇到了这个情况。你可以手动安装，直接下载Python官网的安装包，然后安装，不过要安装到C:\Python36这个路径下面。

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

FastRTPS 依赖于boost库。在[这里](https://sourceforge.net/projects/boost/files/boost-binaries/1.61.0/boost_1_61_0-msvc-14.0-64.exe/download?use_mirror=nchc)下载安装包后安装。

这个安装包默认安装在C:\local。安装完成后添加下面的环境变量到系统中。
```
PATH=C:\local\boost_1_61_0\lib64-msvc-14.0
```
#### Adlink OpenSplice
如果你想使用默认的DDS程序即上面的FastRTPS那么你不需要进行下面的操作。
如果你想使用OpenSplice作为DDS软件。你需要下载最新的版本(ROS2要求的最低版本为6.7.170912).解压到C:\dev\opensplice67，注意文件夹的层级。你的文件夹结构应该如下图所示
![](/assets/splice.png)

### 安装OpenCV
之后的教程例子有些依赖于OpenCV. 你可以下载一个预先编译好的版本
https://github.com/ros2/ros2/releases/download/release-beta2/opencv-2.4.13.2-vc14.VS2015.zip
如果你使用的是预先编译好的ROS版本。那么你需要设置下面几个环境变量来告诉ROS2在哪里找OpenCV的库。假设你把OpenCV解压到C:\dev\文件夹下。你需要在PATH环境变量里面添加下面的变量`c:\dev\opencv-2.4.13.2-vc14.VS2015\x64\vc14\bin`

### 安装依赖
有些依赖文件并不在Chocolatey的软件库里面为了简化安装过程，我们提供了下面的Chocolatey软件包。

你可以在[这里](https://github.com/ros2/choco-packages/releases/latest)下载对应的软件包。
* asio.1.10.6.nupkg
* eigen-3.3.3.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.4.1.0.nupkg
下载完成之后重新打开一个具有管理员权限的命令行程序，然后运行下面的语句
```
choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2
```
注意把`<PATH\TO\DOWNLOADS>`替换成你实际的下载位置。
你还需要安装`pip`和`yaml`
```
python -m pip install -U pyyaml setuptools
```
安装时可能会出错，一般是编码的问题，你可以执行下面的语句设置命令行程序的编码
```
chcp 65001  //换成65001代码页
chcp 437    //换成美国英语
```
设置完成之后再执行上面的指令安装。

### 下载ROS2
* 进入ROS2的发布页面: https://github.com/ros2/ros2/releases
* 下载最新的Windows软件包
* 解压这个zip文件（推荐解压到C:\dev\ros2里面）

## 设置ROS2的环境
打开一个命令行程序然后source ROS2 的配置文件来自动配置好工作空间。
注意下面的指令只能通过cmd程序执行，powershell没法运行。
```
call C:\dev\ros2\local_setup.bat
```
执行过程中可能会提示找不到路径的错误。没关系可以继续进行下面的操作。
如果你下载了一个有OpenSplice支持的版本，而且你想用OpenSplice作为DDS程序，那么你可以执行下面的语句。反之则不用执行。
```
call "C:\opensplice67\HDE\x86_64.win64\release.bat"
```
这个语句要在上面一条语句执行完成之后才行执行。

## 尝试运行简单的例子程序
打开一个命令行程序，按照上面的说明设置好ROS2的环境。然后执行一个talker程序
```
ros2 run demo_nodes_cpp talker
```
再打开一个命令行，执行一个listener程序
```
ros2 run demo_nodes_py listener
```
![](/assets/install_windows_talker.png)
![](/assets/install_windows_listener.png)
你可以看到talker程序发布信息,listener程序说明自己接收到了对应的信息。
以前节点的通信通过master去实现，现在可以看到已经不需要master了。而且你可以尝试在不同的电脑上执行这个程序。不同电脑上的节点间不需要任何配置就可以通信了。

### 常见问题
* 如果你由于缺少dll而无法启动例子程序，那么请确认上面的软件包依赖比如OpenCV已经添加到你的PATH环境变量中。
* 如果你忘记了执行local_setup.bat，那么例子程序可能会立即崩溃停止运行。
