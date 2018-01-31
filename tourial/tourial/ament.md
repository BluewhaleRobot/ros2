# 简介
这篇文章会简要介绍如何快速配置和使用一个ament工作空间。
这是一个比较实际的操作教程，并不是为了替换核心文档而写。

## 背景

Ament是catkin编译工具的优化迭代版本。
关于Ament的设计上的信息可以看[这个文档](http://design.ros2.org/articles/ament.html)

ament的源代码可以在[这里](https://github.com/ament)找到

## 开发环境

确保你已经按照之前从源代码编译文档的要求配置好你的开发环境。

## 基础知识

一个Ament工作空间，是一个有着特定目录结构的文件夹。
通常会有一个src子文件夹。
在这个子文件夹下是各个软件包的源代码。
通常这个文件夹初始情况下是空的。

Ament脱离源代码进行编译。
默认情况下它会在src文件夹旁创建一个build和一个install文件夹。
build文件夹会用来存放编译过程中产生的中间文件。
对于每个软件包都会创建一个对应的文件夹，然后cmake在对应的文件夹下运行。
install文件夹是软件包的安装位置

注意:和catkin相比这里没有devel文件夹

## 创建目录结构

下面在 ~/ros2_ws 下创建一个基本的工作空间目录结构

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

下面就是你所预想的目录结构
```
.
└── src

1 目录, 0 文件
```

## 添加一些源代码

在开始之前我们需要先配置一些依赖

```bash
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs import ~/ros2_ws/src < ros2.repos
```

下面是在添加源代码之后~/ros2_ws的目录结构（注意具体提的目录结构，文件夹和文件的数据可能会变化）。

```
.
├── ros2.repos
└── src
    ├── ament
    │   ├── ament_cmake
    │   ├── ament_index
    |   ...
    │   ├── osrf_pycommon
    │   └── uncrustify
    ├── eProsima
    │   ├── Fast-CDR
    │   └── Fast-RTPS
    ├── ros
    │   ├── class_loader
    │   └── console_bridge
    └── ros2
        ├── ament_cmake_ros
        ├── common_interfaces
        ├── demos
        ...
        ├── urdfdom
        ├── urdfdom_headers
        └── vision_opencv

51 directories, 1 file
```

## 开始编译
由于我们把ament源代码放在了工作空间中，我们需要执行ament.py的全路径。
注意：在未来ament将会默认安装在系统中，或者在一个底层的工作空间中。这样就不需要再执行上面的步骤了。

在安装时每个软件包都支持`--symlink-install`安装选项。这样可以通过创建符号链接进行安装。当源码发生变化时，安装位置的文件也会跟着发生变化。（例如python或者其他不需要编译的文件）这样可以实现更快的软件迭代开发。

```bash
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```

## 运行测试程序

为了能够运行测试程序，你需要在编译的时候增加`--build-tests`选项。之后运行下面的指令

```bash
src/ament/ament_tools/scripts/ament.py test
```

如果你在之前编译的时候没有添加`--build-tests`选项,在编译测试的时候，你可以直接跳过编译和安装的步骤来加速编译测试程序的过程

```bash
src/ament/ament_tools/scripts/ament.py test --skip-build --skip-install
```

# source 你的环境

当ament编译完成之后，生成的文件会在install文件夹里面。
为了能够使用你编译的文件，你需要把`install/bin`添加到你的系统路径里面去。
Ament会在instal文件夹中自动生成bash文件，用来帮助你配置环境变量。
这些文件会向你的系统路径和库路径添加必要的元素。

```bash
. install/local_setup.bash
```

注意:这里和catkin有一些不同
这里的`local_setup.*`文件和`setup.*`文件有一些不同。`local_setup.*`文件只会应用当前工作空间的设置。
当你在使用多个工作空间时，你仍然需要source `setup.*`文件。这样所有父工作空间的设置也能添加进来。

## 运行一个例子

当你source 环境之后你可以执行下面的指令。这是由ament编译生成的。

```bash
ros2 run demo_nodes_cpp listener &
ros2 run demo_nodes_cpp talker
```

你会看到数字在不断地增加。

下面我们先停止这两个节点，然后来创建我们自己的工作空间


```bash
^-C
kill %1
```

## 开发你自己的软件包

Ament 使用和catkin一样的package.xml文件。（这个文件在[REP 140](http://www.ros.org/reps/rep-0140.html)中定义）

你可以直接在src文件夹内创建自己的软件包。但是如果你只是想写几个软件包，推荐你重新创建一个工作空间。

## 创建一个工作空间

首先创建一个新的文件夹 ``~/ros2_overlay_ws``

```bash
mkdir -p ~/ros2_overlay_ws/src
cd ~/ros2_overlay_ws/src
```

然后开始之前我们要先下载[ROS2例子](https://github.com/ros2/examples)。我们会这基础上进行修改。

```bash
git clone https://github.com/ros2/examples.git
```

开始编译，我们用debug模式编译，这样就可以获取到debug的相关文件。

```bash
cd ~/ros2_overlay_ws
ament build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

现在我们看看talker程序到底指向那个工作空间的文件。因为系统自身有一个talker程序，之前的工作空间也有一个talker程序，现在的工作空间同样也有一个talker程序。

如果你source了`~/ros2_overlay_ws/install/local_setup.bash`，那么talker程序会指向当前工作空间的文件。

如果你打开了一个新的终端程序，然后想要使用刚才创建的工作空间，那么你只需要
source `~/ros2_overlay_ws/setup.bash`，这个程序会自动的设置好所有的父工作空间的环境变量。

## 创建你自己的软件包

你可以开始创建自己的软甲包了。
`catkin_create_package`的等效软件会被移植到ament里面，但是目前还没有完成。

Ament支持多种编译方式。
推荐的方式是 `ament_cmake`和`ament_python`。
同样还支持纯粹的cmake软件包
将来还会添加更多的[编译方式](https://github.com/ament/ament_tools/blob/master/doc/development/build_types.rst)。

一个像[`demo_nodes_cpp`](https://github.com/ros2/demos/tree/master/demo_nodes_cpp)的软件包使用`ament_cmake`编译方式，同时使用cmake作为编译工具。

## 注意
- 如果你不想编译某个软件包，你可以在那个包的文件夹下放一个叫做`AMENT_IGNORE`的空文件。这样这个软件包就不回被处理。
  "Catch all" 选项和其他的 --cmake-args参数一样应该被放在其他参数之后，或者使用--开头

```bash
ament build . --force-cmake-configure --cmake-args -DCMAKE_BUILD_TYPE=Debug -- --ament-cmake-args -DCMAKE_BUILD_TYPE=Release
```
<br>

- 如果你只是想执行某个软件包的测试程序:

```
ament test --only-packages YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```
