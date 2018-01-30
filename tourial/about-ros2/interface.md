# ROS 接口

## 1. 背景
ROS程序一般通过一种或两种接口进行通信：消息和服务
ROS使用了一种简化的描述语言来描述这些接口。这种描述语言使得ROS的工具更加容易的自动生成对应语言的源代码。

在这篇文章中，我们将介绍支持的类型和如何创建你的 msg/srv文件

## 2. 消息描述说明
消息的描述文件是在ROS软件包msg文件夹内的.msg文件。
.msg文件宝行两个部分：变量域（Fields）和常量(constants)

*这里将Field翻译成变量域以和constants做作对比区分。如果直接翻译成域，会让人不明所以。*

### 2.1 域
每一个域包含两个部分, 类型和名称。中间用空格隔开，例如
```
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

又如
```
int32 my_int
string my_string
```

#### 2.1.1 变量域类型
变量域的类型可以是一下几种
* 内部定义类型
* 有用户自己定义的类型，比如 "geometry_msgs/PoseStamped"

_内部定义的类型现在支持一下几种：_
| 类型名称 | [C++](http://design.ros2.org/articles/generated_interfaces_cpp.html)  | [Python](http://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](http://design.ros2.org/articles/mapping_dds_types.html)
| ------------- | ------------- | ----- | ---- |
| bool | bool | builtins.bool | boolean |
| byte | uint8_t | builtins.bytes* | octet |
| char | char | builtins.str* | char |
| float32 | float | builtins.float* | float |
| float64 | double | builtins.float* | double |
| int8 | int8_t | builtins.int* | octet |
| uint8 | uint8_t | builtins.int* | octet |
| int16 | int16_t | builtins.int* | short |
| uint16 | uint16_t | builtins.int* | unsigned short |
| int32 | int32_t  | builtins.int* | long |
| uint32 | uint32_t | builtins.int* | unsigned long |
| int64 | int64_t | builtins.int* | long long |
| uint64 | uint64_t  | builtins.int* | unsigned long long |
| string | std::string | builtins.str | string |

_每种内部定义类型都可以用来定义数组_
| 类型名称 | [C++](http://design.ros2.org/articles/generated_interfaces_cpp.html)  | [Python](http://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](http://design.ros2.org/articles/mapping_dds_types.html)
| ------------- | ------------- | ----- | ---- |
| static array | std::array<T, N> | builtins.list* | T[N] |
| unbounded dynamic array | std::vector | builtins.list | sequence |
| bounded dynamic array | custom_class<T, N> | builtins.list* | sequence<T, N> |
| bounded string | std::string | builtins.str* | string |

所有比ROS变量定义中范围更广，更加宽松的变量，都会被软件限制在ROS所定义的范围中。

_使用数组和限制类型的消息定义的例子_
```
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_string_up_to_ten_characters each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

#### 2.1.2 变量域名称
变量域名称必须以小写字母开始，同时以下划线作为单词的分割符。不能以下划线结束，也不允许有两个连续的下划线。

#### 2.1.3 变量域默认值
默认值可以设置成变量域类型所允许的任意值。
当前默认值还不能支持字符串数组和复杂类型。（也就是没有出现在内部定义类型里面的，同样也适用于所有的嵌套消息）

定义默认值可以通过在变量域定义中添加第三个元素来实现。也就是
```
变量域名称 变量域类型 变量域默认值
```

比如
```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```
特别说明:
- 字符串类型默认值必须用单引号或者双引号括起来
- 当前的字符串类型是没有被转义的

### 2.2 常量
常量的定义就好像有默认值的变量域定义。除了常量的值是永远不能由程序改变的。常量通过等号进行赋值。也就是
```
常量类型 常量名称=常量值
```
例如
```
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```

特别说明：常量名必须是大写

## 3. 服务定义说明
服务描述由位于ROS包下的srv文件夹内的.srv文件定义。

一个服务描述文件包含了一个请求和一个回应的消息类型。之间用---分割。任意的两个消息类型连接起来，并在中间用---分割都是一个合法的服务描述。

下面是一个非常简单的服务的例子。这个服务接收一个字符串然后返回一个字符串：

```
string str
---
string str
```
当然我们也可以更加复杂一点（如果你想引用来自同一个软件包内的消息类型，那么你一定不要包含这个软件包的名字）：

```
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg
---
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

你不能在一个服务中嵌入另外一个服务。
