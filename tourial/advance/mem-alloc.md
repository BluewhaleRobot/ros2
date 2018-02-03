这篇文章会教你如何给发布者和订阅者写一个自定义的内存分配器。这样当你的ROS节点程序运行时就会通过你的内存分配器给你的程序分配内存，而不会使用默认的内存分配器。
这个例程的源代码可以在[这里](https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/allocator_tutorial.cpp)获取

## 背景
假如你想写实时运行的安全可靠的代码，那么你一定听说过使用 new 来实时分配内存时可能造成的种种危险。因为默认的在各种平台上的内存分配器都是不确定的。

默认情况下。很多标准的C++库数据结构在数据增加的时候都会自动分配内存。比如`std::vector`.然而这些数据结构也接收一个"内存分配器"参数。如果你给其中的一种数据结构指定了内存分配器，那么它就会使用你的内存分配器而不用系统的内存分配器去分配内存。你的内存分配器可以是一个已经提前分配好的内存栈。这样就更适合于实时运行的程序。

在ROS2 C++客户端程序中(rclcpp),我们和C++标准库有着类似的原则。发布者订阅者和执行者接受一个内存分配器参数。这个分配器在运行时控制着整个程序。

## 编写一个内存分配器
为了写一个和ROS2接口兼容的内存分配器，你的分配器必须兼容C++标准库的内存分配器接口。

C++标准库提供了一个叫做 `allocator_traits`的东西。C++标准库的内存分配器只需要实现很少的几个方法就可以按照标准的方式去分配和回收内存。
`allocator_traits`是一个通用的结构，它提供了通过通用内存分配器编写一个内存分配器所需要的其他参数。

例如，下面的对于一个内存分配器的声明就满足 `allocator_traits`(当然，你还是需要实现这个声明中的各种函数)

```
template <class T>
struct custom_allocator {
  using value_type = T;
  custom_allocator() noexcept;
  template <class U> custom_allocator (const custom_allocator<U>&) noexcept;
  T* allocate (std::size_t n);
  void deallocate (T* p, std::size_t n);
};

template <class T, class U>
constexpr bool operator== (const custom_allocator<T>&, const custom_allocator<U>&) noexcept;

template <class T, class U>
constexpr bool operator!= (const custom_allocator<T>&, const custom_allocator<U>&) noexcept;

```

然后你可以通过下面的方式来访问由`allocator_traits`配置的内存分配器内部的各种函数和成员变量：
`std::allocator_traits<custom_allocator<T>>::construct(...)`

想要了解`allocator_traits`的全部功能可以看: http://en.cppreference.com/w/cpp/memory/allocator_traits

然而有些编译器只有部分的C++ 11支持。比如GCC 4.8, 这样就还需要写大量的样板代码来在标准数据结构（比如数据和字符串）中使用. 因为结构并没有在内部使用 `allocator_traits`。因此如果你在使用一个只有部分C++11支持的编译器。你的内存分配器就会需要写成这样:

```
template<typename T>
struct pointer_traits {
  using reference = T &;
  using const_reference = const T &;
};

// Avoid declaring a reference to void with an empty specialization
template<>
struct pointer_traits<void> {
};

template<typename T = void>
struct MyAllocator : public pointer_traits<T> {
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept;

  ~MyAllocator() noexcept;

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept;

  T * allocate(size_t size, const void * = 0);

  void deallocate(T * ptr, size_t size);

  template<typename U>
  struct rebind {
    typedef MyAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept;

template<typename T, typename U>
constexpr bool operator!=(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept;
```

## 写一个主函数例子
当你写了一个内存分配器之后，你必须把它通过一个共享指针传递给你的发布者，订阅者和执行者。

```
  auto alloc = std::make_shared<MyAllocator<void>>();
  auto publisher = node->create_publisher<std_msgs::msg::UInt32>("allocator_example", 10, alloc);
  auto msg_mem_strat =
    std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<std_msgs::msg::UInt32,
    MyAllocator<>>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_example", 10, callback, nullptr, false, msg_mem_strat, alloc);

  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<MyAllocator<>>>(alloc);
  rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);
```

在你的代码执行的过程中所传递的消息也需要通过你的内存分配器来分配。

```
  auto alloc = std::make_shared<MyAllocator<void>>();
```

当你已经实例化一个节点，给这个节点添加一个执行者之后，就是spin的时候了

```
  uint32_t i = 0;
  while (rclcpp::ok()) {
    msg->data = i;
    i++;
    publisher->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }
```

## 在进程内传递内存分配器
尽管我们已经在同一个进程中创建了一个发布者和订阅者，我们还是不能在进程内的发布者和订阅者间传递内存分配器。

进程内管理器是一个通常对用户隐藏的类。但是为了能够传递自定义的内存分配器，我们需要通过rcl context把它暴露出来。进程内管理器使用了几种标准的数据结构，所以如果没有自定义的内存管理器，它就会使用默认的new。

```
  auto context = rclcpp::contexts::default_context::get_global_default_context();
  auto ipm_state =
    std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerState<MyAllocator<>>>();
  // Constructs the intra-process manager with a custom allocator.
  context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(ipm_state);
  auto node = rclcpp::Node::make_shared("allocator_example", true);
```

一定要确保在这样构造节点之后实例化发布者和订阅者。

## 测试和验证代码
如何确认你的自定义内存分配器正在被使用

最简单的方法就是数自定义内存分配器的`allocate`和`deallocate`函数调用的次数，然后把这个次数和`new`和`delete`进行对比。

在自定义内存分配器中添加计数是非常容易的

```
  T * allocate(size_t size, const void * = 0) {
    // ...
    num_allocs++;
    // ...
  }

  void deallocate(T * ptr, size_t size) {
    // ...
    num_deallocs++;
    // ...
  }
```

你可以覆盖全局的new和delete操作

```
void operator delete(void * ptr) noexcept {
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
    ptr = nullptr;
  }
}

void operator delete(void * ptr, size_t) noexcept {
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
    ptr = nullptr;
  }
}
```

这里我们增加的是一个全局的静态变量， `is_running`是一个全局的静态布尔变量，当spin函数执行前会被设值。

[例子程序](https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/allocator_tutorial.cpp)
会输出各变量的值。通过输入下面的指令来执行例子程序

```
allocator_example
```

或者同时启用进程内工作流

```
allocator_example intra-process
```

你会在输出中看到下面的结果

```
Global new was called 15590 times during spin
Global delete was called 15590 times during spin
Allocator new was called 27284 times during spin
Allocator delete was called 27281 times during spin
```

我们获取到了大概2/3的在程序中allocations/deallocation 的调用。但是这剩下的1/3是从哪里来的？

实际上这来自在这个例子中使用的底层的DDS实现。

下面内容超出这篇文章的范围。但是你可以通过查看在ROS2连续集成测试中的代码。里面有代码可以追踪到某个函数调用到底来自于rmw实现还是DDS实现。

https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/test/test_tlsf.cpp#L41

注意上面的测试程序并没有使用我们刚才创建的自定义内存分配器，它使用的是TLSF内存分配器。

## TLSF内存分配器

ROS2 提供了TLSF（两层分离适应）内存分配器支持。它为了满足实时性要求而被设计出来：

https://github.com/ros2/realtime_support/tree/master/tlsf_cpp

注意TLSF内存分配器是dual-GPL/LGPL协议的。

下面是一个使用TLSF内存分配器的例子

https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/example/allocator_example.cpp
