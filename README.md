# ROS-Programming

# callback()&ensp;&ensp;&&ensp;&ensp;ros::spin()&ensp;&ensp;&&ensp;&ensp;ros::spinOnce()
## ros::spin()
循环且监听反馈函数（callback)

循环就是指程序运行到这里，就会一直在这里循环了，这句话后面的程序将不会被执行。

监听反馈函数的意思是，如果这个节点有callback函数，那写一句ros::spin()在这里，就可以在有对应消息到来的时候，运行callback函数里面的内容。
我觉得写这句话适用于写在程序的末尾（因为写在这句话后面的代码不会被执行），适用于订阅节点，且订阅速度没有限制的情况。
```
ros::init(argc, argv, "my_node"); *//初始化节点*
ros::NodeHandle nh;           *//创建节点句柄*
ros::Subscriber sub = nh.subscribe(...);  *//创建消息订阅者*
...
ros::spin();                 *//调用spin(),统一处理消息*
```
在这里，所有的用户回调函数将在spin()调用之后被调用.

ros::spin()不会返回，直到节点被关闭，或者调用ros::shutdown()，或者按下ctrl+C


## ros::spinOnce()
监听反馈函数（callback），只能监听反馈，不能循环。所以当你需要监听一下的时候，就调用一下这个函数。
这个函数比较灵活，尤其是想控制接收速度的时候。
```
ros::Rate r(10); // 10 hz
while (should_continue)
{
  //... do some work, publish some messages, etc. ...
  ros::spinOnce();     *//轮转一次,返回*
  r.sleep();        *//休眠*
}
```
ros::spinOnce()将会在被调用的那一时间点调用所有等待的回调函数.


## 回调函数与ros::spin()  ros::spinOnce()

ROS的消息接收回调机制(callbacks and spinning)原理是这样的：除了用户的主程序以外，ROS的socket连接控制进程会在后台接收订阅的消息，所有接收到的消息并不是立即处理，而是等到spin()或者spinOnce()执行时才集中处理。**消息到来并不会立即执行消息处理回调函数，而是在调用ros::spin()或者spinOnce()之后，才进行消息处理的轮转,消息回调函数统一处理订阅话题的消息。**

ROS的主循环中需要不断调用ros::spin()或ros::spinOnce()，**两者区别在于前者调用后不会再返回，而后者在调用后还可以继续执行之后的程序。**

在使用ros::spin()的情况下，一般来说在初始化时已经设置好所有消息的回调，并且不需要其他背景程序运行。这样一来，每次消息到达时会执行用户的回调函数进行操作，相当于程序是消息事件驱动的；

而在使用ros::spinOnce()的情况下，一般来说仅仅使用回调不足以完成任务，还需要其他辅助程序的执行：比如定时任务、数据处理、用户界面等。

---

# range()和np.arrange()的区别与联系
## range(start, stop[, step])

这是一个通用的函数来创建包含算术级数的列表。它最常用于for循环。参数必须是纯整数。如果省略step参数，则默认为1。

如果省略start参数，则默认为0。完整的形式返回一个普通整数列表。如果步骤是肯定的，最后一个元素是小于停止的最大元素; 如果step是负数，最后一个元素是大于stop的最小元素。 步骤不能为零（否则报错）。
```
>>> range(10)
[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
>>> range(1, 11)
[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
>>> range(0, 30, 5)
[0, 5, 10, 15, 20, 25]
>>> range(0, 10, 3)
[0, 3, 6, 9]
>>> range(0, -10, -1)
[0, -1, -2, -3, -4, -5, -6, -7, -8, -9]
>>> range(0)
[]
>>> range(1, 0)
[]
```
## numpy.arange([start, ]stop, [step, ]dtype=None)

在给定的时间间隔内返回均匀间隔的值。

在半开区间[start， stop）内产生值 （换句话说，包括开始但不包括停止的区间）。对于整数参数，该函数等同于Python内置的 范围函数，但返回一个ndarray而不是一个列表。

当使用非整数步长（如0.1）时，结果往往不一致。这些情况下最好使用linspace。
```
>>> np.arange(3)
array([0, 1, 2])
>>> np.arange(3.0)
array([ 0.,  1.,  2.])
>>> np.arange(3,7)
array([3, 4, 5, 6])
>>> np.arange(3,7,2)
array([3, 5])
```

## 区别
+range()和np.arange()的返回类型不同，range()返回的是range；
+object，而np.arange()返回的是ndarray类型；
+range()不支持步长为小数，而np.arange()支持步长(step)为小数；
+range()和np.arange()都可用于迭代；
+range()和np.arange()都有三个参数，以第一个参数为起点，第三个参数为步长，截止到第二个参数之前的不包括第二个参数的数据序列。
+range()可用于迭代，而np.arange作用远不止于此，它是一个序列，可被当做向量使用。


