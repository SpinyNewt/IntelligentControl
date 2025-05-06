# README

## 代码使用

### 参数设置

* 初始的参数设置为第一问的参数
* 在`get_F.m`中设置F_m
* 在`get_theta.m`中设置$\theta(0)$
* 在`get_system_param.m`中设置专家控制参数

### 运行

* 在`all_in_one.slx`中运行,
	* `pid_plain`为普通PID控制器的结果
	* `pid_expert`为专家PID控制器的结果
	* 在`all_in_one.slx`的放大器中手动设置衰减比例为$1/F_m$，以方便F和其他量一起展示
	* 红线为$F/F_m$， 蓝线为$\theta$， 黄线为$x/50$

## 代码组织架构

* `system_transfer.m` 系统微分传递方程
* `pid_plain.m` 普通PID控制器
* `pid_expert.m`专家PID控制器
* `get_F.m` `get_theta.m` `get_system_param.m` 设置相关参数，已在上一部分说明