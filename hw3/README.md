# README

## 文件运行
* 	`get_system_param.m`中设置物体质量，初始值为0.05，其它参数均已设定
* 	在`sfunction.slx`中运行，蓝线表示$U/100$,黄线表示x，==注意==PD控制器用于对比，不是解决方案

## 文件组织

* `fuzzy.fis` PD模糊控制器
* `fuzzy_p.fis` 积分环节的模糊P控制器
* `get_system_param.m`中设置系统参数，包括物体质量，初始值为0.05
* `system_transfer` 系统的sfunction
* `sfunction.slx` 整体的simulink文件,包含PID控制和用于对比的PD控制方案，PD方案存在静差

## 报告

==推荐阅读html版本报告，如果因为电脑原因存在格式排版问题或图片缺失问题请阅读PDF版本==。
