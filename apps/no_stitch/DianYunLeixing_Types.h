/*
================================================================================
文件：DianYunLeixing_Types.h
模块：点云基础类型

【主要职责】
集中定义 PointT、Cloud、CloudPtr 等项目内点云别名。

【主要调用关系】
几乎所有点云、显示、识别和拼接模块包含。

【线程与状态】
无运行状态。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
#pragma once

/*
模块职责：
定义整个程序共享的点类型和点云智能指针别名，是最底层的数据类型头文件。

主要调用位置：
点云 IO、显示、孔识别和降噪模块都会包含这里。

维护说明：
这里必须保持“轻量”。不要在本文件加入 GUI、孔识别等上层模块，否则容易形成循环依赖。
*/

/*
模块职责：
统一项目中最常用的点类型和点云智能指针别名，避免各模块重复声明 PCL 类型。
任何模块需要普通 XYZRGB 点云时都应包含本文件，不再自行建立同义类型。

主要调用位置：
DianYunHuiHua、持久化、渲染、降噪和孔识别的公共边界。

维护说明：
PointT 是跨模块基础 ABI/类型约定；更换点类型会影响几乎所有模块，不属于普通代码整理。
*/

#ifdef _MSC_VER
#include <corecrt_io.h>
#include <io.h>
// 部分 PCL/第三方库与 MSVC 工具链组合仍会引用 _chsize 名称。
#ifndef _chsize
#define _chsize _chsize_s
#endif
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGB;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;
