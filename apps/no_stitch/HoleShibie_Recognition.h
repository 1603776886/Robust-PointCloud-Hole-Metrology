/*
================================================================================
文件：HoleShibie_Recognition.h
模块：Hole 识别公开入口

【主要职责】
声明手动 seed 驱动的正式 Hole 识别与空间上下文预热入口。

【主要调用关系】
主窗口 onHoleShibie()/预热流程调用。

【线程与状态】
正式识别通常在后台 worker；输入点云按只读处理。

【维护边界】
1. 本文件只暴露 GUI 正式 Hole 识别入口，不承载法线、圆弧、孔壁等数值实现。
2. 初始粗法线、外环支撑法线、Hole 口椭圆反推法线已归入 HoleFaXian_Normal.h；
   HoleWeizi_Pose.h 保留后续位姿、孔壁、锥壁和可见性综合。
3. 本轮结构整理不改变识别阈值、候选排序、ROI、拟合公式、循环次数、浮点表达式或拼接逻辑。
4. 第三方 Qt/PCL/VTK/Eigen 类型与 API 保持原名。
================================================================================
*/
/*
模块职责：
声明当前 GUI 正式使用的手动选孔局部识别入口，以及识别过程需要共享的数据结构。
HoleMiaoshu、HoleShibieResult 和用户种子参数已经放在 HoleLeixing_Types.h，本文件主要保留算法入口和现有结果兼容结构。

主要调用位置：
ZhuChuangKouWindow::onHoleShibie() 调用 shouDongJuBuHoleShibie()；
prewarmHoleShibieKongJianContext() 由界面在用户准备选孔时提前建立完全相同的空间上下文。

维护说明：
新增生产字段优先放 HoleLeixing_Types.h；新增算法实现优先放入已有几何、孔形、候选或手动孔模块，避免继续扩大总编排文件。
任何影响半径、深度、孔型和法向的默认参数都必须写明单位、增大/减小的影响和验证要求。
*/
#pragma once

#include "DianYunLeixing_Types.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <QString>
#include <vector>
#include <array>
#include <string>
#include <tuple>
#include "HoleLeixing_Types.h"
#include <memory>

// 本头文件只声明 GUI 正式识别入口；识别内部数据与算法细节由对应功能模块管理。

// 手动局部 Hole 识别主入口：当前 GUI 正式使用的生产入口。
// 法线流程要点：
// 1) 每份点云先缓存一个“全局粗法线”，只用于给搜索提供大致朝向；它不是最终 Hole 法线。
// 2) 每个 seed 若周围存在可用实体表面，会用轻量的多尺度稳健平面得到“初始局部粗法线”；
//    若局部表面不可用，则继续沿用全局粗法线，不人为制造支撑平面。
// 3) 只有疑难 seed（局部法线与全局差异较大，或常规 frame 没形成可靠 Hole 族）才会调用
//    ROI 主平面 RANSAC 作为第二级搜索坐标兜底；它不参与正常 seed 的最终法线精度竞争。
// 4) Hole 口已经建立后，jingQueTuoYuanFanTuiFaXian() 才根据圆弧/椭圆几何反推精确法线；
//    一旦该结果被采用，后续支撑面法线不会再覆盖它。
// 因此“初始法线”负责找得到 Hole，“椭圆法线”负责最后量得准，两者职责不同。
// 输出 HoleShibieResult.descriptors 用于参数面板、可视化叠加和保存。
HoleShibieResult shouDongJuBuHoleShibie(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const std::vector<ShouDongHoleSeed>& seeds,
    const char* cloudName = nullptr);

/*
识别空间上下文预热。它建立与正式识别完全相同的全云 FLANN 空间索引和确定性全局法向，
只把原本发生在“孔识别”点击后的准备工作提前，不改变任何搜索结果。函数本身是阻塞的，
图形界面应在线程池调用。最终程序固定启用该生产优化，不再提供研究期环境变量开关。
*/
void prewarmHoleShibieKongJianContext(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

// 最终 GUI 头文件只暴露主识别入口和空间预热。
// 未参与正式识别流程的辅助声明不放在本接口中，
// 避免后续维护者误以为这些接口仍然可用。
