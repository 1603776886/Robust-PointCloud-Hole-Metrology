/*
================================================================================
文件：DianYunJiangZao_Filter.cpp
模块：点云降噪实现

【主要职责】
实现统计离群点等基础滤波。

【主要调用关系】
由主窗口的 DianYunJiangZao 流程调用。

【线程与状态】
纯计算。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
实现 GUI 当前使用的统计离群点去除。输入输出都使用程序统一的 CloudPtr，不维护第二套研究期降噪链。

主要调用位置：
ZhuChuangKou_Window.cpp 的预处理和“统计离群”操作。

维护说明：
这里只保留生产 GUI 实际调用的算法。若以后需要新增降噪方式，应从 GUI 的明确功能需求出发新增接口，
不要把临时试验逻辑长期堆在生产文件中。
*/

#include "DianYunJiangZao_Filter.h"

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>

/** 【函数导航】
 * 作用：执行“tongJiLiQun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云降噪实现。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp、DianYunJiangZao_Filter.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr tongJiLiQun(CloudPtr cloud, int meanK, double stddevMult, float voxelLeaf)
{
    if (!cloud || cloud->empty()) {
        return cloud;
    }
    if (cloud->size() < 3U) {
        return CloudPtr(new Cloud(*cloud));
    }
    meanK = std::clamp(meanK, 2, static_cast<int>(cloud->size() - 1U));

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(meanK);
    filter.setStddevMulThresh(stddevMult);

    CloudPtr filtered(new Cloud);
    filter.filter(*filtered);

    // voxelLeaf <= 0 表示只做统计离群，不改变点云采样密度。
    if (voxelLeaf <= 0.0f || !filtered || filtered->empty()) {
        return filtered;
    }

    pcl::PointXYZRGB minPoint;
    pcl::PointXYZRGB maxPoint;
    pcl::getMinMax3D(*filtered, minPoint, maxPoint);

    // voxelLeaf > 0 时保持原有体素规则：下限 0.001，上限取点云最大 Z 的十分之一。
    // 这个上限会直接影响降采样强度；若以后修改，必须同步检查孔口细节是否被过度稀释。
    float leaf = voxelLeaf;
    leaf = std::clamp(leaf, 0.001f, maxPoint.z / 10.0f);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(leaf, leaf, leaf);

    CloudPtr output(new Cloud);
    voxel.filter(*output);
    return output;
}
