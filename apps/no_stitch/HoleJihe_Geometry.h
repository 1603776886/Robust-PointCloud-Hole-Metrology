/*
================================================================================
文件：HoleJihe_Geometry.h
模块：Hole 几何接口

【主要职责】
声明支撑面、Hole 口、半径/深度、内壁、锥壁和最终几何所需输入输出结构与函数。

【主要调用关系】
HoleShibie 与 HoleFenxi 调用。

【线程与状态】
纯计算；不访问 GUI。

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
孔口支撑面、深度/孔径、孔口轮廓，以及当前直孔/锥孔几何计算主链。

维护说明：
本文件按职责合并相互紧密的子模块。维护时请按中文功能分区定位逻辑；同一职责优先在现有分区内扩展，避免把连续算法拆成过细文件。
*/

// ============================================================================
// 功能分区：几何公共工具接口
// ============================================================================
/*
模块职责：
孔几何公共工具。

主要调用位置：
由 HoleShibie_Recognition.cpp 及孔识别子模块复用，提供不持有状态的几何计算。

维护说明：
这里只放可复用纯计算；阈值应由调用模块显式传入或在敏感参数旁说明。
*/
#include "DianYunLeixing_Types.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace holeJihe {

float kasaZhongWeiShu(std::vector<float>& v);

bool kasaNiHeYuan(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius);

void kasaJiaoJunBianJie(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,     float cx, float cy, float cz, float r, float zHalf, int bins,     std::vector<Eigen::Vector2f>& outPts, bool quZuiYuan);

bool kasaJieDuanNiHe(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius, float& medErr);

bool ransacNiHeYuanSanDian(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius, float inlierDist, int maxIter = 80);

bool ransacNiHeSanWeiYuan(const std::vector<Eigen::Vector3f>& pts,     Eigen::Vector3f& center, Eigen::Vector3f& normal, float& radius,     float inlierDist, int maxIter);

float shouDongZhongWeiShu(std::vector<float>& values);

float shouDongFenWeiShu(std::vector<float>& values, float q);

}

// ============================================================================
// 功能分区：支撑面估计接口
// ============================================================================
/*
模块职责：
孔口支撑面计算子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，为孔口中心、法向和深度提供局部参考面。

维护说明：
平面内点距离、迭代次数和采样范围属于敏感参数，应注明毫米单位及增减后果。
*/

namespace holeZhicheng {

float shouDongZhuZaiFaXiangGao(const CloudPtr& localCloud);

float shouDongJinLinZhuZaiFaXiangGao(const CloudPtr& localCloud, float cx, float cy, float radius);

float shouDongJinLinZhiChengMianFaXiangGao(     const CloudPtr& localCloud,     float cx,     float cy,     float radius,     bool& outPlaneUsed,     int& outSupportSectors,     float& outResidual);

}

// ============================================================================
// 功能分区：孔深与孔壁接口
// ============================================================================
/*
模块职责：
孔深与孔壁几何子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，处理沿孔轴方向的深度/内孔证据。

维护说明：
深度搜索范围、层间距和半径阈值会直接改变测量结果，不能在纯整理轮中改默认值。
*/
#include <array>

namespace holeShendu {

/** 【类型导航注释】
 * ShouDongFenCengTongJi：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShouDongFenCengTongJi {
    float t = 0.0f;
    float z = 0.0f;
    float radius = 0.0f;
    float radiusInner = 0.0f;
    float radiusOuter = 0.0f;
    int pts = 0;
    int sectors = 0;
    int maxSectorGap = 16;
};

/** 【类型导航注释】
 * ShouDongJingXiangShenDu：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShouDongJingXiangShenDu {
    bool valid = false;
    float innerMed = 0.0f;
    float ringMed = 0.0f;
    float outerMed = 0.0f;
    float contrast = 0.0f;
    int innerPts = 0;
    int ringPts = 0;
    int outerPts = 0;
    float ringCoverage = 0.0f;
    int ringSectorCount = 0;
    int ringMaxSectorGap = 16;
};

bool shouDongFenCengBanJing(     const CloudPtr& localCloud,     float cx,     float cy,     float seedR,     float z,     float halfThickness,     ShouDongFenCengTongJi& out,     float zCeil = std::numeric_limits<float>::max());

float shouDongZhongWeiBanJing(const std::vector<ShouDongFenCengTongJi>& layers, size_t begin, size_t end, int radiusMode = 0);

float shouDongJinLinShenDuBanJing(const std::vector<ShouDongFenCengTongJi>& layers, float targetT, float band, int radiusMode = 0);

ShouDongJingXiangShenDu shouDongJingXiangShenDu(     const CloudPtr& localCloud,     float cx,     float cy,     float radius,     float zCeil = std::numeric_limits<float>::max());

}

// ============================================================================
// 功能分区：孔口轮廓接口
// ============================================================================
/*
模块职责：
孔口轮廓计算子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，提取和整理物理孔口轮廓证据。

维护说明：
半径、覆盖率和轮廓门限会影响最终孔口几何，修改时必须保持正式 comparator 约束。
*/
#include "HoleLeixing_Types.h"
#include <queue>
#include <pcl/kdtree/kdtree_flann.h>

namespace holeKou {

using holeShendu::ShouDongFenCengTongJi;

/** 【类型导航注释】
 * ShouDongZhuiHoleKouBaoLuoJingXiu：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShouDongZhuiHoleKouBaoLuoJingXiu {
    bool usable = false;
    float radius = 0.0f;
    float grow = 0.0f;
    float score = 0.0f;
    float spread = 0.0f;
    float shrink = 0.0f;
    float coverage = 0.0f;
    int layers = 0;
};

/** 【类型导航注释】
 * ShouDongZhuiBiKouBuWaiTui：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShouDongZhuiBiKouBuWaiTui {
    bool usable = false;
    float radius = 0.0f;
    float grow = 0.0f;
    float score = 0.0f;
    float slope = 0.0f;
    float rms = 0.0f;
    int layers = 0;
};

/** 【类型导航注释】
 * ShouDongWangGeHoleKou：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShouDongWangGeHoleKou {
    bool executed = false;
    bool stable = false;
    float cx = 0.0f;
    float cy = 0.0f;
    float radius = 0.0f;
    float score = 0.0f;
    float residual = 0.0f;
    float centerShift = 0.0f;
    int voidCells = 0;
    int edgeCells = 0;
    int sectors = 0;
};

ShouDongZhuiHoleKouBaoLuoJingXiu shouDongGuJiZhuiHoleKouBuBaoLuo(     const std::vector<ShouDongFenCengTongJi>& layers,     float currentRTop,     float seedMaxRadius);

ShouDongZhuiBiKouBuWaiTui shouDongGuJiZhuiBiKouBuWaiTui(     const std::vector<ShouDongFenCengTongJi>& layers,     float currentOutputRTop,     float seedMaxRadius);

ShouDongWangGeHoleKou guJiWangGeHoleKouYinYing(     const CloudPtr& localCloud,     float seedCx,     float seedCy,     float seedR,     float mouthZ,     const ShouDongHoleSeed& seed);

ShouDongWangGeHoleKou guJiDaBanJingHoleKouWaiBianJie(     const CloudPtr& localCloud,     float seedCx,     float seedCy,     float seedR,     float mouthZ,     const ShouDongHoleSeed& seed);

}

// ============================================================================
// 功能分区：表面与截面基础几何
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/
#include <string>

namespace HoleJiheSurfacePouMian {

/** 【类型导航注释】
 * Sample：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

/** 【类型导航注释】
 * Input：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double topU = 0.0;
    double topV = 0.0;
    double topW = 0.0;
    double initialTopRadius = 0.0;

    double maxCenterShift = 6.0;
    double centerShiftPenalty = 0.02;
};

/** 【类型导航注释】
 * Layer：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Layer {
    double depth = 0.0;
    double radius = 0.0;
    double coverage = 0.0;
    double span = 0.0;
    int points = 0;
    int sectors = 0;
};

/** 【类型导航注释】
 * Result：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool surfaceValid = false;
    bool profileValid = false;
    bool bottomValid = false;

    int holeType = 0;
    int inwardPolarity = 0;

    double centerU = 0.0;
    double centerV = 0.0;
    double centerShift = 0.0;

    double initialRadius = 0.0;
    double surfaceRadius = 0.0;
    double surfaceMad = 0.0;
    int surfaceSectors = 0;

    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double radiusSlope = 0.0;
    double radiusShrink = 0.0;
    double monotonicRatio = 0.0;
    double confidence = 0.0;

    int validLayers = 0;
    std::vector<Layer> layers;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);

}

// ============================================================================
// 功能分区：内壁连续跟踪
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheNeiWallTrack {

/** 【类型导航注释】
 * Sample：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

/** 【类型导航注释】
 * Input：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double topU = 0.0;
    double topV = 0.0;
    double topW = 0.0;
    double seedU = 0.0;
    double seedV = 0.0;
    double initialTopRadius = 0.0;
};

/** 【类型导航注释】
 * Layer：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Layer {
    double depth = 0.0;
    double radius = 0.0;
    double coverage = 0.0;
    double span = 0.0;
    double radialMad = 0.0;
    int points = 0;
    int sectors = 0;
};

/** 【类型导航注释】
 * Result：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool surfaceValid = false;
    bool profileValid = false;
    bool bottomValid = false;

    int holeType = 0;
    int inwardPolarity = 0;

    double centerU = 0.0;
    double centerV = 0.0;
    double centerShift = 0.0;
    double seedDistance = 0.0;
    double topW = 0.0;
    double axialShift = 0.0;

    double initialRadius = 0.0;
    double surfaceRadius = 0.0;
    double surfaceMad = 0.0;
    int surfaceSectors = 0;

    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double radiusSlope = 0.0;
    double radiusShrink = 0.0;
    double monotonicRatio = 0.0;
    double confidence = 0.0;

    int validLayers = 0;
    int candidateCount = 0;
    std::vector<Layer> layers;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);

}

// ============================================================================
// 功能分区：锥孔保守补充判定
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheZhuiBuJiu {

/** 【类型导航注释】
 * Sample：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

/** 【类型导航注释】
 * Input：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double topU = 0.0;
    double topV = 0.0;
    double topW = 0.0;
    double seedU = 0.0;
    double seedV = 0.0;
    double initialTopRadius = 0.0;

    double baselineMaxCenterShift = 6.0;
    double baselineCenterShiftPenalty = 0.02;
};

/** 【类型导航注释】
 * Layer：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Layer {
    double depth = 0.0;
    double radius = 0.0;
    double coverage = 0.0;
    double span = 0.0;
    int points = 0;
    int sectors = 0;
};

/** 【类型导航注释】
 * Result：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool surfaceValid = false;
    bool profileValid = false;
    bool bottomValid = false;
    bool coneHuiFuUsed = false;

    int holeType = 0;
    int inwardPolarity = 0;
    int baselineType = 0;
    int huiFuType = 0;

    double centerU = 0.0;
    double centerV = 0.0;
    double centerShift = 0.0;
    double seedDistance = 0.0;

    double initialRadius = 0.0;
    double surfaceRadius = 0.0;
    double surfaceMad = 0.0;
    int surfaceSectors = 0;

    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double radiusSlope = 0.0;
    double radiusShrink = 0.0;
    double monotonicRatio = 0.0;
    double confidence = 0.0;

    int validLayers = 0;
    int huiFuLayers = 0;
    double huiFuTopRadius = 0.0;
    double huiFuBottomRadius = 0.0;
    double huiFuDepth = 0.0;
    double huiFuSlope = 0.0;
    double huiFuShrink = 0.0;
    double huiFuMonotonicRatio = 0.0;
    double huiFuConfidence = 0.0;
    double huiFuRadiusDelta = 0.0;

    std::vector<Layer> layers;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);

}

// ============================================================================
// 功能分区：中心与半径细化
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheRadiusJingXiu {

/** 【类型导航注释】
 * Sample：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

/** 【类型导航注释】
 * Input：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double topU = 0.0;
    double topV = 0.0;
    double topW = 0.0;
    double seedU = 0.0;
    double seedV = 0.0;
    double initialTopRadius = 0.0;
    double baselineMaxCenterShift = 6.0;
    double baselineCenterShiftPenalty = 0.02;
};

/** 【类型导航注释】
 * Layer：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Layer {
    double depth = 0.0;
    double radius = 0.0;
    double coverage = 0.0;
    double span = 0.0;
    int points = 0;
    int sectors = 0;
};

/** 【类型导航注释】
 * Result：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool surfaceValid = false;
    bool profileValid = false;
    bool bottomValid = false;
    bool coneHuiFuUsed = false;
    bool radiusRefineUsed = false;
    bool centerRefineUsed = false;

    int holeType = 0;
    int inwardPolarity = 0;
    int baselineType = 0;
    int huiFuType = 0;
    int refineType = 0;

    double centerU = 0.0;
    double centerV = 0.0;
    double centerShift = 0.0;
    double seedDistance = 0.0;

    double initialRadius = 0.0;
    double surfaceRadius = 0.0;
    double surfaceMad = 0.0;
    int surfaceSectors = 0;

    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double radiusSlope = 0.0;
    double radiusShrink = 0.0;
    double monotonicRatio = 0.0;
    double confidence = 0.0;

    int validLayers = 0;
    int huiFuLayers = 0;
    double huiFuTopRadius = 0.0;
    double huiFuBottomRadius = 0.0;
    double huiFuDepth = 0.0;
    double huiFuSlope = 0.0;
    double huiFuShrink = 0.0;
    double huiFuMonotonicRatio = 0.0;
    double huiFuConfidence = 0.0;
    double huiFuRadiusDelta = 0.0;

    double radiusBeforeRefine = 0.0;
    double refinedRadius = 0.0;
    double adjustedSurfaceRadius = 0.0;
    double refineCenterShift = 0.0;
    double refineSurfaceRadius = 0.0;
    double refineSurfaceMad = 0.0;
    int refineSurfaceSectors = 0;
    int refineLayers = 0;
    double refineSlope = 0.0;
    double refineShrink = 0.0;
    double refineMonotonicRatio = 0.0;

    std::vector<Layer> layers;
    std::string radiusMode;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);

}

// ============================================================================
// 功能分区：最终孔几何接口
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheFinal {

/** 【类型导航注释】
 * Sample：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

/** 【类型导航注释】
 * Input：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double topU = 0.0;
    double topV = 0.0;
    double topW = 0.0;
    double seedU = 0.0;
    double seedV = 0.0;
    double initialTopRadius = 0.0;

    bool canonicalMode = false;
    double canonicalCenterU = 0.0;
    double canonicalCenterV = 0.0;
};

/** 【类型导航注释】
 * Result：Hole 几何接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool surfaceValid = false;
    bool profileValid = false;
    bool bottomValid = false;
    bool partialCone = false;

    bool axialAnchorValid = false;
    bool axialAnchorUsed = false;
    double inputTopW = 0.0;
    double topW = 0.0;
    double axialShift = 0.0;
    double axialMad = 0.0;
    int axialSupport = 0;
    int axialCandidateCount = 0;
    double axialCandidateOffset = 0.0;

    bool canonicalMode = false;
    bool rawSeedGateUsed = false;
    bool rawSeedScoreUsed = false;
    double canonicalCenterError = 0.0;

    bool voidValid = false;
    bool voidRecoveryUsed = false;
    bool voidRelaxedSearch = false;
    double voidCenterU = 0.0;
    double voidCenterV = 0.0;
    double voidCenterShift = 0.0;
    double voidRadius = 0.0;
    double voidCircularity = 0.0;
    double voidMinDistance = 0.0;
    double voidSeedBoundaryError = 0.0;
    int voidAreaCells = 0;
    bool voidTouchesGrid = false;
    int voidCandidateCount = 0;

    bool shallowConeHuiFuUsed = false;
    std::string shallowConeMode;

    bool coneCenterRecoveryUsed = false;
    double coneCenterShift = 0.0;
    double coneCenterRadius = 0.0;
    int coneCenterLayers = 0;

    bool wallEvidenceValid = false;
    int wallEvidenceLayers = 0;
    int wallEvidencePolarity = 0;
    double wallEvidenceSlope = 0.0;
    double wallEvidenceShrink = 0.0;
    double wallEvidenceMonotonic = 0.0;
    bool wallEvidenceCone = false;
    bool wallEvidenceStable = false;

    int holeType = 0;
    int inwardPolarity = 0;
    double centerU = 0.0;
    double centerV = 0.0;
    double centerShift = 0.0;
    double initialRadius = 0.0;
    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double confidence = 0.0;

    HoleJiheRadiusJingXiu::Result base;
    std::string takeoverMode;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);

}

