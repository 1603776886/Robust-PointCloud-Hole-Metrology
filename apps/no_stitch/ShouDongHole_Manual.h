/*
================================================================================
文件：ShouDongHole_Manual.h
模块：手动 Hole 内部接口

【主要职责】
声明 seed 局部 mask、候选、Hole 口检测、局部几何和位姿支撑的数据结构与函数。

【主要调用关系】
HoleShibie_Recognition 调用；两个实现 cpp 分别负责几何检测与位姿支撑。

【线程与状态】
纯计算。

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
统一声明手动选孔所需的数据结构和公开计算接口，包括二维孔口检测、点级几何、支撑面、位姿和 PCL 适配。

主要调用位置：
HoleShibie_Recognition.cpp 是上层调用者；各 ShouDongHole_*.cpp 按几何检测与位姿支持职责实现本头文件接口。

维护说明：
本头文件只放跨实现文件共享的数据和公开函数。局部辅助函数留在对应 .cpp，避免再次拆成大量只含少量代码的小文件。
*/

// ============================================================================
// 功能分区：手动孔核心数据与流程
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace shouDongHole {

/** 【类型导航注释】
 * ErZhiMask：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ErZhiMask {
    int width = 0;
    int height = 0;
    double resolution = 0.25;
    double originX = 0.0;
    double originY = 0.0;
    std::vector<std::uint8_t> data;

    /** 【函数导航】
     * 作用：执行“valid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：手动 Hole 内部接口。
     * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、DianYun_IO.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool valid() const noexcept {
        return width > 0 && height > 0 && data.size() == static_cast<std::size_t>(width * height);
    }
    /** 【函数导航】
     * 作用：执行“at”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：手动 Hole 内部接口。
     * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp、HoleCanshuShuchu_Export.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::uint8_t at(int x, int y) const noexcept {
        return data[static_cast<std::size_t>(y * width + x)];
    }
};

/** 【类型导航注释】
 * HouXuanCenter：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HouXuanCenter {
    double px = 0.0;
    double py = 0.0;
    std::string source;
    double distanceValue = 0.0;
};

/** 【类型导航注释】
 * MuBanJieGuo：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct MuBanJieGuo {
    bool valid = false;
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    double score = -1e30;
    double ring = 0.0;
    double inner = 1.0;
    double core = 1.0;
    double ratio = 0.0;
    std::string proposalSource;
};

/** 【类型导航注释】
 * GaoduLevelJieGuo：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct GaoduLevelJieGuo {
    MuBanJieGuo circle;
    double shift = 0.0;
    int surfacePointCount = 0;
    double rawScore = -1e30;
    double levelScore = -1e30;
};

/** 【类型导航注释】
 * LunKuoHouXuan：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct LunKuoHouXuan {
    std::string kind;
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    double correctedRadius = 0.0;
    double rmse = 0.0;
    double coverage = 0.0;
    double circularity = 0.0;
    double ratio = 0.0;
    double polarity = 0.0;
    double score = -1e30;
};

std::vector<LunKuoHouXuan> extractLunKuoHouXuan(
    const ErZhiMask& mask,
    double seedX,
    double seedY);

bool choosePositiveJiXingLunKuo(
    const std::vector<LunKuoHouXuan>& candidates,
    double resolution,
    LunKuoHouXuan& chosen);

MuBanJieGuo scoreTemplateExhaustive(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    double searchRadius = 13.0,
    double minRadius = 1.5,
    double maxRadius = 9.2);

MuBanJieGuo crosscheckIncompleteLunKuo(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    const LunKuoHouXuan& contour);

/** 【类型导航注释】
 * HoleKouLevelResult：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleKouLevelResult {
    bool valid = false;
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    std::string source;
    LunKuoHouXuan contour;
    MuBanJieGuo templateResult;
};

HoleKouLevelResult detectHoleKouAtLevel(
    const ErZhiMask& mask,
    double seedX,
    double seedY);

std::vector<HouXuanCenter> proposeHouXuanCenters(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    double searchRadius = 13.0,
    int maxCandidates = 30);

MuBanJieGuo scoreSparseTemplate(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    const std::vector<HouXuanCenter>& candidates,
    double searchRadius = 13.0,
    double minRadius = 1.5,
    double maxRadius = 9.2);

double cheapGaoduLevelScore(int surfacePointCount, double shift) noexcept;

GaoduLevelJieGuo attachGaoduScore(
    const MuBanJieGuo& circle,
    double shift,
    int surfacePointCount,
    double seedX,
    double seedY) noexcept;

const GaoduLevelJieGuo* chooseBestLevel(const std::vector<GaoduLevelJieGuo>& levels) noexcept;

}

// ============================================================================
// 功能分区：手动选点与邻域处理
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/

namespace shouDongHole {

/** 【类型导航注释】
 * Point3d：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Point3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/** 【类型导航注释】
 * PingMianModel：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingMianModel {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

/** 【类型导航注释】
 * Vec3d：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 1.0;
};

/** 【类型导航注释】
 * CircleJihe：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CircleJihe {
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    std::size_t supportPoints = 0;
};

/** 【类型导航注释】
 * NormalJihe：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct NormalJihe {
    Vec3d normal;
    double tiltDegrees = 0.0;
    double deltaFromGlobalDegrees = 0.0;
    std::size_t supportPoints = 0;
};

/** 【类型导航注释】
 * JingXiuHoleKouJihe：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JingXiuHoleKouJihe {
    CircleJihe surfaceFit;
    CircleJihe acceptedSurface;
    CircleJihe finalCircle;
    NormalJihe normalFit;
    bool depthCenterUsed = false;
};

ErZhiMask makeSurfaceMask(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& plane,
    double halfWidth = 21.0,
    double resolution = 0.25,
    double planeTolerance = 0.42,
    int* surfacePointCount = nullptr);

CircleJihe refineCircleSurface(
    const std::vector<Point3d>& points,
    double centerX,
    double centerY,
    double radius,
    const PingMianModel& plane,
    double planeTolerance = 0.48);

CircleJihe refineCenterAtFirstDepth(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& mouthPlane,
    double centerX,
    double centerY,
    double radius);

NormalJihe fitSectorBalancedSurfaceNormal(
    const std::vector<Point3d>& points,
    double centerX,
    double centerY,
    double radius,
    const Vec3d& globalNormal,
    const PingMianModel& mouthPlane,
    double planeTolerance = 0.35,
    double maxDeltaDegrees = 8.0);

JingXiuHoleKouJihe refineMouthGeometry(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& mouthPlane,
    const Vec3d& globalNormal,
    double mouthCenterX,
    double mouthCenterY,
    double mouthRadius,
    const std::string& mouthSource);

/** 【类型导航注释】
 * HoleJingXiu：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleJingXiu {
    bool valid = false;
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    int layerCount = 0;
    double depthBegin = 0.0;
    double depthEnd = 0.0;
    double residual = 0.0;
    bool stableOrderRetry = false;
    int stableOrderMode = 0;
    bool stableRetryEligible = false;
    int detectedLayerCount = 0;
    int deepSmallLayerCount = 0;
    int shallowOuterWallLayerCount = 0;
};

HoleJingXiu refineChamferedStraightBore(
    const std::vector<Point3d>& points,
    const PingMianModel& mouthPlane,
    double mouthCenterX,
    double mouthCenterY,
    double mouthRadius);

std::vector<Point3d> loadXyz64DianYun(const std::string& path);

}

// ============================================================================
// 功能分区：手动孔检测器
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/

namespace shouDongHole {

/** 【类型导航注释】
 * GaoduTrialState：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_Manual.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct GaoduTrialState {
    double shift = 0.0;
    int surfacePointCount = 0;
    bool hasContour = false;
    double contourScore = -1e30;
    double levelScore = -1e30;
};

/** 【类型导航注释】
 * FullJianCeResult：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct FullJianCeResult {
    bool valid = false;
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    Vec3d normal;
    double tiltDegrees = 0.0;
    double selectedShift = 0.0;
    double rawScore = -1e30;
    double levelScore = -1e30;
    int surfacePointCount = 0;
    std::string source;
    bool depthCenterUsed = false;
    // ActivePath 活动路径内部证据
    bool persistentFamilyValid = false;
    double persistentFamilyRadius = 0.0;

    double persistentFamilyCenterX = 0.0;
    double persistentFamilyCenterY = 0.0;
    double persistentFamilyCenterZ = 0.0;
    int persistentFamilyLevels = 0;
    double persistentFamilyRawScore = -1e30;
    double primaryMouthRadius = 0.0;
    double primaryMouthRmse = 0.0;
    double primaryMouthPolarity = 0.0;
    double primaryMouthCoverage = 0.0;
    double persistentFamilyCenterGap = 0.0;
    int outerHuiFuGateMask = 0;
    bool boreEvidenceEvaluated = false;
    bool boreEvidenceValid = false;

    std::vector<double> heightHypotheses;
    std::vector<GaoduTrialState> trials;
};

std::vector<double> generateHeightHypotheses(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal,
    double radialDistance = 14.0,
    double low = -2.5,
    double high = 2.5,
    double binWidth = 0.15,
    int maxPeaks = 4);

FullJianCeResult detectShouDongHoleJihe(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal);

}

// ============================================================================
// 功能分区：孔口支撑面
// ============================================================================
/*
模块职责：
工程辅助模块。

主要调用位置：
由项目内相邻业务模块按接口调用；具体入口以头文件声明和调用点为准。

维护说明：
整理目标是降低耦合和提高可读性；未经过专项验证，不改变已有算法默认值、数据顺序和外部接口语义。
*/

namespace shouDongHole {
namespace MouthSupportPlane {

/** 【类型导航注释】
 * Result：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    Point3d correctedCenter;
    Vec3d normal;

    double axialShift = 0.0;
    double planeRmse = 0.0;
    double planeMad = 0.0;
    double planeCoverage = 0.0;
    int planeSupport = 0;
    double normalDeltaDegrees = 0.0;

    double innerBelowFraction = 0.0;
    double innerNearFraction = 0.0;
    double innerAboveFraction = 0.0;
    double outerNearFraction = 0.0;

    int selectedBand = -1;
    double selectedScore = 0.0;
    std::string reason;
};

Result estimate(
    const std::vector<Point3d>& points,
    const Point3d& candidateCenter,
    const Vec3d& referenceNormal,
    double mouthRadius);

}
}

// ============================================================================
// 功能分区：手动孔位姿
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/

namespace shouDongHole {

/** 【类型导航注释】
 * LocalPlaneState：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct LocalPlaneState {
    bool valid = false;
    int candidatePoints = 0;
    int inlierPoints = 0;
    double inlierRatio = 0.0;
    double medianResidual = 0.0;
    double planeRatio = 0.0;
    double deltaFromGlobalDegrees = 0.0;
    double tiltDegrees = 0.0;
    Vec3d normal;
    std::string reason;
};

/** 【类型导航注释】
 * WeiziJianCeResult：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct WeiziJianCeResult {
    bool valid = false;
    FullJianCeResult geometry;
    double centerZ = 0.0;
    double initialCenterZ = 0.0;
    bool usedLocalPose = false;
    bool usedMouthSupportPlane = false;
    LocalPlaneState localPlane;
    MouthSupportPlane::Result mouthSupportPlane;
};

LocalPlaneState estimateLocalSupportPlane(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal,
    double radius = 14.0,
    double threshold = 0.18,
    int iterations = 80,
    std::size_t maxPoints = 2000);

WeiziJianCeResult detectShouDongHoleWeizi(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal);

}

// ============================================================================
// 功能分区：PCL 适配层
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/
#include "DianYunLeixing_Types.h"

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace shouDongHole {

/** 【类型导航注释】
 * PclJianCeResult：手动 Hole 内部接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PclJianCeResult {
    FullJianCeResult geometry;
    double centerZ = 0.0;
    double initialCenterZ = 0.0;
    bool usedLocalPose = false;
    bool usedMouthSupportPlane = false;
    LocalPlaneState localPlane;
    MouthSupportPlane::Result mouthSupportPlane;
    Vec3d globalNormal;
    std::size_t finitePointCount = 0;
    std::string error;

    /** 【函数导航】
     * 作用：执行“valid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：手动 Hole 内部接口。
     * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、DianYun_IO.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool valid() const { return error.empty() && geometry.valid; }
};

// 对一份点云只建立一次XYZ缓存和整云鲁棒法向。位姿模块会先检查点击附近
// 是否存在可信倾斜支撑面；可信时转入局部坐标，否则保持全局坐标路径。
PclJianCeResult detectShouDongHoleJihePcl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& seedPoint);

// 供内部状态读取当前缓存所使用的整云法向。
Vec3d estimateGlobalNormalDeterministicPcl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

Vec3d estimateGlobalNormalDeterministicPclWithTree(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZRGB>& existingTree);

Vec3d estimateGlobalNormalDeterministicPclWithTreeUncached(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZRGB>& existingTree);

}

