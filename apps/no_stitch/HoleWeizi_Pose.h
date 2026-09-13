/*
================================================================================
文件：HoleWeizi_Pose.h
模块：Hole 位姿与孔轴计算

【主要职责】
处理“已经有 Hole 几何以后”的位姿计算：Hole 轴、内喉、锥壁、连续锥孔、最终表面/孔轴模型等。
法线基础算法已经拆到 HoleFaXian_Normal.h，本文件不再同时承担“粗法线 + 椭圆法线”的实现细节。

【主要调用关系】
HoleShibie_Recognition.cpp 负责总编排；HoleFenxi_Analysis.cpp、HoleJihe_Geometry.cpp 等模块复用这里的位姿结果。

【维护原则】
- HoleFaXian_Normal.h：只回答“法线朝哪里”。
- HoleWeizi_Pose.h：回答“Hole 轴、孔壁、内喉、锥壁如何组合成最终位姿”。
- 识别阈值、ROI、迭代次数和浮点表达式都属于生产行为；仅做结构整理时不得顺手修改。
================================================================================
*/
#pragma once

#include "HoleFaXian_Normal.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

// ============================================================================
// 功能分区：Hole 轴后拟合
// ============================================================================
/*
模块职责：
工程辅助模块。

主要调用位置：
由项目内相邻业务模块按接口调用；具体入口以头文件声明和调用点为准。

维护说明：
整理目标是降低耦合和提高可读性；未经过专项验证，不改变已有算法默认值、数据顺序和外部接口语义。
*/

namespace HoleAxisHouFit {

using HoleFaXian::PingMianJieGuo;
using HoleFaXian::DianYangBen;

/** 【类型导航注释】
 * Result：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    int quality = 0;
    int supportCount = 0;
    int coveredSectors = 0;
    double coverage = 0.0;
    double rmse = 0.0;
    double mad = 0.0;
    double normalDeltaDegrees = 0.0;
    const char* mode = "POSTFIT_AXIS_UNAVAILABLE";
};

/** 【函数导航】
 * 作用：拟合/求解“fitTightCandidate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline PingMianJieGuo fitTightCandidate(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    double innerRadius,
    double outerRadius)
{
    PingMianJieGuo invalid;
    invalid.mode = "CanonicalPostfit_POSTFIT_TIGHT_INVALID";
    invalid.annulusInnerRadius = innerRadius;
    invalid.annulusOuterRadius = outerRadius;
    if (neighborhood.empty() || !mouthCenter.allFinite()
        || !(innerRadius > 0.0) || !(outerRadius > innerRadius)) {
        return invalid;
    }
    if (!HoleFaXian::finiteVec(initialNormal))
        initialNormal = Eigen::Vector3d::UnitZ();
    initialNormal.normalize();

    Eigen::Vector3d axisU, axisV;
    HoleFaXian::makeAxes(initialNormal, axisU, axisV);

    /** 【类型导航注释】
     * WaiHuanYangBen：Hole 位姿计算中的自定义 结构体。
     * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct WaiHuanYangBen {
        DianYangBen sample;
        double axial = 0.0;
        int sector = 0;
    };
    constexpr int kSectors = 36;
    std::vector<WaiHuanYangBen> raw;
    raw.reserve(neighborhood.size());
    std::array<std::vector<double>, kSectors> sectorAxial;

    for (const DianYangBen& sample : neighborhood) {
        const Eigen::Vector3d delta = sample.point - mouthCenter;
        const double axial = delta.dot(initialNormal);
        if (axial < -1.20 || axial > 1.70) continue;
        const double u = delta.dot(axisU);
        const double v = delta.dot(axisV);
        const double radial = std::hypot(u, v);
        if (radial < innerRadius || radial > outerRadius) continue;
        double angle = std::atan2(v, u);
        if (angle < 0.0) angle += 2.0 * 3.14159265358979323846;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * 3.14159265358979323846) * kSectors)), 0, kSectors - 1);
        raw.push_back({sample, axial, sector});
        sectorAxial[static_cast<std::size_t>(sector)].push_back(axial);
    }

    auto robustFit = [&](const std::vector<DianYangBen>& annulus, const char* mode) {
        PingMianJieGuo result;
        result.mode = mode;
        result.annulusInnerRadius = innerRadius;
        result.annulusOuterRadius = outerRadius;
        if (annulus.size() < 50U) return result;
        const int minSupport = std::max(50, static_cast<int>(0.08 * annulus.size()));
        result = HoleFaXian::robustPlane(
            annulus, initialNormal, mouthCenter, outerRadius,
            minSupport, 10, 0.65, mode);
        result.radiusUsed = outerRadius;
        result.annulusInnerRadius = innerRadius;
        result.annulusOuterRadius = outerRadius;
        result.quality = HoleFaXian::classifyOuterAnnulusQuality(result);
        if (result.quality == 0) result.valid = false;
        return result;
    };

    std::array<double, kSectors> q70Target{};
    std::array<double, kSectors> topBandTarget{};
    std::array<unsigned char, kSectors> q70Usable{};
    std::array<unsigned char, kSectors> topBandUsable{};
    int q70Sectors = 0;
    int topBandSectors = 0;

    for (int sector = 0; sector < kSectors; ++sector) {
        auto& values = sectorAxial[static_cast<std::size_t>(sector)];
        if (values.size() < 3U) continue;
        std::sort(values.begin(), values.end());

        const std::size_t qIndex = static_cast<std::size_t>(std::floor(
            0.70 * static_cast<double>(values.size() - 1U)));
        q70Target[static_cast<std::size_t>(sector)] = values[qIndex];
        q70Usable[static_cast<std::size_t>(sector)] = 1;
        ++q70Sectors;

        const std::size_t minimumBandSupport = std::max<std::size_t>(
            3U, static_cast<std::size_t>(std::ceil(0.12 * values.size())));
        constexpr double kBandWidth = 0.30;
        std::size_t low = 0U;
        std::size_t selectedLow = 0U;
        std::size_t selectedHigh = 0U;
        bool found = false;
        for (std::size_t high = 0U; high < values.size(); ++high) {
            while (low < high && values[high] - values[low] > kBandWidth) ++low;
            if (high - low + 1U < minimumBandSupport) continue;
            selectedLow = low;
            selectedHigh = high;
            found = true;
        }
        if (found) {
            const std::size_t count = selectedHigh - selectedLow + 1U;
            const std::size_t middle = selectedLow + count / 2U;
            double target = values[middle];
            if ((count & 1U) == 0U)
                target = 0.5 * (values[middle - 1U] + values[middle]);
            topBandTarget[static_cast<std::size_t>(sector)] = target;
            topBandUsable[static_cast<std::size_t>(sector)] = 1;
            ++topBandSectors;
        }
    }

    auto collect = [&](const std::array<double, kSectors>& targets,
                       const std::array<unsigned char, kSectors>& usable,
                       int usableSectors) {
        std::vector<DianYangBen> annulus;
        if (usableSectors < 10) return annulus;
        annulus.reserve(raw.size());
        for (const WaiHuanYangBen& item : raw) {
            if (!usable[static_cast<std::size_t>(item.sector)]) continue;
            const double target = targets[static_cast<std::size_t>(item.sector)];
            if (item.axial < target - 0.30 || item.axial > target + 0.25) continue;
            annulus.push_back(item.sample);
        }
        return annulus;
    };

    const std::vector<DianYangBen> q70 = collect(q70Target, q70Usable, q70Sectors);
    const std::vector<DianYangBen> topBand = collect(
        topBandTarget, topBandUsable, topBandSectors);
    PingMianJieGuo q70Fit = robustFit(q70, "CanonicalPostfit_POSTFIT_Q70");
    PingMianJieGuo topFit = robustFit(topBand, "CanonicalPostfit_POSTFIT_TOP_BAND");

    if (q70Fit.valid && topFit.valid) {
        if (q70Fit.quality != topFit.quality)
            return q70Fit.quality > topFit.quality ? q70Fit : topFit;
        constexpr double kRmseTie = 0.002;
        if (q70Fit.rmse + kRmseTie < topFit.rmse) return q70Fit;
        if (topFit.rmse + kRmseTie < q70Fit.rmse) return topFit;
        return q70Fit.score >= topFit.score ? q70Fit : topFit;
    }
    if (q70Fit.valid) return q70Fit;
    if (topFit.valid) return topFit;

    std::vector<DianYangBen> rawSamples;
    rawSamples.reserve(raw.size());
    for (const WaiHuanYangBen& item : raw) rawSamples.push_back(item.sample);
    return robustFit(rawSamples, "CanonicalPostfit_POSTFIT_RAW_FALLBACK");
}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimate(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    double topRadius)
{
    Result out;
    if (neighborhood.empty() || !mouthCenter.allFinite() || !(topRadius > 0.0))
        return out;
    if (!HoleFaXian::finiteVec(initialNormal))
        initialNormal = Eigen::Vector3d::UnitZ();
    initialNormal.normalize();

    const double innerRadius = std::max(1.10 * topRadius, topRadius + 0.35);
    const double outerRadius = std::max(2.00 * topRadius, topRadius + 3.0);
    PingMianJieGuo fit = fitTightCandidate(
        neighborhood, mouthCenter, initialNormal, innerRadius, outerRadius);
    if (!fit.valid || fit.quality < 2 || !fit.normal.allFinite()) return out;

    Eigen::Vector3d normal = fit.normal.normalized();
    if (normal.dot(initialNormal) < 0.0) normal = -normal;
    out.valid = true;
    out.normal = normal;
    out.quality = fit.quality;
    out.supportCount = fit.supportCount;
    out.coveredSectors = fit.coveredSectors;
    out.coverage = fit.coverage;
    out.rmse = fit.rmse;
    out.mad = fit.mad;
    out.normalDeltaDegrees = fit.normalDeltaDegrees;
    out.mode = fit.mode;
    return out;
}

}

// ============================================================================
// 功能分区：孔口位姿基础估计
// ============================================================================
/*
模块职责：
孔口统一姿态估计模块。

主要调用位置：
由正式孔识别后段调用，综合孔壁和孔口证据形成规范姿态/半径结果。

维护说明：
该链处于生产几何热路径；候选顺序、评分、网格分辨率和阈值都属于冻结语义，纯整理不改。
*/
#include <Eigen/Cholesky>

//

namespace HoleWeiziBase {

using HoleFaXian::DianYangBen;
using HoleFaXian::PingMianJieGuo;

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr int kSectors = 72;
inline constexpr int kHistogramBins = 72;
inline constexpr int kMaximumSlices = 12;

/** 【类型导航注释】
 * QiePianNiHe：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct QiePianNiHe {
    bool valid = false;
    double depth = 0.0;
    double halfThickness = 0.0;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Vector3d centerWorld = Eigen::Vector3d::Zero();
    double radius = 0.0;
    int inputCount = 0;
    int supportCount = 0;
    int coveredSectors = 0;
    double coverage = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double score = -std::numeric_limits<double>::infinity();

    double axisResidual = std::numeric_limits<double>::infinity();
    double axisWeight = 0.0;
    bool axisUsed = false;
    bool transitionPolluted = false;
    bool axisRejected = false;
    const char* source = "NO_VALID_SLICE";
};

/** 【类型导航注释】
 * HoleBiModel：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleBiModel {
    bool valid = false;
    Eigen::Vector3d intercept = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisIn = -Eigen::Vector3d::UnitZ();
    int validSlices = 0;
    int totalSupport = 0;
    int medianCoveredSectors = 0;
    double meanCoverage = 0.0;
    double meanRmse = 0.0;
    double centerLineRmse = std::numeric_limits<double>::infinity();
    double allSliceCenterLineRmse = std::numeric_limits<double>::infinity();
    double axisInlierThreshold = 0.0;
    int axisUsedSlices = 0;
    int axisRejectedSlices = 0;
    int transitionSlices = 0;
    int stableStartSlice = 0;
    double radiusAtZero = 0.0;
    double radiusSlope = 0.0;
    double radiusMad = 0.0;
    double monotonicity = 0.0;
    double confidence = 0.0;
    std::vector<QiePianNiHe> slices;
};

/** 【类型导航注释】
 * WaiEvidence：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct WaiEvidence {
    bool valid = false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    int quality = 0;
    int supportCount = 0;
    int coveredSectors = 0;
    double coverage = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double mad = std::numeric_limits<double>::infinity();
    double confidence = 0.0;
    const char* source = "PhysicalMouthContour_OUTER_UNAVAILABLE";
};

/** 【类型导航注释】
 * HoleKouContourPoint：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleKouContourPoint {
    bool valid = false;
    bool evidenceUsed = false;
    int supportCount = 0;
    double angle = 0.0;
    double axialDepth = 0.0;
    double localRadius = 0.0;
    double planeResidual = 0.0;
    double evidenceResidual = 0.0;
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
};

/** 【类型导航注释】
 * HoleKouContour：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleKouContour {
    bool executed = false;
    bool valid = false;
    int validPoints = 0;
    double coverage = 0.0;
    double minimumAxialDepth = 0.0;
    double maximumAxialDepth = 0.0;
    double medianAxialDepth = 0.0;
    double axialSpan = 0.0;
    double minimumLocalRadius = 0.0;
    double maximumLocalRadius = 0.0;
    double planarRingMaximumPlaneResidual = 0.0;
    double maximumPlaneResidual = 0.0;
    double canonicalRadius = 0.0;
    double evidenceRadius = 0.0;
    double evidenceScale = std::numeric_limits<double>::infinity();
    double evidenceCenterShift = 0.0;
    int evidenceRawSectors = 0;
    int evidenceUsedSectors = 0;
    int evidenceSupportCount = 0;
    double confidence = 0.0;
    std::array<HoleKouContourPoint, kSectors> points{};
    const char* source = "PhysicalMouthContour_CONTOUR_UNAVAILABLE";
    const char* decision = "NOT_EXECUTED";
};

/** 【类型导航注释】
 * Result：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool executed = false;
    bool valid = false;
    // 位姿模块内部的中心候选。不同 evidence source 可能来自机械上口、内喉截面或孔壁模型；
    // HoleShibie_Recognition 在最终提交时负责把它与 canonical mouth 平面统一。
    // 因历史接口保留 centerTop 字段名，但维护时不能再把它理解为无条件的最终上口 Z。
    Eigen::Vector3d centerTop = Eigen::Vector3d::Zero();
    Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axisIn = -Eigen::Vector3d::UnitZ();
    double topRadius = 0.0;
    double originalRadius = 0.0;
    double radiusShift = 0.0;
    double wallRadiusAtZero = 0.0;
    double lockedMouthDepth = 0.0;
    bool semanticRadiusLocked = true;
    double wallRadiusWeight = 1.0;
    double centerEvidenceWeight = 1.0;
    double centerShift = 0.0;
    double axisCorrectionDegrees = 0.0;
    double confidence = 0.0;
    HoleBiModel wall;
    WaiEvidence outer;
    HoleKouContour contour;
    const char* source = "PhysicalMouthContour_WALL_SURFACE_EVIDENCE_WEAK";
};

/** 【函数导航】
 * 作用：执行“finiteVec”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool finiteVec(const Eigen::Vector3d& value) {
    return value.allFinite() && value.squaredNorm() > 1e-18;
}

/** 【函数导航】
 * 作用：执行“normalizeOr”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Eigen::Vector3d normalizeOr(Eigen::Vector3d value,
                                   const Eigen::Vector3d& tuoDi) {
    if (!finiteVec(value)) value = tuoDi;
    if (!finiteVec(value)) value = Eigen::Vector3d::UnitZ();
    value.normalize();
    return value;
}

/** 【函数导航】
 * 作用：构建“makeAxes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline void makeAxes(const Eigen::Vector3d& axis,
                     Eigen::Vector3d& u,
                     Eigen::Vector3d& v) {
    const Eigen::Vector3d a = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d helper = std::abs(a.x()) < 0.85
        ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    u = a.cross(helper);
    if (!finiteVec(u)) u = a.cross(Eigen::Vector3d::UnitZ());
    u.normalize();
    v = a.cross(u).normalized();
}

inline double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(),
                     values.begin() + static_cast<std::ptrdiff_t>(middle),
                     values.end());
    double out = values[middle];
    if ((values.size() & 1U) == 0U) {
        const auto lower = std::max_element(
            values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        out = 0.5 * (out + *lower);
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“medianInPlace”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double medianInPlace(std::vector<double>& values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(),
                     values.begin() + static_cast<std::ptrdiff_t>(middle),
                     values.end());
    double out = values[middle];
    if ((values.size() & 1U) == 0U) {
        const auto lower = std::max_element(
            values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        out = 0.5 * (out + *lower);
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“angleDegrees”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double angleDegrees(Eigen::Vector3d left, Eigen::Vector3d right) {
    left = normalizeOr(left, Eigen::Vector3d::UnitZ());
    right = normalizeOr(right, left);
    return std::acos(std::clamp(std::abs(left.dot(right)), 0.0, 1.0))
        * 180.0 / kPi;
}

/** 【函数导航】
 * 作用：执行“boundedAxisStep”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Eigen::Vector3d boundedAxisStep(
    const Eigen::Vector3d& current,
    Eigen::Vector3d candidate,
    double maximumStepDegrees)
{
    const Eigen::Vector3d from = normalizeOr(current, -Eigen::Vector3d::UnitZ());
    candidate = normalizeOr(candidate, from);
    if (candidate.dot(from) < 0.0) candidate = -candidate;
    const double cosine = std::clamp(from.dot(candidate), -1.0, 1.0);
    const double angle = std::acos(cosine);
    const double maximum = maximumStepDegrees * kPi / 180.0;
    if (!(angle > maximum) || angle < 1e-12) return candidate;
    const double t = maximum / angle;
    const double sine = std::sin(angle);
    if (std::abs(sine) < 1e-12)
        return normalizeOr((1.0 - t) * from + t * candidate, from);
    return normalizeOr(
        (std::sin((1.0 - t) * angle) / sine) * from
        + (std::sin(t * angle) / sine) * candidate, from);
}

/** 【类型导航注释】
 * TouYingPoint：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct TouYingPoint {
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
    double radius = 0.0;
};

/** 【函数导航】
 * 作用：执行“projectNeighborhood”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::vector<TouYingPoint> projectNeighborhood(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& axis,
    double maximumRadius,
    double minimumDepth,
    double maximumDepth)
{
    Eigen::Vector3d u, v;
    makeAxes(axis, u, v);
    std::vector<TouYingPoint> out;
    out.reserve(samples.size());
    for (const DianYangBen& sample : samples) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - origin;
        const double depth = delta.dot(axis);
        if (depth < minimumDepth || depth > maximumDepth) continue;
        const double x = delta.dot(u);
        const double y = delta.dot(v);
        const double radius = std::hypot(x, y);
        if (radius > maximumRadius) continue;
        out.push_back(TouYingPoint{x, y, depth, radius});
    }
    return out;
}

/** 【函数导航】
 * 作用：拟合/求解“solveCircleAlgebraic”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool solveCircleAlgebraic(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<unsigned char>& active,
    Eigen::Vector2d& center,
    double& radius)
{
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    int count = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!active.empty() && !active[i]) continue;
        const double x = points[i].x();
        const double y = points[i].y();
        const Eigen::Vector3d row(2.0 * x, 2.0 * y, 1.0);
        normal.noalias() += row * row.transpose();
        rhs.noalias() += row * (x * x + y * y);
        ++count;
    }
    if (count < 3) return false;
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(normal);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::Vector3d solution = ldlt.solve(rhs);
    if (!solution.allFinite()) return false;
    center = solution.head<2>();
    const double r2 = solution.z() + center.squaredNorm();
    if (!(r2 > 1e-8) || !std::isfinite(r2)) return false;
    radius = std::sqrt(r2);
    return std::isfinite(radius);
}

/** 【函数导航】
 * 作用：拟合/求解“fitCircleRobust”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline QiePianNiHe fitCircleRobust(
    const std::vector<Eigen::Vector2d>& points,
    double depth,
    double halfThickness,
    double maximumRadius)
{
    QiePianNiHe out;
    out.depth = depth;
    out.halfThickness = halfThickness;
    out.inputCount = static_cast<int>(points.size());
    if (points.size() < 14U) return out;

    std::vector<unsigned char> active(points.size(), 1U);
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double radius = 0.0;
    for (int iteration = 0; iteration < 5; ++iteration) {
        if (!solveCircleAlgebraic(points, active, center, radius)) return out;
        if (!(radius > 0.45) || radius > maximumRadius || !center.allFinite())
            return out;
        std::vector<double> residuals;
        residuals.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!active[i]) continue;
            residuals.push_back(std::abs((points[i] - center).norm() - radius));
        }
        if (residuals.size() < 12U) return out;
        const double residualMedian = median(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (double residual : residuals)
            deviations.push_back(std::abs(residual - residualMedian));
        const double scale = std::max(0.025, 1.4826 * median(deviations));
        const double limit = std::clamp(residualMedian + 2.8 * scale, 0.07, 0.28);
        int kept = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double residual = std::abs((points[i] - center).norm() - radius);
            active[i] = std::isfinite(residual) && residual <= limit ? 1U : 0U;
            kept += active[i] ? 1 : 0;
        }
        if (kept < 12) return out;
    }
    if (!solveCircleAlgebraic(points, active, center, radius)) return out;

    std::array<unsigned char, kSectors> sectors{};
    double squaredError = 0.0;
    int support = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!active[i]) continue;
        const Eigen::Vector2d delta = points[i] - center;
        const double radial = delta.norm();
        const double residual = radial - radius;
        squaredError += residual * residual;
        double angle = std::atan2(delta.y(), delta.x());
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        sectors[static_cast<std::size_t>(sector)] = 1U;
        ++support;
    }
    int covered = 0;
    for (unsigned char value : sectors) covered += value ? 1 : 0;
    const double coverage = static_cast<double>(covered)
        / static_cast<double>(kSectors);
    const double rmse = support > 0
        ? std::sqrt(squaredError / static_cast<double>(support))
        : std::numeric_limits<double>::infinity();
    if (support < 12 || coverage < 0.18 || rmse > 0.30) return out;

    out.valid = true;
    out.center = center;
    out.radius = radius;
    out.supportCount = support;
    out.coveredSectors = covered;
    out.coverage = coverage;
    out.rmse = rmse;
    out.score = 1.6 * coverage + 0.012 * std::sqrt(static_cast<double>(support))
        - 2.2 * rmse - 0.20 * halfThickness;
    return out;
}

/** 【函数导航】
 * 作用：拟合/求解“fitBestSlice”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline QiePianNiHe fitBestSlice(
    const std::vector<TouYingPoint>& projected,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& axis,
    double targetDepth,
    double expectedRadius,
    double maximumRadius)
{
    static constexpr std::array<double, 4> kHalfThicknesses{{0.20, 0.28, 0.38, 0.50}};
    QiePianNiHe best;
    auto consider = [&](const QiePianNiHe& fit) {
        if (!fit.valid) return;
        if (!best.valid) { best = fit; return; }
        const double scoreDelta = fit.score - best.score;
        const bool outerTie = std::abs(scoreDelta) <= 0.03
            && fit.radius > best.radius + 0.18
            && fit.coverage + 0.04 >= best.coverage
            && fit.rmse <= best.rmse + 0.025;
        if (scoreDelta > 0.03 || outerTie) best = fit;
    };
    Eigen::Vector3d u, v;
    makeAxes(axis, u, v);

    for (double halfThickness : kHalfThicknesses) {
        const double low = std::max(0.45, expectedRadius - 2.0);
        const double high = std::min(maximumRadius, expectedRadius + 2.0);
        if (!(high > low + 0.4)) continue;
        std::array<int, kHistogramBins> histogram{};
        int radialCount = 0;
        for (const TouYingPoint& point : projected) {
            if (std::abs(point.depth - targetDepth) > halfThickness
                || point.radius < low || point.radius > high) continue;
            const int bin = std::clamp(static_cast<int>(std::floor(
                (point.radius - low) / (high - low)
                * static_cast<double>(kHistogramBins))), 0, kHistogramBins - 1);
            ++histogram[static_cast<std::size_t>(bin)];
            ++radialCount;
        }
        if (radialCount < 20) continue;

        std::array<int, kHistogramBins> smoothed{};
        int maximumPeak = 0;
        for (int bin = 0; bin < kHistogramBins; ++bin) {
            int value = 0;
            for (int offset = -2; offset <= 2; ++offset) {
                const int index = std::clamp(bin + offset, 0, kHistogramBins - 1);
                value += (3 - std::abs(offset))
                    * histogram[static_cast<std::size_t>(index)];
            }
            smoothed[static_cast<std::size_t>(bin)] = value;
            maximumPeak = std::max(maximumPeak, value);
        }
        if (maximumPeak <= 0) continue;

        std::array<int, kHistogramBins> order{};
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int left, int right) {
            return smoothed[static_cast<std::size_t>(left)]
                > smoothed[static_cast<std::size_t>(right)];
        });
        std::vector<int> candidateBins;
        candidateBins.reserve(4U);
        for (int bin : order) {
            if (smoothed[static_cast<std::size_t>(bin)]
                < static_cast<int>(std::ceil(0.42 * maximumPeak))) break;
            bool separated = true;
            for (int existing : candidateBins)
                if (std::abs(existing - bin) < 4) separated = false;
            if (!separated) continue;
            candidateBins.push_back(bin);
            if (candidateBins.size() >= 3U) break;
        }

        int priorBin = -1;
        double priorPeak = -1.0;
        for (int bin = 0; bin < kHistogramBins; ++bin) {
            if (smoothed[static_cast<std::size_t>(bin)]
                < static_cast<int>(std::ceil(0.32 * maximumPeak))) continue;
            const double candidateRadius = low + (static_cast<double>(bin) + 0.5)
                / static_cast<double>(kHistogramBins) * (high - low);
            const double priorScale = std::max(0.35, 0.22 * expectedRadius);
            const double prior = std::exp(-0.5 * std::pow(
                (candidateRadius - expectedRadius) / priorScale, 2.0));
            const double value = smoothed[static_cast<std::size_t>(bin)]
                * (0.35 + 0.65 * prior);
            if (value > priorPeak) { priorPeak = value; priorBin = bin; }
        }
        if (priorBin >= 0) {
            bool separated = true;
            for (int existing : candidateBins)
                if (std::abs(existing - priorBin) < 3) separated = false;
            if (separated) candidateBins.push_back(priorBin);
        }

        double localBestCoverage = 0.0;
        for (int bin : candidateBins) {
            const double modeRadius = low + (static_cast<double>(bin) + 0.5)
                / static_cast<double>(kHistogramBins) * (high - low);
            const double radialBand = std::clamp(0.055 * modeRadius, 0.16, 0.36);
            std::vector<Eigen::Vector2d> points2d;
            points2d.reserve(256U);
            for (const TouYingPoint& point : projected) {
                if (std::abs(point.depth - targetDepth) <= halfThickness
                    && std::abs(point.radius - modeRadius) <= radialBand)
                    points2d.emplace_back(point.x, point.y);
            }
            QiePianNiHe fit = fitCircleRobust(points2d, targetDepth,
                                           halfThickness, maximumRadius);
            if (!fit.valid) continue;
            localBestCoverage = std::max(localBestCoverage, fit.coverage);
            fit.centerWorld = origin + axis * targetDepth
                + u * fit.center.x() + v * fit.center.y();
            fit.source = "RADIAL_MODE_MULTI_THICKNESS";
            const double priorPenalty = 0.08 * std::abs(fit.radius - expectedRadius)
                / std::max(0.50, 0.20 * expectedRadius);
            const double centerPenalty = 0.08 * std::max(
                0.0, fit.center.norm() - 0.15 * expectedRadius);
            fit.score = 1.70 * fit.coverage
                + 0.012 * std::sqrt(static_cast<double>(fit.supportCount))
                - 2.20 * fit.rmse - 0.20 * halfThickness
                - priorPenalty - centerPenalty;
            consider(fit);
        }

        if (localBestCoverage < 0.55) {
            const double broadBand = std::max(0.75, 0.18 * expectedRadius);
            std::vector<Eigen::Vector2d> broadPoints;
            broadPoints.reserve(512U);
            for (const TouYingPoint& point : projected) {
                if (std::abs(point.depth - targetDepth) <= halfThickness
                    && point.radius >= std::max(0.45, expectedRadius - broadBand)
                    && point.radius <= expectedRadius + broadBand)
                    broadPoints.emplace_back(point.x, point.y);
            }
            QiePianNiHe fit = fitCircleRobust(
                broadPoints, targetDepth, halfThickness, maximumRadius);
            if (fit.valid) {
                fit.centerWorld = origin + axis * targetDepth
                    + u * fit.center.x() + v * fit.center.y();
                fit.source = "WIDE_ANNULUS_PARTIAL_ARC_FUSION";
                const double priorPenalty = 0.10
                    * std::abs(fit.radius - expectedRadius)
                    / std::max(0.50, 0.20 * expectedRadius);
                const double centerPenalty = 0.06 * std::max(
                    0.0, fit.center.norm() - 0.30 * expectedRadius);
                fit.score = 1.65 * fit.coverage
                    + 0.012 * std::sqrt(static_cast<double>(fit.supportCount))
                    - 2.20 * fit.rmse - 0.20 * halfThickness
                    - priorPenalty - centerPenalty - 0.03;
                consider(fit);
            }
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：拟合/求解“fitDepthParameterizedLine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool fitDepthParameterizedLine(
    const std::vector<QiePianNiHe>& slices,
    const std::vector<double>& weights,
    Eigen::Vector3d& intercept,
    Eigen::Vector3d& slope)
{
    if (slices.size() < 2U || weights.size() != slices.size()) return false;
    double sumW = 0.0;
    double meanDepth = 0.0;
    Eigen::Vector3d meanCenter = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < slices.size(); ++i) {
        const double w = std::max(0.0, weights[i]);
        if (!(w > 0.0)) continue;
        sumW += w;
        meanDepth += w * slices[i].depth;
        meanCenter += w * slices[i].centerWorld;
    }
    if (!(sumW > 0.0)) return false;
    meanDepth /= sumW;
    meanCenter /= sumW;

    double denominator = 0.0;
    slope.setZero();
    for (std::size_t i = 0; i < slices.size(); ++i) {
        const double w = std::max(0.0, weights[i]);
        if (!(w > 0.0)) continue;
        const double ds = slices[i].depth - meanDepth;
        denominator += w * ds * ds;
        slope += w * ds * (slices[i].centerWorld - meanCenter);
    }
    if (!(denominator > 1e-10)) return false;
    slope /= denominator;
    intercept = meanCenter - slope * meanDepth;
    return intercept.allFinite() && slope.allFinite();
}

/** 【函数导航】
 * 作用：执行“orthogonalLineResiduals”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::vector<double> orthogonalLineResiduals(
    const std::vector<QiePianNiHe>& slices,
    const Eigen::Vector3d& intercept,
    const Eigen::Vector3d& axis)
{
    std::vector<double> residuals(slices.size(), 0.0);
    const Eigen::Vector3d unit = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    for (std::size_t i = 0; i < slices.size(); ++i) {
        const Eigen::Vector3d delta = slices[i].centerWorld - intercept;
        residuals[i] = (delta - unit * delta.dot(unit)).norm();
    }
    return residuals;
}

/** 【函数导航】
 * 作用：执行“longestContiguousRun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline int longestContiguousRun(const std::vector<unsigned char>& mask,
                                int& runStart)
{
    int best = 0;
    int current = 0;
    int currentStart = 0;
    runStart = 0;
    for (std::size_t i = 0; i < mask.size(); ++i) {
        if (mask[i]) {
            if (current == 0) currentStart = static_cast<int>(i);
            ++current;
            if (current > best) {
                best = current;
                runStart = currentStart;
            }
        } else {
            current = 0;
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“robustLineFit”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool robustLineFit(
    std::vector<QiePianNiHe>& slices,
    const Eigen::Vector3d& referenceAxis,
    Eigen::Vector3d& intercept,
    Eigen::Vector3d& axis,
    double& rmse,
    double& allSliceRmse,
    double& inlierThreshold,
    int& stableStartSlice,
    int& usedSlices,
    int& rejectedSlices,
    int& transitionSlices)
{
    const std::size_t n = slices.size();
    if (n < 2U) return false;

    std::vector<double> quality(n, 1.0);
    std::vector<double> uncertainty;
    uncertainty.reserve(n);
    std::vector<double> radii;
    radii.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double support = std::max(12.0,
            static_cast<double>(slices[i].supportCount)
            * std::max(0.20, slices[i].coverage));
        const double sigma = slices[i].rmse / std::sqrt(support);
        uncertainty.push_back(std::isfinite(sigma) ? sigma : 0.02);
        radii.push_back(slices[i].radius);
        const double denominator = slices[i].rmse * slices[i].rmse + 0.0025;
        quality[i] = std::max(1e-6,
            std::pow(std::max(0.10, slices[i].coverage), 1.5)
            * std::sqrt(std::max(1.0,
                static_cast<double>(slices[i].supportCount)))
            / denominator);
    }
    const double medianQuality = std::max(1e-9, median(quality));
    for (double& value : quality)
        value = std::clamp(value / medianQuality, 0.30, 3.00);

    // 这一行保留既有严格比较顺序。
    inlierThreshold = std::clamp(std::max({
        0.018,
        1.85 * median(uncertainty),
        0.0030 * std::max(0.5, median(radii))}), 0.018, 0.075);

    Eigen::Vector3d allIntercept = Eigen::Vector3d::Zero();
    Eigen::Vector3d allSlope = referenceAxis;
    if (!fitDepthParameterizedLine(slices, quality, allIntercept, allSlope))
        return false;
    Eigen::Vector3d allAxis = normalizeOr(allSlope, referenceAxis);
    if (allAxis.dot(referenceAxis) < 0.0) allAxis = -allAxis;
    const std::vector<double> allResiduals = orthogonalLineResiduals(
        slices, allIntercept, allAxis);
    double allSquared = 0.0;
    for (double value : allResiduals) allSquared += value * value;
    allSliceRmse = std::sqrt(allSquared / static_cast<double>(n));

    const int minimumRun = n < 4U ? static_cast<int>(n)
        : (n >= 6U ? 4 : static_cast<int>(n) - 1);
    bool foundCandidate = false;
    int bestRunLength = 0;
    int bestRunStart = 0;
    int bestBlockBegin = 0;
    int bestBlockEnd = static_cast<int>(n);
    double bestInlierQuality = -1.0;
    double bestBlockRmse = std::numeric_limits<double>::infinity();
    Eigen::Vector3d bestIntercept = allIntercept;
    Eigen::Vector3d bestAxis = allAxis;
    std::vector<double> bestResiduals = allResiduals;
    std::vector<unsigned char> bestInliers(n, 1U);

    for (int begin = 0; begin <= static_cast<int>(n) - minimumRun; ++begin) {
        for (int end = begin + minimumRun; end <= static_cast<int>(n); ++end) {
            std::vector<double> blockWeights(n, 0.0);
            for (int i = begin; i < end; ++i)
                blockWeights[static_cast<std::size_t>(i)] = quality[static_cast<std::size_t>(i)];
            Eigen::Vector3d candidateIntercept = Eigen::Vector3d::Zero();
            Eigen::Vector3d candidateSlope = referenceAxis;
            if (!fitDepthParameterizedLine(
                    slices, blockWeights, candidateIntercept, candidateSlope)) {
                continue;
            }
            Eigen::Vector3d candidateAxis = normalizeOr(
                candidateSlope, referenceAxis);
            if (candidateAxis.dot(referenceAxis) < 0.0)
                candidateAxis = -candidateAxis;
            const std::vector<double> residuals = orthogonalLineResiduals(
                slices, candidateIntercept, candidateAxis);
            std::vector<unsigned char> inliers(n, 0U);
            double inlierQuality = 0.0;
            for (std::size_t i = 0; i < n; ++i) {
                if (residuals[i] <= inlierThreshold) {
                    inliers[i] = 1U;
                    inlierQuality += quality[i];
                }
            }
            int runStart = 0;
            const int runLength = longestContiguousRun(inliers, runStart);
            double blockSquared = 0.0;
            for (int i = begin; i < end; ++i) {
                const double residual = residuals[static_cast<std::size_t>(i)];
                blockSquared += residual * residual;
            }
            const double blockRmse = std::sqrt(
                blockSquared / static_cast<double>(end - begin));

            const bool better = !foundCandidate
                || runLength > bestRunLength
                || (runLength == bestRunLength
                    && inlierQuality > bestInlierQuality + 1e-9)
                || (runLength == bestRunLength
                    && std::abs(inlierQuality - bestInlierQuality) <= 1e-9
                    && blockRmse < bestBlockRmse - 1e-12)
                || (runLength == bestRunLength
                    && std::abs(inlierQuality - bestInlierQuality) <= 1e-9
                    && std::abs(blockRmse - bestBlockRmse) <= 1e-12
                    && (end - begin) > (bestBlockEnd - bestBlockBegin));
            if (!better) continue;
            foundCandidate = true;
            bestRunLength = runLength;
            bestRunStart = runStart;
            bestBlockBegin = begin;
            bestBlockEnd = end;
            bestInlierQuality = inlierQuality;
            bestBlockRmse = blockRmse;
            bestIntercept = candidateIntercept;
            bestAxis = candidateAxis;
            bestResiduals = residuals;
            bestInliers = inliers;
        }
    }

    if (!foundCandidate || bestRunLength < minimumRun) {
        bestRunStart = 0;
        bestRunLength = static_cast<int>(n);
        bestIntercept = allIntercept;
        bestAxis = allAxis;
        bestResiduals = allResiduals;
        std::fill(bestInliers.begin(), bestInliers.end(), 1U);
    }

    stableStartSlice = bestRunStart;
    std::vector<unsigned char> active(n, 1U);
    std::vector<double> transitionWeight(n, 1.0);

    if (bestRunStart >= 2) {
        std::vector<double> stableResiduals;
        for (int i = bestRunStart;
             i < bestRunStart + bestRunLength && i < static_cast<int>(n); ++i) {
            stableResiduals.push_back(bestResiduals[static_cast<std::size_t>(i)]);
        }
        const double stableMedian = std::max(0.003, median(stableResiduals));
        std::vector<double> prefixResiduals;
        Eigen::Vector3d directionSum = Eigen::Vector3d::Zero();
        int directionCount = 0;
        for (int i = 0; i < bestRunStart; ++i) {
            const Eigen::Vector3d delta = slices[static_cast<std::size_t>(i)].centerWorld
                - bestIntercept;
            const Eigen::Vector3d transverse = delta
                - bestAxis * delta.dot(bestAxis);
            const double residual = transverse.norm();
            prefixResiduals.push_back(residual);
            if (residual > 1e-9) {
                directionSum += transverse / residual;
                ++directionCount;
            }
        }
        const double coherence = directionCount > 0
            ? directionSum.norm() / static_cast<double>(directionCount) : 0.0;
        const double prefixMedian = median(prefixResiduals);
        const bool coherentTransition = coherence >= 0.82
            && prefixMedian >= std::max(1.60 * inlierThreshold,
                                       3.0 * stableMedian)
            && bestBlockRmse <= 0.70 * std::max(allSliceRmse, 1e-6);
        if (coherentTransition) {
            for (int i = 0; i < bestRunStart; ++i) {
                QiePianNiHe& slice = slices[static_cast<std::size_t>(i)];
                slice.transitionPolluted = true;
                transitionWeight[static_cast<std::size_t>(i)] = 0.25;
                ++transitionSlices;
                if (bestResiduals[static_cast<std::size_t>(i)]
                    > 1.55 * inlierThreshold) {
                    active[static_cast<std::size_t>(i)] = 0U;
                    slice.axisRejected = true;
                }
            }
        } else {
            stableStartSlice = 0;
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        if (!bestInliers[i] && !slices[i].transitionPolluted)
            active[i] = 0U;
    }
    auto countActive = [&]() {
        return static_cast<int>(std::count(active.begin(), active.end(), 1U));
    };
    if (countActive() < minimumRun) {
        std::fill(active.begin(), active.end(), 0U);
        for (int i = bestRunStart;
             i < bestRunStart + bestRunLength && i < static_cast<int>(n); ++i) {
            active[static_cast<std::size_t>(i)] = 1U;
            slices[static_cast<std::size_t>(i)].axisRejected = false;
        }
    }

    std::vector<double> robustWeight(n, 1.0);
    Eigen::Vector3d fittedSlope = referenceAxis;
    for (int iteration = 0; iteration < 6; ++iteration) {
        std::vector<double> fitWeights(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            if (!active[i]) continue;
            fitWeights[i] = quality[i] * transitionWeight[i] * robustWeight[i];
        }
        if (!fitDepthParameterizedLine(
                slices, fitWeights, intercept, fittedSlope)) {
            return false;
        }
        axis = normalizeOr(fittedSlope, referenceAxis);
        if (axis.dot(referenceAxis) < 0.0) axis = -axis;
        const std::vector<double> residuals = orthogonalLineResiduals(
            slices, intercept, axis);
        std::vector<double> activeResiduals;
        for (std::size_t i = 0; i < n; ++i)
            if (active[i]) activeResiduals.push_back(residuals[i]);
        const double residualMedian = median(activeResiduals);
        std::vector<double> deviations;
        deviations.reserve(activeResiduals.size());
        for (double value : activeResiduals)
            deviations.push_back(std::abs(value - residualMedian));
        const double scale = std::max(0.006, 1.4826 * median(deviations));
        const double huberLimit = std::max(inlierThreshold, 2.5 * scale);
        for (std::size_t i = 0; i < n; ++i) {
            if (!active[i]) {
                robustWeight[i] = 0.0;
                continue;
            }
            robustWeight[i] = residuals[i] <= huberLimit
                ? 1.0 : huberLimit / std::max(residuals[i], 1e-9);
        }

        if (iteration >= 1 && countActive() > minimumRun) {
            const double rejectLimit = residualMedian
                + std::max(inlierThreshold, 2.5 * scale);
            std::vector<std::pair<double, std::size_t>> candidates;
            for (std::size_t i = 0; i < n; ++i) {
                if (active[i] && residuals[i] > rejectLimit)
                    candidates.emplace_back(residuals[i], i);
            }
            std::sort(candidates.begin(), candidates.end(),
                [](const auto& left, const auto& right) {
                    return left.first > right.first;
                });
            for (const auto& candidate : candidates) {
                if (countActive() <= minimumRun) break;
                active[candidate.second] = 0U;
                slices[candidate.second].axisRejected = true;
            }
        }
    }

    const std::vector<double> finalResiduals = orthogonalLineResiduals(
        slices, intercept, axis);
    double squared = 0.0;
    usedSlices = 0;
    rejectedSlices = 0;
    for (std::size_t i = 0; i < n; ++i) {
        slices[i].axisResidual = finalResiduals[i];
        slices[i].axisUsed = active[i] != 0U;
        slices[i].axisRejected = !slices[i].axisUsed;
        slices[i].axisWeight = slices[i].axisUsed
            ? quality[i] * transitionWeight[i] * robustWeight[i] : 0.0;
        if (slices[i].axisUsed) {
            squared += finalResiduals[i] * finalResiduals[i];
            ++usedSlices;
        } else {
            ++rejectedSlices;
        }
    }
    if (usedSlices < minimumRun) return false;
    rmse = std::sqrt(squared / static_cast<double>(usedSlices));
    return intercept.allFinite() && axis.allFinite() && std::isfinite(rmse);
}

/** 【函数导航】
 * 作用：拟合/求解“fitRadiusModel”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline void fitRadiusModel(const std::vector<QiePianNiHe>& slices,
                           int holeType,
                           double originalRadius,
                           double& radiusAtZero,
                           double& slope,
                           double& radiusMad,
                           double& monotonicity)
{
    radiusAtZero = originalRadius;
    slope = 0.0;
    radiusMad = 0.0;
    monotonicity = 0.0;
    if (slices.empty()) return;

    std::vector<double> radii;
    radii.reserve(slices.size());
    for (const QiePianNiHe& slice : slices) radii.push_back(slice.radius);

    if (holeType == 1) {

        std::vector<QiePianNiHe> usable = slices;
        const double maximumCoverage = std::max_element(
            usable.begin(), usable.end(), [](const QiePianNiHe& left,
                                             const QiePianNiHe& right) {
                return left.coverage < right.coverage;
            })->coverage;
        while (usable.size() >= 2U) {
            const QiePianNiHe& previous = usable[usable.size() - 2U];
            const QiePianNiHe& last = usable.back();
            const double terminalJump = std::abs(previous.radius - last.radius);
            const bool largeTerminalJump = terminalJump
                > std::max(0.24, 0.055 * originalRadius);
            const bool coverageCollapsed = last.coverage
                < 0.45 * maximumCoverage;
            if (!(largeTerminalJump && coverageCollapsed)) break;
            usable.pop_back();
        }
        if (usable.empty()) usable = slices;

        const QiePianNiHe& last = usable.back();
        radiusAtZero = last.radius;
        if (usable.size() >= 2U) {
            const QiePianNiHe& previous = usable[usable.size() - 2U];
            const double closeLimit = std::max(0.12, 0.030 * originalRadius);
            if (std::abs(last.radius - previous.radius) <= closeLimit) {
                const double w0 = previous.coverage / (previous.rmse + 0.05);
                const double w1 = last.coverage / (last.rmse + 0.05);
                radiusAtZero = (w0 * previous.radius + w1 * last.radius)
                    / std::max(1e-9, w0 + w1);
            }
        }

        std::vector<double> tailRadii;
        const std::size_t tailBegin = usable.size() > 3U
            ? usable.size() - 3U : 0U;
        for (std::size_t i = tailBegin; i < usable.size(); ++i)
            tailRadii.push_back(usable[i].radius);
        std::vector<double> deviations;
        for (double radius : tailRadii)
            deviations.push_back(std::abs(radius - radiusAtZero));
        radiusMad = deviations.empty() ? 0.0 : 1.4826 * median(deviations);

        if (usable.size() >= 2U) {
            const double meanD = std::accumulate(usable.begin(), usable.end(), 0.0,
                [](double sum, const QiePianNiHe& slice){ return sum + slice.depth; })
                / static_cast<double>(usable.size());
            const double meanR = std::accumulate(usable.begin(), usable.end(), 0.0,
                [](double sum, const QiePianNiHe& slice){ return sum + slice.radius; })
                / static_cast<double>(usable.size());
            double covariance = 0.0, variance = 0.0;
            for (const QiePianNiHe& slice : usable) {
                covariance += (slice.depth - meanD) * (slice.radius - meanR);
                variance += (slice.depth - meanD) * (slice.depth - meanD);
            }
            if (variance > 1e-12) slope = covariance / variance;
        }
        monotonicity = 1.0;
        return;
    }

    std::vector<double> weights(slices.size(), 1.0);
    double intercept = originalRadius;
    for (int iteration = 0; iteration < 5; ++iteration) {
        double sw=0.0, sd=0.0, sr=0.0, sdd=0.0, sdr=0.0;
        for (std::size_t i=0;i<slices.size();++i) {
            const double base = slices[i].coverage / (slices[i].rmse + 0.05);
            const double w = std::max(1e-6, base * weights[i]);
            const double d = slices[i].depth;
            const double r = slices[i].radius;
            sw+=w; sd+=w*d; sr+=w*r; sdd+=w*d*d; sdr+=w*d*r;
        }
        const double determinant = sw*sdd - sd*sd;
        if (std::abs(determinant) < 1e-10) break;
        intercept = (sr*sdd - sd*sdr) / determinant;
        slope = (sw*sdr - sd*sr) / determinant;
        std::vector<double> residuals;
        residuals.reserve(slices.size());
        for (const QiePianNiHe& slice : slices)
            residuals.push_back(std::abs(slice.radius
                - (intercept + slope * slice.depth)));
        const double med = median(residuals);
        std::vector<double> deviations;
        for (double residual : residuals)
            deviations.push_back(std::abs(residual-med));
        const double scale = std::max(0.025, 1.4826*median(deviations));
        for (std::size_t i=0;i<weights.size();++i) {
            const double ratio = residuals[i]/(2.5*scale);
            weights[i] = ratio<=1.0 ? 1.0 : 1.0/ratio;
        }
        radiusMad = 1.4826*median(deviations);
    }
    radiusAtZero = intercept;
    int good=0, pairs=0;
    for (std::size_t i=1;i<slices.size();++i) {
        ++pairs;
        if (slices[i].radius <= slices[i-1].radius + 0.10) ++good;
    }
    monotonicity = pairs>0 ? static_cast<double>(good)/pairs : 0.0;
}

/** 【函数导航】
 * 作用：估计“estimateOuter”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline WaiEvidence estimateOuter(const std::vector<DianYangBen>& samples,
                                   const Eigen::Vector3d& center,
                                   Eigen::Vector3d surfaceNormal,
                                   double radius)
{
    WaiEvidence out;
    surfaceNormal = normalizeOr(surfaceNormal, Eigen::Vector3d::UnitZ());
    const double inner = std::max(1.10 * radius, radius + 0.35);
    const double outer = std::max(2.10 * radius, radius + 3.2);
    const PingMianJieGuo fit = HoleAxisHouFit::fitTightCandidate(
        samples, center, surfaceNormal, inner, outer);
    if (!fit.valid || !fit.normal.allFinite()) return out;
    out.center = fit.center;
    out.normal = normalizeOr(fit.normal, surfaceNormal);
    if (out.normal.dot(surfaceNormal) < 0.0) out.normal = -out.normal;
    out.quality = fit.quality;
    out.supportCount = fit.supportCount;
    out.coveredSectors = fit.coveredSectors;
    out.coverage = fit.coverage;
    out.rmse = fit.rmse;
    out.mad = fit.mad;
    out.source = fit.mode;
    out.valid = fit.quality >= 2 && fit.supportCount >= 120
        && fit.coverage >= 0.55 && fit.rmse <= 0.16
        && angleDegrees(surfaceNormal, out.normal) <= 10.0;
    if (out.valid) {
        const double coverageScore = std::clamp((out.coverage-0.55)/0.45,0.0,1.0);
        const double residualScore = std::clamp((0.16-out.rmse)/0.13,0.0,1.0);
        out.confidence = 0.55*coverageScore + 0.45*residualScore;
    }
    return out;
}

/** 【函数导航】
 * 作用：估计“estimateWall”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline HoleBiModel estimateWall(const std::vector<DianYangBen>& samples,
                              const Eigen::Vector3d& origin,
                              Eigen::Vector3d initialAxis,
                              double originalRadius,
                              double bottomRadius,
                              double depth,
                              int holeType)
{
    HoleBiModel out;
    initialAxis = normalizeOr(initialAxis, -Eigen::Vector3d::UnitZ());
    Eigen::Vector3d axis = initialAxis;
    Eigen::Vector3d workingOrigin = origin;

    const bool cone = holeType == 2;
    const double usableDepth = cone && depth > 0.8
        ? std::clamp(depth, 1.2, 6.0)
        : std::clamp(0.55 * originalRadius, 1.0, 2.0);
    const double coneStart = std::max(0.40, 0.15 * usableDepth);
    const double coneEnd = std::max(coneStart + 0.75,
        std::min(usableDepth - 0.55, 0.72 * usableDepth));
    const int sliceCount = cone ? 8
        : std::clamp(static_cast<int>(std::ceil(usableDepth / 0.25)),
                     4, kMaximumSlices);

    const int evaluations = cone ? 2 : 1;

    std::vector<QiePianNiHe> finalSlices;
    Eigen::Vector3d finalIntercept = origin;
    double finalLineRmse = std::numeric_limits<double>::infinity();
    double finalAllLineRmse = std::numeric_limits<double>::infinity();
    double finalInlierThreshold = 0.0;
    int finalStableStart = 0;
    int finalUsedSlices = 0;
    int finalRejectedSlices = 0;
    int finalTransitionSlices = 0;

    for (int evaluation = 0; evaluation < evaluations; ++evaluation) {
        const double maximumTarget = cone ? coneEnd : usableDepth;
        const auto projected = projectNeighborhood(
            samples, workingOrigin, axis, originalRadius + 3.0,
            -0.65, maximumTarget + 0.65);
        std::vector<QiePianNiHe> slices;
        slices.reserve(static_cast<std::size_t>(sliceCount));
        double predictedRadius = originalRadius;
        for (int i = 0; i < sliceCount; ++i) {
            const double targetDepth = cone
                ? coneStart + (coneEnd - coneStart)
                    * static_cast<double>(i)
                    / static_cast<double>(std::max(1, sliceCount - 1))
                : 0.25 + static_cast<double>(i) * 0.25;
            if (!cone && targetDepth > usableDepth) break;
            if (cone && depth > 0.5 && bottomRadius > 0.3)
                predictedRadius = originalRadius
                    + (bottomRadius - originalRadius) * (targetDepth / depth);
            QiePianNiHe fit = fitBestSlice(projected, workingOrigin, axis,
                targetDepth, predictedRadius, originalRadius + 2.0);
            if (!fit.valid) continue;
            slices.push_back(fit);
            predictedRadius = cone
                ? 0.55 * predictedRadius + 0.45 * fit.radius
                : 0.75 * predictedRadius + 0.25 * fit.radius;
        }
        if (slices.empty()) break;
        std::sort(slices.begin(), slices.end(),
            [](const QiePianNiHe& left, const QiePianNiHe& right) {
                return left.depth < right.depth;
            });

        Eigen::Vector3d intercept = workingOrigin;
        Eigen::Vector3d candidateAxis = axis;
        double lineRmse = std::numeric_limits<double>::infinity();
        double allLineRmse = std::numeric_limits<double>::infinity();
        double inlierThreshold = 0.0;
        int stableStart = 0;
        int usedSliceCount = 0;
        int rejectedSliceCount = 0;
        int transitionSliceCount = 0;
        const bool lineValid = slices.size() >= 2U
            && robustLineFit(slices, axis, intercept, candidateAxis, lineRmse,
                             allLineRmse, inlierThreshold, stableStart,
                             usedSliceCount, rejectedSliceCount,
                             transitionSliceCount);
        if (lineValid) {

            const double perPassLimit = cone ? 2.0 : 1.5;
            const double totalLimit = cone ? 8.0 : 4.0;
            Eigen::Vector3d nextAxis = boundedAxisStep(
                axis, candidateAxis, perPassLimit);
            nextAxis = boundedAxisStep(initialAxis, nextAxis, totalLimit);
            if (nextAxis.dot(initialAxis) < 0.0) nextAxis = -nextAxis;

            finalIntercept = intercept;
            finalLineRmse = lineRmse;
            finalAllLineRmse = allLineRmse;
            finalInlierThreshold = inlierThreshold;
            finalStableStart = stableStart;
            finalUsedSlices = usedSliceCount;
            finalRejectedSlices = rejectedSliceCount;
            finalTransitionSlices = transitionSliceCount;
            if (evaluation + 1 < evaluations) {
                axis = nextAxis;
                Eigen::Vector3d shift = intercept - workingOrigin;
                shift -= axis * shift.dot(axis);
                const double maximumShift = 0.8;
                if (shift.norm() > maximumShift)
                    shift *= maximumShift / shift.norm();
                workingOrigin += shift;
                if (cone) {
                    Eigen::Vector3d totalShift = workingOrigin - origin;
                    totalShift -= axis * totalShift.dot(axis);
                    if (totalShift.norm() > 1.2)
                        workingOrigin = origin + totalShift
                            * (1.2 / totalShift.norm());
                }
            } else {
                axis = nextAxis;
            }
        } else {
            finalIntercept = workingOrigin;
        }
        finalSlices = slices;
    }
    if (finalSlices.empty()) return out;

    std::vector<QiePianNiHe> modelSlices;
    modelSlices.reserve(finalSlices.size());
    for (const QiePianNiHe& slice : finalSlices)
        if (slice.axisUsed) modelSlices.push_back(slice);
    if (modelSlices.empty()) modelSlices = finalSlices;

    double radiusAtZero = originalRadius;
    double slope = 0.0;
    double radiusMad = 0.0;
    double monotonicity = 0.0;
    fitRadiusModel(finalSlices, holeType, originalRadius,
                   radiusAtZero, slope, radiusMad, monotonicity);
    double sumCoverage = 0.0;
    double sumRmse = 0.0;
    std::vector<double> sectorCounts;
    for (const QiePianNiHe& slice : modelSlices) {
        sumCoverage += slice.coverage;
        sumRmse += slice.rmse;
        out.totalSupport += slice.supportCount;
        sectorCounts.push_back(static_cast<double>(slice.coveredSectors));
    }
    out.validSlices = static_cast<int>(finalSlices.size());
    out.axisUsedSlices = finalUsedSlices > 0
        ? finalUsedSlices : static_cast<int>(modelSlices.size());
    out.axisRejectedSlices = finalRejectedSlices;
    out.transitionSlices = finalTransitionSlices;
    out.stableStartSlice = finalStableStart;
    out.axisInlierThreshold = finalInlierThreshold;
    out.allSliceCenterLineRmse = std::isfinite(finalAllLineRmse)
        ? finalAllLineRmse : finalLineRmse;
    out.meanCoverage = sumCoverage / modelSlices.size();
    out.meanRmse = sumRmse / modelSlices.size();
    out.medianCoveredSectors = static_cast<int>(std::lround(median(sectorCounts)));
    out.intercept = finalIntercept;
    out.axisIn = axis;
    out.centerLineRmse = std::isfinite(finalLineRmse) ? finalLineRmse : 0.0;
    out.radiusAtZero = radiusAtZero;
    out.radiusSlope = slope;
    out.radiusMad = radiusMad;
    out.monotonicity = monotonicity;
    out.slices = finalSlices;

    const bool typeGeometry = holeType == 1
        ? (out.axisUsedSlices >= 1 && out.meanCoverage >= 0.32
           && out.meanRmse <= 0.20)
        : (out.axisUsedSlices >= 4 && out.meanCoverage >= 0.45
           && out.meanRmse <= 0.22 && slope < -0.05
           && monotonicity >= 0.60);
    const bool lineGeometry = out.validSlices < 2
        || out.centerLineRmse <= 0.28;
    out.valid = typeGeometry && lineGeometry && radiusAtZero > 0.5;
    const double sliceScore = std::clamp(
        (out.axisUsedSlices - 1.0) / 7.0, 0.0, 1.0);
    const double coverageScore = std::clamp(
        (out.meanCoverage - 0.25) / 0.70, 0.0, 1.0);
    const double rmseScore = std::clamp(
        (0.24 - out.meanRmse) / 0.20, 0.0, 1.0);
    const double lineScore = out.validSlices < 2 ? 0.45
        : std::clamp((0.30 - out.centerLineRmse) / 0.25, 0.0, 1.0);
    out.confidence = 0.25 * sliceScore + 0.30 * coverageScore
        + 0.25 * rmseScore + 0.20 * lineScore;
    return out;
}

/** 【函数导航】
 * 作用：估计“estimateLockedRadiusSurfaceIntersectionContour”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline HoleKouContour estimateLockedRadiusSurfaceIntersectionContour(
    const Eigen::Vector3d& centerTop,
    Eigen::Vector3d axisIn,
    double lockedRadius,
    double wallRadiusSlope,
    int holeType,
    const WaiEvidence& outer)
{
    HoleKouContour out;
    out.executed = true;
    if (!centerTop.allFinite() || !(lockedRadius > 0.5) || !outer.valid) {
        out.decision = "REJECT_MISSING_LOCKED_GEOMETRY_OR_OUTER_SURFACE";
        return out;
    }

    axisIn = normalizeOr(axisIn, -Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d planeNormal = normalizeOr(
        outer.normal, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d u, v;
    makeAxes(axisIn, u, v);

    double slope = 0.0;
    if (holeType == 2) {
        if (!(wallRadiusSlope < -0.05) || !std::isfinite(wallRadiusSlope)) {
            out.decision = "REJECT_CONE_SLOPE_UNAVAILABLE";
            return out;
        }
        slope = wallRadiusSlope;
    }

    std::vector<double> depths;
    std::vector<double> radii;
    depths.reserve(kSectors);
    radii.reserve(kSectors);
    const double upstreamLimit = std::clamp(
        0.34 * lockedRadius + 0.35, 1.00, 2.50);
    const double inwardLimit = std::clamp(
        0.20 * lockedRadius + 0.30, 0.70, 1.60);
    const double radialLimit = std::clamp(
        0.22 * lockedRadius + 0.25, 0.55, 1.75);

    for (int sector = 0; sector < kSectors; ++sector) {
        const double angle = (static_cast<double>(sector) + 0.5)
            * 2.0 * kPi / static_cast<double>(kSectors);
        const Eigen::Vector3d radialDirection =
            std::cos(angle) * u + std::sin(angle) * v;
        const Eigen::Vector3d planarRingPoint =
            centerTop + lockedRadius * radialDirection;
        out.planarRingMaximumPlaneResidual = std::max(
            out.planarRingMaximumPlaneResidual,
            std::abs((planarRingPoint - outer.center).dot(planeNormal)));
        const double denominator = planeNormal.dot(
            axisIn + slope * radialDirection);
        HoleKouContourPoint point;
        point.angle = angle;
        if (std::abs(denominator) <= 0.25) {
            out.points[static_cast<std::size_t>(sector)] = point;
            continue;
        }
        const double numerator = planeNormal.dot(
            outer.center - centerTop - lockedRadius * radialDirection);
        const double axialDepth = numerator / denominator;
        const double localRadius = lockedRadius + slope * axialDepth;
        if (!std::isfinite(axialDepth) || !std::isfinite(localRadius)
            || axialDepth < -upstreamLimit || axialDepth > inwardLimit
            || localRadius <= 0.5
            || std::abs(localRadius - lockedRadius) > radialLimit) {
            out.points[static_cast<std::size_t>(sector)] = point;
            continue;
        }
        point.valid = true;
        point.axialDepth = axialDepth;
        point.localRadius = localRadius;
        point.world = centerTop + axisIn * axialDepth
            + radialDirection * localRadius;
        point.planeResidual = (point.world - outer.center).dot(planeNormal);
        if (!point.world.allFinite()) {
            point.valid = false;
            out.points[static_cast<std::size_t>(sector)] = point;
            continue;
        }
        out.points[static_cast<std::size_t>(sector)] = point;
        depths.push_back(axialDepth);
        radii.push_back(localRadius);
        ++out.validPoints;
        out.maximumPlaneResidual = std::max(
            out.maximumPlaneResidual, std::abs(point.planeResidual));
    }

    out.coverage = static_cast<double>(out.validPoints)
        / static_cast<double>(kSectors);
    if (out.validPoints < 48) {
        out.decision = "REJECT_TOO_FEW_SURFACE_INTERSECTION_POINTS";
        return out;
    }
    out.minimumAxialDepth = *std::min_element(depths.begin(), depths.end());
    out.maximumAxialDepth = *std::max_element(depths.begin(), depths.end());
    out.medianAxialDepth = median(depths);
    out.axialSpan = out.maximumAxialDepth - out.minimumAxialDepth;
    out.minimumLocalRadius = *std::min_element(radii.begin(), radii.end());
    out.maximumLocalRadius = *std::max_element(radii.begin(), radii.end());

    const double maximumSpan = std::clamp(
        0.42 * lockedRadius + 0.35, 1.10, 2.80);
    const bool qualityGate = out.coverage >= 0.66
        && out.axialSpan <= maximumSpan
        && out.maximumPlaneResidual <= 0.01
        && std::abs(out.medianAxialDepth) <= 1.20;
    if (!qualityGate) {
        if (out.coverage < 0.66)
            out.decision = "REJECT_CONTOUR_COVERAGE";
        else if (out.axialSpan > maximumSpan)
            out.decision = "REJECT_CONTOUR_AXIAL_SPAN";
        else if (out.maximumPlaneResidual > 0.01)
            out.decision = "REJECT_CONTOUR_PLANE_RESIDUAL";
        else
            out.decision = "REJECT_CONTOUR_MEDIAN_OFFSET";
        return out;
    }

    const double coverageScore = std::clamp(
        (out.coverage - 0.66) / 0.34, 0.0, 1.0);
    const double spanScore = std::clamp(
        (maximumSpan - out.axialSpan) / maximumSpan, 0.0, 1.0);
    const double surfaceScore = outer.rmse > 0.0 && std::isfinite(outer.rmse)
        ? std::clamp((0.22 - outer.rmse) / 0.18, 0.0, 1.0) : 0.35;
    out.confidence = 0.40 * coverageScore
        + 0.35 * spanScore + 0.25 * surfaceScore;
    out.valid = true;
    out.source = holeType == 2
        ? "PhysicalMouthContour_LOCKED_CONE_SURFACE_INTERSECTION_CONTOUR"
        : "PhysicalMouthContour_LOCKED_CYLINDER_SURFACE_INTERSECTION_CONTOUR";
    out.decision = "APPLY_PHYSICAL_MOUTH_CONTOUR_KEEP_CANONICAL_RADIUS";
    return out;
}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimate(const std::vector<DianYangBen>& samples,
                       const Eigen::Vector3d& mouthCenter,
                       Eigen::Vector3d recognitionSurface,
                       double topRadius,
                       double bottomRadius,
                       double depth,
                       int inwardPolarity,
                       int holeType)
{
    Result out;
    out.executed = true;
    out.centerTop = mouthCenter;
    out.topRadius = topRadius;
    out.originalRadius = topRadius;
    out.radiusShift = 0.0;
    out.wallRadiusWeight = 0.0;
    if (samples.empty() || !mouthCenter.allFinite() || !(topRadius > 0.5)
        || inwardPolarity == 0 || (holeType != 1 && holeType != 2)) {
        return out;
    }

    recognitionSurface = normalizeOr(
        recognitionSurface, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d initialAxis = normalizeOr(
        recognitionSurface * static_cast<double>(inwardPolarity),
        -recognitionSurface);

    out.outer = estimateOuter(samples, mouthCenter,
                              recognitionSurface, topRadius);
    Eigen::Vector3d wallSeedAxis = initialAxis;
    if (out.outer.valid && holeType == 1) {
        wallSeedAxis = normalizeOr(
            out.outer.normal * static_cast<double>(inwardPolarity),
            initialAxis);
        if (wallSeedAxis.dot(initialAxis) < 0.0) wallSeedAxis = -wallSeedAxis;
    }
    out.wall = estimateWall(samples, mouthCenter, wallSeedAxis,
                            topRadius, bottomRadius, depth, holeType);
    out.wallRadiusAtZero = out.wall.radiusAtZero;

    if (holeType == 2 && !out.wall.valid) return out;
    if (holeType == 1 && !out.outer.valid && !out.wall.valid) return out;

    Eigen::Vector3d finalAxis = initialAxis;
    Eigen::Vector3d center = mouthCenter;
    double lockedMouthDepth = 0.0;

    if (holeType == 1) {

        if (out.outer.valid) {
            Eigen::Vector3d outerAxis = normalizeOr(
                out.outer.normal * static_cast<double>(inwardPolarity),
                initialAxis);
            if (outerAxis.dot(initialAxis) < 0.0) outerAxis = -outerAxis;
            finalAxis = outerAxis;

            if (out.wall.valid && out.wall.validSlices >= 2
                && angleDegrees(outerAxis, out.wall.axisIn) <= 2.50) {

                finalAxis = normalizeOr(
                    0.78 * outerAxis + 0.22 * out.wall.axisIn,
                    outerAxis);
                if (finalAxis.dot(initialAxis) < 0.0) finalAxis = -finalAxis;
                center = out.wall.intercept;
                out.centerEvidenceWeight = 0.22;
            } else {
                center = mouthCenter;
                out.centerEvidenceWeight = 0.0;
            }

            const double denominator = finalAxis.dot(out.outer.normal);
            if (std::abs(denominator) <= 0.35) return out;
            lockedMouthDepth = (out.outer.center - center).dot(out.outer.normal)
                / denominator;
            if (!std::isfinite(lockedMouthDepth)
                || std::abs(lockedMouthDepth) > 1.50) {
                return out;
            }
            center += finalAxis * lockedMouthDepth;
            out.wallRadiusWeight = 0.0;
        } else {
            finalAxis = out.wall.axisIn;
            center = out.wall.intercept;
            out.centerEvidenceWeight = 1.0;
            out.wallRadiusWeight = 1.0;
        }
    } else {

        finalAxis = out.wall.axisIn;
        if (finalAxis.dot(initialAxis) < 0.0) finalAxis = -finalAxis;
        if (!(out.wall.radiusSlope < -0.05)
            || !std::isfinite(out.wall.radiusAtZero)) {
            return out;
        }

        const double coverageEvidence = std::clamp(
            (out.wall.meanCoverage - 0.45) / 0.40, 0.20, 1.0);
        const double lineEvidence = std::clamp(
            (0.30 - out.wall.centerLineRmse) / 0.25, 0.20, 1.0);
        out.centerEvidenceWeight = std::clamp(
            std::sqrt(coverageEvidence * lineEvidence), 0.25, 1.0);
        center = mouthCenter + out.centerEvidenceWeight
            * (out.wall.intercept - mouthCenter);

        lockedMouthDepth = (topRadius - out.wall.radiusAtZero)
            / out.wall.radiusSlope;

        const double upstreamLimit = std::clamp(
            0.22 * topRadius + 0.35, 0.75, 1.60);
        const double inwardLimit = std::clamp(
            0.10 * topRadius + 0.20, 0.45, 0.85);
        if (!std::isfinite(lockedMouthDepth)
            || lockedMouthDepth < -upstreamLimit
            || lockedMouthDepth > inwardLimit) {
            return out;
        }
        center += finalAxis * lockedMouthDepth;
        out.wallRadiusWeight = 1.0;
    }

    if (finalAxis.dot(initialAxis) < 0.0) finalAxis = -finalAxis;
    const double correction = angleDegrees(initialAxis, finalAxis);
    if (correction > 8.0) return out;

    const double centerShift = (center - mouthCenter).norm();
    if (!center.allFinite() || centerShift > 1.80) return out;

    out.centerTop = center;
    out.lockedMouthDepth = lockedMouthDepth;
    out.axisIn = finalAxis;
    out.surfaceNormal = normalizeOr(
        finalAxis * static_cast<double>(inwardPolarity), recognitionSurface);
    if (out.surfaceNormal.dot(recognitionSurface) < 0.0)
        out.surfaceNormal = -out.surfaceNormal;

    out.contour = estimateLockedRadiusSurfaceIntersectionContour(
        center, finalAxis, topRadius, out.wall.radiusSlope, holeType, out.outer);

    out.topRadius = topRadius;
    out.radiusShift = 0.0;
    out.centerShift = centerShift;
    out.axisCorrectionDegrees = correction;
    out.confidence = 0.72 * (out.wall.valid ? out.wall.confidence : 0.45)
        + 0.28 * (out.outer.valid ? out.outer.confidence : 0.35);
    out.valid = true;

    if (holeType == 1 && out.outer.valid)
        out.source = "PhysicalMouthContour_STRAIGHT_LOCKED_RADIUS_SURFACE_AXIS_INTERSECTION";
    else if (holeType == 2)
        out.source = "PhysicalMouthContour_CONE_LOCKED_RADIUS_WALL_INTERSECTION";
    else
        out.source = "PhysicalMouthContour_STRAIGHT_LOCKED_RADIUS_WALL_AXIS";
    return out;
}

}

// ============================================================================
// 功能分区：孔口位姿粗估计
// ============================================================================
/*
模块职责：
孔口统一姿态估计模块。

主要调用位置：
由正式孔识别后段调用，综合孔壁和孔口证据形成规范姿态/半径结果。

维护说明：
该链处于生产几何热路径；候选顺序、评分、网格分辨率和阈值都属于冻结语义，纯整理不改。
*/
#include <thread>

//

//

//

namespace HoleWeiziCu {

using namespace HoleWeiziBase;

inline constexpr int kInnerHistogramBins = 96;
inline constexpr int kConeDepthBins = 12;

/** 【类型导航注释】
 * CuZhongXinHuiFu：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CuZhongXinHuiFu {
    bool executed = false;
    bool valid = false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double shift = 0.0;
    double boundaryRadius = 0.0;
    double boundaryMad = std::numeric_limits<double>::infinity();
    double coverage = 0.0;
    double innerDensity = 0.0;
    double annulusDensity = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    int surfacePoints = 0;
    int evaluatedCandidates = 0;
    long long visitedSurfacePoints = 0;
    int parallelWorkers = 1;
    const char* decision = "NOT_EXECUTED";
};

/** 【类型导航注释】
 * NeiYuanZhuZu：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct NeiYuanZhuZu {
    bool executed = false;
    bool valid = false;
    double radius = 0.0;
    double radiusMad = std::numeric_limits<double>::infinity();
    double radiusSlope = 0.0;
    double depthStart = 0.0;
    double depthEnd = 0.0;
    double depthSpan = 0.0;
    double meanCoverage = 0.0;
    double medianRmse = std::numeric_limits<double>::infinity();
    double centerLineRmse = std::numeric_limits<double>::infinity();
    double quality = -std::numeric_limits<double>::infinity();
    int candidateCount = 0;
    int familyCount = 0;
    int histogramWindows = 0;
    long long histogramPointVisits = 0;
    long long fitPointVisits = 0;
    int usedSlices = 0;
    int rejectedSlices = 0;
    Eigen::Vector3d intercept = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisIn = -Eigen::Vector3d::UnitZ();
    std::vector<QiePianNiHe> slices;
    const char* decision = "NOT_EXECUTED";
};

/** 【类型导航注释】
 * RawZhuiWallFit：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct RawZhuiWallFit {
    bool executed = false;
    bool valid = false;
    Eigen::Vector3d centerAtLockedRadius = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisIn = -Eigen::Vector3d::UnitZ();
    double radiusAtZero = 0.0;
    double radiusSlope = 0.0;
    double lockedMouthDepth = 0.0;
    double scale = std::numeric_limits<double>::infinity();
    double rmse = std::numeric_limits<double>::infinity();
    double noDriftRmse = std::numeric_limits<double>::infinity();
    double baselineEvenRmse = std::numeric_limits<double>::infinity();
    double baselineOddRmse = std::numeric_limits<double>::infinity();
    double evenRmse = std::numeric_limits<double>::infinity();
    double oddRmse = std::numeric_limits<double>::infinity();
    double evenImprovement = 0.0;
    double oddImprovement = 0.0;
    double improvement = 0.0;
    double axisCorrectionDegrees = 0.0;
    double centerShift = 0.0;
    int cellCount = 0;
    int roiPoints = 0;
    int axisEvaluations = 0;
    int parallelWorkers = 1;
    int coveredSectors = 0;
    int coveredDepthBins = 0;
    std::array<int, kSectors> sectorSupport{};
    std::array<double, kSectors> sectorResidual{};
    const char* decision = "NOT_EXECUTED";
};


/** 【类型导航注释】
 * Result：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result : HoleWeiziBase::Result {
    double coneFamilyConeFamilyAngleDeg = 0.0;
    CuZhongXinHuiFu coarseCenter;
    NeiYuanZhuZu innerCylinder;
    RawZhuiWallFit rawCone;
    bool measuredInnerRadiusValid = false;
    double measuredInnerRadius = 0.0;
};

/** 【函数导航】
 * 作用：执行“candidateQuality”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double candidateQuality(const QiePianNiHe& fit, double targetRadius,
                               double tolerance) {
    return fit.coverage - 1.6 * fit.rmse
        + 0.0015 * std::sqrt(static_cast<double>(std::max(1, fit.supportCount)))
        - 0.06 * fit.halfThickness
        - 0.8 * std::abs(fit.radius - targetRadius) / std::max(1e-6, tolerance);
}

/** 【类型导航注释】
 * PingMianPoint2D：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingMianPoint2D {
    double x = 0.0;
    double y = 0.0;
};

/** 【类型导航注释】
 * PingMianPointGrid：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingMianPointGrid {
    const std::vector<PingMianPoint2D>* points = nullptr;
    double minimumX = 0.0;
    double minimumY = 0.0;
    double cellSize = 0.50;
    int width = 0;
    int height = 0;
    std::vector<std::vector<int>> cells;

    /** 【函数导航】
     * 作用：构建“build”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 位姿计算。
     * 主要引用/调用位置：HoleCanshuShuchu_Export.h、ZhuChuangKou_Window.cpp、HoleCanshuShuchu_Export.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool build(const std::vector<PingMianPoint2D>& source, double requestedCellSize) {
        points = &source;
        if (source.empty()) return false;
        cellSize = std::max(0.20, requestedCellSize);
        double maximumX = source.front().x;
        double maximumY = source.front().y;
        minimumX = source.front().x;
        minimumY = source.front().y;
        for (const PingMianPoint2D& point : source) {
            minimumX = std::min(minimumX, point.x);
            minimumY = std::min(minimumY, point.y);
            maximumX = std::max(maximumX, point.x);
            maximumY = std::max(maximumY, point.y);
        }
        width = std::max(1, static_cast<int>(std::floor(
            (maximumX - minimumX) / cellSize)) + 1);
        height = std::max(1, static_cast<int>(std::floor(
            (maximumY - minimumY) / cellSize)) + 1);
        cells.assign(static_cast<std::size_t>(width * height), {});
        for (std::size_t index = 0; index < source.size(); ++index) {
            const PingMianPoint2D& point = source[index];
            const int ix = std::clamp(static_cast<int>(std::floor(
                (point.x - minimumX) / cellSize)), 0, width - 1);
            const int iy = std::clamp(static_cast<int>(std::floor(
                (point.y - minimumY) / cellSize)), 0, height - 1);
            cells[static_cast<std::size_t>(iy * width + ix)]
                .push_back(static_cast<int>(index));
        }
        return true;
    }

    /** 【函数导航】
     * 作用：执行“visitSquare”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 位姿计算。
     * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    template <typename Visitor>
    void visitSquare(double centerX, double centerY, double extent,
                     Visitor&& visitor) const {
        if (!points || cells.empty()) return;
        const int minimumCellX = std::clamp(static_cast<int>(std::floor(
            (centerX - extent - minimumX) / cellSize)), 0, width - 1);
        const int maximumCellX = std::clamp(static_cast<int>(std::floor(
            (centerX + extent - minimumX) / cellSize)), 0, width - 1);
        const int minimumCellY = std::clamp(static_cast<int>(std::floor(
            (centerY - extent - minimumY) / cellSize)), 0, height - 1);
        const int maximumCellY = std::clamp(static_cast<int>(std::floor(
            (centerY + extent - minimumY) / cellSize)), 0, height - 1);
        for (int iy = minimumCellY; iy <= maximumCellY; ++iy) {
            for (int ix = minimumCellX; ix <= maximumCellX; ++ix) {
                const auto& bucket = cells[static_cast<std::size_t>(iy * width + ix)];
                for (int index : bucket)
                    visitor((*points)[static_cast<std::size_t>(index)]);
            }
        }
    }

    /** 【函数导航】
     * 作用：执行“visitCircle”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 位姿计算。
     * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    template <typename Visitor>
    void visitCircle(double centerX, double centerY, double radius,
                     Visitor&& visitor) const {
        if (!points || cells.empty() || !(radius > 0.0)) return;
        const int minimumCellX = std::clamp(static_cast<int>(std::floor(
            (centerX - radius - minimumX) / cellSize)), 0, width - 1);
        const int maximumCellX = std::clamp(static_cast<int>(std::floor(
            (centerX + radius - minimumX) / cellSize)), 0, width - 1);
        const int minimumCellY = std::clamp(static_cast<int>(std::floor(
            (centerY - radius - minimumY) / cellSize)), 0, height - 1);
        const int maximumCellY = std::clamp(static_cast<int>(std::floor(
            (centerY + radius - minimumY) / cellSize)), 0, height - 1);
        const double radiusSquared = radius * radius;
        for (int iy = minimumCellY; iy <= maximumCellY; ++iy) {
            const double cellMinY = minimumY + static_cast<double>(iy) * cellSize;
            const double cellMaxY = cellMinY + cellSize;
            const double nearestY = std::clamp(centerY, cellMinY, cellMaxY);
            const double dy = nearestY - centerY;
            for (int ix = minimumCellX; ix <= maximumCellX; ++ix) {
                const double cellMinX = minimumX + static_cast<double>(ix) * cellSize;
                const double cellMaxX = cellMinX + cellSize;
                const double nearestX = std::clamp(centerX, cellMinX, cellMaxX);
                const double dx = nearestX - centerX;
                if (dx * dx + dy * dy > radiusSquared) continue;
                const auto& bucket = cells[static_cast<std::size_t>(iy * width + ix)];
                for (int index : bucket)
                    visitor((*points)[static_cast<std::size_t>(index)]);
            }
        }
    }
};

/** 【类型导航注释】
 * CuHouXuanScore：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CuHouXuanScore {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    double mad = std::numeric_limits<double>::infinity();
    double coverage = 0.0;
    double innerDensity = 0.0;
    double annulusDensity = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：给一个粗中心候选打分。函数只评估“这个中心附近是否呈现内空、外环有边界”的几何对比，不改变 Hole 半径和最终法线。
 * 调用位置：recoverCoarseVoidCenter() 生成二维中心网格后批量调用；该阶段是 pose 日志中的 coarseCenter/coarseWork 主要计算来源。
 * 输入：surface/grid 为同一局部支撑面投影；candidateX/Y、lockedRadius 单位均为 mm；visitedPoints 只累计内部工作量。
 * 输出：CuHouXuanScore，包含边界半径、MAD、覆盖度、内外密度和综合 score；valid=false 表示证据不足。
 * 性能：一次 Hole 会评估约 922 个候选，所以本函数必须避免每候选堆分配；不能为了提速改变 24 扇区、48 径向 bin 或 score 公式。
 */
inline CuHouXuanScore evaluateCoarseCenterCandidate(
    const std::vector<PingMianPoint2D>& surface,
    const PingMianPointGrid& grid,
    double candidateX,
    double candidateY,
    double lockedRadius,
    long long& visitedPoints)
{
    CuHouXuanScore out;
    out.x = candidateX;
    out.y = candidateY;
    if (surface.size() < 80U || !(lockedRadius > 0.5)) return out;
    static constexpr int kCenterSectors = 24;
    static constexpr int kCenterBins = 48;
    std::array<int, kCenterSectors * kCenterBins> histogram{};
    const double radialLow = 0.50 * lockedRadius;
    const double radialHigh = 1.60 * lockedRadius;
    const double innerLimit = 0.72 * lockedRadius;
    const double annulusLow = 1.05 * lockedRadius;
    const double annulusHigh = 1.55 * lockedRadius;
    const double radialLowSquared = radialLow * radialLow;
    const double radialHighSquared = radialHigh * radialHigh;
    const double innerLimitSquared = innerLimit * innerLimit;
    const double annulusLowSquared = annulusLow * annulusLow;
    const double annulusHighSquared = annulusHigh * annulusHigh;
    const double binWidth = (radialHigh - radialLow)
        / static_cast<double>(kCenterBins);
    int innerCount = 0;
    int annulusCount = 0;
    grid.visitCircle(candidateX, candidateY, radialHigh,
        [&](const PingMianPoint2D& point) {
        ++visitedPoints;
        const double dx = point.x - candidateX;
        const double dy = point.y - candidateY;
        const double radiusSquared = dx * dx + dy * dy;
        if (radiusSquared < innerLimitSquared) ++innerCount;
        if (radiusSquared > annulusLowSquared
            && radiusSquared < annulusHighSquared) ++annulusCount;
        if (radiusSquared < radialLowSquared
            || radiusSquared >= radialHighSquared) return;
        const double radius = std::sqrt(radiusSquared);
        double angle = std::atan2(dy, dx);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kCenterSectors))),
            0, kCenterSectors - 1);
        const int bin = std::clamp(static_cast<int>(std::floor(
            (radius - radialLow) / binWidth)), 0, kCenterBins - 1);
        ++histogram[static_cast<std::size_t>(sector * kCenterBins + bin)];
    });
    // 每个角扇区最多只产生一个边界半径，数量上限固定为 24。
    // 这里使用栈上定长数组，避免 922 个粗中心候选反复创建/释放两个小 vector。
    // 注意：候选顺序、边界半径公式、median/MAD 定义和后续 score 完全不变。
    std::array<double, kCenterSectors> boundaries{};
    std::size_t boundaryCount = 0U;
    for (int sector = 0; sector < kCenterSectors; ++sector) {
        for (int bin = 0; bin < kCenterBins - 3; ++bin) {
            int count = 0;
            for (int offset = 0; offset < 4; ++offset)
                count += histogram[static_cast<std::size_t>(
                    sector * kCenterBins + bin + offset)];
            if (count < 3) continue;
            boundaries[boundaryCount++] = radialLow
                + (static_cast<double>(bin) + 0.5) * binWidth;
            break;
        }
    }
    if (boundaryCount < 14U) return out;

    // 与 medianInPlace 的定义保持一致，只处理数组中实际写入的前 boundaryCount 个元素。
    // nth_element 会重排元素，但 median/MAD 只关心数值集合，不依赖原扇区顺序。
    auto smallMedian = [](auto& values, std::size_t count) {
        if (count == 0U) return 0.0;
        const std::size_t middle = count / 2U;
        auto begin = values.begin();
        auto end = begin + static_cast<std::ptrdiff_t>(count);
        auto middleIt = begin + static_cast<std::ptrdiff_t>(middle);
        std::nth_element(begin, middleIt, end);
        double value = *middleIt;
        if ((count & 1U) == 0U) {
            const auto lower = std::max_element(begin, middleIt);
            value = 0.5 * (value + *lower);
        }
        return value;
    };

    const double boundaryRadius = smallMedian(boundaries, boundaryCount);
    std::array<double, kCenterSectors> deviations{};
    for (std::size_t index = 0U; index < boundaryCount; ++index)
        deviations[index] = std::abs(boundaries[index] - boundaryRadius);
    const double boundaryMad = 1.4826 * smallMedian(deviations, boundaryCount);
    const double coverage = static_cast<double>(boundaryCount)
        / static_cast<double>(kCenterSectors);
    const double innerArea = kPi * std::pow(0.72 * lockedRadius, 2.0);
    const double annulusArea = kPi * (std::pow(1.55 * lockedRadius, 2.0)
        - std::pow(1.05 * lockedRadius, 2.0));
    const double innerDensity = static_cast<double>(innerCount)
        / std::max(1e-6, innerArea);
    const double annulusDensity = static_cast<double>(annulusCount)
        / std::max(1e-6, annulusArea);
    const double contrast = std::log1p(annulusDensity / (innerDensity + 0.25));
    const double score = 1.8 * coverage + 0.45 * contrast
        + 0.02 * annulusDensity - 2.2 * boundaryMad / lockedRadius
        - 1.8 * std::abs(boundaryRadius - lockedRadius) / lockedRadius
        - 0.10 * innerDensity;
    out.valid = std::isfinite(score);
    out.radius = boundaryRadius;
    out.mad = boundaryMad;
    out.coverage = coverage;
    out.innerDensity = innerDensity;
    out.annulusDensity = annulusDensity;
    out.score = score;
    return out;
}

/** 【函数导航】
 * 作用：执行“recoverCoarseVoidCenter”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline CuZhongXinHuiFu recoverCoarseVoidCenter(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& initialCenter,
    Eigen::Vector3d recognitionSurface,
    double lockedRadius,
    WaiEvidence outer)
{
    CuZhongXinHuiFu out;
    out.executed = true;
    if (samples.empty() || !initialCenter.allFinite() || !(lockedRadius > 0.5)) {
        out.decision = "REJECT_COARSE_CENTER_INVALID_INPUT";
        return out;
    }
    recognitionSurface = normalizeOr(recognitionSurface, Eigen::Vector3d::UnitZ());
    if (!outer.valid)
        outer = estimateOuter(samples, initialCenter, recognitionSurface, lockedRadius);
    if (!outer.valid) {
        out.decision = "REJECT_COARSE_CENTER_NO_OUTER_PLANE";
        return out;
    }
    Eigen::Vector3d u, v;
    makeAxes(outer.normal, u, v);
    const double planeBand = std::clamp(
        std::max(0.10, 2.0 * outer.rmse), 0.10, 0.22);
    const double searchExtent = std::clamp(1.80 * lockedRadius + 0.80, 4.0, 6.5);
    const double collectionExtent = searchExtent + 1.75 * lockedRadius;
    std::vector<PingMianPoint2D> surface;
    surface.reserve(samples.size());
    for (const DianYangBen& sample : samples) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - outer.center;
        const double planeResidual = delta.dot(outer.normal);
        if (std::abs(planeResidual) > planeBand) continue;
        const double x = delta.dot(u);
        const double y = delta.dot(v);
        if (std::abs(x) > collectionExtent || std::abs(y) > collectionExtent)
            continue;
        surface.push_back(PingMianPoint2D{x, y});
    }
    out.surfacePoints = static_cast<int>(surface.size());
    if (surface.size() < 100U) {
        out.decision = "REJECT_COARSE_CENTER_TOO_FEW_SURFACE_POINTS";
        return out;
    }
    PingMianPointGrid surfaceGrid;
    if (!surfaceGrid.build(surface, 0.50)) {
        out.decision = "REJECT_COARSE_CENTER_GRID_BUILD";
        return out;
    }
    const Eigen::Vector3d initialDelta = initialCenter - outer.center;
    const double initialX = initialDelta.dot(u);
    const double initialY = initialDelta.dot(v);
    CuHouXuanScore best;
    auto searchGrid = [&](double centerX, double centerY,
                          double extent, double step) {
        const int count = static_cast<int>(std::floor(2.0 * extent / step));
        std::vector<std::pair<double, double>> coordinates;
        coordinates.reserve(static_cast<std::size_t>((count + 1) * (count + 1)));
        for (int ix = 0; ix <= count; ++ix) {
            const double x = centerX - extent + static_cast<double>(ix) * step;
            for (int iy = 0; iy <= count; ++iy) {
                const double y = centerY - extent + static_cast<double>(iy) * step;
                coordinates.emplace_back(x, y);
            }
        }

        std::vector<CuHouXuanScore> evaluations(coordinates.size());
        std::vector<long long> visits(coordinates.size(), 0);
        const unsigned hardware = std::max(1U, std::thread::hardware_concurrency());
        const std::size_t requestedWorkers = coordinates.size() >= 128U
            ? std::min<std::size_t>(8U, static_cast<std::size_t>(hardware)) : 1U;
        const std::size_t workers = std::max<std::size_t>(
            1U, std::min<std::size_t>(requestedWorkers, coordinates.size()));
        out.parallelWorkers = std::max(out.parallelWorkers, static_cast<int>(workers));

        auto evaluateRange = [&](std::size_t begin, std::size_t end) {
            for (std::size_t index = begin; index < end; ++index) {
                evaluations[index] = evaluateCoarseCenterCandidate(
                    surface, surfaceGrid, coordinates[index].first,
                    coordinates[index].second, lockedRadius, visits[index]);
            }
        };

        if (workers == 1U) {
            evaluateRange(0U, coordinates.size());
        } else {
            std::vector<std::thread> threads;
            threads.reserve(workers);
            const std::size_t block = (coordinates.size() + workers - 1U) / workers;
            for (std::size_t worker = 0; worker < workers; ++worker) {
                const std::size_t begin = worker * block;
                const std::size_t end = std::min(coordinates.size(), begin + block);
                if (begin >= end) break;
                threads.emplace_back(evaluateRange, begin, end);
            }
            for (std::thread& thread : threads) thread.join();
        }

        for (std::size_t index = 0; index < evaluations.size(); ++index) {
            out.visitedSurfacePoints += visits[index];
            ++out.evaluatedCandidates;
            const CuHouXuanScore& candidate = evaluations[index];
            if (candidate.valid && (!best.valid || candidate.score > best.score))
                best = candidate;
        }
    };
    searchGrid(initialX, initialY, searchExtent, 0.45);
    if (best.valid) searchGrid(best.x, best.y, 0.60, 0.15);
    if (!best.valid) {
        out.decision = "REJECT_COARSE_CENTER_NO_VALID_GRID_CANDIDATE";
        return out;
    }
    out.center = outer.center + u * best.x + v * best.y;
    out.shift = (out.center - initialCenter).norm();
    out.boundaryRadius = best.radius;
    out.boundaryMad = best.mad;
    out.coverage = best.coverage;
    out.innerDensity = best.innerDensity;
    out.annulusDensity = best.annulusDensity;
    out.score = best.score;
    const double maximumShift = std::clamp(1.55 * lockedRadius, 3.0, 6.5);
    const bool gate = out.shift <= maximumShift
        && out.coverage >= 0.58
        && out.boundaryRadius >= 0.70 * lockedRadius
        && out.boundaryRadius <= 1.28 * lockedRadius
        && out.boundaryMad <= std::max(0.50, 0.15 * lockedRadius)
        && out.annulusDensity >= 1.15 * out.innerDensity;
    if (!gate) {
        if (out.shift > maximumShift)
            out.decision = "REJECT_COARSE_CENTER_SHIFT";
        else if (out.coverage < 0.58)
            out.decision = "REJECT_COARSE_CENTER_COVERAGE";
        else if (out.boundaryMad > std::max(0.50, 0.15 * lockedRadius))
            out.decision = "REJECT_COARSE_CENTER_BOUNDARY_SCATTER";
        else
            out.decision = "REJECT_COARSE_CENTER_VOID_CONTRAST";
        return out;
    }
    out.valid = true;
    out.decision = "APPLY_COARSE_SUPPORT_VOID_CENTER_RECOVERY";
    return out;
}

/** 【函数导航】
 * 作用：执行“generateInnerCandidates”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::vector<std::vector<QiePianNiHe>> generateInnerCandidates(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& origin,
    Eigen::Vector3d axis,
    double lockedRadius,
    int& candidateCount,
    int& histogramWindows,
    long long& histogramPointVisits,
    long long& fitPointVisits)
{
    candidateCount = 0;
    histogramWindows = 0;
    histogramPointVisits = 0;
    fitPointVisits = 0;
    axis = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    const double maximumDepth = std::clamp(0.55 * lockedRadius, 1.0, 2.0);
    const auto projected = projectNeighborhood(
        samples, origin, axis, 1.20 * lockedRadius + 0.45,
        -0.15, maximumDepth + 0.25);
    std::vector<std::vector<QiePianNiHe>> byDepth;
    static constexpr std::array<double, 4> kHalfWidths{{0.14, 0.20, 0.28, 0.38}};
    const double radialLow = std::max(0.45, 0.62 * lockedRadius);
    const double radialHigh = 1.20 * lockedRadius + 0.35;
    if (projected.size() < 20U || !(radialHigh > radialLow + 0.4)) return byDepth;

    std::vector<double> targetDepths;
    for (double targetDepth = 0.20;
         targetDepth <= maximumDepth + 1e-9; targetDepth += 0.12) {
        targetDepths.push_back(targetDepth);
    }
    using WindowIndices = std::vector<std::size_t>;
    std::vector<std::array<WindowIndices, 4>> windows(
        targetDepths.size());
    std::vector<int> radialBins(projected.size(), -1);
    for (std::size_t pointIndex = 0; pointIndex < projected.size(); ++pointIndex) {
        const TouYingPoint& point = projected[pointIndex];
        if (point.radius < radialLow || point.radius > radialHigh) continue;
        radialBins[pointIndex] = std::clamp(static_cast<int>(std::floor(
            (point.radius - radialLow) / (radialHigh - radialLow)
            * static_cast<double>(kInnerHistogramBins))),
            0, kInnerHistogramBins - 1);

        for (std::size_t depthIndex = 0; depthIndex < targetDepths.size(); ++depthIndex) {
            for (std::size_t halfIndex = 0; halfIndex < kHalfWidths.size(); ++halfIndex) {
                if (std::abs(point.depth - targetDepths[depthIndex])
                    <= kHalfWidths[halfIndex]) {
                    windows[depthIndex][halfIndex].push_back(pointIndex);
                }
            }
        }
    }

    Eigen::Vector3d u, v;
    makeAxes(axis, u, v);
    for (std::size_t depthIndex = 0; depthIndex < targetDepths.size(); ++depthIndex) {
        const double targetDepth = targetDepths[depthIndex];
        std::vector<QiePianNiHe> depthCandidates;
        for (std::size_t halfIndex = 0; halfIndex < kHalfWidths.size(); ++halfIndex) {
            const double halfWidth = kHalfWidths[halfIndex];
            const WindowIndices& window = windows[depthIndex][halfIndex];
            std::array<int, kInnerHistogramBins> histogram{};
            const int inputCount = static_cast<int>(window.size());
            ++histogramWindows;
            histogramPointVisits += static_cast<long long>(window.size());
            for (std::size_t pointIndex : window) {
                const int bin = radialBins[pointIndex];
                ++histogram[static_cast<std::size_t>(bin)];
            }
            if (inputCount < 18) continue;
            std::array<int, kInnerHistogramBins> smoothed{};
            int maximumPeak = 0;
            for (int bin = 0; bin < kInnerHistogramBins; ++bin) {
                int value = 0;
                for (int offset = -2; offset <= 2; ++offset) {
                    const int index = std::clamp(bin + offset, 0, kInnerHistogramBins - 1);
                    value += (3 - std::abs(offset))
                        * histogram[static_cast<std::size_t>(index)];
                }
                smoothed[static_cast<std::size_t>(bin)] = value;
                maximumPeak = std::max(maximumPeak, value);
            }
            if (maximumPeak <= 0) continue;
            std::array<int, kInnerHistogramBins> order{};
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](int left, int right) {
                return smoothed[static_cast<std::size_t>(left)]
                    > smoothed[static_cast<std::size_t>(right)];
            });
            std::vector<int> bins;
            for (int bin : order) {
                if (smoothed[static_cast<std::size_t>(bin)]
                    < std::max(4, static_cast<int>(std::ceil(0.24 * maximumPeak)))) break;
                bool separated = true;
                for (int previous : bins)
                    if (std::abs(previous - bin) < 3) separated = false;
                if (!separated) continue;
                bins.push_back(bin);
                if (bins.size() >= 7U) break;
            }
            for (int bin : bins) {
                const double modeRadius = radialLow
                    + (static_cast<double>(bin) + 0.5)
                    / static_cast<double>(kInnerHistogramBins)
                    * (radialHigh - radialLow);
                const double band = std::clamp(0.04 * modeRadius, 0.12, 0.25);
                std::vector<Eigen::Vector2d> points;
                fitPointVisits += static_cast<long long>(window.size());
                for (std::size_t pointIndex : window) {
                    const TouYingPoint& point = projected[pointIndex];
                    if (std::abs(point.radius - modeRadius) <= band) {
                        points.emplace_back(point.x, point.y);
                    }
                }
                QiePianNiHe fit = fitCircleRobust(
                    points, targetDepth, halfWidth, 1.25 * lockedRadius + 0.50);
                if (!fit.valid
                    || fit.radius < 0.68 * lockedRadius
                    || fit.radius > 1.16 * lockedRadius
                    || fit.center.norm() > std::max(0.85, 0.22 * lockedRadius)
                    || fit.coverage < 0.20 || fit.rmse > 0.22) continue;
                fit.centerWorld = origin + u * fit.center.x()
                    + v * fit.center.y() + axis * targetDepth;
                fit.source = "WallEvidence_INNER_RADIUS_MODE";
                depthCandidates.push_back(fit);
                ++candidateCount;
            }
        }

        std::sort(depthCandidates.begin(), depthCandidates.end(),
            [](const QiePianNiHe& left, const QiePianNiHe& right) {
                return left.radius < right.radius;
            });
        std::vector<QiePianNiHe> deduplicated;
        const double duplicateTolerance = std::max(0.07, 0.018 * lockedRadius);
        for (const QiePianNiHe& fit : depthCandidates) {
            int same = -1;
            for (std::size_t i = 0; i < deduplicated.size(); ++i) {
                if (std::abs(deduplicated[i].radius - fit.radius) < duplicateTolerance) {
                    same = static_cast<int>(i);
                    break;
                }
            }
            if (same < 0) {
                deduplicated.push_back(fit);
            } else {
                const double oldQuality = deduplicated[static_cast<std::size_t>(same)].coverage
                    - 1.7 * deduplicated[static_cast<std::size_t>(same)].rmse
                    + 0.001 * std::sqrt(static_cast<double>(deduplicated[static_cast<std::size_t>(same)].supportCount))
                    - 0.08 * deduplicated[static_cast<std::size_t>(same)].halfThickness;
                const double newQuality = fit.coverage - 1.7 * fit.rmse
                    + 0.001 * std::sqrt(static_cast<double>(fit.supportCount))
                    - 0.08 * fit.halfThickness;
                if (newQuality > oldQuality)
                    deduplicated[static_cast<std::size_t>(same)] = fit;
            }
        }
        byDepth.push_back(std::move(deduplicated));
    }
    return byDepth;
}

/** 【函数导航】
 * 作用：评估/审核“evaluateFamily”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool evaluateFamily(std::vector<QiePianNiHe> slices,
                           Eigen::Vector3d referenceAxis,
                           double lockedRadius,
                           NeiYuanZhuZu& family) {
    if (slices.size() < 3U) return false;
    std::sort(slices.begin(), slices.end(), [](const QiePianNiHe& a, const QiePianNiHe& b) {
        return a.depth < b.depth;
    });
    const double depthSpan = slices.back().depth - slices.front().depth;
    if (depthSpan < 0.24 || slices.front().depth > 0.80) return false;
    std::vector<double> radii, rmses;
    radii.reserve(slices.size()); rmses.reserve(slices.size());
    double coverage = 0.0;
    int support = 0;
    for (const QiePianNiHe& slice : slices) {
        radii.push_back(slice.radius);
        rmses.push_back(slice.rmse);
        coverage += slice.coverage;
        support += slice.supportCount;
    }
    const double radius = median(radii);
    std::vector<double> deviations;
    for (double value : radii) deviations.push_back(std::abs(value - radius));
    const double radiusMad = 1.4826 * median(deviations);
    coverage /= static_cast<double>(slices.size());
    const double medianRmse = median(rmses);

    double sumD = 0.0, sumR = 0.0, sumDD = 0.0, sumDR = 0.0;
    for (const QiePianNiHe& slice : slices) {
        sumD += slice.depth; sumR += slice.radius;
        sumDD += slice.depth * slice.depth;
        sumDR += slice.depth * slice.radius;
    }
    const double n = static_cast<double>(slices.size());
    const double determinant = n * sumDD - sumD * sumD;
    const double slope = std::abs(determinant) > 1e-10
        ? (n * sumDR - sumD * sumR) / determinant : 0.0;
    if (coverage < 0.20 || medianRmse > 0.18
        || radiusMad > std::max(0.15, 0.035 * lockedRadius)
        || std::abs(slope) > std::max(0.65, 0.14 * lockedRadius)) return false;

    Eigen::Vector3d intercept = slices.front().centerWorld;
    Eigen::Vector3d candidateAxis = referenceAxis;
    double lineRmse = std::numeric_limits<double>::infinity();
    double allRmse = lineRmse, threshold = 0.0;
    int stableStart = 0, used = 0, rejected = 0, transition = 0;
    if (!robustLineFit(slices, referenceAxis, intercept, candidateAxis,
                       lineRmse, allRmse, threshold, stableStart,
                       used, rejected, transition)) return false;
    if (lineRmse > std::max(0.25, 0.065 * lockedRadius)) return false;

    family.executed = true;
    family.valid = true;
    family.radius = radius;
    family.radiusMad = radiusMad;
    family.radiusSlope = slope;
    family.depthStart = slices.front().depth;
    family.depthEnd = slices.back().depth;
    family.depthSpan = depthSpan;
    family.meanCoverage = coverage;
    family.medianRmse = medianRmse;
    family.centerLineRmse = lineRmse;
    family.usedSlices = used;
    family.rejectedSlices = rejected;
    family.intercept = intercept;
    family.axisIn = candidateAxis;
    family.slices = std::move(slices);
    family.quality = 1.2 * coverage - 2.0 * medianRmse
        + 0.08 * std::min(8.0, n) + 0.001 * std::sqrt(static_cast<double>(support))
        - 0.8 * radiusMad - 0.12 * std::abs(slope) - 0.5 * lineRmse;
    family.decision = "VALID_MINIMUM_PERSISTENT_INNER_CYLINDER_FAMILY";
    return true;
}

/** 【函数导航】
 * 作用：估计“estimateMinimumInnerCylinder”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline NeiYuanZhuZu estimateMinimumInnerCylinder(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& origin,
    Eigen::Vector3d initialAxis,
    double lockedRadius) {
    NeiYuanZhuZu out;
    out.executed = true;
    if (samples.empty() || !origin.allFinite() || !(lockedRadius > 0.5)) {
        out.decision = "REJECT_INVALID_INPUT";
        return out;
    }
    initialAxis = normalizeOr(initialAxis, -Eigen::Vector3d::UnitZ());
    int candidateCount = 0;
    int histogramWindows = 0;
    long long histogramPointVisits = 0;
    long long fitPointVisits = 0;
    const auto byDepth = generateInnerCandidates(
        samples, origin, initialAxis, lockedRadius, candidateCount,
        histogramWindows, histogramPointVisits, fitPointVisits);
    out.candidateCount = candidateCount;
    out.histogramWindows = histogramWindows;
    out.histogramPointVisits = histogramPointVisits;
    out.fitPointVisits = fitPointVisits;
    if (candidateCount < 3) {
        out.decision = "REJECT_TOO_FEW_INNER_CYLINDER_CANDIDATES";
        return out;
    }

    std::vector<NeiYuanZhuZu> families;
    const double radiusTolerance = std::max(0.14, 0.032 * lockedRadius);
    for (const auto& depthCandidates : byDepth) {
        for (const QiePianNiHe& seed : depthCandidates) {
            double targetRadius = seed.radius;
            std::vector<std::pair<int, QiePianNiHe>> selected;
            for (int iteration = 0; iteration < 4; ++iteration) {
                selected.clear();
                for (std::size_t depthIndex = 0; depthIndex < byDepth.size(); ++depthIndex) {
                    bool found = false;
                    QiePianNiHe best;
                    double bestQuality = -std::numeric_limits<double>::infinity();
                    for (const QiePianNiHe& candidate : byDepth[depthIndex]) {
                        if (std::abs(candidate.radius - targetRadius) > radiusTolerance) continue;
                        const double quality = candidateQuality(
                            candidate, targetRadius, radiusTolerance);
                        if (!found || quality > bestQuality) {
                            found = true; best = candidate; bestQuality = quality;
                        }
                    }
                    if (found) selected.emplace_back(
                        static_cast<int>(depthIndex), best);
                }
                if (selected.size() < 3U) break;
                std::vector<double> values;
                for (const auto& item : selected) values.push_back(item.second.radius);
                targetRadius = median(values);
            }
            if (selected.size() < 3U) continue;

            std::vector<QiePianNiHe> component;
            int previousIndex = -100;
            auto flush = [&]() {
                if (component.size() >= 3U) {
                    NeiYuanZhuZu family;
                    if (evaluateFamily(component, initialAxis, lockedRadius, family))
                        families.push_back(std::move(family));
                }
                component.clear();
            };
            for (const auto& item : selected) {
                if (!component.empty() && item.first - previousIndex > 2) flush();
                component.push_back(item.second);
                previousIndex = item.first;
            }
            flush();
        }
    }
    out.familyCount = static_cast<int>(families.size());
    if (families.empty()) {
        out.decision = "REJECT_NO_PERSISTENT_INNER_CYLINDER_FAMILY";
        return out;
    }
    double bestQuality = -std::numeric_limits<double>::infinity();
    for (const NeiYuanZhuZu& family : families)
        bestQuality = std::max(bestQuality, family.quality);
    const NeiYuanZhuZu* selected = nullptr;
    for (const NeiYuanZhuZu& family : families) {

        if (family.quality < bestQuality - 0.45
            || family.meanCoverage < 0.27 || family.medianRmse > 0.18) continue;
        if (!selected || family.radius < selected->radius - 1e-9
            || (std::abs(family.radius - selected->radius) <= 1e-9
                && family.radiusMad < selected->radiusMad)) selected = &family;
    }
    if (!selected) {
        out.decision = "REJECT_MINIMUM_FAMILY_OUTSIDE_QUALITY_ENVELOPE";
        return out;
    }
    out = *selected;
    out.candidateCount = candidateCount;
    out.familyCount = static_cast<int>(families.size());
    out.histogramWindows = histogramWindows;
    out.histogramPointVisits = histogramPointVisits;
    out.fitPointVisits = fitPointVisits;
    out.decision = "APPLY_MINIMUM_PERSISTENT_INNER_CYLINDER";
    return out;
}

/** 【类型导航注释】
 * ZhuiObservationPoint：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhuiObservationPoint {
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
    double radius = 0.0;
    int sector = 0;
};

/** 【函数导航】
 * 作用：执行“sortedRadiusMedian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double sortedRadiusMedian(const std::vector<ZhuiObservationPoint>& points,
                                 std::size_t begin, std::size_t end) {
    if (begin >= end || end > points.size()) return 0.0;
    const std::size_t count = end - begin;
    const std::size_t middle = begin + count / 2U;
    double value = points[middle].radius;
    if ((count & 1U) == 0U)
        value = 0.5 * (value + points[middle - 1U].radius);
    return value;
}

/** 【类型导航注释】
 * ZhuiAxisEvaluation：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhuiAxisEvaluation {
    bool valid = false;
    Eigen::Vector3d axis = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d u = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v = Eigen::Vector3d::UnitY();
    Eigen::Vector4d parameters = Eigen::Vector4d::Zero();
    double rmse = std::numeric_limits<double>::infinity();
    double scale = std::numeric_limits<double>::infinity();
    double evenRmse = std::numeric_limits<double>::infinity();
    double oddRmse = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();
    int cells = 0;
    int sectors = 0;
    int depthBins = 0;
    std::array<int, kSectors> sectorSupport{};
    std::array<double, kSectors> sectorResidual{};
};

/** 【类型导航注释】
 * ZhuiAxisEvaluationScratch：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhuiAxisEvaluationScratch {
    using CellPoints = std::vector<ZhuiObservationPoint>;
    std::array<CellPoints, kConeDepthBins * kSectors> cellPoints;
    std::vector<ZhuiObservationPoint> observations;
    std::vector<double> residuals;
    std::vector<double> deviations;
    std::array<std::vector<double>, kSectors> sectorResiduals;

    /** 【函数导航】
     * 作用：清理/重置“reset”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 位姿计算。
     * 主要引用/调用位置：HoleShibie_Recognition.cpp、DianYun_IO.cpp、DianYunXianshi_View.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void reset() {
        for (auto& cell : cellPoints) cell.clear();
        observations.clear();
        residuals.clear();
        deviations.clear();
        for (auto& sector : sectorResiduals) sector.clear();
    }
};

/** 【函数导航】
 * 作用：评估/审核“evaluateConeAxisIndependent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline ZhuiAxisEvaluation evaluateConeAxisIndependent(
    const std::vector<Eigen::Vector3d>& roiDeltas,
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double radiusAtZero,
    double radiusSlope,
    double depthStart,
    double depthEnd,
    double lockedRadius,
    double priorAngleDegrees)
{
    ZhuiAxisEvaluation out;
    (void)center; // ROI 已在调用端一次性转换为相对中心坐标。
    axis = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    out.axis = axis;
    makeAxes(axis, out.u, out.v);
    const double radialLow = std::max(0.45, 0.52 * lockedRadius);
    const double radialHigh = 1.22 * lockedRadius + 0.45;
    // 同一线程会连续评估数十个轴候选；复用 cell/residual 工作区，
    // 避免每个候选重新构造 12×72 个小 vector。只改变内存生命周期，不改变点序或计算公式。
    static thread_local ZhuiAxisEvaluationScratch scratch;
    scratch.reset();
    auto& cellPoints = scratch.cellPoints;
    using CellPoints = ZhuiAxisEvaluationScratch::CellPoints;
    for (const Eigen::Vector3d& delta : roiDeltas) {
        const double depth = delta.dot(axis);
        if (depth < depthStart - 0.08 || depth > depthEnd + 0.08) continue;
        const double x = delta.dot(out.u);
        const double y = delta.dot(out.v);
        const double radius = std::hypot(x, y);
        if (radius < radialLow || radius > radialHigh) continue;
        double angle = std::atan2(y, x);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        const int depthBin = std::clamp(static_cast<int>(std::floor(
            (depth - depthStart) / (depthEnd - depthStart)
            * static_cast<double>(kConeDepthBins))), 0, kConeDepthBins - 1);
        cellPoints[static_cast<std::size_t>(depthBin * kSectors + sector)]
            .push_back(ZhuiObservationPoint{x, y, depth, radius, sector});
    }

    auto& observations = scratch.observations;
    if (observations.capacity() < static_cast<std::size_t>(kConeDepthBins * kSectors))
        observations.reserve(kConeDepthBins * kSectors);
    std::array<unsigned char, kSectors> coveredSectors{};
    std::array<unsigned char, kConeDepthBins> coveredDepths{};
    const double modeWindow = std::clamp(0.040 * lockedRadius, 0.16, 0.24);
    for (int depthBin = 0; depthBin < kConeDepthBins; ++depthBin) {
        for (int sector = 0; sector < kSectors; ++sector) {
            CellPoints& points = cellPoints[static_cast<std::size_t>(
                depthBin * kSectors + sector)];
            if (points.empty()) continue;
            std::sort(points.begin(), points.end(),
                [](const ZhuiObservationPoint& left,
                   const ZhuiObservationPoint& right) {
                    return left.radius < right.radius;
                });
            const double cellMedian = sortedRadiusMedian(points, 0U, points.size());
            std::size_t bestBegin = 0U, bestEnd = 1U;
            double bestMedianDistance = std::numeric_limits<double>::infinity();
            std::size_t end = 0U;
            for (std::size_t begin = 0U; begin < points.size(); ++begin) {
                if (end < begin) end = begin;
                while (end < points.size()
                    && points[end].radius - points[begin].radius <= modeWindow) {
                    ++end;
                }
                const std::size_t count = end - begin;
                const double windowMedian = sortedRadiusMedian(points, begin, end);
                const double medianDistance = std::abs(windowMedian - cellMedian);
                const std::size_t bestCount = bestEnd - bestBegin;
                if (count > bestCount
                    || (count == bestCount
                        && medianDistance < bestMedianDistance)) {
                    bestBegin = begin;
                    bestEnd = end;
                    bestMedianDistance = medianDistance;
                }
            }
            const double targetRadius = sortedRadiusMedian(points, bestBegin, bestEnd);
            const ZhuiObservationPoint* selected = nullptr;
            double selectedDistance = std::numeric_limits<double>::infinity();
            for (std::size_t i = bestBegin; i < bestEnd; ++i) {
                const double distance = std::abs(points[i].radius - targetRadius);
                if (!selected || distance < selectedDistance) {
                    selected = &points[i];
                    selectedDistance = distance;
                }
            }
            if (!selected) continue;
            observations.push_back(*selected);
            coveredSectors[static_cast<std::size_t>(sector)] = 1U;
            coveredDepths[static_cast<std::size_t>(depthBin)] = 1U;
        }
    }
    out.cells = static_cast<int>(observations.size());
    for (unsigned char value : coveredSectors) out.sectors += value ? 1 : 0;
    for (unsigned char value : coveredDepths) out.depthBins += value ? 1 : 0;
    if (out.cells < 120 || out.sectors < 48 || out.depthBins < 6) return out;

    Eigen::Vector4d parameters(0.0, 0.0, radiusAtZero, radiusSlope);
    auto residualFor = [](const ZhuiObservationPoint& point,
                          const Eigen::Vector4d& p) {
        return std::hypot(point.x - p(0), point.y - p(1))
            - p(2) - p(3) * point.depth;
    };
    auto& residuals = scratch.residuals;
    auto& deviations = scratch.deviations;
    if (residuals.capacity() < observations.size()) residuals.reserve(observations.size());
    if (deviations.capacity() < observations.size()) deviations.reserve(observations.size());
    for (int iteration = 0; iteration < 6; ++iteration) {
        residuals.clear();
        for (const auto& point : observations)
            residuals.push_back(residualFor(point, parameters));
        const double residualMedian = median(residuals);
        deviations.clear();
        for (double residual : residuals)
            deviations.push_back(std::abs(residual - residualMedian));
        const double scale = std::max(0.025, 1.4826 * median(deviations));
        Eigen::Matrix4d normal = Eigen::Matrix4d::Zero();
        Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& point = observations[i];
            const double dx = point.x - parameters(0);
            const double dy = point.y - parameters(1);
            const double radius = std::max(1e-6, std::hypot(dx, dy));
            const double centeredResidual = residuals[i] - residualMedian;
            const double weight = std::min(1.0,
                2.2 * scale / std::max(1e-9, std::abs(centeredResidual)));
            const Eigen::Vector4d row(-dx / radius, -dy / radius,
                                      -1.0, -point.depth);
            normal.noalias() += weight * row * row.transpose();
            rhs.noalias() -= weight * row * residuals[i];
        }
        normal.diagonal().array() += 1e-8;
        const Eigen::LDLT<Eigen::Matrix4d> ldlt(normal);
        if (ldlt.info() != Eigen::Success) return out;
        Eigen::Vector4d step = ldlt.solve(rhs);
        if (!step.allFinite()) return out;
        step(0) = std::clamp(step(0), -0.14, 0.14);
        step(1) = std::clamp(step(1), -0.14, 0.14);
        step(2) = std::clamp(step(2), -0.18, 0.18);
        step(3) = std::clamp(step(3), -0.12, 0.12);
        parameters += step;
        if (step.norm() < 1e-5) break;
    }
    if (!(parameters(3) < -0.05)) return out;

    residuals.clear();
    deviations.clear();
    auto& sectorResiduals = scratch.sectorResiduals;
    for (auto& sector : sectorResiduals) sector.clear();
    double squared = 0.0, evenSquared = 0.0, oddSquared = 0.0;
    int evenCount = 0, oddCount = 0;
    for (const auto& point : observations) {
        const double residual = residualFor(point, parameters);
        residuals.push_back(residual);
        squared += residual * residual;
        sectorResiduals[static_cast<std::size_t>(point.sector)].push_back(residual);
        if ((point.sector & 1) == 0) {
            evenSquared += residual * residual;
            ++evenCount;
        } else {
            oddSquared += residual * residual;
            ++oddCount;
        }
    }
    const double residualMedian = median(residuals);
    for (double residual : residuals)
        deviations.push_back(std::abs(residual - residualMedian));
    out.scale = 1.4826 * medianInPlace(deviations);
    out.rmse = std::sqrt(squared / static_cast<double>(residuals.size()));
    out.evenRmse = evenCount > 0
        ? std::sqrt(evenSquared / static_cast<double>(evenCount))
        : std::numeric_limits<double>::infinity();
    out.oddRmse = oddCount > 0
        ? std::sqrt(oddSquared / static_cast<double>(oddCount))
        : std::numeric_limits<double>::infinity();
    out.parameters = parameters;
    out.score = out.rmse + 0.12 * out.scale
        + 0.04 * std::abs(out.evenRmse - out.oddRmse)
        + 0.00035 * priorAngleDegrees * priorAngleDegrees;

    for (int sector = 0; sector < kSectors; ++sector) {
        std::vector<double>& sectorValues = sectorResiduals[static_cast<std::size_t>(sector)];
        out.sectorSupport[static_cast<std::size_t>(sector)]
            = static_cast<int>(sectorValues.size());
        out.sectorResidual[static_cast<std::size_t>(sector)]
            = sectorValues.empty() ? 0.0 : medianInPlace(sectorValues);
    }
    out.valid = std::isfinite(out.score);
    return out;
}

/** 【函数导航】
 * 作用：精修“refineConeFromRawWall”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline RawZhuiWallFit refineConeFromRawWall(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double lockedRadius,
    double radiusAtZero,
    double radiusSlope,
    const HoleBiModel& wall)
{
    RawZhuiWallFit out;
    out.executed = true;
    if (samples.empty() || !center.allFinite() || !(lockedRadius > 0.5)
        || !(radiusSlope < -0.05) || wall.slices.size() < 4U) {
        out.decision = "REJECT_CONE_RAW_WALL_INPUT";
        return out;
    }
    axis = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    const double depthStart = wall.slices.front().depth;
    const double depthEnd = wall.slices.back().depth;
    if (!(depthEnd > depthStart + 0.45)) {
        out.decision = "REJECT_CONE_RAW_WALL_DEPTH_SPAN";
        return out;
    }

    const double radialHigh = 1.22 * lockedRadius + 0.45;
    const double maximumDepth = std::max(std::abs(depthStart - 0.08),
                                         std::abs(depthEnd + 0.08));
    const double distanceLimitSquared = radialHigh * radialHigh
        + maximumDepth * maximumDepth + 1e-10;
    std::vector<Eigen::Vector3d> coneRoiDeltas;
    coneRoiDeltas.reserve(samples.size());
    for (const DianYangBen& sample : samples) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - center;
        if (delta.squaredNorm() <= distanceLimitSquared)
            coneRoiDeltas.push_back(delta);
    }
    out.roiPoints = static_cast<int>(coneRoiDeltas.size());
    auto evaluateAxis = [&](const Eigen::Vector3d& candidate,
                            double correction) {
        ++out.axisEvaluations;
        return evaluateConeAxisIndependent(
            coneRoiDeltas, center, candidate, radiusAtZero, radiusSlope,
            depthStart, depthEnd, lockedRadius, correction);
    };
    ZhuiAxisEvaluation baseline = evaluateAxis(axis, 0.0);
    if (!baseline.valid) {
        out.decision = "REJECT_CONE_INDEPENDENT_WALL_BASELINE_COVERAGE";
        return out;
    }
    ZhuiAxisEvaluation best = baseline;
    auto searchStage = [&](double extent, int count) {
        const Eigen::Vector3d stageAxis = best.axis;
        Eigen::Vector3d stageU, stageV;
        makeAxes(stageAxis, stageU, stageV);
        ZhuiAxisEvaluation stageBest = best;

        /** 【类型导航注释】
         * ZhouRenWu：Hole 位姿计算中的自定义 结构体。
         * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
         * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
         */
        struct ZhouRenWu {
            Eigen::Vector3d candidate = -Eigen::Vector3d::UnitZ();
            double correction = 0.0;
        };
        std::vector<ZhouRenWu> tasks;
        tasks.reserve(static_cast<std::size_t>(count * count));
        for (int iu = 0; iu < count; ++iu) {
            const double offsetU = count > 1
                ? -extent + 2.0 * extent * static_cast<double>(iu)
                    / static_cast<double>(count - 1) : 0.0;
            for (int iv = 0; iv < count; ++iv) {
                const double offsetV = count > 1
                    ? -extent + 2.0 * extent * static_cast<double>(iv)
                        / static_cast<double>(count - 1) : 0.0;

                if (std::abs(offsetU) < 1e-15 && std::abs(offsetV) < 1e-15)
                    continue;
                const Eigen::Vector3d candidate = normalizeOr(
                    stageAxis + stageU * offsetU + stageV * offsetV, stageAxis);
                const double correction = angleDegrees(axis, candidate);
                if (correction > 3.25) continue;
                tasks.push_back(ZhouRenWu{candidate, correction});
            }
        }

        std::vector<ZhuiAxisEvaluation> evaluations(tasks.size());
        const unsigned hardware = std::max(1U, std::thread::hardware_concurrency());
        const std::size_t requestedWorkers = tasks.size() >= 8U
            ? std::min<std::size_t>(8U, static_cast<std::size_t>(hardware)) : 1U;
        const std::size_t workers = std::max<std::size_t>(
            1U, std::min<std::size_t>(requestedWorkers, tasks.size()));
        out.parallelWorkers = std::max(out.parallelWorkers, static_cast<int>(workers));
        out.axisEvaluations += static_cast<int>(tasks.size());

        auto evaluateRange = [&](std::size_t begin, std::size_t end) {
            for (std::size_t index = begin; index < end; ++index) {
                evaluations[index] = evaluateConeAxisIndependent(
                    coneRoiDeltas, center, tasks[index].candidate,
                    radiusAtZero, radiusSlope, depthStart, depthEnd,
                    lockedRadius, tasks[index].correction);
            }
        };
        if (workers == 1U) {
            evaluateRange(0U, tasks.size());
        } else {
            std::vector<std::thread> threads;
            threads.reserve(workers);
            const std::size_t block = (tasks.size() + workers - 1U) / workers;
            for (std::size_t worker = 0; worker < workers; ++worker) {
                const std::size_t begin = worker * block;
                const std::size_t end = std::min(tasks.size(), begin + block);
                if (begin >= end) break;
                threads.emplace_back(evaluateRange, begin, end);
            }
            for (std::thread& thread : threads) thread.join();
        }

        for (std::size_t index = 0; index < evaluations.size(); ++index) {
            ZhuiAxisEvaluation& evaluation = evaluations[index];
            if (evaluation.valid && evaluation.score < stageBest.score)
                stageBest = std::move(evaluation);
        }
        best = std::move(stageBest);
    };
    searchStage(0.050, 5);
    searchStage(0.015, 5);
    searchStage(0.006, 3);

    out.cellCount = best.cells;
    out.coveredSectors = best.sectors;
    out.coveredDepthBins = best.depthBins;
    out.noDriftRmse = baseline.rmse;
    out.baselineEvenRmse = baseline.evenRmse;
    out.baselineOddRmse = baseline.oddRmse;
    out.rmse = best.rmse;
    out.scale = best.scale;
    out.evenRmse = best.evenRmse;
    out.oddRmse = best.oddRmse;
    out.improvement = baseline.rmse > 1e-9
        ? (baseline.rmse - best.rmse) / baseline.rmse : 0.0;
    out.evenImprovement = baseline.evenRmse > 1e-9
        ? (baseline.evenRmse - best.evenRmse) / baseline.evenRmse : 0.0;
    out.oddImprovement = baseline.oddRmse > 1e-9
        ? (baseline.oddRmse - best.oddRmse) / baseline.oddRmse : 0.0;
    out.axisIn = best.axis;
    out.radiusAtZero = best.parameters(2);
    out.radiusSlope = best.parameters(3);
    out.axisCorrectionDegrees = angleDegrees(axis, best.axis);
    out.lockedMouthDepth = (lockedRadius - out.radiusAtZero) / out.radiusSlope;
    out.centerAtLockedRadius = center
        + best.u * best.parameters(0) + best.v * best.parameters(1)
        + best.axis * out.lockedMouthDepth;
    out.centerShift = (out.centerAtLockedRadius - center).norm();
    out.sectorSupport = best.sectorSupport;
    out.sectorResidual = best.sectorResidual;
    const bool crossValidated = out.axisCorrectionDegrees <= 0.20
        || (out.improvement >= 0.04
            && out.evenImprovement >= 0.015
            && out.oddImprovement >= 0.015);
    const bool gate = out.cellCount >= 120 && out.coveredSectors >= 48
        && out.coveredDepthBins >= 6 && out.scale <= 0.12 && out.rmse <= 0.14
        && out.axisCorrectionDegrees <= 3.25 && out.centerShift <= 0.75
        && out.lockedMouthDepth >= -0.90 && out.lockedMouthDepth <= 0.60
        && crossValidated;
    if (!gate) {
        if (out.scale > 0.12 || out.rmse > 0.14)
            out.decision = "REJECT_CONE_INDEPENDENT_WALL_RESIDUAL";
        else if (out.axisCorrectionDegrees > 3.25)
            out.decision = "REJECT_CONE_INDEPENDENT_WALL_AXIS_STEP";
        else if (out.centerShift > 0.75)
            out.decision = "REJECT_CONE_INDEPENDENT_WALL_CENTER_SHIFT";
        else if (out.lockedMouthDepth < -0.90 || out.lockedMouthDepth > 0.60)
            out.decision = "REJECT_CONE_INDEPENDENT_WALL_MOUTH_DEPTH";
        else
            out.decision = "REJECT_CONE_ALTERNATING_SECTOR_CROSS_VALIDATION";
        return out;
    }
    out.valid = true;
    out.decision = "APPLY_MODEL_INDEPENDENT_CONE_WALL_CROSS_VALIDATED_AXIS";
    return out;
}

/** 【函数导航】
 * 作用：执行“annotateContourWithConeEvidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline void annotateContourWithConeEvidence(HoleKouContour& contour,
                                            const RawZhuiWallFit& rawCone) {
    if (!contour.valid || !rawCone.valid) return;
    int supported = 0;
    for (int sector = 0; sector < kSectors; ++sector) {
        HoleKouContourPoint& point = contour.points[static_cast<std::size_t>(sector)];
        const int support = rawCone.sectorSupport[static_cast<std::size_t>(sector)];
        point.supportCount = support;
        point.evidenceUsed = support > 0;
        point.evidenceResidual = rawCone.sectorResidual[static_cast<std::size_t>(sector)];
        supported += support;
    }
    contour.evidenceSupportCount = supported;
    contour.evidenceRawSectors = rawCone.coveredSectors;
    contour.evidenceUsedSectors = rawCone.coveredSectors;
    contour.evidenceScale = rawCone.scale;
    contour.source = "WallEvidence_RAW_CONE_WALL_SURFACE_INTERSECTION";
    contour.decision = "APPLY_RAW_SUPPORTED_CONE_CONTOUR_KEEP_SEMANTIC_RADIUS";
}

/** 【函数导航】
 * 作用：构建“makeMeasuredThroatCircle”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline HoleKouContour makeMeasuredThroatCircle(
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double measuredRadius,
    double canonicalRadius,
    double throatDepth,
    int supportCount)
{
    HoleKouContour contour;
    contour.executed = true;
    if (!center.allFinite() || !(measuredRadius > 0.45)) {
        contour.decision = "REJECT_INVALID_MEASURED_THROAT";
        return contour;
    }
    axis = normalizeOr(axis, -Eigen::Vector3d::UnitZ());
    Eigen::Vector3d u, v;
    makeAxes(axis, u, v);
    std::vector<double> depths;
    depths.reserve(kSectors);
    for (int sector = 0; sector < kSectors; ++sector) {
        const double angle = (static_cast<double>(sector) + 0.5)
            * 2.0 * kPi / static_cast<double>(kSectors);
        HoleKouContourPoint& point = contour.points[static_cast<std::size_t>(sector)];
        point.valid = true;
        point.evidenceUsed = true;
        point.supportCount = supportCount / std::max(1, kSectors);
        point.angle = angle;
        point.axialDepth = throatDepth;
        point.localRadius = measuredRadius;
        point.planeResidual = 0.0;
        point.evidenceResidual = 0.0;
        point.world = center + u * (measuredRadius * std::cos(angle))
            + v * (measuredRadius * std::sin(angle));
        depths.push_back(throatDepth);
    }
    contour.valid = true;
    contour.validPoints = kSectors;
    contour.coverage = 1.0;
    contour.minimumAxialDepth = throatDepth;
    contour.maximumAxialDepth = throatDepth;
    contour.medianAxialDepth = throatDepth;
    contour.axialSpan = 0.0;
    contour.minimumLocalRadius = measuredRadius;
    contour.maximumLocalRadius = measuredRadius;
    contour.planarRingMaximumPlaneResidual = 0.0;
    contour.maximumPlaneResidual = 0.0;
    contour.canonicalRadius = canonicalRadius;
    contour.evidenceRadius = measuredRadius;
    contour.evidenceScale = 0.0;
    contour.evidenceSupportCount = supportCount;
    contour.evidenceRawSectors = kSectors;
    contour.evidenceUsedSectors = kSectors;
    contour.confidence = 1.0;
    contour.source = "WallEvidence_MEASURED_MINIMUM_INNER_THROAT_CIRCLE";
    contour.decision = "APPLY_MEASURED_THROAT_NOT_OUTER_SURFACE_PROJECTION";
    return contour;
}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimate(const std::vector<DianYangBen>& samples,
                       const Eigen::Vector3d& mouthCenter,
                       Eigen::Vector3d recognitionSurface,
                       double topRadius,
                       double bottomRadius,
                       double depth,
                       int inwardPolarity,
                       int holeType) {
    const HoleWeiziBase::Result base = HoleWeiziBase::estimate(
        samples, mouthCenter, recognitionSurface, topRadius, bottomRadius,
        depth, inwardPolarity, holeType);
    Result out;
    static_cast<HoleWeiziBase::Result&>(out) = base;
    auto finish = [&]() { return out; };
    out.topRadius = topRadius;
    out.originalRadius = topRadius;
    out.radiusShift = 0.0;
    if (samples.empty() || !mouthCenter.allFinite() || !(topRadius > 0.5)
        || inwardPolarity == 0 || (holeType != 1 && holeType != 2)) return finish();

    recognitionSurface = normalizeOr(recognitionSurface, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d initialAxis = normalizeOr(
        recognitionSurface * static_cast<double>(inwardPolarity),
        -recognitionSurface);

    if (holeType == 1) {
        WaiEvidence outer = out.outer;
        if (!outer.valid)
            outer = estimateOuter(samples, mouthCenter, recognitionSurface, topRadius);
        out.coarseCenter = recoverCoarseVoidCenter(
            samples, mouthCenter, recognitionSurface, topRadius, outer);
        const Eigen::Vector3d cylinderSeed = out.coarseCenter.valid
            ? out.coarseCenter.center : mouthCenter;
        out.innerCylinder = estimateMinimumInnerCylinder(
            samples, cylinderSeed, initialAxis, topRadius);
        if (!out.innerCylinder.valid) return finish();
        Eigen::Vector3d finalAxis = boundedAxisStep(
            initialAxis, out.innerCylinder.axisIn, 3.0);
        if (finalAxis.dot(initialAxis) < 0.0) finalAxis = -finalAxis;

        // 这里得到的是“最小内喉截面中心”，不是机械外表面的 Hole 上口中心。
        // 它对孔轴、内壁半径和切向圆心非常有价值，但其 depthStart 本身就是向孔内的深度。
        // HoleShibie_Recognition 在提交最终 centerTop 时会把该位姿中心沿最终孔轴与
        // canonical mouth 平面求交，因此本层保留它作为内部测量证据，不再让调用方误把
        // “内喉截面所在高度”解释成机械上口 Z。
        Eigen::Vector3d zuiXiaoNeiHouCenter = out.innerCylinder.intercept
            + finalAxis * out.innerCylinder.depthStart;
        const double maximumCenterShift = out.coarseCenter.valid
            ? std::clamp(1.70 * topRadius, 3.5, 7.0) : 2.0;
        if ((zuiXiaoNeiHouCenter - mouthCenter).norm() > maximumCenterShift) return finish();
        int totalSupport = 0;
        for (const QiePianNiHe& slice : out.innerCylinder.slices)
            totalSupport += slice.supportCount;
        HoleKouContour contour = makeMeasuredThroatCircle(
            zuiXiaoNeiHouCenter, finalAxis, out.innerCylinder.radius, topRadius,
            out.innerCylinder.depthStart, totalSupport);
        if (!contour.valid) return finish();
        contour.evidenceScale = out.innerCylinder.radiusMad;
        contour.evidenceCenterShift = (zuiXiaoNeiHouCenter - mouthCenter).norm();
        contour.evidenceRawSectors = out.innerCylinder.candidateCount;
        contour.evidenceUsedSectors = out.innerCylinder.usedSlices;

        // Result 在 HoleWeizi 层返回的是“位姿中心候选”；最终机械上口平面由调用层统一。
        out.centerTop = zuiXiaoNeiHouCenter;
        out.axisIn = finalAxis;
        out.surfaceNormal = finalAxis * static_cast<double>(inwardPolarity);
        if (out.surfaceNormal.dot(recognitionSurface) < 0.0)
            out.surfaceNormal = -out.surfaceNormal;
        out.outer = outer;
        out.contour = contour;
        out.measuredInnerRadiusValid = true;
        out.measuredInnerRadius = out.innerCylinder.radius;
        out.centerShift = (zuiXiaoNeiHouCenter - mouthCenter).norm();
        out.axisCorrectionDegrees = angleDegrees(initialAxis, finalAxis);
        out.confidence = std::clamp(0.40 * out.innerCylinder.meanCoverage
            + 0.25 * std::clamp((0.18 - out.innerCylinder.medianRmse) / 0.15, 0.0, 1.0)
            + 0.20 * std::clamp((0.16 - out.innerCylinder.radiusMad) / 0.16, 0.0, 1.0)
            + 0.15 * (out.coarseCenter.valid ? out.coarseCenter.coverage : 0.0),
            0.0, 1.0);
        out.valid = true;
        out.executed = true;
        out.source = "WallEvidence_STRAIGHT_COARSE_RECENTER_MINIMUM_INNER_THROAT";
    } else {
        if (!out.valid || !out.wall.valid || !out.outer.valid) return finish();
        out.rawCone = refineConeFromRawWall(
            samples, out.centerTop, out.axisIn, topRadius,
            out.wall.radiusAtZero, out.wall.radiusSlope, out.wall);

        out.coneFamilyConeFamilyAngleDeg =
            angleDegrees(out.axisIn, out.rawCone.axisIn);
        if (!out.rawCone.valid) return finish();
        HoleKouContour contour = estimateLockedRadiusSurfaceIntersectionContour(
            out.rawCone.centerAtLockedRadius, out.rawCone.axisIn,
            topRadius, out.rawCone.radiusSlope, 2, out.outer);
        if (!contour.valid) return finish();
        annotateContourWithConeEvidence(contour, out.rawCone);
        out.centerTop = out.rawCone.centerAtLockedRadius;
        out.axisIn = out.rawCone.axisIn;
        out.surfaceNormal = out.axisIn * static_cast<double>(inwardPolarity);
        if (out.surfaceNormal.dot(recognitionSurface) < 0.0)
            out.surfaceNormal = -out.surfaceNormal;
        out.wall.radiusAtZero = out.rawCone.radiusAtZero;
        out.wall.radiusSlope = out.rawCone.radiusSlope;
        out.contour = contour;
        out.centerShift = (out.centerTop - mouthCenter).norm();
        out.axisCorrectionDegrees = angleDegrees(initialAxis, out.axisIn);
        out.confidence = std::clamp(
            0.45 * std::clamp((0.14 - out.rawCone.rmse) / 0.10, 0.0, 1.0)
            + 0.30 * std::clamp((0.12 - out.rawCone.scale) / 0.09, 0.0, 1.0)
            + 0.25 * std::clamp(static_cast<double>(out.rawCone.coveredSectors) / 72.0, 0.0, 1.0),
            0.0, 1.0);
        out.valid = true;
        out.executed = true;
        out.source = "WallEvidence_CONE_MODEL_INDEPENDENT_WALL_CROSS_VALIDATED";
    }
    out.topRadius = topRadius;
    out.originalRadius = topRadius;
    out.radiusShift = 0.0;
    return finish();
}

}

// ============================================================================
// 功能分区：孔口位姿最终估计
// ============================================================================
/*
模块职责：
孔口统一姿态估计模块。

主要调用位置：
由正式孔识别后段调用，综合孔壁和孔口证据形成规范姿态/半径结果。

维护说明：
该链处于生产几何热路径；候选顺序、评分、网格分辨率和阈值都属于冻结语义，纯整理不改。
*/

//

//

//

namespace HoleWeiziFinal {

using HoleWeiziBase::HoleKouContour;
using HoleWeiziBase::HoleKouContourPoint;
using HoleWeiziBase::WaiEvidence;
using HoleWeiziBase::DianYangBen;
using HoleWeiziBase::QiePianNiHe;
using HoleWeiziBase::angleDegrees;
using HoleWeiziBase::fitCircleRobust;
using HoleWeiziBase::kPi;
using HoleWeiziBase::kSectors;
using HoleWeiziBase::makeAxes;
using HoleWeiziBase::median;
using HoleWeiziBase::normalizeOr;
using Result = HoleWeiziCu::Result;

struct ErCiZhichengSurface {
    bool valid = false;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d u = Eigen::Vector3d::UnitX();
    Eigen::Vector3d v = Eigen::Vector3d::UnitY();
    Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
    Eigen::Matrix<double, 6, 1> coefficients =
        Eigen::Matrix<double, 6, 1>::Zero();
    double residualMedian = 0.0;
    double residualMad = std::numeric_limits<double>::infinity();
    double rmse = std::numeric_limits<double>::infinity();
    double surfaceTolerance = 0.0;
    int supportCount = 0;
    int coveredSectors = 0;
    double coverage = 0.0;
};

/** 【类型导航注释】
 * BiaoMianPoint：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct BiaoMianPoint {
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double radial = 0.0;
    double angle = 0.0;
    double surfaceResidual = 0.0;
};

/** 【类型导航注释】
 * ZhiHoleBiaoMianBian：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhiHoleBiaoMianBian {
    bool executed = false;
    bool valid = false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d surfaceNormal = Eigen::Vector3d::UnitZ();
    double radius = 0.0;
    double fitRmse = std::numeric_limits<double>::infinity();
    double radiusMad = std::numeric_limits<double>::infinity();
    double centerShift = 0.0;
    double throatGap = 0.0;
    int boundaryCandidates = 0;
    int observedSectors = 0;
    int supportCount = 0;
    HoleKouContour contour;
    const char* decision = "NOT_EXECUTED";
};

/** 【函数导航】
 * 作用：执行“quadraticRow”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Eigen::Matrix<double, 6, 1> quadraticRow(double x, double y) {
    Eigen::Matrix<double, 6, 1> row;
    row << 1.0, x, y, x * x, x * y, y * y;
    return row;
}

/** 【函数导航】
 * 作用：评估/审核“evaluateSurface”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double evaluateSurface(const ErCiZhichengSurface& surface,
                              double x,
                              double y) {
    return quadraticRow(x, y).dot(surface.coefficients);
}

/** 【函数导航】
 * 作用：评估/审核“evaluateSurfaceNormal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Eigen::Vector3d evaluateSurfaceNormal(
    const ErCiZhichengSurface& surface,
    double x,
    double y)
{
    const auto& c = surface.coefficients;
    const double dx = c(1) + 2.0 * c(3) * x + c(4) * y;
    const double dy = c(2) + c(4) * x + 2.0 * c(5) * y;
    Eigen::Vector3d normal = surface.n - dx * surface.u - dy * surface.v;
    normal = normalizeOr(normal, surface.n);
    if (normal.dot(surface.n) < 0.0) normal = -normal;
    return normal;
}

/** 【函数导航】
 * 作用：拟合/求解“solveQuadratic”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool solveQuadratic(const std::vector<BiaoMianPoint>& points,
                           const std::vector<unsigned char>& active,
                           Eigen::Matrix<double, 6, 1>& coefficients)
{
    Eigen::Matrix<double, 6, 6> normal =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> rhs =
        Eigen::Matrix<double, 6, 1>::Zero();
    int count = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!active.empty() && !active[i]) continue;
        const auto row = quadraticRow(points[i].x, points[i].y);
        normal.noalias() += row * row.transpose();
        rhs.noalias() += row * points[i].w;
        ++count;
    }
    if (count < 24) return false;
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(normal);
    if (ldlt.info() != Eigen::Success) return false;
    coefficients = ldlt.solve(rhs);
    return coefficients.allFinite();
}

/** 【函数导航】
 * 作用：拟合/求解“fitQuadraticSupportSurface”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline ErCiZhichengSurface fitQuadraticSupportSurface(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& centerSeed,
    const WaiEvidence& outer,
    double lockedRadius)
{
    ErCiZhichengSurface out;
    if (samples.empty() || !centerSeed.allFinite() || !outer.valid
        || !(lockedRadius > 0.5)) return out;

    out.n = normalizeOr(outer.normal, Eigen::Vector3d::UnitZ());
    makeAxes(out.n, out.u, out.v);
    out.origin = centerSeed - out.n * (centerSeed - outer.center).dot(out.n);

    const double radialLow = std::max(1.10 * lockedRadius,
                                      lockedRadius + 0.35);
    const double radialHigh = std::max(2.10 * lockedRadius,
                                       lockedRadius + 3.20);
    const double heightLimit = std::clamp(0.34 * lockedRadius + 0.45,
                                          0.85, 1.80);

    std::vector<BiaoMianPoint> points;
    points.reserve(samples.size());
    std::array<unsigned char, kSectors> annulusSectors{};
    for (const DianYangBen& sample : samples) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - out.origin;
        const double x = delta.dot(out.u);
        const double y = delta.dot(out.v);
        const double w = delta.dot(out.n);
        const double radial = std::hypot(x, y);
        if (radial < radialLow || radial > radialHigh
            || std::abs(w) > heightLimit) continue;
        double angle = std::atan2(y, x);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        annulusSectors[static_cast<std::size_t>(sector)] = 1U;
        points.push_back(BiaoMianPoint{sample.point, x, y, w, radial, angle, 0.0});
    }
    if (points.size() < 120U) return out;

    std::vector<unsigned char> active(points.size(), 1U);
    Eigen::Matrix<double, 6, 1> coefficients =
        Eigen::Matrix<double, 6, 1>::Zero();
    double residualMedian = 0.0;
    double residualMad = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < 5; ++iteration) {
        if (!solveQuadratic(points, active, coefficients)) return out;
        std::vector<double> residuals;
        residuals.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!active[i]) continue;
            residuals.push_back(points[i].w
                - quadraticRow(points[i].x, points[i].y).dot(coefficients));
        }
        if (residuals.size() < 80U) return out;
        residualMedian = median(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (double residual : residuals)
            deviations.push_back(std::abs(residual - residualMedian));
        residualMad = 1.4826 * median(deviations);
        const double limit = std::clamp(3.0 * residualMad + 0.02,
                                        0.07, 0.25);
        int kept = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double residual = points[i].w
                - quadraticRow(points[i].x, points[i].y).dot(coefficients);
            points[i].surfaceResidual = residual - residualMedian;
            active[i] = std::abs(points[i].surfaceResidual) <= limit ? 1U : 0U;
            kept += active[i] ? 1 : 0;
        }
        if (kept < 80) return out;
    }
    if (!solveQuadratic(points, active, coefficients)) return out;

    double squaredError = 0.0;
    int support = 0;
    std::array<unsigned char, kSectors> supportSectors{};
    std::vector<double> finalResiduals;
    finalResiduals.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!active[i]) continue;
        const double residual = points[i].w
            - quadraticRow(points[i].x, points[i].y).dot(coefficients);
        finalResiduals.push_back(residual);
        squaredError += residual * residual;
        const int sector = std::clamp(static_cast<int>(std::floor(
            points[i].angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        supportSectors[static_cast<std::size_t>(sector)] = 1U;
        ++support;
    }
    residualMedian = median(finalResiduals);
    std::vector<double> deviations;
    deviations.reserve(finalResiduals.size());
    for (double residual : finalResiduals)
        deviations.push_back(std::abs(residual - residualMedian));
    residualMad = 1.4826 * median(deviations);
    const double rmse = support > 0
        ? std::sqrt(squaredError / static_cast<double>(support))
        : std::numeric_limits<double>::infinity();
    int covered = 0;
    for (unsigned char value : supportSectors) covered += value ? 1 : 0;
    const double coverage = static_cast<double>(covered)
        / static_cast<double>(kSectors);

    out.coefficients = coefficients;
    out.residualMedian = residualMedian;
    out.residualMad = residualMad;
    out.rmse = rmse;
    out.surfaceTolerance = std::clamp(3.0 * residualMad + 0.03,
                                      0.08, 0.22);
    out.supportCount = support;
    out.coveredSectors = covered;
    out.coverage = coverage;
    out.valid = support >= 120 && covered >= 42 && coverage >= 0.58
        && rmse <= 0.12 && residualMad <= 0.08;
    return out;
}

/** 【函数导航】
 * 作用：执行“recoverStraightSurfaceRim”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline ZhiHoleBiaoMianBian recoverStraightSurfaceRim(
    const std::vector<DianYangBen>& samples,
    const Eigen::Vector3d& centerSeed,
    const WaiEvidence& outer,
    double lockedRadius,
    double measuredInnerRadius)
{
    ZhiHoleBiaoMianBian out;
    out.executed = true;
    const ErCiZhichengSurface surface = fitQuadraticSupportSurface(
        samples, centerSeed, outer, lockedRadius);
    if (!surface.valid) {
        out.decision = "REJECT_QUADRATIC_SUPPORT_SURFACE";
        return out;
    }

    const double radialLow = std::max(0.70 * lockedRadius,
        measuredInnerRadius > 0.5 ? measuredInnerRadius - 0.12
                                  : 0.70 * lockedRadius);
    const double radialHigh = std::max(1.55 * lockedRadius,
                                       lockedRadius + 2.20);
    std::array<int, kSectors> selectedIndex{};
    selectedIndex.fill(-1);
    std::array<double, kSectors> selectedRadius{};
    selectedRadius.fill(std::numeric_limits<double>::infinity());
    std::vector<BiaoMianPoint> projected;
    projected.reserve(samples.size());

    for (const DianYangBen& sample : samples) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - surface.origin;
        const double x = delta.dot(surface.u);
        const double y = delta.dot(surface.v);
        const double w = delta.dot(surface.n);
        const double radial = std::hypot(x, y);
        if (radial < radialLow || radial > radialHigh) continue;
        const double residual = w - evaluateSurface(surface, x, y)
            - surface.residualMedian;
        if (std::abs(residual) > surface.surfaceTolerance) continue;
        double angle = std::atan2(y, x);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        const int index = static_cast<int>(projected.size());
        projected.push_back(BiaoMianPoint{
            sample.point, x, y, w, radial, angle, residual});
        if (radial < selectedRadius[static_cast<std::size_t>(sector)]) {
            selectedRadius[static_cast<std::size_t>(sector)] = radial;
            selectedIndex[static_cast<std::size_t>(sector)] = index;
        }
    }

    std::vector<Eigen::Vector2d> boundaryPoints;
    boundaryPoints.reserve(kSectors);
    for (int sector = 0; sector < kSectors; ++sector) {
        const int index = selectedIndex[static_cast<std::size_t>(sector)];
        if (index < 0) continue;
        const BiaoMianPoint& point = projected[static_cast<std::size_t>(index)];
        boundaryPoints.emplace_back(point.x, point.y);
    }
    out.boundaryCandidates = static_cast<int>(boundaryPoints.size());
    if (boundaryPoints.size() < 36U) {
        out.decision = "REJECT_TOO_FEW_SURFACE_BOUNDARY_SECTORS";
        return out;
    }

    const QiePianNiHe circle = fitCircleRobust(
        boundaryPoints, 0.0, 0.0, radialHigh + 0.50);
    if (!circle.valid) {
        out.decision = "REJECT_SURFACE_BOUNDARY_CIRCLE";
        return out;
    }
    const double centerShift = circle.center.norm();
    const double throatGap = circle.radius - measuredInnerRadius;
    const double minimumGap = std::max(0.18, 0.045 * lockedRadius);
    if (circle.coverage < 0.50 || circle.supportCount < 30
        || circle.rmse > 0.24
        || circle.radius < 0.78 * lockedRadius
        || circle.radius > std::min(radialHigh, 1.55 * lockedRadius)
        || centerShift > std::max(0.60, 0.24 * lockedRadius)
        || !(throatGap >= minimumGap)) {
        out.decision = "REJECT_SURFACE_RIM_QUALITY_OR_THROAT_SEPARATION";
        return out;
    }

    const double centerW = evaluateSurface(surface,
        circle.center.x(), circle.center.y());
    out.center = surface.origin + surface.u * circle.center.x()
        + surface.v * circle.center.y() + surface.n * centerW;
    out.surfaceNormal = evaluateSurfaceNormal(surface,
        circle.center.x(), circle.center.y());
    out.radius = circle.radius;
    out.fitRmse = circle.rmse;
    out.centerShift = centerShift;
    out.throatGap = throatGap;

    std::array<int, kSectors> evidenceIndex{};
    evidenceIndex.fill(-1);
    std::array<double, kSectors> evidenceError{};
    evidenceError.fill(std::numeric_limits<double>::infinity());
    std::array<int, kSectors> evidenceSupport{};
    const double evidenceBand = std::clamp(
        std::max(0.18, 2.8 * circle.rmse), 0.18, 0.38);
    std::vector<double> observedRadii;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        const double dx = projected[i].x - circle.center.x();
        const double dy = projected[i].y - circle.center.y();
        const double radial = std::hypot(dx, dy);
        double angle = std::atan2(dy, dx);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSectors))),
            0, kSectors - 1);
        const double error = std::abs(radial - circle.radius);
        if (error <= evidenceBand) {
            ++evidenceSupport[static_cast<std::size_t>(sector)];
            if (error < evidenceError[static_cast<std::size_t>(sector)]) {
                evidenceError[static_cast<std::size_t>(sector)] = error;
                evidenceIndex[static_cast<std::size_t>(sector)] =
                    static_cast<int>(i);
            }
        }
    }

    HoleKouContour contour;
    contour.executed = true;
    std::vector<double> depths;
    depths.reserve(kSectors);
    int observed = 0;
    int totalSupport = 0;
    double maximumPlaneResidual = 0.0;
    double maximumEvidenceResidual = 0.0;
    for (int sector = 0; sector < kSectors; ++sector) {
        const double angle = (static_cast<double>(sector) + 0.5)
            * 2.0 * kPi / static_cast<double>(kSectors);
        const double x = circle.center.x() + circle.radius * std::cos(angle);
        const double y = circle.center.y() + circle.radius * std::sin(angle);
        const double w = evaluateSurface(surface, x, y);
        HoleKouContourPoint point;
        point.valid = true;
        point.angle = angle;
        point.world = surface.origin + surface.u * x
            + surface.v * y + surface.n * w;
        point.axialDepth = (point.world - out.center).dot(-out.surfaceNormal);
        point.localRadius = circle.radius;
        point.planeResidual = (point.world - out.center).dot(out.surfaceNormal);
        point.supportCount = evidenceSupport[static_cast<std::size_t>(sector)];
        const int evidence = evidenceIndex[static_cast<std::size_t>(sector)];
        if (evidence >= 0) {
            const BiaoMianPoint& observedPoint =
                projected[static_cast<std::size_t>(evidence)];
            const double dx = observedPoint.x - circle.center.x();
            const double dy = observedPoint.y - circle.center.y();
            const double observedRadius = std::hypot(dx, dy);
            point.evidenceUsed = true;
            point.evidenceResidual = observedRadius - circle.radius;
            observedRadii.push_back(observedRadius);
            ++observed;
            totalSupport += point.supportCount;
            maximumEvidenceResidual = std::max(maximumEvidenceResidual,
                std::abs(point.evidenceResidual));
        }
        maximumPlaneResidual = std::max(maximumPlaneResidual,
            std::abs(point.planeResidual));
        contour.points[static_cast<std::size_t>(sector)] = point;
        depths.push_back(point.axialDepth);
    }
    if (observed < 36) {
        out.decision = "REJECT_TOO_FEW_EVIDENCE_BACKED_RIM_SECTORS";
        return out;
    }

    const double observedMedian = median(observedRadii);
    std::vector<double> radiusDeviations;
    radiusDeviations.reserve(observedRadii.size());
    for (double radius : observedRadii)
        radiusDeviations.push_back(std::abs(radius - observedMedian));
    out.radiusMad = 1.4826 * median(radiusDeviations);
    if (out.radiusMad > std::max(0.24, 0.075 * lockedRadius)
        || maximumEvidenceResidual > 0.42) {
        out.decision = "REJECT_UNSTABLE_SURFACE_RIM_EVIDENCE";
        return out;
    }

    contour.valid = true;
    contour.validPoints = kSectors;
    contour.coverage = 1.0;
    contour.minimumAxialDepth = *std::min_element(depths.begin(), depths.end());
    contour.maximumAxialDepth = *std::max_element(depths.begin(), depths.end());
    contour.medianAxialDepth = median(depths);
    contour.axialSpan = contour.maximumAxialDepth - contour.minimumAxialDepth;
    contour.minimumLocalRadius = circle.radius;
    contour.maximumLocalRadius = circle.radius;
    contour.planarRingMaximumPlaneResidual = maximumPlaneResidual;
    contour.maximumPlaneResidual = maximumPlaneResidual;
    contour.canonicalRadius = lockedRadius;
    contour.evidenceRadius = circle.radius;
    contour.evidenceScale = circle.rmse;
    contour.evidenceCenterShift = centerShift;
    contour.evidenceRawSectors = out.boundaryCandidates;
    contour.evidenceUsedSectors = observed;
    contour.evidenceSupportCount = totalSupport;
    contour.confidence = std::clamp(
        0.35 * surface.coverage
        + 0.30 * static_cast<double>(observed) / static_cast<double>(kSectors)
        + 0.20 * std::clamp((0.24 - circle.rmse) / 0.20, 0.0, 1.0)
        + 0.15 * std::clamp((0.24 - out.radiusMad) / 0.20, 0.0, 1.0),
        0.0, 1.0);
    contour.source = "SurfaceAxisModel_QUADRATIC_SUPPORT_SURFACE_UPPER_RIM";
    contour.decision = "APPLY_VISIBILITY_AWARE_STRAIGHT_UPPER_RIM";

    out.valid = true;
    out.observedSectors = observed;
    out.supportCount = totalSupport;
    out.contour = contour;
    out.decision = "APPLY_VISIBILITY_AWARE_STRAIGHT_UPPER_RIM";
    return out;
}

/** 【函数导航】
 * 作用：执行“medianCoveredSectors”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline int medianCoveredSectors(const HoleWeiziCu::NeiYuanZhuZu& family) {
    if (family.slices.empty()) return 0;
    std::vector<double> sectors;
    sectors.reserve(family.slices.size());
    for (const QiePianNiHe& slice : family.slices)
        sectors.push_back(static_cast<double>(slice.coveredSectors));
    return static_cast<int>(std::lround(median(sectors)));
}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimate(const std::vector<DianYangBen>& samples,
                       const Eigen::Vector3d& mouthCenter,
                       Eigen::Vector3d recognitionSurface,
                       double topRadius,
                       double bottomRadius,
                       double depth,
                       int inwardPolarity,
                       int holeType)
{
    Result out = HoleWeiziCu::estimate(
        samples, mouthCenter, recognitionSurface, topRadius, bottomRadius,
        depth, inwardPolarity, holeType);
    if (holeType != 1 || inwardPolarity == 0 || !out.valid
        || !out.outer.valid || !out.innerCylinder.valid
        || !out.measuredInnerRadiusValid) {        return out;
    }

    const int medianSectors = medianCoveredSectors(out.innerCylinder);

    const bool wallEvidenceStrong = out.innerCylinder.usedSlices >= 6
        && out.innerCylinder.meanCoverage >= 0.55
        && medianSectors >= 36;
    if (wallEvidenceStrong) {        return out;
    }

    const ZhiHoleBiaoMianBian rim = recoverStraightSurfaceRim(
        samples, out.centerTop, out.outer, topRadius, out.measuredInnerRadius);    if (!rim.valid) {        return out;
    }

    recognitionSurface = normalizeOr(recognitionSurface,
        Eigen::Vector3d::UnitZ());
    Eigen::Vector3d initialAxis = normalizeOr(
        recognitionSurface * static_cast<double>(inwardPolarity),
        -recognitionSurface);
    Eigen::Vector3d surfaceNormal = normalizeOr(rim.surfaceNormal,
        out.outer.normal);
    if (surfaceNormal.dot(recognitionSurface) < 0.0)
        surfaceNormal = -surfaceNormal;
    Eigen::Vector3d surfaceAxis = surfaceNormal
        * static_cast<double>(inwardPolarity);
    surfaceAxis = normalizeOr(surfaceAxis, initialAxis);
    if (surfaceAxis.dot(initialAxis) < 0.0) surfaceAxis = -surfaceAxis;

    const bool wallEvidenceClearlyWeak = out.innerCylinder.usedSlices <= 2
        || out.innerCylinder.meanCoverage < 0.30
        || medianSectors < 18;
    if (!wallEvidenceClearlyWeak) {
        const double familyAngleDeg = angleDegrees(out.axisIn, surfaceAxis);
        if (familyAngleDeg <= 1.5) {            return out;
        }
    }

    out.centerTop = rim.center;
    out.axisIn = surfaceAxis;
    out.surfaceNormal = surfaceNormal;
    out.contour = rim.contour;
    out.centerShift = (rim.center - mouthCenter).norm();
    out.axisCorrectionDegrees = angleDegrees(initialAxis, surfaceAxis);
    out.lockedMouthDepth = 0.0;
    out.centerEvidenceWeight = static_cast<double>(rim.observedSectors)
        / static_cast<double>(kSectors);
    out.wallRadiusWeight = 1.0;
    out.confidence = std::clamp(
        0.55 * rim.contour.confidence
        + 0.25 * out.outer.confidence
        + 0.20 * std::clamp((0.20 - out.innerCylinder.medianRmse) / 0.16,
                            0.0, 1.0),
        0.0, 1.0);
    out.outer.center = rim.center;
    out.outer.normal = surfaceNormal;
    out.outer.source = "SurfaceAxisModel_QUADRATIC_SUPPORT_SURFACE";
    out.source = "SurfaceAxisModel_STRAIGHT_VISIBILITY_AWARE_UPPER_RIM_WEAK_WALL_AXIS";    return out;
}

}

// ============================================================================
// 功能分区：连续锥孔轴线拟合
// ============================================================================
/*
模块职责：
锥孔连续性辅助证据模块。

主要调用位置：
由识别/研究链采集锥壁连续性信息，主要用于验证候选方向。

维护说明：
辅助证据默认不应直接改变生产判定，正式接入前必须单独通过等价性门。
*/
//

//
#include <Eigen/Dense>

#include <string>

namespace LianXuZhuiWeizi {

/** 【类型导航注释】
 * Result：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool executed = false;
    bool valid = false;
    Eigen::Vector3d axis{0.0, 0.0, 1.0};
    Eigen::Vector3d centerAtLockedRadius{0.0, 0.0, 0.0};
    double axisCorrectionDeg = 0.0;
    double r0 = 0.0;
    double k = 0.0;
    double objective = 0.0;
    double rmse = 0.0;
    double supportWeightSum = 0.0;
    int iterations = 0;
    std::string reason = "NOT_EXECUTED";
};

namespace detail {

inline double sigmoid(double x) {
    if (x >= 0.0) {
        const double ex = std::exp(-x);
        return 1.0 / (1.0 + ex);
    }
    const double ex = std::exp(x);
    return ex / (1.0 + ex);
}

/** 【类型导航注释】
 * Params：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Params {
    double cu = 0.0;
    double cv = 0.0;
    double tu = 0.0;
    double tv = 0.0;
    double r0 = 0.0;
    double k = 0.0;
};

/** 【类型导航注释】
 * DongJieZhuangTai：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct DongJieZhuangTai {
    Eigen::Vector3d mouthCenter{0.0, 0.0, 0.0};
    Eigen::Vector3d a0{0.0, 0.0, 1.0};
    Eigen::Vector3d u{1.0, 0.0, 0.0};
    Eigen::Vector3d v{0.0, 1.0, 0.0};
    double topRadius = 0.0;
    double bottomRadius = 0.0;
    double depth = 0.0;
    double k0 = -0.5;
    double r0Lo = 0.0;
    double r0Hi = 0.0;
    std::vector<double> w0;
};

/** 【函数导航】
 * 作用：执行“residual”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double residual(const Eigen::Vector3d& p, const Params& x,
                       const DongJieZhuangTai& f) {
    const Eigen::Vector3d c =
        f.mouthCenter + x.cu * f.u + x.cv * f.v;
    const Eigen::Vector3d a =
        (f.a0 + x.tu * f.u + x.tv * f.v).normalized();
    const Eigen::Vector3d q = p - c;
    const double s = q.dot(a);
    const double rho = (q - a * s).norm();
    return rho - (x.r0 + x.k * s);
}

/** 【函数导航】
 * 作用：执行“clampParams”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Params clampParams(const Params& x, const DongJieZhuangTai& f) {
    Params out = x;
    out.cu = std::clamp(out.cu, -1.0, 1.0);
    out.cv = std::clamp(out.cv, -1.0, 1.0);
    out.tu = std::clamp(out.tu, -0.15, 0.15);
    out.tv = std::clamp(out.tv, -0.15, 0.15);
    out.r0 = std::clamp(out.r0, f.r0Lo, f.r0Hi);
    out.k = std::clamp(out.k, -4.5, -0.05);
    return out;
}

}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimate(const std::vector<HoleFaXian::DianYangBen>& samples,
                       const Eigen::Vector3d& mouthCenter,
                       const Eigen::Vector3d& recognitionSurface,
                       double topRadius,
                       double bottomRadius,
                       double depth,
                       int inwardPolarity,
                       int holeType)
{
    Result out;
    if (holeType != 2 || inwardPolarity == 0) {
        out.executed = false;
        out.reason = "NOT_EXECUTED";
        return out;
    }
    out.executed = true;
    if (samples.empty() || !mouthCenter.allFinite()
        || !recognitionSurface.allFinite()
        || recognitionSurface.squaredNorm() < 1e-12 || !(topRadius > 0.5)
        || !(depth > 0.25) || !std::isfinite(bottomRadius)) {
        out.reason = "INVALID_INPUT";
        return out;
    }

    detail::DongJieZhuangTai f;
    f.mouthCenter = mouthCenter;
    const Eigen::Vector3d n = recognitionSurface.normalized();
    f.a0 = (n * static_cast<double>(inwardPolarity)).normalized();
    HoleWeiziBase::makeAxes(f.a0, f.u, f.v);
    f.topRadius = topRadius;
    f.bottomRadius = bottomRadius;
    f.depth = depth;
    f.k0 = (bottomRadius - topRadius) / depth;
    f.r0Lo = topRadius - 1.5;
    f.r0Hi = topRadius + 1.5;

    const double low = 0.28;
    const double high = std::min(0.80 * depth, depth - 0.25);
    const double logisticWidth = 0.09;
    const double radialSigma = 0.75;
    const double cauchyScale = 0.12;

    f.w0.resize(samples.size());
    double weightSum = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Eigen::Vector3d q = samples[i].point - mouthCenter;
        const double s0 = q.dot(f.a0);
        const double rho0 = (q - f.a0 * s0).norm();
        const double rExpected = topRadius + f.k0 * s0;
        const double wDepth =
            detail::sigmoid((s0 - low) / logisticWidth)
            * detail::sigmoid((high - s0) / logisticWidth);
        const double wRadial =
            std::exp(-0.5 * ((rho0 - rExpected) / radialSigma)
                     * ((rho0 - rExpected) / radialSigma));
        f.w0[i] = wDepth * wRadial;
        weightSum += f.w0[i];
    }
    out.supportWeightSum = weightSum;
    if (!(weightSum > 1e-9)) {
        out.reason = "NO_SUPPORT_WEIGHT";        return out;
    }

    detail::Params x;
    x.cu = 0.0;
    x.cv = 0.0;
    x.tu = 0.0;
    x.tv = 0.0;
    x.r0 = topRadius;
    x.k = std::clamp(f.k0, -4.5, -0.05);

    auto evaluate = [&](const detail::Params& p) {
        const Eigen::Vector3d candidateCenter =
            f.mouthCenter + p.cu * f.u + p.cv * f.v;
        const Eigen::Vector3d candidateAxis =
            (f.a0 + p.tu * f.u + p.tv * f.v).normalized();
        double candidateObjective = 0.0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const Eigen::Vector3d q = samples[i].point - candidateCenter;
            const double s = q.dot(candidateAxis);
            const double rho = (q - candidateAxis * s).norm();
            const double ei = rho - (p.r0 + p.k * s);
            const double scaled = ei / cauchyScale;
            candidateObjective += f.w0[i] * 0.5 * cauchyScale * cauchyScale
                * std::log1p(scaled * scaled);
        }
        return candidateObjective;
    };

    double objective = evaluate(x);
    double damping = 1e-6;
    const double stepH = 1e-5;
    out.iterations = 0;
    bool converged = false;
    for (int iteration = 0; iteration < 60 && !converged; ++iteration) {

        const Eigen::Vector3d c =
            f.mouthCenter + x.cu * f.u + x.cv * f.v;
        const Eigen::Vector3d b =
            f.a0 + x.tu * f.u + x.tv * f.v;
        const double bNorm = b.norm();
        if (!(bNorm > 1e-12) || !std::isfinite(bNorm)) break;
        const Eigen::Vector3d a = b / bNorm;
        const double aDotU = a.dot(f.u);
        const double aDotV = a.dot(f.v);
        const Eigen::Vector3d dAxisDTu =
            (f.u - a * aDotU) / bNorm;
        const Eigen::Vector3d dAxisDTv =
            (f.v - a * aDotV) / bNorm;

        Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const Eigen::Vector3d q = samples[i].point - c;
            const double s = q.dot(a);
            const Eigen::Vector3d radial = q - a * s;
            const double rho = radial.norm();
            const double ei = rho - (x.r0 + x.k * s);
            const double scaled = ei / cauchyScale;
            const double robustWeight = 1.0 / (1.0 + scaled * scaled);

            Eigen::Matrix<double, 6, 1> jac;
            if (rho > 1e-12 && std::isfinite(rho)) {
                jac(0) = -radial.dot(f.u) / rho + x.k * aDotU;
                jac(1) = -radial.dot(f.v) / rho + x.k * aDotV;
                jac(2) = -(s / rho + x.k) * q.dot(dAxisDTu);
                jac(3) = -(s / rho + x.k) * q.dot(dAxisDTv);
            } else {

                detail::Params plus = x;
                detail::Params minus = x;
                plus.cu += stepH; minus.cu -= stepH;
                jac(0) = (detail::residual(samples[i].point, plus, f)
                          - detail::residual(samples[i].point, minus, f))
                    / (2.0 * stepH);
                plus = x; minus = x;
                plus.cv += stepH; minus.cv -= stepH;
                jac(1) = (detail::residual(samples[i].point, plus, f)
                          - detail::residual(samples[i].point, minus, f))
                    / (2.0 * stepH);
                plus = x; minus = x;
                plus.tu += stepH; minus.tu -= stepH;
                jac(2) = (detail::residual(samples[i].point, plus, f)
                          - detail::residual(samples[i].point, minus, f))
                    / (2.0 * stepH);
                plus = x; minus = x;
                plus.tv += stepH; minus.tv -= stepH;
                jac(3) = (detail::residual(samples[i].point, plus, f)
                          - detail::residual(samples[i].point, minus, f))
                    / (2.0 * stepH);
            }
            jac(4) = -1.0;
            jac(5) = -s;
            const double w = f.w0[i] * robustWeight;
            gradient += w * ei * jac;
            hessian += w * (jac * jac.transpose());
        }

        Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
        bool solved = false;
        for (int inner = 0; inner < 8 && !solved; ++inner) {
            Eigen::Matrix<double, 6, 6> system = hessian;
            for (int j = 0; j < 6; ++j) {
                system(j, j) += damping * std::max(1e-12, std::abs(hessian(j, j)));
            }
            Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu(system);
            if (lu.isInvertible()) {
                delta = lu.solve(-gradient);
                solved = true;
            } else {
                damping = std::min(damping * 10.0, 1e9);
            }
        }
        if (!solved) break;

        const detail::Params candidate = detail::clampParams(
            detail::Params{x.cu + delta(0), x.cv + delta(1), x.tu + delta(2),
                           x.tv + delta(3), x.r0 + delta(4), x.k + delta(5)},
            f);
        const double candidateObjective = evaluate(candidate);
        const bool accepted = candidateObjective < objective;
        if (accepted) {
            x = candidate;
            const double change = objective - candidateObjective;
            objective = candidateObjective;
            damping = std::max(damping / 3.0, 1e-9);
            const double stepNorm = delta.cwiseAbs().maxCoeff();
            if (stepNorm < 1e-7 || change < 1e-10 * (1.0 + std::abs(objective))) {
                converged = true;
            }
        } else {
            damping = std::min(damping * 10.0, 1e9);
        }
        out.iterations = iteration + 1;
        if (damping >= 1e9 && !accepted) break;
    }

    const Eigen::Vector3d anchor =
        f.mouthCenter + x.cu * f.u + x.cv * f.v;
    const Eigen::Vector3d axis =
        (f.a0 + x.tu * f.u + x.tv * f.v).normalized();
    const double dot = std::clamp(axis.dot(f.a0), -1.0, 1.0);
    const double correctionDeg =
        std::acos(dot) * 180.0 / 3.14159265358979323846;

    out.axis = axis;
    out.axisCorrectionDeg = correctionDeg;
    out.r0 = x.r0;
    out.k = x.k;
    out.objective = objective;
    double weightedSquared = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Eigen::Vector3d q = samples[i].point - anchor;
        const double s = q.dot(axis);
        const double rho = (q - axis * s).norm();
        const double ei = rho - (x.r0 + x.k * s);
        weightedSquared += f.w0[i] * ei * ei;
    }
    out.rmse = std::sqrt(weightedSquared / weightSum);
    out.centerAtLockedRadius = anchor
        + axis * ((topRadius - x.r0) / x.k);
    out.valid = axis.allFinite() && out.centerAtLockedRadius.allFinite()
        && std::isfinite(out.r0) && std::isfinite(out.k)
        && std::isfinite(out.rmse);
    out.reason = out.valid ? "CONTINUOUS_CONE_AXIS" : "NON_FINITE_RESULT";    return out;
}

}

// ============================================================================
// 功能分区：孔口法向平衡保护
// ============================================================================
/*
模块职责：
孔口法向/支撑坐标辅助模块。

主要调用位置：
由孔姿态估计链调用，在局部表面上构造更稳健的参考方向。

维护说明：
邻域、扇区和残差参数会影响斜孔法向；普通整理只允许注释和结构等价改动。
*/
//


namespace holeNormalPingHeng {

/** 【类型导航注释】
 * Vec3：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x*s,a.y*s,a.z*s}; }
inline Vec3 operator/(const Vec3& a, double s) { return {a.x/s,a.y/s,a.z/s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline double norm2(const Vec3& a) { return dot(a,a); }
inline double norm(const Vec3& a) { return std::sqrt(norm2(a)); }
/** 【函数导航】
 * 作用：执行“normalized”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp、ZhuChuangKou_Window.cpp、DianYunXianshi_View.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Vec3 normalized(const Vec3& a) {
    const double n = norm(a);
    return n > 1e-12 ? a/n : Vec3{0,0,1};
}
inline double clampd(double x, double a, double b) { return std::max(a,std::min(b,x)); }
/** 【函数导航】
 * 作用：执行“angleDeg”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double angleDeg(const Vec3& a, const Vec3& b) {
    return std::acos(clampd(dot(normalized(a),normalized(b)),-1.0,1.0))*180.0/3.14159265358979323846;
}

inline double median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    const std::size_t n=v.size(), k=n/2;
    std::nth_element(v.begin(),v.begin()+k,v.end());
    const double hi=v[k];
    if (n&1) return hi;
    std::nth_element(v.begin(),v.begin()+k-1,v.begin()+k);
    return 0.5*(v[k-1]+hi);
}
inline double mad(const std::vector<double>& v, double med) {
    std::vector<double> d; d.reserve(v.size());
    for(double x:v) d.push_back(std::abs(x-med));
    return median(std::move(d));
}

/** 【函数导航】
 * 作用：执行“smallestEigenvector”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Vec3 smallestEigenvector(std::array<std::array<double,3>,3> a) {
    std::array<std::array<double,3>,3> v{{{{1,0,0}},{{0,1,0}},{{0,0,1}}}};
    for(int it=0; it<32; ++it) {
        int p=0,q=1;
        double m=std::abs(a[0][1]);
        if(std::abs(a[0][2])>m){p=0;q=2;m=std::abs(a[0][2]);}
        if(std::abs(a[1][2])>m){p=1;q=2;m=std::abs(a[1][2]);}
        if(m<1e-12) break;
        const double app=a[p][p], aqq=a[q][q], apq=a[p][q];
        const double phi=0.5*std::atan2(2.0*apq, aqq-app);
        const double c=std::cos(phi), s=std::sin(phi);
        for(int k=0;k<3;++k){
            const double aik=a[k][p], aiq=a[k][q];
            a[k][p]=c*aik-s*aiq; a[k][q]=s*aik+c*aiq;
        }
        for(int k=0;k<3;++k){
            const double apk=a[p][k], aqk=a[q][k];
            a[p][k]=c*apk-s*aqk; a[q][k]=s*apk+c*aqk;
        }
        a[p][q]=a[q][p]=0.0;
        for(int k=0;k<3;++k){
            const double vip=v[k][p], viq=v[k][q];
            v[k][p]=c*vip-s*viq; v[k][q]=s*vip+c*viq;
        }
    }
    int idx=0;
    if(a[1][1]<a[idx][idx]) idx=1;
    if(a[2][2]<a[idx][idx]) idx=2;
    return normalized({v[0][idx],v[1][idx],v[2][idx]});
}

/** 【函数导航】
 * 作用：执行“weightedPlaneNormal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Vec3 weightedPlaneNormal(const std::vector<Vec3>& pts, const std::vector<double>& w) {
    double sw=0; Vec3 c{};
    for(std::size_t i=0;i<pts.size();++i){ sw+=w[i]; c=c+pts[i]*w[i]; }
    if(sw<=1e-12) return {0,0,1};
    c=c/sw;
    std::array<std::array<double,3>,3> C{};
    for(std::size_t i=0;i<pts.size();++i){
        const Vec3 d=pts[i]-c; const double wi=w[i];
        C[0][0]+=wi*d.x*d.x; C[0][1]+=wi*d.x*d.y; C[0][2]+=wi*d.x*d.z;
        C[1][1]+=wi*d.y*d.y; C[1][2]+=wi*d.y*d.z; C[2][2]+=wi*d.z*d.z;
    }
    C[1][0]=C[0][1]; C[2][0]=C[0][2]; C[2][1]=C[1][2];
    return smallestEigenvector(C);
}

// 孔口外环法向保护参数。半径尺度均相对 rTop，距离单位为毫米，角度单位为度。
/** 【类型导航注释】
 * Config：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Config {
    int sectors = 24;                     // 外环按方位角分成的扇区数；增大角向分辨率，也提高每扇区缺点概率。
    double innerScale = 1.15;             // 外支撑环内边界 = rTop×该比例；减小可能混入孔口/孔壁点。
    double outerScale = 2.60;             // 外支撑环外边界 = rTop×该比例；增大可取更多面点，也更容易跨到邻近结构。
    int minPointsPerSector = 3;           // 每个有效扇区最少点数；提高更稳健但会降低稀疏云覆盖率。
    double prefilterSigma = 3.0;          // 全环深度预筛的 MAD 倍数；降低会更强地剔除离群高度。
    double minPrefilterBandMm = 0.18;     // 全环预筛最小深度带宽，单位毫米；防止低噪声区域门限收缩过度。
    double sectorSigma = 2.5;             // 单扇区深度筛选 MAD 倍数；降低会收紧每扇区平面一致性。
    double minSectorBandMm = 0.10;        // 单扇区最小深度带宽，单位毫米。
    double minCoverage = 0.55;            // 有效扇区覆盖比例下限；提高更可靠，但缺边/遮挡孔更易失败。
    double maxGapDeg = 120.0;             // 允许的最大连续方位缺口，单位度；降低会要求更完整的外表面。
    double maxNormalDeltaDeg = 6.0;       // 输出法向相对输入法向允许的最大修正，单位度；增大可能过度改姿态。
    double huberK = 1.5;                  // IRLS Huber 阈值倍数；降低更抑制异常扇区。
    int irlsIterations = 4;               // 法向稳健拟合最大迭代次数；增加主要增加耗时。
};

/** 【类型导航注释】
 * Result：Hole 位姿计算中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    Vec3 normal{0,0,1};
    int usedSectors = 0;
    int totalSectors = 0;
    double coverage = 0.0;
    double maxGapDeg = 360.0;
    double residualMedian = 0.0;
    double residualMad = 0.0;
    double deltaFromInputDeg = 0.0;
    int annulusPoints = 0;
    int prefilteredPoints = 0;
    const char* decision = "INVALID";
};

inline Result estimateBalancedOuterNormal(
    const std::vector<Vec3>& cloud,
    const Vec3& centerTop,
    double rTop,
    Vec3 initialNormal,
    const Config& cfg = Config{})
{
    Result out; out.totalSectors=cfg.sectors;
    initialNormal=normalized(initialNormal);
    if(rTop<=0 || cfg.sectors<8) { out.decision="BAD_INPUT"; return out; }

    Vec3 helper = std::abs(initialNormal.z)<0.85 ? Vec3{0,0,1} : Vec3{1,0,0};
    Vec3 u=normalized(cross(helper,initialNormal));
    Vec3 v=normalized(cross(initialNormal,u));

    /** 【类型导航注释】
     * DianYangBen：Hole 位姿计算中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct DianYangBen{Vec3 p; double x,y,d,ang;};
    std::vector<DianYangBen> ann;
    std::vector<double> depths;
    ann.reserve(cloud.size()/8+1); depths.reserve(ann.capacity());
    const double r0=cfg.innerScale*rTop, r1=cfg.outerScale*rTop;
    for(const Vec3& p:cloud){
        const Vec3 q=p-centerTop;
        const double x=dot(q,u), y=dot(q,v), rr=std::hypot(x,y);
        if(rr<r0 || rr>r1) continue;
        const double d=dot(q,initialNormal);
        if(!std::isfinite(d)) continue;
        double a=std::atan2(y,x); if(a<0) a+=2.0*3.14159265358979323846;
        ann.push_back({p,x,y,d,a}); depths.push_back(d);
    }
    out.annulusPoints=(int)ann.size();
    if((int)ann.size()<cfg.sectors*cfg.minPointsPerSector){out.decision="ANNULUS_TOO_SPARSE";return out;}

    const double gmed=median(depths);
    const double gmad=mad(depths,gmed);
    const double gscale=std::max(cfg.minPrefilterBandMm, cfg.prefilterSigma*1.4826*std::max(gmad,1e-6));

    std::vector<std::vector<DianYangBen>> bins(cfg.sectors);
    for(const auto& s:ann){
        if(std::abs(s.d-gmed)>gscale) continue;
        int k=(int)std::floor(s.ang/(2.0*3.14159265358979323846)*cfg.sectors);
        k=std::max(0,std::min(cfg.sectors-1,k));
        bins[k].push_back(s); ++out.prefilteredPoints;
    }

    std::vector<Vec3> reps;
    std::vector<int> repSector;
    for(int k=0;k<cfg.sectors;++k){
        auto& b=bins[k];
        if((int)b.size()<cfg.minPointsPerSector) continue;
        std::vector<double> ds; ds.reserve(b.size());
        for(auto& s:b) ds.push_back(s.d);
        const double smed=median(ds), smad=mad(ds,smed);
        const double sb=std::max(cfg.minSectorBandMm,cfg.sectorSigma*1.4826*std::max(smad,1e-6));
        Vec3 c{}; int n=0;
        for(auto& s:b){ if(std::abs(s.d-smed)<=sb){c=c+s.p;++n;} }
        if(n<cfg.minPointsPerSector) continue;
        reps.push_back(c/(double)n); repSector.push_back(k);
    }
    out.usedSectors=(int)reps.size();
    out.coverage=(double)out.usedSectors/cfg.sectors;
    if(out.usedSectors<3 || out.coverage<cfg.minCoverage){out.decision="SECTOR_COVERAGE_WEAK";return out;}

    std::sort(repSector.begin(),repSector.end());
    int maxGap=0;
    for(std::size_t i=0;i<repSector.size();++i){
        const int a=repSector[i];
        const int b=repSector[(i+1)%repSector.size()] + ((i+1)==repSector.size()?cfg.sectors:0);
        maxGap=std::max(maxGap,b-a-1);
    }
    out.maxGapDeg=(maxGap+1)*(360.0/cfg.sectors);
    if(out.maxGapDeg>cfg.maxGapDeg){out.decision="AZIMUTH_GAP_TOO_LARGE";return out;}

    std::vector<double> w(reps.size(),1.0);
    Vec3 n=weightedPlaneNormal(reps,w);
    if(dot(n,initialNormal)<0) n=n*(-1.0);
    for(int it=0;it<cfg.irlsIterations;++it){
        Vec3 c{}; for(const auto&p:reps)c=c+p; c=c/(double)reps.size();
        std::vector<double> rs; rs.reserve(reps.size());
        for(const auto&p:reps)rs.push_back(dot(p-c,n));
        const double rm=median(rs), rmad=mad(rs,rm);
        const double s=std::max(1e-5,1.4826*rmad);
        for(std::size_t i=0;i<rs.size();++i){
            const double a=std::abs(rs[i]-rm)/(cfg.huberK*s);
            w[i]=a<=1.0?1.0:1.0/a;
        }
        n=weightedPlaneNormal(reps,w); if(dot(n,initialNormal)<0)n=n*(-1.0);
        out.residualMedian=rm; out.residualMad=rmad;
    }
    out.deltaFromInputDeg=angleDeg(n,initialNormal);
    if(out.deltaFromInputDeg>cfg.maxNormalDeltaDeg){out.decision="NORMAL_JUMP_TOO_LARGE";return out;}
    out.valid=true; out.normal=n; out.decision="APPLY_BALANCED_OUTER_SURFACE_NORMAL";
    return out;
}

/** 【函数导航】
 * 作用：执行“maxAllowedWallCorrectionDeg”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline double maxAllowedWallCorrectionDeg(double nominalMaxDeg,
                                          double wallAngularCoverage,
                                          const Result& surfaceGuard)
{
    if(!surfaceGuard.valid) return 0.0;

    const double q=clampd((wallAngularCoverage-0.50)/0.20,0.0,1.0);
    return nominalMaxDeg*q;
}

}

