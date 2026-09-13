/*
================================================================================
文件：HoleFaXian_Normal.h
模块：Hole 法线计算

【这个文件解决什么问题】
本文件只负责“法线方向”相关的数学计算，把原先混在 HoleWeizi_Pose.h 里的法线基础算法单独归类。
这里同时保留“识别前的粗法线”和“识别后的椭圆精确法线”，但两者职责严格不同：

1. 初始粗法线（guJiChuShiJuBuFaXian）
   - 调用时机：还没有确定真实 Hole 口之前。
   - 目的：只给 mouth-search 建立一个大致正确的局部坐标系，让倾斜 Hole 不至于因为观察方向过差而漏检。
   - 数据来源：seed 周围可能存在的支撑表面。
   - 精度要求：只要求方向大致正确，不拥有最终法线解释权。

2. 外环支撑法线（guJiDuoChiDuWaiHuanFaXian）
   - 调用时机：已经有 Hole 口中心/半径以后，且需要表面法线作为兜底时。
   - 目的：在 Hole 外侧实体表面上做多尺度稳健平面拟合。
   - 注意：如果后面的椭圆法线已经被正式采用，外环支撑法线只能作为辅助证据，不能覆盖椭圆结果。

3. 椭圆反推精确法线（jingQueTuoYuanFanTuiFaXian）
   - 调用时机：真实 Hole 口圆弧/椭圆已经建立以后。
   - 目的：根据“圆在错误观察平面中会表现为椭圆”的几何关系，一次反推出更精确的 Hole 法线。
   - 它不是第二个初始法线，也不负责找 Hole；它属于找到 Hole 之后的精确测量阶段。

【无支撑面的情况】
如果 seed 周围没有足够的支撑面，初始局部法线会返回无效；上层识别继续使用整份点云预先缓存的全局粗法线。
只要 mouth-search 仍能形成可靠 Hole 口圆弧，后续椭圆反推仍可以独立把法线精修到更准确方向。
因此“没有局部支撑面”不会强迫程序伪造一个局部平面法线。

【性能边界】
本文件中的初始法线是搜索辅助，不应为了追求最终精度而无限增加迭代；最终角度精度由后续 Hole 口椭圆和孔壁证据负责。
最终稳定版保持这一边界：粗法线只服务搜索，椭圆法线负责 Hole 建立后的精确角度；
任何性能整理只能减少临时分配/复制，不改变阈值、循环次数、计算顺序或浮点表达式。
================================================================================
*/
#pragma once

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

namespace HoleFaXian {

/** 【类型导航注释】
 * DianYangBen：法线模块统一使用的三维点样本；point 是世界坐标，originalIndex 用于需要回查原始点云时保持索引对应。
 * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct DianYangBen {
    Eigen::Vector3d point{0.0, 0.0, 0.0};
    int originalIndex = -1;
};

/** 【类型导航注释】
 * PingMianJieGuo：粗法线/支撑面拟合结果；包含中心、法线、覆盖度、残差和质量等级。
 * 调用者必须先检查 valid；normal 只代表本阶段测量，不自动拥有最终 Hole 法线解释权。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingMianJieGuo {
    bool valid = false;
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    Eigen::Vector3d normal{0.0, 0.0, 1.0};
    int inputCount = 0;
    int supportCount = 0;
    int coveredSectors = 0;
    double coverage = 0.0;
    double rmse = 0.0;
    double mad = 0.0;
    double radiusUsed = 0.0;
    double annulusInnerRadius = 0.0;
    double annulusOuterRadius = 0.0;
    int quality = 0;
    double normalDeltaDegrees = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    const char* mode = "INVALID";
};

/** 【类型导航注释】
 * DuoChiDuWaiHuanJieGuo：同一 Hole 外环在多组半径尺度上的平面结果集合；best 是按现有质量规则选出的候选。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct DuoChiDuWaiHuanJieGuo {
    PingMianJieGuo best;
    std::array<PingMianJieGuo, 3> candidates{};
    int selectedIndex = -1;
    int usableCandidateCount = 0;
    double normalSpreadDegrees = 0.0;
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

inline double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double value = values[middle];
    if ((values.size() & 1U) == 0U) {
        const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        value = 0.5 * (value + *lower);
    }
    return value;
}

/** 【函数导航】
 * 作用：执行“orientNormal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Eigen::Vector3d orientNormal(Eigen::Vector3d normal, Eigen::Vector3d reference) {
    if (!finiteVec(normal)) normal = Eigen::Vector3d::UnitZ();
    normal.normalize();
    if (!finiteVec(reference)) reference = Eigen::Vector3d::UnitZ();
    reference.normalize();
    if (normal.dot(reference) < 0.0) normal = -normal;
    return normal;
}

/** 【函数导航】
 * 作用：构建“makeAxes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline void makeAxes(const Eigen::Vector3d& normal, Eigen::Vector3d& axisU, Eigen::Vector3d& axisV) {
    const Eigen::Vector3d helper = std::abs(normal.x()) < 0.85
        ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    axisU = normal.cross(helper).normalized();
    axisV = normal.cross(axisU).normalized();
}

/** 【稳健平面基础拟合】
 * 作用：在给定局部点集合上估计一个“粗支撑平面”。连续 5 次用 PCA 求最小特征向量，并用中位数/MAD 删除离群点。
 * 调用位置：初始 seed 粗法线、Hole 外环支撑法线都会复用本函数；它是两者共享的数学底座，不直接参与最终 Hole 类型判定。
 * 输入：input 为世界坐标样本；referenceNormal 用来统一法线正反方向；coverageOrigin/coverageRadius 用于计算角向覆盖；距离单位均为 mm。
 * 输出：PingMianJieGuo。valid 成立还需同时满足 minimumSupport、minimumSectors 和 maximumRmse。
 * 维护风险：5 次迭代、MAD 门限、角扇区和 score 会影响困难 seed 的初始观察方向；普通结构优化不得修改这些数值。
 * 性能：signedDistances/absoluteDistances 求中位数后不再使用，因此最终版把它们 move 给 median，避免一次额外 vector 复制。
 */
inline PingMianJieGuo robustPlane(
    const std::vector<DianYangBen>& input,
    Eigen::Vector3d referenceNormal,
    const Eigen::Vector3d& coverageOrigin,
    double coverageRadius,
    int minimumSupport,
    int minimumSectors,
    double maximumRmse,
    const char* mode)
{
    PingMianJieGuo result;
    result.inputCount = static_cast<int>(input.size());
    result.mode = mode;
    if (input.size() < static_cast<std::size_t>(minimumSupport)) return result;
    if (!finiteVec(referenceNormal)) referenceNormal = Eigen::Vector3d::UnitZ();
    referenceNormal.normalize();

    std::vector<unsigned char> active(input.size(), 1);
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = referenceNormal;
    double mad = 0.0;
    for (int iteration = 0; iteration < 5; ++iteration) {
        center.setZero();
        int count = 0;
        for (std::size_t i = 0; i < input.size(); ++i) {
            if (!active[i]) continue;
            center += input[i].point;
            ++count;
        }
        if (count < minimumSupport) return result;
        center /= static_cast<double>(count);

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < input.size(); ++i) {
            if (!active[i]) continue;
            const Eigen::Vector3d delta = input[i].point - center;
            covariance.noalias() += delta * delta.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) return result;
        normal = orientNormal(solver.eigenvectors().col(0), referenceNormal);

        std::vector<double> signedDistances;
        signedDistances.reserve(input.size());
        for (const DianYangBen& sample : input)
            signedDistances.push_back((sample.point - center).dot(normal));
        const double offset = median(std::move(signedDistances));
        center += normal * offset;

        std::vector<double> absoluteDistances;
        absoluteDistances.reserve(input.size());
        for (const DianYangBen& sample : input)
            absoluteDistances.push_back(std::abs((sample.point - center).dot(normal)));
        mad = median(std::move(absoluteDistances));
        const double limit = std::clamp(3.5 * mad + 0.10, 0.20, 0.85);
        int remaining = 0;
        for (std::size_t i = 0; i < input.size(); ++i) {
            active[i] = std::abs((input[i].point - center).dot(normal)) <= limit ? 1 : 0;
            remaining += active[i] ? 1 : 0;
        }
        if (remaining < minimumSupport) return result;
    }

    int support = 0;
    double squaredError = 0.0;
    Eigen::Vector3d supportCenter = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (!active[i]) continue;
        const double distance = (input[i].point - center).dot(normal);
        squaredError += distance * distance;
        supportCenter += input[i].point;
        ++support;
    }
    if (support < minimumSupport) return result;
    supportCenter /= static_cast<double>(support);
    center += normal * (supportCenter - center).dot(normal);
    const double rmse = std::sqrt(squaredError / static_cast<double>(support));

    Eigen::Vector3d axisU, axisV;
    makeAxes(normal, axisU, axisV);
    constexpr int kSectors = 36;
    std::array<unsigned char, kSectors> sectorHit{};
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (!active[i]) continue;
        const Eigen::Vector3d delta = input[i].point - coverageOrigin;
        const double u = delta.dot(axisU);
        const double v = delta.dot(axisV);
        const double radial = std::hypot(u, v);
        if (radial > coverageRadius || radial < 0.15 * coverageRadius) continue;
        double angle = std::atan2(v, u);
        if (angle < 0.0) angle += 2.0 * 3.14159265358979323846;
        int sector = static_cast<int>(std::floor(angle / (2.0 * 3.14159265358979323846) * kSectors));
        sector = std::clamp(sector, 0, kSectors - 1);
        sectorHit[static_cast<std::size_t>(sector)] = 1;
    }
    const int covered = static_cast<int>(std::count(sectorHit.begin(), sectorHit.end(), 1));
    const double coverage = static_cast<double>(covered) / static_cast<double>(kSectors);
    if (covered < minimumSectors || rmse > maximumRmse) return result;

    const double cosine = std::clamp(normal.dot(referenceNormal), -1.0, 1.0);
    result.valid = true;
    result.center = center;
    result.normal = normal;
    result.supportCount = support;
    result.coveredSectors = covered;
    result.coverage = coverage;
    result.rmse = rmse;
    result.mad = mad;
    result.normalDeltaDegrees = std::acos(cosine) * 180.0 / 3.14159265358979323846;
    result.score = 3.0 * coverage + 0.35 * std::log1p(static_cast<double>(support))
        - 5.0 * rmse - 0.008 * result.normalDeltaDegrees;
    return result;
}

/**
 * @brief 估计单个 seed 附近的“初始粗法线”。
 *
 * 调用位置：HoleShibie_Recognition.cpp 的 mouth-search 前置阶段。
 * 输入含义：neighborhood 是 seed 周围已经收集好的局部点；initialNormal 通常来自整云缓存的全局粗法线。
 * 方法：在 9/12/15/18 mm 四个尺度上做稳健平面拟合，选择覆盖、残差和方向一致性最好的一个。
 * 输出用途：只用于构造 Hole 口搜索坐标系，不直接写入最终 Hole 法线。
 * 无支撑面：四个尺度都不能形成可靠平面时 valid=false，上层继续使用全局粗法线。
 * 性能说明：这是每 seed 的常规粗估计；若以后要提速，应优先测量本函数耗时后再决定是否减少尺度，不能把它与后面的椭圆精修混为一谈。
 */
inline PingMianJieGuo guJiChuShiJuBuFaXian(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& seed,
    Eigen::Vector3d initialNormal)
{
    PingMianJieGuo best;
    if (neighborhood.empty() || !seed.allFinite()) return best;
    if (!finiteVec(initialNormal)) initialNormal = Eigen::Vector3d::UnitZ();
    initialNormal.normalize();
    constexpr std::array<double, 4> radii{{9.0, 12.0, 15.0, 18.0}};
    for (double radius : radii) {
        std::vector<DianYangBen> selected;
        selected.reserve(neighborhood.size());
        const double radiusSquared = radius * radius;
        for (const DianYangBen& sample : neighborhood) {
            if ((sample.point - seed).squaredNorm() <= radiusSquared)
                selected.push_back(sample);
        }
        const int minSupport = std::max(100, static_cast<int>(0.08 * selected.size()));
        PingMianJieGuo candidate = robustPlane(
            selected, initialNormal, seed, radius,
            minSupport, 18, 0.32, "MULTISCALE_SEED_SUPPORT_PLANE");
        candidate.radiusUsed = radius;
        if (!candidate.valid) continue;

        if (std::abs(candidate.normal.z()) < 0.55) continue;
        if (!best.valid || candidate.score > best.score) best = candidate;
    }
    return best;
}

/** 【函数导航】
 * 作用：评估/审核“classifyOuterAnnulusQuality”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline int classifyOuterAnnulusQuality(const PingMianJieGuo& result) {
    if (!result.valid) return 0;
    if (result.supportCount >= 120 && result.coverage >= 0.60
        && result.rmse <= 0.35 && result.normalDeltaDegrees <= 10.0)
        return 3;
    if (result.supportCount >= 80 && result.coverage >= 0.45
        && result.rmse <= 0.45 && result.normalDeltaDegrees <= 12.0)
        return 2;
    if (result.supportCount >= 50 && result.coverage >= 0.28
        && result.rmse <= 0.65 && result.normalDeltaDegrees <= 15.0)
        return 1;
    return 0;
}

/** 【函数导航】
 * 作用：估计“guJiWaiHuanFaXianHouXuan”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline PingMianJieGuo guJiWaiHuanFaXianHouXuan(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    double innerRadius,
    double outerRadius,
    const char* mode)
{
    PingMianJieGuo invalid;
    invalid.mode = mode;
    invalid.annulusInnerRadius = innerRadius;
    invalid.annulusOuterRadius = outerRadius;
    if (neighborhood.empty() || !mouthCenter.allFinite()
        || !(innerRadius > 0.0) || !(outerRadius > innerRadius)) return invalid;
    if (!finiteVec(initialNormal)) initialNormal = Eigen::Vector3d::UnitZ();
    initialNormal.normalize();
    Eigen::Vector3d axisU, axisV;
    makeAxes(initialNormal, axisU, axisV);

    std::vector<DianYangBen> annulus;
    annulus.reserve(neighborhood.size());
    for (const DianYangBen& sample : neighborhood) {
        const Eigen::Vector3d delta = sample.point - mouthCenter;
        const double axial = delta.dot(initialNormal);

        if (axial < -0.55 || axial > 1.10) continue;
        const double radial = std::hypot(delta.dot(axisU), delta.dot(axisV));
        if (radial < innerRadius || radial > outerRadius) continue;
        annulus.push_back(sample);
    }

    const int minSupport = std::max(50, static_cast<int>(0.08 * annulus.size()));
    PingMianJieGuo result = robustPlane(
        annulus, initialNormal, mouthCenter, outerRadius,
        minSupport, 10, 0.65, mode);
    result.radiusUsed = outerRadius;
    result.annulusInnerRadius = innerRadius;
    result.annulusOuterRadius = outerRadius;
    result.quality = classifyOuterAnnulusQuality(result);
    if (result.quality == 0) result.valid = false;
    return result;
}

/** 【Hole 建立后的多尺度外环支撑法线】
 * 作用：已知 mouthCenter/topRadius 后，只取 Hole 外侧实体环带，在三组内外半径上拟合支撑面并选质量最佳结果。
 * 调用位置：HoleShibie_Recognition.cpp 的后续位姿/表面支撑阶段；不是 mouth-search 前的初始法线。
 * 输入：mouthCenter 为已解析 Hole 口中心，topRadius 为当前 Hole 口半径，单位 mm；initialNormal 只作为正反方向和初始参考。
 * 输出：DuoChiDuWaiHuanJieGuo，保留三组候选、选中索引、可用候选数量和候选间法线离散角。
 * 维护边界：它用于表面支撑和辅助复核；一旦椭圆精确法线已经拥有最终解释权，本结果不能反向覆盖精确法线。
 */
inline DuoChiDuWaiHuanJieGuo guJiDuoChiDuWaiHuanFaXian(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    double topRadius)
{
    DuoChiDuWaiHuanJieGuo output;
    if (neighborhood.empty() || !mouthCenter.allFinite() || !(topRadius > 0.0))
        return output;
    if (!finiteVec(initialNormal)) initialNormal = Eigen::Vector3d::UnitZ();
    initialNormal.normalize();

    const std::array<double, 3> inner{{
        std::max(1.10 * topRadius, topRadius + 0.35),
        std::max(1.30 * topRadius, topRadius + 0.80),
        topRadius + 2.0
    }};
    const std::array<double, 3> outer{{
        std::max(2.00 * topRadius, topRadius + 3.0),
        std::max(2.80 * topRadius, topRadius + 6.0),
        topRadius + 8.0
    }};
    constexpr std::array<const char*, 3> modes{{
        "OuterAnnulusFrame_ANNULUS_A_TIGHT",
        "OuterAnnulusFrame_ANNULUS_B_WIDE",
        "OuterAnnulusFrame_ANNULUS_C_ABSOLUTE"
    }};

    for (std::size_t index = 0; index < output.candidates.size(); ++index) {
        output.candidates[index] = guJiWaiHuanFaXianHouXuan(
            neighborhood, mouthCenter, initialNormal,
            inner[index], outer[index], modes[index]);
        const PingMianJieGuo& candidate = output.candidates[index];
        if (!candidate.valid) continue;
        if (candidate.quality >= 2) ++output.usableCandidateCount;
        if (output.selectedIndex < 0) {
            output.selectedIndex = static_cast<int>(index);
            output.best = candidate;
            continue;
        }
        const PingMianJieGuo& current = output.best;
        if (candidate.quality > current.quality
            || (candidate.quality == current.quality && candidate.score > current.score)) {
            output.selectedIndex = static_cast<int>(index);
            output.best = candidate;
        }
    }

    for (std::size_t left = 0; left < output.candidates.size(); ++left) {
        if (!output.candidates[left].valid || output.candidates[left].quality < 2) continue;
        for (std::size_t right = left + 1; right < output.candidates.size(); ++right) {
            if (!output.candidates[right].valid || output.candidates[right].quality < 2) continue;
            const double cosine = std::clamp(
                output.candidates[left].normal.dot(output.candidates[right].normal), -1.0, 1.0);
            output.normalSpreadDegrees = std::max(
                output.normalSpreadDegrees,
                std::acos(cosine) * 180.0 / 3.14159265358979323846);
        }
    }

    if (output.selectedIndex >= 0 && output.usableCandidateCount >= 2
        && output.normalSpreadDegrees > 5.0) {
        output.best.quality = 1;
    }
    return output;
}

/** 【函数导航】
 * 作用：估计“guJiWaiHuanFaXian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline PingMianJieGuo guJiWaiHuanFaXian(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    double topRadius)
{
    return guJiDuoChiDuWaiHuanFaXian(
        neighborhood, mouthCenter, initialNormal, topRadius).best;
}


// ============================================================================
// 精确法线：Hole 口椭圆反推
// ============================================================================
// 这一分区与上面的“初始粗法线”处在不同阶段。此时 Hole 口圆弧已经建立，
// 初始法线只作为局部坐标基准和法线正负方向参考；真正的角度修正量来自椭圆轴比/长轴方向。
// 它不会帮助 mouth-search 找 Hole，也不会在没有 Hole 口证据时凭空计算法线。
/** 【类型导航注释】
 * YuanHuFaXianJieGuo：Hole 口圆弧/椭圆反推法线的精确测量结果；仅在圆弧几何证据成立后生成。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct YuanHuFaXianJieGuo {
    bool attempted = false;
    bool valid = false;
    bool used = false;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    int edgeSectors = 0;
    double edgeCoverage = 0.0;
    double ellipseAxisRatio = 1.0;
    double ellipseTiltDegrees = 0.0;
    double ellipseModelRmse = std::numeric_limits<double>::infinity();
    double initialCircleRmse = std::numeric_limits<double>::infinity();
    double finalCircleRmse = std::numeric_limits<double>::infinity();
    double initialCoverage = 0.0;
    double finalCoverage = 0.0;
    double initialScore = -std::numeric_limits<double>::infinity();
    double finalScore = -std::numeric_limits<double>::infinity();
    double normalDeltaDegrees = 0.0;
    double mirrorScoreGap = 0.0;
    const char* mode = "ARC_NORMAL_INVALID";
};

/** 【类型导航注释】
 * YuanHuYuanXingScore：给定观察法线下的 Hole 口圆弧圆形度评分，用于比较法线微调前后的投影质量。
 * 主要使用位置：HoleWeizi_Pose.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct YuanHuYuanXingScore {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    double coverage = 0.0;
    double continuousCoverage = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double clean = 0.0;
    double radius = 0.0;
};

/** 【函数导航】
 * 作用：执行“zuiChangLianXuYuanHu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline int zuiChangLianXuYuanHu(const std::array<unsigned char, 36>& hit) {
    int best = 0;
    int current = 0;
    for (int index = 0; index < 72; ++index) {
        if (hit[static_cast<std::size_t>(index % 36)] != 0U) {
            current = std::min(36, current + 1);
            best = std::max(best, current);
        } else {
            current = 0;
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：评估/审核“pingGuYuanHuYuanXing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 位姿计算。
 * 主要引用/调用位置：HoleWeizi_Pose.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline YuanHuYuanXingScore pingGuYuanHuYuanXing(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d normal,
    double topRadius)
{
    YuanHuYuanXingScore out;
    if (neighborhood.empty() || !mouthCenter.allFinite() || !(topRadius > 1.0))
        return out;
    if (!finiteVec(normal)) return out;
    normal.normalize();
    if (normal.z() < 0.0) normal = -normal;

    Eigen::Vector3d axisU, axisV;
    makeAxes(normal, axisU, axisV);
    std::array<double, 36> nearest;
    nearest.fill(std::numeric_limits<double>::infinity());
    const double sliceHalf = std::clamp(0.13 * topRadius, 0.42, 0.70);
    const double maximumRadial = std::max(topRadius + 1.6, 1.45 * topRadius);
    int corePoints = 0;
    int ringSupport = 0;
    for (const DianYangBen& sample : neighborhood) {
        if (!sample.point.allFinite()) continue;
        const Eigen::Vector3d delta = sample.point - mouthCenter;
        const double axial = delta.dot(normal);
        if (std::abs(axial) > sliceHalf) continue;
        const double u = delta.dot(axisU);
        const double v = delta.dot(axisV);
        const double radial = std::hypot(u, v);
        if (radial < 0.70 * topRadius) ++corePoints;
        if (std::abs(radial - topRadius) <= std::max(0.55, 0.18 * topRadius))
            ++ringSupport;
        if (radial < 0.45 || radial > maximumRadial) continue;
        double angle = std::atan2(v, u);
        if (angle < 0.0) angle += 2.0 * 3.14159265358979323846;
        int sector = static_cast<int>(std::floor(
            angle / (2.0 * 3.14159265358979323846) * 36.0));
        sector = std::clamp(sector, 0, 35);
        double& first = nearest[static_cast<std::size_t>(sector)];
        if (radial < first) first = radial;
    }

    std::vector<double> ring;
    ring.reserve(36);
    std::array<unsigned char, 36> hit{};
    const double selectHalf = std::max(0.60, 0.22 * topRadius);
    for (int sector = 0; sector < 36; ++sector) {
        const double radial = nearest[static_cast<std::size_t>(sector)];
        if (!std::isfinite(radial) || std::abs(radial - topRadius) > selectHalf)
            continue;
        hit[static_cast<std::size_t>(sector)] = 1U;
        ring.push_back(radial);
    }
    if (ring.size() < 9U) return out;

    const double radius = median(ring);
    double squaredError = 0.0;
    for (double radial : ring) {
        const double difference = radial - radius;
        squaredError += difference * difference;
    }
    const double rmse = std::sqrt(squaredError / static_cast<double>(ring.size()));
    const double coverage = static_cast<double>(ring.size()) / 36.0;
    const double continuous = static_cast<double>(zuiChangLianXuYuanHu(hit)) / 36.0;
    const double coreRatio = static_cast<double>(corePoints)
        / static_cast<double>(std::max(1, ringSupport));
    const double clean = std::clamp(1.0 - coreRatio, 0.0, 1.0);
    if (coverage < 0.25 || continuous < 0.16 || clean < 0.72) return out;

    out.valid = true;
    out.coverage = coverage;
    out.continuousCoverage = continuous;
    out.rmse = rmse;
    out.clean = clean;
    out.radius = radius;
    out.score = 4.0 * coverage + 2.0 * continuous + 1.8 * clean
        - 3.2 * rmse - 0.50 * std::abs(radius - topRadius);
    return out;
}

// Hole 口候选已经在真实第一边缘上完成中心约束椭圆拟合。
// 本函数不再检测第二遍圆弧，也不扫描角度：只把椭圆轴比和长轴方向解析成世界法线，
// 再用同一局部点云检查“圆化后是否真的更像圆”。所以它是精确测量器，不是第二套 Hole 检测器。
/** 【Hole 口椭圆反推精确法线】
 * 作用：Hole 口已经建立后，根据圆在当前观察平面中的椭圆轴比和主轴方向解析出更准确的空间法线，再用同一批局部点检查“校正后是否更接近圆”。
 * 调用位置：HoleShibie_Recognition.cpp 的精确测量阶段；不会在 seed 初始搜索阶段运行，也不会负责重新寻找 Hole。
 * 输入：mouthCenter/topRadius 是已锁定 Hole 几何；frameU/frameV/initialNormal 是当前观察坐标；ellipse* 是上游已拟合的椭圆证据。
 * 输出：YuanHuFaXianJieGuo。used=true 才表示精确法线正式采用；信息不足时保留 initialNormal，而不是强行制造角度。
 * 维护边界：本函数与初始 PCA/RANSAC 粗法线职责不同；不要再叠加第二套角度网格搜索，否则会重复计算并可能改变最终精度。
 */
inline YuanHuFaXianJieGuo jingQueTuoYuanFanTuiFaXian(
    const std::vector<DianYangBen>& neighborhood,
    const Eigen::Vector3d& mouthCenter,
    Eigen::Vector3d initialNormal,
    Eigen::Vector3d frameU,
    Eigen::Vector3d frameV,
    double topRadius,
    double ellipseAxisRatio,
    double ellipseTiltU,
    double ellipseTiltV,
    double ellipseModelRmse,
    double centerDriftU,
    double centerDriftV,
    double edgeCoverage)
{
    YuanHuFaXianJieGuo out;
    out.attempted = true;
    out.ellipseAxisRatio = ellipseAxisRatio;
    out.ellipseModelRmse = ellipseModelRmse;
    out.edgeCoverage = edgeCoverage;
    if (neighborhood.size() < 40U || !mouthCenter.allFinite()
        || !(topRadius > 1.0) || !finiteVec(initialNormal)
        || !finiteVec(frameU) || !finiteVec(frameV)
        || !(ellipseAxisRatio >= 0.35 && ellipseAxisRatio <= 1.0)) {
        out.mode = "ARC_NORMAL_REJECT_INTEGRATED_EVIDENCE";
        return out;
    }

    initialNormal.normalize();
    if (initialNormal.z() < 0.0) initialNormal = -initialNormal;
    frameU -= initialNormal * frameU.dot(initialNormal);
    if (!finiteVec(frameU)) {
        out.mode = "ARC_NORMAL_REJECT_FRAME_U";
        return out;
    }
    frameU.normalize();
    frameV -= initialNormal * frameV.dot(initialNormal);
    frameV -= frameU * frameV.dot(frameU);
    if (!finiteVec(frameV)) frameV = initialNormal.cross(frameU);
    if (!finiteVec(frameV)) {
        out.mode = "ARC_NORMAL_REJECT_FRAME_V";
        return out;
    }
    frameV.normalize();
    if (initialNormal.dot(frameU.cross(frameV)) < 0.0) frameV = -frameV;
    out.normal = initialNormal;

    const YuanHuYuanXingScore initial = pingGuYuanHuYuanXing(
        neighborhood, mouthCenter, initialNormal, topRadius);
    out.initialCircleRmse = initial.rmse;
    out.initialCoverage = initial.coverage;
    out.initialScore = initial.score;

    // 接近圆时没有稳定的椭圆主轴，说明粗法向已经足够；不为了“必须修正”而制造方向。
    if (ellipseAxisRatio >= 0.995) {
        out.valid = initial.valid;
        out.finalCircleRmse = initial.rmse;
        out.finalCoverage = initial.coverage;
        out.finalScore = initial.score;
        out.ellipseTiltDegrees = 0.0;
        out.mode = "ARC_NORMAL_KEEP_INITIAL_NEAR_CIRCLE";
        return out;
    }

    const double directionNorm = std::hypot(ellipseTiltU, ellipseTiltV);
    if (!(directionNorm > 1e-9)) {
        out.mode = "ARC_NORMAL_REJECT_ELLIPSE_DIRECTION";
        return out;
    }
    ellipseTiltU /= directionNorm;
    ellipseTiltV /= directionNorm;
    Eigen::Vector3d tiltWorld = frameU * ellipseTiltU + frameV * ellipseTiltV;
    if (!finiteVec(tiltWorld)) {
        out.mode = "ARC_NORMAL_REJECT_WORLD_TILT_DIRECTION";
        return out;
    }
    tiltWorld.normalize();

    const double sine = std::sqrt(std::max(
        0.0, 1.0 - ellipseAxisRatio * ellipseAxisRatio));
    out.ellipseTiltDegrees = std::acos(std::clamp(ellipseAxisRatio, 0.0, 1.0))
        * 180.0 / 3.14159265358979323846;

    // 斜切同一圆柱/锥孔的多个局部层时，截面中心会沿真实孔轴在粗平面中的投影方向漂移。
    // 若这个多层证据足够明显，就直接用它确定椭圆长轴的正负号；仍然只计算一个法向。
    const double driftMagnitude = std::hypot(centerDriftU, centerDriftV);
    double driftAlignment = 0.0;
    if (driftMagnitude > 1e-9) {
        driftAlignment = std::abs(
            (centerDriftU * ellipseTiltU + centerDriftV * ellipseTiltV) / driftMagnitude);
    }
    const double expectedDrift = sine / std::max(ellipseAxisRatio, 1e-9); // tan(倾角)
    const double driftScaleRatio = expectedDrift > 0.03
        ? driftMagnitude / expectedDrift : std::numeric_limits<double>::infinity();
    // 两层中心恰好抖了1 mm也可能产生很大的假斜率，所以还要与椭圆给出的 tan(倾角) 同量级。
    const bool driftResolvesSign = driftMagnitude >= 0.04
        && driftAlignment >= 0.55
        && driftScaleRatio >= 0.30 && driftScaleRatio <= 2.60;

    std::array<Eigen::Vector3d, 2> directNormals{{
        ellipseAxisRatio * initialNormal + sine * tiltWorld,
        ellipseAxisRatio * initialNormal - sine * tiltWorld
    }};
    for (Eigen::Vector3d& candidate : directNormals) {
        if (finiteVec(candidate)) candidate.normalize();
    }

    int selected = -1;
    YuanHuYuanXingScore selectedScore;
    if (driftResolvesSign) {
        const double signedProjection = centerDriftU * ellipseTiltU
            + centerDriftV * ellipseTiltV;
        selected = signedProjection >= 0.0 ? 0 : 1;
        if (!finiteVec(directNormals[static_cast<std::size_t>(selected)])
            || directNormals[static_cast<std::size_t>(selected)].z() <= 0.0) {
            out.mode = "ARC_NORMAL_REJECT_DRIFT_DIRECTED_Z";
            return out;
        }
        selectedScore = pingGuYuanHuYuanXing(
            neighborhood, mouthCenter,
            directNormals[static_cast<std::size_t>(selected)], topRadius);
        if (!selectedScore.valid) {
            out.mode = "ARC_NORMAL_REJECT_DRIFT_DIRECTED_GEOMETRY";
            return out;
        }
        out.mode = "ARC_NORMAL_DIRECT_ELLIPSE_LAYER_DRIFT";
    } else {
        // 只有在多层中心漂移不足以消除数学上的 ± 二义性时，才各验证一次两个解析解。
        // 这里没有角度网格、没有迭代优化：倾角和方向都已经由椭圆一次求出。
        std::array<YuanHuYuanXingScore, 2> scores{};
        std::array<bool, 2> usable{{false, false}};
        for (std::size_t index = 0; index < directNormals.size(); ++index) {
            const Eigen::Vector3d& candidate = directNormals[index];
            if (!finiteVec(candidate) || candidate.z() <= 0.0) continue;
            scores[index] = pingGuYuanHuYuanXing(
                neighborhood, mouthCenter, candidate, topRadius);
            usable[index] = scores[index].valid;
        }
        if (usable[0] && usable[1]) {
            const double difference = scores[0].score - scores[1].score;
            out.mirrorScoreGap = std::abs(difference);
            if (std::abs(difference) > 0.025)
                selected = difference > 0.0 ? 0 : 1;
            else
                selected = directNormals[0].z() >= directNormals[1].z() ? 0 : 1;
        } else if (usable[0]) {
            selected = 0;
        } else if (usable[1]) {
            selected = 1;
        }
        if (selected < 0) {
            out.mode = "ARC_NORMAL_REJECT_DIRECT_SOLUTIONS";
            return out;
        }
        selectedScore = scores[static_cast<std::size_t>(selected)];
        out.mode = "ARC_NORMAL_DIRECT_ELLIPSE_MIRROR_GEOMETRY";
    }

    const Eigen::Vector3d selectedNormal = directNormals[static_cast<std::size_t>(selected)];
    const double cosine = std::clamp(initialNormal.dot(selectedNormal), -1.0, 1.0);
    const double deltaDegrees = std::acos(cosine)
        * 180.0 / 3.14159265358979323846;

    const bool initialInvalid = !initial.valid;
    const bool geometryImproved = initialInvalid
        || selectedScore.score >= initial.score + 0.04
        || selectedScore.rmse <= initial.rmse * 0.95
        || selectedScore.coverage >= initial.coverage + 0.055;
    const bool physicallyUsable = selectedNormal.z() > 0.0;

    out.valid = true;
    out.normal = (geometryImproved && physicallyUsable)
        ? selectedNormal : initialNormal;
    out.used = geometryImproved && physicallyUsable && deltaDegrees >= 0.15;
    out.normalDeltaDegrees = out.used ? deltaDegrees : 0.0;
    out.finalCircleRmse = out.used ? selectedScore.rmse : initial.rmse;
    out.finalCoverage = out.used ? selectedScore.coverage : initial.coverage;
    out.finalScore = out.used ? selectedScore.score : initial.score;
    if (!out.used)
        out.mode = "ARC_NORMAL_KEEP_INITIAL_NO_GEOMETRIC_GAIN";
    return out;
}

}
