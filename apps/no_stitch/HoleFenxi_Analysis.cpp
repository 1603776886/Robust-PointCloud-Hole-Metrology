/*
================================================================================
文件：HoleFenxi_Analysis.cpp
模块：Hole 分析实现

【主要职责】
实现候选聚类排序、直孔/锥孔判定、二次分析、补充判定与一致性检查。

【主要调用关系】
由 HoleShibie_Recognition.cpp 的阶段化流程调用。

【线程与状态】
纯计算；排序/tie-break 属于生产行为。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
孔候选聚类子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，把局部几何证据聚合为稳定候选。

维护说明：
聚类距离、支持点数等阈值会改变候选集合；调整前先写明单位，再做全量等价性验证。
*/
#include "HoleFenxi_Analysis.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <condition_variable>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <utility>

namespace HoleHouXuanJuLei {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 最终 GUI 版固定采用已经正式化的生产配置：
// 稀疏层处理开启、bitset 连通域开启、layer 使用 4 个工作线程、不启用持久线程池。
constexpr bool kUseSparseLayers = true;
constexpr bool kUseBitsetComponents = true;
constexpr int kLayerWorkerCount = 4;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value)
{
    return std::isfinite(value);
}

// GUI 点击只承担“选择哪个孔”的职责。归属依据始终是种子到候选机械孔口边线的
// 最短径向距离 |distance(seed, center) - radius|，而不是到圆心的距离。
// 捕获宽度直接与用户配置的最大孔半径联动：最大孔越大，允许点击离孔沿越远；
// 点云栅格越稀，额外保留少量采样间距容差。这里只扩大“点击归属范围”，不改变
// 圆拟合、孔壁、内喉、锥壁、法向和最终尺寸的任何几何阈值。
/** 【函数导航】
 * 作用：执行“seedEdgeOwnershipLimit”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double seedEdgeOwnershipLimit(double radius, double cellSize, double maxHoleRadius)
{
    (void)radius;
    const double configuredMaxRadius = std::clamp(maxHoleRadius, 2.0, 30.0);
    const double spacingAllowance = 3.0 * std::clamp(cellSize, 0.15, 0.50);
    const double configuredReach = 0.70 * configuredMaxRadius + spacingAllowance;
    return std::clamp(configuredReach, 4.0, 12.0);
}

/** 【函数导航】
 * 作用：执行“seedBelongsToMouth”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool seedBelongsToMouth(
    double radialDistance, double radius, double cellSize, double maxHoleRadius)
{
    if (!(radius > 0.0) || !finite(radialDistance)) return false;
    const double edgeDistance = std::abs(radialDistance - radius);
    return edgeDistance <= seedEdgeOwnershipLimit(radius, cellSize, maxHoleRadius);
}

// 跨层持续几何族优先，seed 只承担弱归属约束，避免局部短弧抢占稳定孔口。
// 这里不替换现有孔沿归属：普通候选仍走 seedBelongsToMouth。只有一个跨层稳定、
// 覆盖充分、中心/半径抖动小的候选在几何分数上明显胜过当前孔沿最近候选时，
// 才允许在固定 ROI 内用较宽的中心关联接管。这样提高 seed 方位容错，同时不改变
// 1.51 残缺锥孔现有的开放圆弧门、SurfaceProfile、canonical 和最终孔型链。
/** 【函数导航】
 * 作用：执行“persistentFamilyCenterLimit”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double persistentFamilyCenterLimit(double maxHoleRadius)
{
    // Seed 只承担固定 ROI 内的目标归属，不应再用一个与 GUI 最大孔径脱节的 14 mm 常数截断真实孔。
    // 默认 maxHoleRadius=10 mm 时允许候选中心距 seed 20 mm；用户增大最大孔径时按 2R 联动。
    // 真正可达范围仍由当前固定 ROI 内已有候选限制，因此这里不会扩大成整云搜索。
    const double configuredMaxRadius = std::clamp(maxHoleRadius, 2.0, 30.0);
    return std::max(20.0, 2.0 * configuredMaxRadius);
}

/** 【函数导航】
 * 作用：执行“persistentFamilyEligible”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool persistentFamilyEligible(const Cluster& cluster, double radialDistance, const Input& input)
{
    if (!cluster.stable || !cluster.mouthAttachmentValid || cluster.deepContinuation) return false;
    if (!finite(radialDistance) || radialDistance > persistentFamilyCenterLimit(input.maxHoleRadius)) return false;
    // 这里只是“接管资格”，不是全局孔径门。小孔仍可按原孔沿路径正常识别；
    // 但 R≈1.x mm 的局部短弧不能仅凭几何分数反抢半径更合理、跨层持续的真实 Hole 口。
    const double radius = cluster.consensusTopRadius;
    if (!(radius >= 2.0) || radius > 0.80 * std::clamp(input.maxHoleRadius, 2.0, 30.0)) return false;
    if (cluster.supportLayers < 2 || cluster.meanCoverage < 0.50
        || cluster.trajectoryContinuity < 0.90) return false;
    if (cluster.centerStd > std::max(0.30, 0.075 * radius)) return false;
    if (cluster.radiusStd > std::max(0.35, 0.10 * radius)) return false;
    if (cluster.meanResidual > std::max(0.22, 0.060 * radius)) return false;
    return true;
}

/** 【函数导航】
 * 作用：执行“persistentFamilyScore”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double persistentFamilyScore(const Cluster& cluster, double radialDistance)
{
    // 几何持续性已经浓缩在 cluster.score 中；这里继续把它作为候选稳定性的主证据之一，
    // seed 到中心只保留 0.018/mm 的弱关联，不再用孔沿距离压倒几何证据。
    return cluster.score - 0.018 * radialDistance;
}

double median(std::vector<double> values)
{
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double out = values[middle];
    if ((values.size() & 1U) == 0U) {
        const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        out = 0.5 * (out + *lower);
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“medianInPlace”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double medianInPlace(std::vector<double>& values)
{
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double out = values[middle];
    if ((values.size() & 1U) == 0U) {
        const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        out = 0.5 * (out + *lower);
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“quantile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double quantile(std::vector<double> values, double q)
{
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    q = std::clamp(q, 0.0, 1.0);
    const double position = q * static_cast<double>(values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(position));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(position));
    const double t = position - static_cast<double>(lo);
    return values[lo] * (1.0 - t) + values[hi] * t;
}

/** 【函数导航】
 * 作用：执行“robustStd”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double robustStd(const std::vector<double>& values, double center)
{
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double value : values) {
        const double delta = value - center;
        sum += delta * delta;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

/** 【类型导航注释】
 * BanJingQuShi：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct BanJingQuShi {
    bool valid = false;
    int layers = 0;
    double slope = 0.0;
    double rmse = 0.0;
    double monotonicity = 0.0;
    double span = 0.0;
};

/** 【函数导航】
 * 作用：拟合/求解“fitRadiusTrend”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
BanJingQuShi fitRadiusTrend(const std::vector<HouXuan>& ordered)
{
    BanJingQuShi out;
    std::vector<const HouXuan*> perLayer;
    perLayer.reserve(ordered.size());
    for (const HouXuan& candidate : ordered) {
        if (perLayer.empty() || perLayer.back()->layerIndex != candidate.layerIndex) {
            perLayer.push_back(&candidate);
        } else if (candidate.score > perLayer.back()->score) {
            perLayer.back() = &candidate;
        }
    }
    if (perLayer.size() < 3) return out;

    double meanW = 0.0;
    double meanR = 0.0;
    for (const HouXuan* candidate : perLayer) {
        meanW += candidate->layerW;
        meanR += candidate->radius;
    }
    meanW /= static_cast<double>(perLayer.size());
    meanR /= static_cast<double>(perLayer.size());
    double numerator = 0.0;
    double denominator = 0.0;
    for (const HouXuan* candidate : perLayer) {
        const double dw = candidate->layerW - meanW;
        numerator += dw * (candidate->radius - meanR);
        denominator += dw * dw;
    }
    if (denominator <= 1e-9) return out;
    out.slope = numerator / denominator;
    const double intercept = meanR - out.slope * meanW;
    double sse = 0.0;
    double minRadius = std::numeric_limits<double>::infinity();
    double maxRadius = -std::numeric_limits<double>::infinity();
    int monotonic = 0;
    for (std::size_t i = 0; i < perLayer.size(); ++i) {
        const double prediction = intercept + out.slope * perLayer[i]->layerW;
        const double error = perLayer[i]->radius - prediction;
        sse += error * error;
        minRadius = std::min(minRadius, perLayer[i]->radius);
        maxRadius = std::max(maxRadius, perLayer[i]->radius);
        if (i > 0) {
            const double delta = perLayer[i]->radius - perLayer[i - 1]->radius;
            const double signedDelta = out.slope >= 0.0 ? delta : -delta;
            if (signedDelta >= -0.25) ++monotonic;
        }
    }
    out.valid = true;
    out.layers = static_cast<int>(perLayer.size());
    out.rmse = std::sqrt(sse / static_cast<double>(perLayer.size()));
    out.monotonicity = perLayer.size() > 1
        ? static_cast<double>(monotonic) / static_cast<double>(perLayer.size() - 1) : 1.0;
    out.span = maxRadius - minRadius;
    return out;
}

/** 【函数导航】
 * 作用：估计“estimateDominantSupportPlaneW”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double estimateDominantSupportPlaneW(const std::vector<Sample>& samples)
{
    if (samples.empty()) return 0.0;
    constexpr double kBin = 0.25;
    std::vector<double> finiteW;
    finiteW.reserve(samples.size());
    for (const Sample& sample : samples) {
        if (finite(sample.w)) finiteW.push_back(sample.w);
    }
    if (finiteW.empty()) return 0.0;

    const auto minMax = std::minmax_element(finiteW.begin(), finiteW.end());
    const double minW = *minMax.first;
    const double maxW = *minMax.second;
    const int bins = std::max(1, static_cast<int>(std::ceil((maxW - minW) / kBin)) + 1);
    std::vector<int> counts(static_cast<std::size_t>(bins), 0);
    for (double w : finiteW) {
        const int bin = std::clamp(static_cast<int>(std::floor((w - minW) / kBin)), 0, bins - 1);
        ++counts[static_cast<std::size_t>(bin)];
    }
    const int bestBin = static_cast<int>(std::distance(
        counts.begin(), std::max_element(counts.begin(), counts.end())));
    std::vector<double> inBin;
    inBin.reserve(static_cast<std::size_t>(counts[static_cast<std::size_t>(bestBin)]));
    const double low = minW + static_cast<double>(bestBin) * kBin;
    const double high = low + kBin;
    for (double w : finiteW) {
        if (w >= low && w < high) inBin.push_back(w);
    }
    return inBin.empty() ? low + 0.5 * kBin : median(std::move(inBin));
}

// 全 ROI 的“点数最多层”在严重残缺孔里可能落到孔底/背面：上口只剩约三四成时，
// 大面积深层平面反而拥有更多点。孔口搜索不能因此只围绕深层生成圆弧。
// 这里在点击附近的有限径向范围内再估一次局部支撑层；它仍然只是同一候选系统的
// 轴向锚点，不代表残缺孔专用分支。搜索法向统一约定世界 Z 朝外，因此真实外表面
// 在局部 W 上应位于孔内深层的外侧（更大的 W）。
/** 【函数导航】
 * 作用：估计“estimateSeedLocalSupportPlaneW”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double estimateSeedLocalSupportPlaneW(
    const std::vector<Sample>& samples,
    double seedU, double seedV, double radialLimit, double tuoDiW)
{
    if (samples.empty() || !finite(seedU) || !finite(seedV)) return tuoDiW;
    const double localRadius = std::clamp(0.40 * radialLimit, 8.0, 12.0);
    const double localRadius2 = localRadius * localRadius;
    std::vector<double> localW;
    localW.reserve(std::min<std::size_t>(samples.size(), 4096U));
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double du = sample.u - seedU;
        const double dv = sample.v - seedV;
        if (du * du + dv * dv <= localRadius2) localW.push_back(sample.w);
    }
    // 点数太少时不让局部统计制造一个不可靠的新支撑层。
    if (localW.size() < 80U) return tuoDiW;

    // 不使用“从最小 W 起算的固定直方图格子”。严重残缺孔的上口/孔底峰值可能只差
    // 很少几个点，格子相位平移 0.05~0.10 mm 就会让主峰互换。这里改成 0.30 mm
    // 滑动窗口寻找局部密度峰；随后在不低于最强峰 65% 支撑的峰里选 W 最大者。
    // 因为所有正式搜索法向都统一为世界 Z 朝外，W 最大的强峰对应外表面，而不是孔底。
    std::sort(localW.begin(), localW.end());
    constexpr double windowWidth = 0.30;
    std::size_t right = 0U;
    std::size_t maxSupport = 0U;
    /** 【类型导航注释】
     * Window：Hole 分析实现中的自定义 结构体。
     * 主要使用位置：ZhuChuangKou_Window.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Window { std::size_t begin=0U; std::size_t end=0U; std::size_t support=0U; double center=0.0; };
    std::vector<Window> windows;
    windows.reserve(localW.size());
    for (std::size_t left = 0U; left < localW.size(); ++left) {
        if (right < left) right = left;
        while (right < localW.size() && localW[right] < localW[left] + windowWidth) ++right;
        const std::size_t support = right - left;
        if (support == 0U) continue;
        const double center = 0.5 * (localW[left] + localW[right - 1U]);
        windows.push_back({left, right, support, center});
        maxSupport = std::max(maxSupport, support);
    }
    if (maxSupport < 20U || windows.empty()) return tuoDiW;
    const std::size_t strongSupport = std::max<std::size_t>(
        20U, static_cast<std::size_t>(std::ceil(0.65 * static_cast<double>(maxSupport))));
    const Window* selected = nullptr;
    for (const Window& window : windows) {
        if (window.support < strongSupport) continue;
        if (!selected || window.center > selected->center) selected = &window;
    }
    if (!selected) return tuoDiW;
    std::vector<double> peak(
        localW.begin() + static_cast<std::ptrdiff_t>(selected->begin),
        localW.begin() + static_cast<std::ptrdiff_t>(selected->end));
    return peak.empty() ? selected->center : median(std::move(peak));
}


/** 【函数导航】
 * 作用：执行“zuiChangLianXuYuanHu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int zuiChangLianXuYuanHu(const std::vector<unsigned char>& hits)
{
    if (hits.empty()) return 0;
    const int count = static_cast<int>(hits.size());
    int best = 0;
    int current = 0;
    for (int i = 0; i < 2 * count; ++i) {
        if (hits[static_cast<std::size_t>(i % count)] != 0) {
            current = std::min(count, current + 1);
            best = std::max(best, current);
        } else {
            current = 0;
        }
    }
    return std::min(best, count);
}



/** 【类型导航注释】
 * YuanHuLianXuTongJi：开放/残缺 Hole 口沿的周向连续段统计。
 * maxLianXuSector：单段最长连续扇区数；
 * youXiaoDuanSectorHe：所有“至少连续 2 个扇区”的有效圆弧段总扇区数；
 * youXiaoDuanShu：有效圆弧段数量。
 *
 * 本结构只用于“圆弧是否具备圆形证据”的资格判断，不改变圆拟合、法向、跨层聚类、
 * seed 归属、孔型或深度。单个孤立扇区不计入多段合计，避免稀疏噪点凑总角度。
 */
struct YuanHuLianXuTongJi {
    int maxLianXuSector = 0;
    int youXiaoDuanSectorHe = 0;
    int youXiaoDuanShu = 0;
};

/** 【函数导航】
 * 作用：统计环形扇区上的连续圆弧段，并正确处理 0°/360° 首尾相接。
 * 规则：单段连续弧 >= 90°，或者至少两段连续弧的有效连续角合计 >= 180°。
 * 调用位置：openArcCandidatesAtLayer() 的开放/残缺圆弧资格门。
 */
YuanHuLianXuTongJi tongJiYuanHuLianXuDuan(const std::vector<unsigned char>& hits)
{
    YuanHuLianXuTongJi out;
    if (hits.empty()) return out;
    const int count = static_cast<int>(hits.size());

    bool allHit = true;
    int firstZero = -1;
    for (int i = 0; i < count; ++i) {
        if (hits[static_cast<std::size_t>(i)] == 0) {
            allHit = false;
            firstZero = i;
            break;
        }
    }
    if (allHit) {
        out.maxLianXuSector = count;
        out.youXiaoDuanSectorHe = count;
        out.youXiaoDuanShu = 1;
        return out;
    }

    int run = 0;
    auto finishRun = [&]() {
        if (run <= 0) return;
        out.maxLianXuSector = std::max(out.maxLianXuSector, run);
        if (run >= 2) {
            out.youXiaoDuanSectorHe += run;
            ++out.youXiaoDuanShu;
        }
        run = 0;
    };

    // 从一个空扇区之后开始线性扫描一整圈，避免跨 0° 的同一段被拆成两段。
    const int start = (firstZero + 1) % count;
    for (int k = 0; k < count; ++k) {
        const int idx = (start + k) % count;
        if (hits[static_cast<std::size_t>(idx)] != 0) ++run;
        else finishRun();
    }
    finishRun();
    return out;
}

/** 【类型导航注释】
 * KaiFangYuanHuTuoYuanZhengJu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct KaiFangYuanHuTuoYuanZhengJu {
    bool valid = false;
    int sectorCount = 0;
    double axisRatio = 1.0;
    double tiltU = 0.0;
    double tiltV = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：拟合/求解“solveSymmetric3x3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool solveSymmetric3x3(
    const std::array<std::array<double, 3>, 3>& inputMatrix,
    const std::array<double, 3>& inputRhs,
    std::array<double, 3>& solution)
{
    double augmented[3][4]{};
    double largestEntry = 0.0;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            augmented[row][column] = inputMatrix[static_cast<std::size_t>(row)]
                [static_cast<std::size_t>(column)];
            largestEntry = std::max(largestEntry, std::abs(augmented[row][column]));
        }
        augmented[row][3] = inputRhs[static_cast<std::size_t>(row)];
    }
    if (!(largestEntry > 1e-12)) return false;

    for (int pivot = 0; pivot < 3; ++pivot) {
        int bestRow = pivot;
        for (int row = pivot + 1; row < 3; ++row) {
            if (std::abs(augmented[row][pivot]) > std::abs(augmented[bestRow][pivot]))
                bestRow = row;
        }
        if (std::abs(augmented[bestRow][pivot]) <= 1e-7 * largestEntry) return false;
        if (bestRow != pivot) {
            for (int column = pivot; column < 4; ++column)
                std::swap(augmented[pivot][column], augmented[bestRow][column]);
        }
        const double pivotValue = augmented[pivot][pivot];
        for (int column = pivot; column < 4; ++column)
            augmented[pivot][column] /= pivotValue;
        for (int row = 0; row < 3; ++row) {
            if (row == pivot) continue;
            const double factor = augmented[row][pivot];
            for (int column = pivot; column < 4; ++column)
                augmented[row][column] -= factor * augmented[pivot][column];
        }
    }
    for (int row = 0; row < 3; ++row)
        solution[static_cast<std::size_t>(row)] = augmented[row][3];
    return finite(solution[0]) && finite(solution[1]) && finite(solution[2]);
}

// 这一步直接复用“开放孔候选”已经确定的第一边缘，不另起一套外部法向搜索。
// 当前候选圆心由“孔内清洁 + 第一边缘 + 点击归属”产生；椭圆只作为该候选的直接几何证据，
// 最终是否采用还会在簇级上口筛选和原始点几何复核中再次审核。这里解中心固定的椭圆二次型：
// 1/r² = q11*cos²(a) + 2*q12*cos(a)sin(a) + q22*sin²(a)。
// 三个系数一次线性求解即可得到长短轴及其方向；没有任何 ±角度 枚举。
/** 【函数导航】
 * 作用：拟合/求解“fitOpenArcCenteredEllipse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
KaiFangYuanHuTuoYuanZhengJu fitOpenArcCenteredEllipse(
    const std::vector<double>& nearest,
    const std::vector<double>& nearestAngle,
    double radius)
{
    KaiFangYuanHuTuoYuanZhengJu out;
    if (nearest.size() != nearestAngle.size() || nearest.size() < 12 || !(radius > 1.0))
        return out;

    const double halfWidth = std::clamp(0.18 * radius, 0.55, 0.95);
    std::array<std::array<double, 3>, 3> normal{};
    std::array<double, 3> rhs{};
    std::vector<std::pair<double, double>> observations;
    observations.reserve(nearest.size());
    for (std::size_t sector = 0; sector < nearest.size(); ++sector) {
        const double radial = nearest[sector];
        if (!finite(radial) || std::abs(radial - radius) > halfWidth) continue;
        const double angle = nearestAngle[sector];
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const std::array<double, 3> row{{
            cosine * cosine, 2.0 * cosine * sine, sine * sine}};
        const double value = 1.0 / (radial * radial);
        for (int r = 0; r < 3; ++r) {
            rhs[static_cast<std::size_t>(r)] += row[static_cast<std::size_t>(r)] * value;
            for (int c = 0; c < 3; ++c)
                normal[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]
                    += row[static_cast<std::size_t>(r)] * row[static_cast<std::size_t>(c)];
        }
        observations.emplace_back(angle, radial);
    }
    if (observations.size() < 9) return out;

    std::array<double, 3> q{};
    if (!solveSymmetric3x3(normal, rhs, q)) return out;
    const double q11 = q[0];
    const double q12 = q[1];
    const double q22 = q[2];
    const double traceHalf = 0.5 * (q11 + q22);
    const double spread = std::hypot(0.5 * (q11 - q22), q12);
    const double lambdaSmall = traceHalf - spread;
    const double lambdaLarge = traceHalf + spread;
    if (!(lambdaSmall > 1e-12) || !(lambdaLarge >= lambdaSmall)) return out;

    const double axisRatio = std::sqrt(std::clamp(lambdaSmall / lambdaLarge, 0.0, 1.0));
    const double majorRadius = 1.0 / std::sqrt(lambdaSmall);
    if (!(axisRatio >= 0.35 && axisRatio <= 1.0)
        || majorRadius < 0.55 * radius || majorRadius > 1.65 * radius) return out;

    double tiltU = 1.0;
    double tiltV = 0.0;
    if (std::abs(q12) > 1e-12) {
        tiltU = q12;
        tiltV = lambdaSmall - q11;
        const double norm = std::hypot(tiltU, tiltV);
        if (!(norm > 1e-12)) return out;
        tiltU /= norm;
        tiltV /= norm;
    } else if (q22 < q11) {
        tiltU = 0.0;
        tiltV = 1.0;
    }

    double squaredError = 0.0;
    for (const auto& observation : observations) {
        const double angle = observation.first;
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const double inverseRadiusSquared = q11 * cosine * cosine
            + 2.0 * q12 * cosine * sine + q22 * sine * sine;
        if (!(inverseRadiusSquared > 1e-12)) return out;
        const double predicted = 1.0 / std::sqrt(inverseRadiusSquared);
        const double difference = observation.second - predicted;
        squaredError += difference * difference;
    }
    const double rmse = std::sqrt(squaredError / static_cast<double>(observations.size()));
    if (rmse > std::max(0.40, 0.16 * radius)) return out;

    out.valid = true;
    out.sectorCount = static_cast<int>(observations.size());
    out.axisRatio = axisRatio;
    out.tiltU = tiltU;
    out.tiltV = tiltV;
    out.rmse = rmse;
    return out;
}


/** 【类型导航注释】
 * KaiFangYuanHuYuanZhengJu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct KaiFangYuanHuYuanZhengJu {
    bool valid = false;
    double centerU = 0.0;
    double centerV = 0.0;
    double radius = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    std::vector<std::pair<double, double>> edgePoints;
};

/** 【函数导航】
 * 作用：精修“refineOpenArcCircleFromFirstEdge”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
KaiFangYuanHuYuanZhengJu refineOpenArcCircleFromFirstEdge(
    const std::vector<double>& nearest,
    const std::vector<double>& nearestAngle,
    double initialCenterU,
    double initialCenterV,
    double initialRadius,
    double maxRadiusAllowed)
{
    KaiFangYuanHuYuanZhengJu out;
    if (nearest.size() != nearestAngle.size() || nearest.size() < 9 || !(initialRadius > 1.0))
        return out;
    const double halfWidth = std::clamp(0.12 * initialRadius, 0.42, 0.85);
    std::array<std::array<double, 3>, 3> normal{};
    std::array<double, 3> rhs{};
    out.edgePoints.reserve(nearest.size());
    for (std::size_t i = 0; i < nearest.size(); ++i) {
        const double radial = nearest[i];
        if (!finite(radial) || std::abs(radial - initialRadius) > halfWidth) continue;
        const double x = radial * std::cos(nearestAngle[i]);
        const double y = radial * std::sin(nearestAngle[i]);
        const std::array<double, 3> row{{x, y, 1.0}};
        const double value = -(x * x + y * y);
        for (int r = 0; r < 3; ++r) {
            rhs[static_cast<std::size_t>(r)] += row[static_cast<std::size_t>(r)] * value;
            for (int c = 0; c < 3; ++c)
                normal[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]
                    += row[static_cast<std::size_t>(r)] * row[static_cast<std::size_t>(c)];
        }
        out.edgePoints.emplace_back(initialCenterU + x, initialCenterV + y);
    }
    if (out.edgePoints.size() < 9) return out;
    std::array<double, 3> q{};
    if (!solveSymmetric3x3(normal, rhs, q)) return out;
    const double offsetU = -0.5 * q[0];
    const double offsetV = -0.5 * q[1];
    const double radiusSquared = offsetU * offsetU + offsetV * offsetV - q[2];
    if (!(radiusSquared > 1.0)) return out;
    const double radius = std::sqrt(radiusSquared);
    const double shift = std::hypot(offsetU, offsetV);
    if (!(radius >= 1.0 && radius <= maxRadiusAllowed) || shift > 2.5) return out;
    double squared = 0.0;
    const double refinedU = initialCenterU + offsetU;
    const double refinedV = initialCenterV + offsetV;
    for (const auto& point : out.edgePoints) {
        const double error = std::hypot(point.first - refinedU, point.second - refinedV) - radius;
        squared += error * error;
    }
    const double rmse = std::sqrt(squared / static_cast<double>(out.edgePoints.size()));
    if (rmse > std::max(0.38, 0.10 * radius)) return out;
    out.valid = true;
    out.centerU = refinedU;
    out.centerV = refinedV;
    out.radius = radius;
    out.rmse = rmse;
    return out;
}

/** 【函数导航】
 * 作用：拟合/求解“fitOpenArcCenteredEllipsePoints”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
KaiFangYuanHuTuoYuanZhengJu fitOpenArcCenteredEllipsePoints(
    const std::vector<std::pair<double, double>>& points,
    double centerU,
    double centerV,
    double radius)
{
    std::vector<double> radial;
    std::vector<double> angle;
    radial.reserve(points.size());
    angle.reserve(points.size());
    for (const auto& point : points) {
        const double du = point.first - centerU;
        const double dv = point.second - centerV;
        radial.push_back(std::hypot(du, dv));
        angle.push_back(std::atan2(dv, du));
    }
    return fitOpenArcCenteredEllipse(radial, angle, radius);
}

// 开放孔口回退：不从“封闭空洞面积”猜圆，而沿用正常孔内/孔外语义，
// 从候选中心向外读取每个方向的第一条有效边界。第一边界是开放候选的定义本身，
// 后续统一交给支撑面归属、多层稳定和点击归属审核，保持单一判定链。
/** 【函数导航】
 * 作用：执行“openArcCandidatesAtLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HouXuan> openArcCandidatesAtLayer(
    const std::vector<Sample>& samples,
    const Input& input,
    double layerW,
    int layerIndex)
{
    const double band = std::clamp(input.layerBand, 0.25, 0.65);
    const double extent = std::clamp(input.radialHalfExtent, 16.0, 50.0);
    const int sectorTotal = std::clamp(input.sectors, 24, 72);
    const double maxRadius = std::clamp(input.maxHoleRadius * 1.12, 2.0, 33.6);
    // 单阶段搜索：默认 10 mm 孔时把圆心可达范围由约 12 mm 温和放到约 13 mm。
    // 更大孔径模式按最大孔半径联动，但永远受当前 ROI 边界约束。
    const double searchSpan = std::min(
        std::max(8.0, maxRadius + std::max(1.8, 0.10 * maxRadius)),
        std::max(8.0, extent - 2.0));
    const double centerStep = 0.50; // 只在部分可见边缘生成时使用；0.5 mm 兼顾圆弧圆心精度与速度。

    std::vector<std::pair<double, double>> layerPoints;
    layerPoints.reserve(2048);
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        if (std::abs(sample.w - layerW) > band) continue;
        if (std::abs(sample.u - input.seedU) > extent
            || std::abs(sample.v - input.seedV) > extent) continue;
        layerPoints.emplace_back(sample.u, sample.v);
    }
    if (layerPoints.size() < 35) return {};

    // 开放弧回退只需要查看候选中心最大有效半径内的点。先按小网格分桶，
    // 每个候选中心仅扫描与其搜索半径相交的桶；这是精确空间裁剪，不改变几何结果。
    const double pointReach = maxRadius + 1.35;
    const double bucketSize = 4.0;
    const double bucketExtent = searchSpan + pointReach + bucketSize;
    const double bucketMinU = input.seedU - bucketExtent;
    const double bucketMinV = input.seedV - bucketExtent;
    const int bucketCols = std::max(1,
        static_cast<int>(std::ceil(2.0 * bucketExtent / bucketSize)) + 1);
    const int bucketRows = bucketCols;
    std::vector<std::vector<std::pair<double, double>>> pointBuckets(
        static_cast<std::size_t>(bucketCols * bucketRows));
    for (const auto& point : layerPoints) {
        const int bx = std::clamp(static_cast<int>(std::floor(
            (point.first - bucketMinU) / bucketSize)), 0, bucketCols - 1);
        const int by = std::clamp(static_cast<int>(std::floor(
            (point.second - bucketMinV) / bucketSize)), 0, bucketRows - 1);
        pointBuckets[static_cast<std::size_t>(by * bucketCols + bx)].push_back(point);
    }

    /** 【类型导航注释】
     * Ranked：Hole 分析实现中的自定义 结构体。
     * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Ranked { HouXuan candidate; double rank = -std::numeric_limits<double>::infinity(); };
    std::vector<Ranked> ranked;
    ranked.reserve(48);
    std::vector<double> nearest(static_cast<std::size_t>(sectorTotal));
    std::vector<double> nearestAngle(static_cast<std::size_t>(sectorTotal), 0.0);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(sectorTotal));
    std::vector<unsigned char> hits(static_cast<std::size_t>(sectorTotal));
    std::vector<double> ringRadii;
    ringRadii.reserve(static_cast<std::size_t>(sectorTotal));

    const double centerBaseU = std::round(input.seedU / centerStep) * centerStep;
    const double centerBaseV = std::round(input.seedV / centerStep) * centerStep;
    const int centerSteps = static_cast<int>(std::ceil(searchSpan / centerStep));
    for (int gy = -centerSteps; gy <= centerSteps; ++gy) {
        const double centerV = centerBaseV + static_cast<double>(gy) * centerStep;
        for (int gx = -centerSteps; gx <= centerSteps; ++gx) {
            const double centerU = centerBaseU + static_cast<double>(gx) * centerStep;
            const double seedCenterDistance = std::hypot(centerU - input.seedU, centerV - input.seedV);
            if (seedCenterDistance > searchSpan) continue;

            std::fill(nearest.begin(), nearest.end(), std::numeric_limits<double>::infinity());
            std::fill(nearestAngle.begin(), nearestAngle.end(), 0.0);
            const int minBx = std::clamp(static_cast<int>(std::floor(
                (centerU - pointReach - bucketMinU) / bucketSize)), 0, bucketCols - 1);
            const int maxBx = std::clamp(static_cast<int>(std::floor(
                (centerU + pointReach - bucketMinU) / bucketSize)), 0, bucketCols - 1);
            const int minBy = std::clamp(static_cast<int>(std::floor(
                (centerV - pointReach - bucketMinV) / bucketSize)), 0, bucketRows - 1);
            const int maxBy = std::clamp(static_cast<int>(std::floor(
                (centerV + pointReach - bucketMinV) / bucketSize)), 0, bucketRows - 1);
            for (int by = minBy; by <= maxBy; ++by) {
                for (int bx = minBx; bx <= maxBx; ++bx) {
                    // 整个空间桶若与候选中心的最大搜索圆都不相交，则桶内任意点都不可能参与。
                    // 这是严格几何裁剪，只减少无效遍历，不改变进入后续统计的点集合。
                    const double cellMinU = bucketMinU + static_cast<double>(bx) * bucketSize;
                    const double cellMaxU = cellMinU + bucketSize;
                    const double cellMinV = bucketMinV + static_cast<double>(by) * bucketSize;
                    const double cellMaxV = cellMinV + bucketSize;
                    const double nearestU = std::clamp(centerU, cellMinU, cellMaxU);
                    const double nearestV = std::clamp(centerV, cellMinV, cellMaxV);
                    const double cellDu = nearestU - centerU;
                    const double cellDv = nearestV - centerV;
                    if (cellDu * cellDu + cellDv * cellDv > pointReach * pointReach) continue;

                    const auto& bucket = pointBuckets[
                        static_cast<std::size_t>(by * bucketCols + bx)];
                    for (const auto& point : bucket) {
                        const double du = point.first - centerU;
                        const double dv = point.second - centerV;
                        const double radialSquared = du * du + dv * dv;
                        if (radialSquared < 0.45 * 0.45
                            || radialSquared > pointReach * pointReach) continue;
                        const double radial = std::sqrt(radialSquared);
                        double angle = std::atan2(dv, du);
                        if (angle < 0.0) angle += 2.0 * kPi;
                        int sector = static_cast<int>(std::floor(
                            angle / (2.0 * kPi) * sectorTotal));
                        sector = std::clamp(sector, 0, sectorTotal - 1);
                        double& first = nearest[static_cast<std::size_t>(sector)];
                        if (radial < first) {
                            first = radial;
                            nearestAngle[static_cast<std::size_t>(sector)] = angle;
                        }
                    }
                }
            }

            values.clear();
            for (double radial : nearest) {
                if (finite(radial) && radial >= 1.0 && radial <= maxRadius) values.push_back(radial);
            }
            if (values.size() < static_cast<std::size_t>(std::max(9, sectorTotal / 4))) continue;

            double bestRadius = 0.0;
            double bestRmse = std::numeric_limits<double>::infinity();
            double bestCoverage = 0.0;
            double bestContinuous = 0.0;
            double bestClean = 0.0;
            double bestFirst = 0.0;
            int bestHitCount = 0;
            double bestScore = -std::numeric_limits<double>::infinity();

            for (double seedRadius : values) {
                if (seedRadius < 1.0 || seedRadius > maxRadius) continue;
                const double halfWidth = std::clamp(0.12 * seedRadius, 0.42, 0.85);
                std::fill(hits.begin(), hits.end(), 0);
                ringRadii.clear();
                for (int sector = 0; sector < sectorTotal; ++sector) {
                    const double radial = nearest[static_cast<std::size_t>(sector)];
                    if (!finite(radial) || std::abs(radial - seedRadius) > halfWidth) continue;
                    hits[static_cast<std::size_t>(sector)] = 1;
                    ringRadii.push_back(radial);
                }
                const int hitCount = static_cast<int>(ringRadii.size());
                // 单段 90° 在 36 扇区时正好是 9 个扇区，因此预筛下限改为 25%。
                // 这里只改变残缺圆弧的角度资格，不改变后续圆拟合和物理审核。
                if (hitCount < std::max(9, static_cast<int>(std::ceil(0.25 * sectorTotal)))) continue;
                const double radius = median(ringRadii);
                if (!(radius >= 1.0 && radius <= maxRadius)) continue;
                double squared = 0.0;
                for (double radial : ringRadii) {
                    const double d = radial - radius;
                    squared += d * d;
                }
                const double rmse = std::sqrt(squared / static_cast<double>(ringRadii.size()));
                const double rmseLimit = std::max(0.24, 0.075 * radius);
                if (rmse > rmseLimit) continue;

                const double coverage = static_cast<double>(hitCount) / static_cast<double>(sectorTotal);
                const YuanHuLianXuTongJi arcStats = tongJiYuanHuLianXuDuan(hits);
                const double continuous = static_cast<double>(arcStats.maxLianXuSector)
                    / static_cast<double>(sectorTotal);

                // 开放/残缺圆弧只增加这一条周向资格：
                // A. 单段连续有效圆弧 >= 90°；
                // B. 至少 2 段有效连续圆弧，且这些连续段在同一候选圆上的合计角度 >= 180°。
                // 每段至少连续 2 个扇区，孤立点不能参加 B 的 180° 合计。
                // 角度直接由 sectorTotal 换算，36/72 扇区都遵守同一物理角度。
                const double sectorDegrees = 360.0 / static_cast<double>(sectorTotal);
                const bool singleArc90Valid =
                    static_cast<double>(arcStats.maxLianXuSector) * sectorDegrees
                    >= 90.0 - 1e-9;
                const bool multiArc180Valid = arcStats.youXiaoDuanShu >= 2
                    && static_cast<double>(arcStats.youXiaoDuanSectorHe) * sectorDegrees
                    >= 180.0 - 1e-9;
                if (!singleArc90Valid && !multiArc180Valid) continue;

                int cleanSectors = 0;
                int firstEdgeSectors = 0;
                const double coreRadius = std::max(0.75, 0.70 * radius);
                const double firstEdgeInner = std::max(0.55, radius - std::max(0.55, 0.16 * radius));
                for (int sector = 0; sector < sectorTotal; ++sector) {
                    const double radial = nearest[static_cast<std::size_t>(sector)];
                    if (!finite(radial) || radial >= coreRadius) ++cleanSectors;
                    if (hits[static_cast<std::size_t>(sector)] != 0
                        && radial >= firstEdgeInner) ++firstEdgeSectors;
                }
                const double clean = static_cast<double>(cleanSectors) / static_cast<double>(sectorTotal);
                const double first = hitCount > 0
                    ? static_cast<double>(firstEdgeSectors) / static_cast<double>(hitCount) : 0.0;
                if (clean < 0.78) continue;

                // 不再使用固定 1.50 mm 的 seed->孔沿硬门。直接完全删除该门的真实 1.51 回归会让
                // 一些 2~3 mm 局部弧进入前 12 个开放候选并扰动原有锥孔簇，因此改为纯尺度关系：
                // seed 落在预测孔沿外时，允许的外距等于该候选自身半径。大孔自然获得更远捕获范围，
                // 小局部弧不会因为 GUI 最大孔径很大就获得同样宽的远距资格；不再存在固定 1.5 mm 常数。
                const double outside = std::max(0.0, seedCenterDistance - radius);
                if (outside > radius) continue;
                const double clickCost = outside <= 0.0
                    ? 0.0 : outside + 0.05 * seedCenterDistance;
                // “第一边缘”已经是候选生成方式本身，不再额外给它重复加分。
                // 真正的候选质量由可见弧、连续性、孔内清洁度、拟合残差和点击归属共同决定。
                const double score = 4.0 * coverage + 2.4 * continuous + 2.8 * clean
                    - 3.5 * rmse - 0.18 * clickCost;
                if (score > bestScore) {
                    bestScore = score;
                    bestRadius = radius;
                    bestRmse = rmse;
                    bestCoverage = coverage;
                    bestContinuous = continuous;
                    bestClean = clean;
                    bestFirst = first;
                    bestHitCount = hitCount;
                }
            }

            if (!(bestRadius > 0.0)) continue;

            // 部分圆弧会让“中心网格”产生多个近邻假设。这里不靠扩大/缩小半径猜中心，
            // 而直接用已经确认的第一边缘点一次解圆方程，把同一真实孔的近邻假设收敛到同一圆心/半径。
            const KaiFangYuanHuYuanZhengJu refinedCircle = refineOpenArcCircleFromFirstEdge(
                nearest, nearestAngle, centerU, centerV, bestRadius, maxRadius);
            const double finalCenterU = refinedCircle.valid ? refinedCircle.centerU : centerU;
            const double finalCenterV = refinedCircle.valid ? refinedCircle.centerV : centerV;
            const double finalRadius = refinedCircle.valid ? refinedCircle.radius : bestRadius;
            const double finalRmse = refinedCircle.valid ? refinedCircle.rmse : bestRmse;

            // 扇区“看起来空”还不够：再直接统计孔内核心点与边缘环带点。
            // 真孔内部允许少量离散噪点，但不能像普通表面或随机缺口那样持续有点。
            const double supportHalfWidth = std::clamp(0.16 * finalRadius, 0.50, 0.95);
            const double coreRadius = std::max(0.75, 0.70 * finalRadius);
            int ringPointCount = 0;
            int corePointCount = 0;
            for (const auto& point : layerPoints) {
                const double radial = std::hypot(point.first - finalCenterU, point.second - finalCenterV);
                if (radial < coreRadius) ++corePointCount;
                if (std::abs(radial - finalRadius) <= supportHalfWidth) ++ringPointCount;
            }
            if (ringPointCount < std::max(18, 2 * bestHitCount)) continue;
            const double corePointRatio = static_cast<double>(corePointCount)
                / static_cast<double>(std::max(1, ringPointCount));
            if (corePointRatio > 0.14) continue;
            const double pointClean = std::clamp(1.0 - corePointRatio, 0.0, 1.0);
            bestClean = std::min(bestClean, pointClean);
            if (bestClean < 0.86) continue;

            HouXuan candidate;
            candidate.valid = true;
            candidate.layerIndex = layerIndex;
            candidate.layerW = layerW;
            candidate.centerU = finalCenterU;
            candidate.centerV = finalCenterV;
            candidate.radius = finalRadius;
            candidate.circularity = std::clamp(1.0 - finalRmse / std::max(0.5, finalRadius), 0.0, 1.0);
            candidate.coverage = bestCoverage;
            candidate.residual = finalRmse;
            candidate.voidArea = kPi * finalRadius * finalRadius * bestClean;
            candidate.wallSupport = bestCoverage * std::sqrt(static_cast<double>(bestHitCount));
            candidate.areaCells = 0;
            candidate.pointCount = ringPointCount;
            candidate.sectorCount = bestHitCount;
            candidate.touchesGrid = false;
            candidate.openArc = true;
            candidate.interiorCleanRatio = bestClean;
            candidate.firstEdgeRatio = bestFirst;
            candidate.continuousArcCoverage = bestContinuous;
            candidate.circleFitRmse = finalRmse;
            const KaiFangYuanHuTuoYuanZhengJu ellipse = refinedCircle.valid
                ? fitOpenArcCenteredEllipsePoints(
                    refinedCircle.edgePoints, finalCenterU, finalCenterV, finalRadius)
                : fitOpenArcCenteredEllipse(nearest, nearestAngle, finalRadius);
            candidate.ellipseValid = ellipse.valid;
            candidate.ellipseAxisRatio = ellipse.axisRatio;
            candidate.ellipseTiltU = ellipse.tiltU;
            candidate.ellipseTiltV = ellipse.tiltV;
            candidate.ellipseModelRmse = ellipse.valid ? ellipse.rmse : 0.0;
            candidate.ellipseSectorCount = ellipse.sectorCount;
            candidate.source = "OPEN_FIRST_EDGE_ARC";
            candidate.score = bestScore;
            ranked.push_back({candidate, bestScore});
        }
    }

    std::sort(ranked.begin(), ranked.end(), [](const Ranked& left, const Ranked& right) {
        if (left.rank != right.rank) return left.rank > right.rank;
        if (left.candidate.centerU != right.candidate.centerU)
            return left.candidate.centerU < right.candidate.centerU;
        return left.candidate.centerV < right.candidate.centerV;
    });
    std::vector<HouXuan> unique;
    unique.reserve(12);
    for (const Ranked& item : ranked) {
        bool duplicate = false;
        for (const HouXuan& existing : unique) {
            if (std::hypot(item.candidate.centerU - existing.centerU,
                           item.candidate.centerV - existing.centerV) < 0.85
                && std::abs(item.candidate.radius - existing.radius) < 0.65) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) unique.push_back(item.candidate);
        if (unique.size() >= 12) break;
    }
    return unique;
}

/** 【函数导航】
 * 作用：执行“stableClusterId”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int stableClusterId(double centerU, double centerV, double topW, double radius)
{
    const std::array<long long, 4> quantized{{
        std::llround(centerU * 2.0),
        std::llround(centerV * 2.0),
        std::llround(topW * 2.0),
        std::llround(radius * 2.0)}};
    std::uint64_t hash = 1469598103934665603ULL;
    for (long long value : quantized) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(value); ++i) {
            hash ^= static_cast<std::uint64_t>(bytes[i]);
            hash *= 1099511628211ULL;
        }
    }
    return static_cast<int>(hash & 0x7fffffffULL);
}

/** 【类型导航注释】
 * BingChaJi：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct BingChaJi {
    explicit BingChaJi(std::size_t count) : parent(count), rank(count, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    /** 【函数导航】
     * 作用：检测/搜索“find”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleFenxi_Analysis.h。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::size_t find(std::size_t value) {
        while (parent[value] != value) {
            parent[value] = parent[parent[value]];
            value = parent[value];
        }
        return value;
    }
    /** 【函数导航】
     * 作用：执行“unite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void unite(std::size_t left, std::size_t right) {
        left = find(left);
        right = find(right);
        if (left == right) return;
        if (rank[left] < rank[right]) std::swap(left, right);
        parent[right] = left;
        if (rank[left] == rank[right]) ++rank[left];
    }
    std::vector<std::size_t> parent;
    std::vector<unsigned char> rank;
};

/** 【类型导航注释】
 * CengGongZuoQu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CengGongZuoQu {
    std::vector<unsigned char> occupied;
    std::vector<unsigned char> plate;
    std::vector<unsigned char> filteredOccupied;
    std::vector<unsigned char> sectorHit;
    std::vector<int> label;
    std::vector<int> floodQueue;
    std::vector<std::pair<double, double>> layerPoints;
    std::vector<double> residuals;

    std::vector<int> touched;
    std::vector<int> touchedPrev;
    std::vector<int> occupiedCells;
    std::vector<int> plateCells;

    std::vector<std::uint64_t> plateBits;
    int plateMinX = 0;
    int plateMaxX = -1;
    int plateMinY = 0;
    int plateMaxY = -1;

    /** 【函数导航】
     * 作用：执行“prepareGrid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void prepareGrid(std::size_t cellCount) {
        occupied.assign(cellCount, 0);
        plate.resize(cellCount);
        filteredOccupied.resize(cellCount);
        label.assign(cellCount, -1);
        floodQueue.clear();
        layerPoints.clear();
        if (layerPoints.capacity() < 2048) layerPoints.reserve(2048);
    }

    /** 【函数导航】
     * 作用：执行“prepareGridSparse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void prepareGridSparse(std::size_t cellCount) {
        occupied.resize(cellCount);
        plate.resize(cellCount);
        filteredOccupied.resize(cellCount);
        label.resize(cellCount);
        touched.reserve(cellCount);
        touchedPrev.reserve(cellCount);
        floodQueue.reserve(cellCount);
        occupiedCells.reserve(4096);
        std::fill(occupied.begin(), occupied.end(), 0);
        std::fill(plate.begin(), plate.end(), 0);
        std::fill(filteredOccupied.begin(), filteredOccupied.end(), 0);
        std::fill(label.begin(), label.end(), -1);
        touched.clear();
        touchedPrev.clear();
        occupiedCells.clear();
        plateCells.clear();
        floodQueue.clear();
        layerPoints.clear();
        if (layerPoints.capacity() < 2048) layerPoints.reserve(2048);
        plateMinX = 0;
        plateMaxX = -1;
        plateMinY = 0;
        plateMaxY = -1;
    }

    /** 【函数导航】
     * 作用：执行“beginSparseLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void beginSparseLayer() {
        touchedPrev.swap(touched);
        for (int idx : touchedPrev) {
            occupied[static_cast<std::size_t>(idx)] = 0;
            plate[static_cast<std::size_t>(idx)] = 0;
            filteredOccupied[static_cast<std::size_t>(idx)] = 0;
            label[static_cast<std::size_t>(idx)] = -1;
        }
        touched.clear();
        occupiedCells.clear();
        plateCells.clear();
        floodQueue.clear();
        layerPoints.clear();
        plateMinX = 0;
        plateMaxX = -1;
        plateMinY = 0;
        plateMaxY = -1;
    }

    /** 【函数导航】
     * 作用：执行“touchSparse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void touchSparse(int idx) {
        touched.push_back(idx);
    }

    /** 【函数导航】
     * 作用：执行“prepareRing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 分析实现。
     * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void prepareRing(int sectorTotal) {
        sectorHit.assign(static_cast<std::size_t>(sectorTotal), 0);
        residuals.clear();
        if (residuals.capacity() < 256) residuals.reserve(256);
    }
};

/** 【函数导航】
 * 作用：执行“candidatesAtLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HouXuan> candidatesAtLayer(
    const std::vector<Sample>& samples,
    const Input& input,
    double gridCenterU,
    double gridCenterV,
    double layerW,
    int layerIndex,
    std::size_t sampleBegin,
    std::size_t sampleEnd,
    CengGongZuoQu& workspace)
{
    const double cell = std::clamp(input.cellSize, 0.15, 0.50);
    const double extent = std::clamp(input.radialHalfExtent, 16.0, 50.0);
    const double band = std::clamp(input.layerBand, 0.25, 0.65);
    const double maxCandidateRadius = std::clamp(input.maxHoleRadius * 1.12, 2.0, 33.6);
    const int grid = std::max(64, static_cast<int>(std::ceil(2.0 * extent / cell)));
    const int cellCount = grid * grid;
    workspace.prepareGrid(static_cast<std::size_t>(cellCount));

    auto& occupied = workspace.occupied;
    auto& plate = workspace.plate;
    auto& label = workspace.label;
    auto& floodQueue = workspace.floodQueue;
    auto& layerPoints = workspace.layerPoints;

    sampleBegin = std::min(sampleBegin, samples.size());
    sampleEnd = std::min(std::max(sampleEnd, sampleBegin), samples.size());
    for (std::size_t sampleIndex = sampleBegin; sampleIndex < sampleEnd; ++sampleIndex) {
        const Sample& sample = samples[sampleIndex];
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;

        if (std::abs(sample.w - layerW) > band) continue;
        const double du = sample.u - gridCenterU;
        const double dv = sample.v - gridCenterV;
        if (du < -extent || du >= extent || dv < -extent || dv >= extent) continue;
        const int x = static_cast<int>(std::floor((du + extent) / cell));
        const int y = static_cast<int>(std::floor((dv + extent) / cell));
        if (x < 0 || x >= grid || y < 0 || y >= grid) continue;
        occupied[static_cast<std::size_t>(y * grid + x)] = 1;
        layerPoints.emplace_back(sample.u, sample.v);
    }

    if (layerPoints.size() < 55) return {};


    std::copy(occupied.begin(), occupied.end(), plate.begin());
    for (int y = 0; y < grid; ++y) {
        for (int x = 0; x < grid; ++x) {
            if (!occupied[static_cast<std::size_t>(y * grid + x)]) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    const int yy = y + dy;
                    if (xx >= 0 && xx < grid && yy >= 0 && yy < grid)
                        plate[static_cast<std::size_t>(yy * grid + xx)] = 1;
                }
            }
        }
    }

    const std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    const std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};
    int nextLabel = 0;
    std::vector<HouXuan> candidates;

    for (int startY = 0; startY < grid; ++startY) {
        for (int startX = 0; startX < grid; ++startX) {
            const int start = startY * grid + startX;
            if (plate[static_cast<std::size_t>(start)]
                || label[static_cast<std::size_t>(start)] >= 0) continue;

            floodQueue.clear();
            floodQueue.push_back(start);
            std::size_t floodHead = 0;
            label[static_cast<std::size_t>(start)] = nextLabel;
            int area = 0;
            bool touches = false;
            double sumU = 0.0;
            double sumV = 0.0;
            double sumUU = 0.0;
            double sumVV = 0.0;
            double sumUV = 0.0;

            while (floodHead < floodQueue.size()) {
                const int index = floodQueue[floodHead++];

                const int y = index / grid;
                const int x = index - y * grid;
                const double localU = (static_cast<double>(x) + 0.5) * cell - extent;
                const double localV = (static_cast<double>(y) + 0.5) * cell - extent;
                ++area;
                sumU += localU;
                sumV += localV;
                sumUU += localU * localU;
                sumVV += localV * localV;
                sumUV += localU * localV;
                touches = touches || x == 0 || y == 0 || x + 1 == grid || y + 1 == grid;

                for (std::size_t neighbor = 0; neighbor < dx.size(); ++neighbor) {
                    const int xx = x + dx[neighbor];
                    const int yy = y + dy[neighbor];
                    if (xx < 0 || xx >= grid || yy < 0 || yy >= grid) continue;
                    const int next = yy * grid + xx;
                    if (plate[static_cast<std::size_t>(next)]
                        || label[static_cast<std::size_t>(next)] >= 0) continue;
                    label[static_cast<std::size_t>(next)] = nextLabel;
                    floodQueue.push_back(next);
                }
            }
            ++nextLabel;
            if (area < 28) continue;

            const double count = static_cast<double>(area);
            const double localCenterU = sumU / count;
            const double localCenterV = sumV / count;
            const double centerU = gridCenterU + localCenterU;
            const double centerV = gridCenterV + localCenterV;
            const double varU = std::max(0.0, sumUU / count - localCenterU * localCenterU);
            const double varV = std::max(0.0, sumVV / count - localCenterV * localCenterV);
            const double covUV = sumUV / count - localCenterU * localCenterV;
            const double trace = varU + varV;
            const double disc = std::sqrt(std::max(0.0,
                (varU - varV) * (varU - varV) + 4.0 * covUV * covUV));
            const double lambdaMax = 0.5 * (trace + disc);
            const double lambdaMin = 0.5 * (trace - disc);
            const double circularity = lambdaMax > 1e-9 ? lambdaMin / lambdaMax : 0.0;
            const double radius = std::sqrt(count * cell * cell / kPi)
                + std::sqrt(2.0) * cell;
            if (!finite(radius) || radius < 1.0 || radius > maxCandidateRadius || circularity < 0.48) continue;

            const double annulusHalfWidth = std::clamp(0.18 * radius, 0.65, 1.25);
            const int sectorTotal = std::clamp(input.sectors, 24, 72);
            workspace.prepareRing(sectorTotal);
            auto& sectorHit = workspace.sectorHit;
            auto& residuals = workspace.residuals;
            int ringPoints = 0;
            for (const auto& point : layerPoints) {
                const double du = point.first - centerU;
                const double dv = point.second - centerV;
                const double radial = std::hypot(du, dv);
                const double residual = std::abs(radial - radius);
                if (residual > annulusHalfWidth) continue;
                double angle = std::atan2(dv, du);
                if (angle < 0.0) angle += 2.0 * kPi;
                int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * sectorTotal));
                sector = std::clamp(sector, 0, sectorTotal - 1);
                sectorHit[static_cast<std::size_t>(sector)] = 1;
                residuals.push_back(residual);
                ++ringPoints;
            }
            const int sectorCount = static_cast<int>(std::count(sectorHit.begin(), sectorHit.end(), 1));
            const double coverage = static_cast<double>(sectorCount) / static_cast<double>(sectorTotal);
            const double residual = medianInPlace(residuals);
            if (sectorCount < 9 || coverage < 0.25 || ringPoints < 16) continue;
            if (touches && (circularity < 0.78 || coverage < 0.65
                || radius > 0.72 * input.maxHoleRadius)) continue;

            HouXuan candidate;
            candidate.valid = true;
            candidate.layerIndex = layerIndex;
            candidate.layerW = layerW;
            candidate.centerU = centerU;
            candidate.centerV = centerV;
            candidate.radius = radius;
            candidate.circularity = circularity;
            candidate.coverage = coverage;
            candidate.residual = residual;
            candidate.voidArea = count * cell * cell;
            candidate.wallSupport = coverage * std::sqrt(static_cast<double>(ringPoints));
            candidate.areaCells = area;
            candidate.pointCount = ringPoints;
            candidate.sectorCount = sectorCount;
            candidate.touchesGrid = touches;
            candidate.score = 4.0 * circularity + 3.0 * coverage
                + 0.15 * std::log1p(static_cast<double>(ringPoints))
                + 0.08 * radius - 0.60 * residual - (touches ? 0.4 : 0.0);
            candidates.push_back(candidate);
        }
    }


    std::sort(candidates.begin(), candidates.end(), [](const HouXuan& left, const HouXuan& right) {
        if (left.score != right.score) return left.score > right.score;
        if (left.centerU != right.centerU) return left.centerU < right.centerU;
        if (left.centerV != right.centerV) return left.centerV < right.centerV;
        return left.radius < right.radius;
    });
    std::vector<HouXuan> unique;
    for (const HouXuan& candidate : candidates) {
        bool duplicate = false;
        for (const HouXuan& existing : unique) {
            if (std::hypot(candidate.centerU - existing.centerU,
                           candidate.centerV - existing.centerV) < 0.70
                && std::abs(candidate.radius - existing.radius) < 0.45) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) unique.push_back(candidate);
        if (unique.size() >= 16) break;
    }
    return unique;
}

/** 【函数导航】
 * 作用：执行“candidatesAtLayerSparse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HouXuan> candidatesAtLayerSparse(
    const std::vector<Sample>& samples,
    const Input& input,
    double gridCenterU,
    double gridCenterV,
    double layerW,
    int layerIndex,
    std::size_t sampleBegin,
    std::size_t sampleEnd,
    CengGongZuoQu& workspace)
{
    const double cell = std::clamp(input.cellSize, 0.15, 0.50);
    const double extent = std::clamp(input.radialHalfExtent, 16.0, 50.0);
    const double band = std::clamp(input.layerBand, 0.25, 0.65);
    const double maxCandidateRadius = std::clamp(input.maxHoleRadius * 1.12, 2.0, 33.6);
    const int grid = std::max(64, static_cast<int>(std::ceil(2.0 * extent / cell)));
    workspace.beginSparseLayer();

    auto& occupied = workspace.occupied;
    auto& plate = workspace.plate;
    auto& label = workspace.label;
    auto& floodQueue = workspace.floodQueue;
    auto& layerPoints = workspace.layerPoints;
    auto& occupiedCells = workspace.occupiedCells;
    const int wordsPerRow = (grid + 63) / 64;
    if (kUseBitsetComponents) {
        const std::size_t wordCount = static_cast<std::size_t>(grid)
            * static_cast<std::size_t>(wordsPerRow);
        workspace.plateBits.resize(wordCount);
        std::fill(workspace.plateBits.begin(), workspace.plateBits.end(), 0ULL);
    }

    sampleBegin = std::min(sampleBegin, samples.size());
    sampleEnd = std::min(std::max(sampleEnd, sampleBegin), samples.size());
    for (std::size_t sampleIndex = sampleBegin; sampleIndex < sampleEnd; ++sampleIndex) {
        const Sample& sample = samples[sampleIndex];
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        if (std::abs(sample.w - layerW) > band) continue;
        const double du = sample.u - gridCenterU;
        const double dv = sample.v - gridCenterV;
        if (du < -extent || du >= extent || dv < -extent || dv >= extent) continue;
        const int x = static_cast<int>(std::floor((du + extent) / cell));
        const int y = static_cast<int>(std::floor((dv + extent) / cell));
        if (x < 0 || x >= grid || y < 0 || y >= grid) continue;
        const int idx = y * grid + x;
        if (!occupied[static_cast<std::size_t>(idx)]) {
            occupied[static_cast<std::size_t>(idx)] = 1;
            workspace.touchSparse(idx);
            occupiedCells.push_back(idx);
        }
        layerPoints.emplace_back(sample.u, sample.v);
    }

    if (layerPoints.size() < 55) {

        return {};
    }


    for (int idx : occupiedCells) {
        if (!occupied[static_cast<std::size_t>(idx)]) continue;
        const int y = idx / grid;
        const int x = idx - y * grid;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int xx = x + dx;
                const int yy = y + dy;
                if (xx < 0 || xx >= grid || yy < 0 || yy >= grid) continue;
                const int nidx = yy * grid + xx;
                if (!plate[static_cast<std::size_t>(nidx)]) {
                    plate[static_cast<std::size_t>(nidx)] = 1;
                    if (kUseBitsetComponents) {
                        workspace.plateBits[static_cast<std::size_t>(yy)
                            * static_cast<std::size_t>(wordsPerRow)
                            + static_cast<std::size_t>(xx >> 6)]
                            |= (1ULL << static_cast<unsigned>(xx & 63));
                    }
                    workspace.touchSparse(nidx);
                    workspace.plateCells.push_back(nidx);
                    if (xx < workspace.plateMinX) workspace.plateMinX = xx;
                    if (xx > workspace.plateMaxX) workspace.plateMaxX = xx;
                    if (yy < workspace.plateMinY) workspace.plateMinY = yy;
                    if (yy > workspace.plateMaxY) workspace.plateMaxY = yy;
                }
            }
        }
    }

    const std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    const std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};
    int nextLabel = 0;
    std::vector<HouXuan> candidates;

    const double hugeRadius = std::max(1.0, maxCandidateRadius - std::sqrt(2.0) * cell);
    const double hugeArea = std::floor(
        kPi * hugeRadius * hugeRadius / (cell * cell)) + 1.0;
    /** 【类型导航注释】
     * XiShuDuan：Hole 分析实现中的自定义 结构体。
     * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct XiShuDuan { int x0; int x1; int comp; };
    /** 【类型导航注释】
     * XiShuLianTong：Hole 分析实现中的自定义 结构体。
     * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct XiShuLianTong { int area = 0; int startX = -1; int startY = -1; };
    std::vector<XiShuDuan> prevRuns;
    std::vector<XiShuDuan> curRuns;
    std::vector<XiShuLianTong> comps;
    prevRuns.reserve(16);
    curRuns.reserve(16);
    for (int y = 0; y < grid; ++y) {
        curRuns.clear();
        if (!kUseBitsetComponents) {
            int x = 0;
            while (x < grid) {
                if (plate[static_cast<std::size_t>(y * grid + x)]) { ++x; continue; }
                const int x0 = x;
                while (x < grid
                    && !plate[static_cast<std::size_t>(y * grid + x)]) ++x;
                curRuns.push_back({x0, x - 1, -1});
            }
        } else {

            int runStart = -1;
            for (int wordIndex = 0; wordIndex < wordsPerRow; ++wordIndex) {
                const int baseX = wordIndex * 64;
                const int validBits = std::min(64, grid - baseX);
                const std::uint64_t validMask = validBits == 64
                    ? ~0ULL
                    : ((1ULL << static_cast<unsigned>(validBits)) - 1ULL);
                const std::uint64_t plateWord = workspace.plateBits[
                    static_cast<std::size_t>(y)
                    * static_cast<std::size_t>(wordsPerRow)
                    + static_cast<std::size_t>(wordIndex)] & validMask;
                const std::uint64_t emptyMask = (~plateWord) & validMask;
                if (emptyMask == 0ULL) {
                    if (runStart >= 0) {
                        curRuns.push_back({runStart, baseX - 1, -1});
                        runStart = -1;
                    }
                    continue;
                }
                if (emptyMask == validMask) {
                    if (runStart < 0) runStart = baseX;
                    continue;
                }
                for (int bit = 0; bit < validBits; ++bit) {
                    const int x = baseX + bit;
                    const bool empty = (emptyMask
                        & (1ULL << static_cast<unsigned>(bit))) != 0ULL;
                    if (empty) {
                        if (runStart < 0) runStart = x;
                    } else if (runStart >= 0) {
                        curRuns.push_back({runStart, x - 1, -1});
                        runStart = -1;
                    }
                }
            }
            if (runStart >= 0) curRuns.push_back({runStart, grid - 1, -1});
        }
        for (XiShuDuan& run : curRuns) {
            int best = -1;
            for (XiShuDuan& prev : prevRuns) {
                if (prev.x0 <= run.x1 + 1 && run.x0 <= prev.x1 + 1) {
                    const int candidate = prev.comp;
                    if (candidate < 0) continue;
                    if (best < 0) {
                        best = candidate;
                    } else if (candidate != best) {
                        XiShuLianTong& target = comps[static_cast<std::size_t>(best)];
                        XiShuLianTong& source = comps[static_cast<std::size_t>(candidate)];
                        target.area += source.area;
                        if (source.startY < target.startY
                            || (source.startY == target.startY
                                && source.startX < target.startX)) {
                            target.startX = source.startX;
                            target.startY = source.startY;
                        }
                        source.area = -1;
                        for (XiShuDuan& prevRelabel : prevRuns) {
                            if (prevRelabel.comp == candidate)
                                prevRelabel.comp = best;
                        }
                        for (XiShuDuan& curRelabel : curRuns) {
                            if (curRelabel.comp == candidate)
                                curRelabel.comp = best;
                        }
                    }
                }
            }
            if (best < 0) {
                best = static_cast<int>(comps.size());
                comps.push_back(XiShuLianTong{});
                comps[static_cast<std::size_t>(best)].startX = run.x0;
                comps[static_cast<std::size_t>(best)].startY = y;
            }
            run.comp = best;
            comps[static_cast<std::size_t>(best)].area += run.x1 - run.x0 + 1;
        }
        prevRuns.swap(curRuns);
    }


    for (std::size_t ci = 0; ci < comps.size(); ++ci) {
        const XiShuLianTong& comp = comps[ci];
        if (comp.area < 28 || static_cast<double>(comp.area) >= hugeArea) continue;
        const int start = comp.startY * grid + comp.startX;
        if (label[static_cast<std::size_t>(start)] >= 0) continue;

        floodQueue.clear();
        floodQueue.push_back(start);
        std::size_t floodHead = 0;
        label[static_cast<std::size_t>(start)] = nextLabel;
        workspace.touchSparse(start);
        int area = 0;
        bool touches = false;
        double sumU = 0.0;
        double sumV = 0.0;
        double sumUU = 0.0;
        double sumVV = 0.0;
        double sumUV = 0.0;
        while (floodHead < floodQueue.size()) {
            const int index = floodQueue[floodHead++];

            const int y = index / grid;
            const int x = index - y * grid;
            const double localU = (static_cast<double>(x) + 0.5) * cell - extent;
            const double localV = (static_cast<double>(y) + 0.5) * cell - extent;
            ++area;
            sumU += localU;
            sumV += localV;
            sumUU += localU * localU;
            sumVV += localV * localV;
            sumUV += localU * localV;
            touches = touches || x == 0 || y == 0
                || x + 1 == grid || y + 1 == grid;
            for (std::size_t neighbor = 0; neighbor < dx.size(); ++neighbor) {
                const int xx = x + dx[neighbor];
                const int yy = y + dy[neighbor];
                if (xx < 0 || xx >= grid || yy < 0 || yy >= grid) continue;
                const int next = yy * grid + xx;
                if (plate[static_cast<std::size_t>(next)]
                    || label[static_cast<std::size_t>(next)] >= 0) continue;
                label[static_cast<std::size_t>(next)] = nextLabel;
                workspace.touchSparse(next);
                floodQueue.push_back(next);
            }
        }
        ++nextLabel;
        if (area < 28) continue;

        const double count = static_cast<double>(area);
        const double localCenterU = sumU / count;
        const double localCenterV = sumV / count;
        const double centerU = gridCenterU + localCenterU;
        const double centerV = gridCenterV + localCenterV;
        const double varU = std::max(0.0, sumUU / count - localCenterU * localCenterU);
        const double varV = std::max(0.0, sumVV / count - localCenterV * localCenterV);
        const double covUV = sumUV / count - localCenterU * localCenterV;
        const double trace = varU + varV;
        const double disc = std::sqrt(std::max(0.0,
            (varU - varV) * (varU - varV) + 4.0 * covUV * covUV));
        const double lambdaMax = 0.5 * (trace + disc);
        const double lambdaMin = 0.5 * (trace - disc);
        const double circularity = lambdaMax > 1e-9 ? lambdaMin / lambdaMax : 0.0;
        const double radius = std::sqrt(count * cell * cell / kPi)
            + std::sqrt(2.0) * cell;
        if (!finite(radius) || radius < 1.0 || radius > maxCandidateRadius || circularity < 0.48) continue;

        const double annulusHalfWidth = std::clamp(0.18 * radius, 0.65, 1.25);
        const int sectorTotal = std::clamp(input.sectors, 24, 72);
        workspace.prepareRing(sectorTotal);
        auto& sectorHit = workspace.sectorHit;
        auto& residuals = workspace.residuals;
        int ringPoints = 0;
        for (const auto& point : layerPoints) {
            const double du = point.first - centerU;
            const double dv = point.second - centerV;
            const double radial = std::hypot(du, dv);
            const double residual = std::abs(radial - radius);
            if (residual > annulusHalfWidth) continue;
            double angle = std::atan2(dv, du);
            if (angle < 0.0) angle += 2.0 * kPi;
            int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * sectorTotal));
            sector = std::clamp(sector, 0, sectorTotal - 1);
            sectorHit[static_cast<std::size_t>(sector)] = 1;
            residuals.push_back(residual);
            ++ringPoints;
        }
        const int sectorCount = static_cast<int>(std::count(sectorHit.begin(), sectorHit.end(), 1));
        const double coverage = static_cast<double>(sectorCount) / static_cast<double>(sectorTotal);
        const double residual = medianInPlace(residuals);
        if (sectorCount < 9 || coverage < 0.25 || ringPoints < 16) continue;
        if (touches && (circularity < 0.78 || coverage < 0.65
            || radius > 0.72 * input.maxHoleRadius)) continue;

        HouXuan candidate;
        candidate.valid = true;
        candidate.layerIndex = layerIndex;
        candidate.layerW = layerW;
        candidate.centerU = centerU;
        candidate.centerV = centerV;
        candidate.radius = radius;
        candidate.circularity = circularity;
        candidate.coverage = coverage;
        candidate.residual = residual;
        candidate.voidArea = count * cell * cell;
        candidate.wallSupport = coverage * std::sqrt(static_cast<double>(ringPoints));
        candidate.areaCells = area;
        candidate.pointCount = ringPoints;
        candidate.sectorCount = sectorCount;
        candidate.touchesGrid = touches;
        candidate.score = 4.0 * circularity + 3.0 * coverage
            + 0.15 * std::log1p(static_cast<double>(ringPoints))
            + 0.08 * radius - 0.60 * residual - (touches ? 0.4 : 0.0);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), [](const HouXuan& left, const HouXuan& right) {
        if (left.score != right.score) return left.score > right.score;
        if (left.centerU != right.centerU) return left.centerU < right.centerU;
        if (left.centerV != right.centerV) return left.centerV < right.centerV;
        return left.radius < right.radius;
    });
    std::vector<HouXuan> unique;
    for (const HouXuan& candidate : candidates) {
        bool duplicate = false;
        for (const HouXuan& existing : unique) {
            if (std::hypot(candidate.centerU - existing.centerU,
                           candidate.centerV - existing.centerV) < 0.70
                && std::abs(candidate.radius - existing.radius) < 0.45) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) unique.push_back(candidate);
        if (unique.size() >= 16) break;
    }

    return unique;
}

/** 【类型导航注释】
 * CengBingXingRenWu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CengBingXingRenWu {
    double layerW = 0.0;
    int layerIndex = 0;
    std::size_t sampleBegin = 0;
    std::size_t sampleEnd = 0;
};



}

/** 【函数导航】
 * 作用：拟合/求解“fitResolvedMouthEllipse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleKouEllipseEvidence fitResolvedMouthEllipse(
    const std::vector<Sample>& samples,
    double centerU, double centerV, double topW, double topRadius)
{
    HoleKouEllipseEvidence out;
    if (samples.size() < 40 || !finite(centerU) || !finite(centerV)
        || !finite(topW) || !(topRadius > 1.0)) return out;

    constexpr int sectorTotal = 36;
    std::vector<double> nearest(static_cast<std::size_t>(sectorTotal),
        std::numeric_limits<double>::infinity());
    std::vector<double> nearestAngle(static_cast<std::size_t>(sectorTotal), 0.0);
    const double layerBand = std::clamp(0.10 * topRadius, 0.28, 0.48);
    const double maximumRadial = std::max(topRadius + 1.7, 1.42 * topRadius);
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        if (std::abs(sample.w - topW) > layerBand) continue;
        const double du = sample.u - centerU;
        const double dv = sample.v - centerV;
        const double radial = std::hypot(du, dv);
        if (radial < 0.45 || radial > maximumRadial) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(sectorTotal)));
        sector = std::clamp(sector, 0, sectorTotal - 1);
        double& first = nearest[static_cast<std::size_t>(sector)];
        if (radial < first) {
            first = radial;
            nearestAngle[static_cast<std::size_t>(sector)] = angle;
        }
    }

    const KaiFangYuanHuTuoYuanZhengJu ellipse = fitOpenArcCenteredEllipse(
        nearest, nearestAngle, topRadius);
    if (!ellipse.valid) return out;
    out.valid = true;
    out.sectorCount = ellipse.sectorCount;
    out.coverage = static_cast<double>(ellipse.sectorCount)
        / static_cast<double>(sectorTotal);
    out.axisRatio = ellipse.axisRatio;
    out.tiltU = ellipse.tiltU;
    out.tiltV = ellipse.tiltV;
    out.modelRmse = ellipse.rmse;
    return out;
}

/** 【函数导航】
 * 作用：评估/审核“evaluateImpl”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluateImpl(const std::vector<Sample>& samples, const Input& input,
                    bool sparse)
{
    Result result;

    result.rawSeedCandidateGateUsed = false;
    result.rawSeedCandidateScoreUsed = false;
    result.rawSeedClusterGeometryUsed = false;
    result.rawSeedClusterSelectUsed = true;

    if (samples.size() < 80 || !finite(input.seedU) || !finite(input.seedV)
        || !finite(input.seedW)) {
        result.exitCode = "CanonicalSearch_INVALID_INPUT";
        result.reason = "insufficient coarse samples or invalid seed coordinates";
        return result;
    }
    const double histogramSupportPlaneW = estimateDominantSupportPlaneW(samples);
    const double seedLocalSupportPlaneW = estimateSeedLocalSupportPlaneW(
        samples, input.seedU, input.seedV, input.radialHalfExtent, histogramSupportPlaneW);

    result.supportPlaneHistogramW = histogramSupportPlaneW;

    const double step = std::clamp(input.layerStep, 0.18, 0.60);
    const double halfDepth = std::clamp(input.axialHalfExtent, 6.0, 20.0);

    const double phaseCell = std::clamp(input.cellSize, 0.15, 0.50);
    const double gridCenterU = std::floor(input.seedU / phaseCell) * phaseCell;
    const double gridCenterV = std::floor(input.seedV / phaseCell) * phaseCell;
    const double startW = std::floor((input.seedW - halfDepth) / step) * step;
    const double endW = input.seedW + halfDepth;
    const std::vector<Sample>* orderedSamples = &samples;
    std::vector<Sample> sortedSamples;
    bool monotonicW = true;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].w < samples[i - 1].w) { monotonicW = false; break; }
    }
    if (!monotonicW) {
        sortedSamples = samples;
        std::stable_sort(sortedSamples.begin(), sortedSamples.end(),
            [](const Sample& left, const Sample& right) {
                if (left.w != right.w) return left.w < right.w;
                if (left.u != right.u) return left.u < right.u;
                if (left.v != right.v) return left.v < right.v;
                return left.originalIndex < right.originalIndex;
            });
        orderedSamples = &sortedSamples;
    }

    const double band = std::clamp(input.layerBand, 0.25, 0.65);
    std::size_t sampleBegin = 0;
    std::size_t sampleEnd = 0;
    int layerIndex = 0;
    CengGongZuoQu layerWorkspace;
    if (sparse) {
        const double sparseExtent = std::clamp(input.radialHalfExtent, 16.0, 50.0);
        const double sparseCell = std::clamp(input.cellSize, 0.15, 0.50);
        const int sparseGrid = std::max(
            64, static_cast<int>(std::ceil(2.0 * sparseExtent / sparseCell)));
        layerWorkspace.prepareGridSparse(
            static_cast<std::size_t>(sparseGrid * sparseGrid));
    }
    std::vector<CengBingXingRenWu> layerTasks;
    if (sparse && kLayerWorkerCount > 1) layerTasks.reserve(256);

    for (double layerW = startW; layerW <= endW + 0.25 * step; layerW += step, ++layerIndex) {

        const double lowerW = layerW - band;
        const double upperW = layerW + band;
        while (sampleBegin < orderedSamples->size()
            && (*orderedSamples)[sampleBegin].w < lowerW) ++sampleBegin;
        if (sampleEnd < sampleBegin) sampleEnd = sampleBegin;
        while (sampleEnd < orderedSamples->size()
            && (*orderedSamples)[sampleEnd].w <= upperW) ++sampleEnd;

        if (sampleEnd - sampleBegin < 55) continue;

        if (sparse && kLayerWorkerCount > 1) {
            layerTasks.push_back(
                CengBingXingRenWu{layerW, layerIndex, sampleBegin, sampleEnd});
            continue;
        }

        std::vector<HouXuan> layer = sparse
            ? candidatesAtLayerSparse(
                  *orderedSamples, input, gridCenterU, gridCenterV, layerW,
                  layerIndex, sampleBegin, sampleEnd, layerWorkspace)
            : candidatesAtLayer(
                  *orderedSamples, input, gridCenterU, gridCenterV, layerW,
                  layerIndex, sampleBegin, sampleEnd, layerWorkspace);
        result.candidates.insert(result.candidates.end(), layer.begin(), layer.end());
    }

    if (!layerTasks.empty()) {
        const int workerCount = std::min(
            kLayerWorkerCount, static_cast<int>(layerTasks.size()));
        std::vector<std::vector<HouXuan>> perLayer(layerTasks.size());

        const double sparseExtent = std::clamp(
            input.radialHalfExtent, 16.0, 50.0);
        const double sparseCell = std::clamp(input.cellSize, 0.15, 0.50);
        const int sparseGrid = std::max(
            64, static_cast<int>(std::ceil(
                2.0 * sparseExtent / sparseCell)));
        const std::size_t sparseCellCount = static_cast<std::size_t>(
            sparseGrid * sparseGrid);


        std::atomic<std::size_t> nextTask{0};
        std::vector<std::thread> workers;
        workers.reserve(static_cast<std::size_t>(workerCount));

        for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex) {
            workers.emplace_back([&, sparseCellCount]() {
                CengGongZuoQu workerWorkspace;
                workerWorkspace.prepareGridSparse(sparseCellCount);
                for (;;) {
                    const std::size_t taskIndex =
                        nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= layerTasks.size()) break;
                    const CengBingXingRenWu& task = layerTasks[taskIndex];
                    perLayer[taskIndex] = candidatesAtLayerSparse(
                        *orderedSamples, input, gridCenterU, gridCenterV,
                        task.layerW, task.layerIndex,
                        task.sampleBegin, task.sampleEnd, workerWorkspace);
                }

            });
        }
        for (std::thread& worker : workers) worker.join();

        for (std::vector<HouXuan>& layer : perLayer) {
            result.candidates.insert(
                result.candidates.end(), layer.begin(), layer.end());
        }
    }

    // 闭合圆与开放圆弧属于同一个孔口候选系统。“残缺”只表示可见度下降，
    // 不代表另一种机械孔型或另一套最终识别算法。开放弧能否省略只由“是否已经有明确的
    // 外表面完整口沿证据”决定：深层内喉、小截面或旁孔都不能阻止开放弧生成。
    bool hasUnambiguousClosedMouthEvidence = false;
    for (const HouXuan& candidate : result.candidates) {
            if (!candidate.valid || candidate.openArc || !(candidate.radius > 0.0)) continue;
            const double radialDistance = std::hypot(
                input.seedU - candidate.centerU, input.seedV - candidate.centerV);
            const bool seedAssociated = seedBelongsToMouth(
                radialDistance, candidate.radius, input.cellSize, input.maxHoleRadius);
            const double supportBand = std::max(1.25, 0.28 * candidate.radius);
            // “是否已经有明确完整孔口”必须相对当前点击附近的真实外表面判断，
            // 不能只看全 ROI 点数最多的深层/底部平面。对于完整孔，局部外表面完整圆
            // 已经是这同一孔口最充分的证据，开放弧只是它的子集，没有必要重复生成；
            // 对严重残缺孔或旁孔，seedAssociated/完整度仍过不了，因此开放弧照常参与。
            const double outerSupportReferenceW = input.seedWIsOuterSurfaceAnchor
                ? std::max({histogramSupportPlaneW, seedLocalSupportPlaneW, input.seedW})
                : std::max(histogramSupportPlaneW, seedLocalSupportPlaneW);
            const bool nearOuterSupport =
                std::abs(candidate.layerW - outerSupportReferenceW) <= supportBand;
            const bool sufficientlyComplete = candidate.coverage >= 0.62
                && candidate.residual <= std::max(0.45, 0.10 * candidate.radius);
            if (seedAssociated && nearOuterSupport && sufficientlyComplete) {
                hasUnambiguousClosedMouthEvidence = true;
                break;
            }
        }
    const bool runOpenArc = input.enableOpenArc
        && (result.candidates.empty() || !hasUnambiguousClosedMouthEvidence);
    if (runOpenArc) {
        result.openArcSearchUsed = true;
        const std::array<double, 6> openLayerOffsets{{-1.25, -0.75, -0.25, 0.25, 0.75, 1.25}};

        // 开放/部分可见口沿围绕两个“同一外表面”的可能轴向锚点取层：
        // 1) 全 ROI 的主密度层；2) 点击附近局部主层。
        // 两者相近时只算一次；相差明显时同时保留，让后续统一簇/真实孔壁审核决定哪个才是孔口。
        // 这解决“残缺上口点少、孔底点多”时主直方图被深层劫持的问题，而不是降低圆弧质量门。
        std::array<double, 2> openSupportAnchors{{histogramSupportPlaneW, seedLocalSupportPlaneW}};
        int openAnchorCount = 1;
        if (input.seedWIsOuterSurfaceAnchor) {
            // GUI 正式链已经把点击深度投影到独立外表面；开放弧只围绕这个外层强支撑面取层。
            // 不能再把“点击附近的深层强峰”作为第二个锚点，否则孔底半圆会重新获得上口资格。
            openSupportAnchors[0] = std::max(
                {histogramSupportPlaneW, seedLocalSupportPlaneW, input.seedW});
            openAnchorCount = 1;
        } else if (std::abs(seedLocalSupportPlaneW - histogramSupportPlaneW) > 0.75) {
            openAnchorCount = 2;
        }

        /** 【类型导航注释】
         * KaiFangYuanHuCengRenWu：Hole 分析实现中的自定义 结构体。
         * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
         * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
         */
        struct KaiFangYuanHuCengRenWu {
            double layerW = 0.0;
            int layerIndex = 0;
        };
        std::vector<KaiFangYuanHuCengRenWu> openTasks;
        openTasks.reserve(static_cast<std::size_t>(openAnchorCount) * openLayerOffsets.size());
        int openLayerIndex = layerIndex + 1000;
        for (int anchorIndex = 0; anchorIndex < openAnchorCount; ++anchorIndex) {
            const double anchorW = openSupportAnchors[static_cast<std::size_t>(anchorIndex)];
            for (double offset : openLayerOffsets)
                openTasks.push_back({anchorW + offset, openLayerIndex++});
        }

        // 各轴向层互不写共享几何状态，可并行生成；结果仍按任务原顺序合并，保证确定性。
        std::vector<std::vector<HouXuan>> openPerLayer(openTasks.size());
        const int openWorkerCount = std::min(
            kLayerWorkerCount, static_cast<int>(openTasks.size()));
        std::atomic<std::size_t> nextOpenTask{0};
        std::vector<std::thread> openWorkers;
        openWorkers.reserve(static_cast<std::size_t>(openWorkerCount));
        for (int workerIndex = 0; workerIndex < openWorkerCount; ++workerIndex) {
            openWorkers.emplace_back([&]() {
                for (;;) {
                    const std::size_t taskIndex =
                        nextOpenTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= openTasks.size()) break;
                    const KaiFangYuanHuCengRenWu& task = openTasks[taskIndex];
                    openPerLayer[taskIndex] = openArcCandidatesAtLayer(
                        *orderedSamples, input, task.layerW, task.layerIndex);
                }
            });
        }
        for (std::thread& worker : openWorkers) worker.join();
        for (std::vector<HouXuan>& open : openPerLayer) {
            result.openArcCandidateCount += static_cast<int>(open.size());
            result.candidates.insert(result.candidates.end(), open.begin(), open.end());
        }
    }

    result.candidateCount = static_cast<int>(result.candidates.size());
    if (result.candidates.empty()) {
        result.exitCode = input.enableOpenArc
            ? "CanonicalSearch_NO_CANDIDATES_WITH_OPEN_ARC" : "CanonicalSearch_NO_CANDIDATES";
        result.reason = input.enableOpenArc
            ? "closed void and first-edge open-arc searches produced no reliable mouth candidates"
            : "seed-independent coarse scan produced no circular void candidates";
        return result;
    }
    BingChaJi sets(result.candidates.size());
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        const HouXuan& left = result.candidates[i];
        for (std::size_t j = i + 1; j < result.candidates.size(); ++j) {
            const HouXuan& right = result.candidates[j];
            const double centerDistance = std::hypot(
                left.centerU - right.centerU, left.centerV - right.centerV);
            const double layerDistance = std::abs(left.layerW - right.layerW);
            const double radiusDistance = std::abs(left.radius - right.radius);

            // 圆弧候选在同一切层会产生多个“圆心/半径假设”，它们不能相互连接后
            // 再被当成多层证据。开放圆弧只允许跨真实高度层建立孔口族，而且半径必须连续。
            // 闭合孔仍使用原有宽松聚类范围，保证完整孔既有行为不被改变。
            const bool eitherOpenArc = left.openArc || right.openArc;
            const bool distinctEvidenceLayers =
                !eitherOpenArc || left.layerIndex != right.layerIndex;
            const double centerLimit = eitherOpenArc
                ? std::max(0.90, 0.14 * (left.radius + right.radius))
                : std::max(1.10, 0.12 * (left.radius + right.radius));
            const double radiusLimit = eitherOpenArc
                ? std::max(0.80, 0.14 * std::max(left.radius, right.radius))
                : std::max(2.4, 0.38 * std::max(left.radius, right.radius));
            if (distinctEvidenceLayers
                && centerDistance <= centerLimit && layerDistance <= 1.55
                && radiusDistance <= radiusLimit) {
                sets.unite(i, j);
            }
        }
    }


    std::vector<std::vector<int>> groups;
    std::vector<std::size_t> roots;
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        const std::size_t root = sets.find(i);
        auto found = std::find(roots.begin(), roots.end(), root);
        if (found == roots.end()) {
            roots.push_back(root);
            groups.push_back({static_cast<int>(i)});
        } else {
            groups[static_cast<std::size_t>(std::distance(roots.begin(), found))]
                .push_back(static_cast<int>(i));
        }
    }
    for (const std::vector<int>& group : groups) {
        if (group.empty()) continue;
        Cluster cluster;
        cluster.candidateIndices = group;
        cluster.candidateCount = static_cast<int>(group.size());
        std::vector<double> centersU;
        std::vector<double> centersV;
        std::vector<double> radii;
        std::vector<double> layersW;
        std::vector<double> coverages;
        std::vector<double> residuals;
        std::vector<double> interiorCleanRatios;
        std::vector<double> firstEdgeRatios;
        std::vector<double> continuousArcCoverages;
        std::vector<const HouXuan*> openArcCandidates;
        std::vector<const HouXuan*> openArcMouthRepresentatives;
        std::set<int> layerIds;
        centersU.reserve(group.size());
        centersV.reserve(group.size());
        radii.reserve(group.size());
        layersW.reserve(group.size());
        for (int index : group) {
            const HouXuan& candidate = result.candidates[static_cast<std::size_t>(index)];
            centersU.push_back(candidate.centerU);
            centersV.push_back(candidate.centerV);
            radii.push_back(candidate.radius);
            layersW.push_back(candidate.layerW);
            coverages.push_back(candidate.coverage);
            residuals.push_back(candidate.residual);
            if (candidate.openArc) {
                ++cluster.openArcCandidateCount;
                interiorCleanRatios.push_back(candidate.interiorCleanRatio);
                firstEdgeRatios.push_back(candidate.firstEdgeRatio);
                continuousArcCoverages.push_back(candidate.continuousArcCoverage);
                openArcCandidates.push_back(&candidate);
            }
            layerIds.insert(candidate.layerIndex);
        }
        // 开放弧同一层会因短弧几何产生多个近邻圆心假设。它们只是同一层的候选，
        // 不能被当成多份独立证据放大 centerStd/radiusStd。
        // 上口先按“接近该簇最大口径”筛出嘴沿候选，再每层只保留一个质量最高代表；
        // 更深/更小的圆弧仍保留在原候选集合中供后续孔壁、锥度和深度分析使用。
        if (!openArcCandidates.empty()) {
            double maxOpenRadius = 0.0;
            for (const HouXuan* candidate : openArcCandidates)
                if (candidate) maxOpenRadius = std::max(maxOpenRadius, candidate->radius);
            const double mouthRadiusFloor = 0.82 * maxOpenRadius;
            auto mouthQuality = [&](const HouXuan* candidate) {
                if (!candidate) return -std::numeric_limits<double>::infinity();
                return candidate->score
                    + 0.20 * candidate->coverage
                    + 0.10 * candidate->radius
                    - 0.15 * candidate->circleFitRmse;
            };
            for (const HouXuan* candidate : openArcCandidates) {
                if (!candidate || candidate->radius + 1e-9 < mouthRadiusFloor) continue;
                auto sameLayer = std::find_if(
                    openArcMouthRepresentatives.begin(), openArcMouthRepresentatives.end(),
                    [&](const HouXuan* existing) {
                        return existing && existing->layerIndex == candidate->layerIndex;
                    });
                if (sameLayer == openArcMouthRepresentatives.end()) {
                    openArcMouthRepresentatives.push_back(candidate);
                } else if (mouthQuality(candidate) > mouthQuality(*sameLayer)) {
                    *sameLayer = candidate;
                }
            }
            if (!openArcMouthRepresentatives.empty()) {
                centersU.clear(); centersV.clear(); radii.clear(); layersW.clear();
                coverages.clear(); residuals.clear(); interiorCleanRatios.clear();
                firstEdgeRatios.clear(); continuousArcCoverages.clear(); layerIds.clear();
                for (const HouXuan* candidate : openArcMouthRepresentatives) {
                    centersU.push_back(candidate->centerU);
                    centersV.push_back(candidate->centerV);
                    radii.push_back(candidate->radius);
                    layersW.push_back(candidate->layerW);
                    coverages.push_back(candidate->coverage);
                    residuals.push_back(candidate->residual);
                    interiorCleanRatios.push_back(candidate->interiorCleanRatio);
                    firstEdgeRatios.push_back(candidate->firstEdgeRatio);
                    continuousArcCoverages.push_back(candidate->continuousArcCoverage);
                    layerIds.insert(candidate->layerIndex);
                }
            }
        }

        cluster.supportLayers = static_cast<int>(layerIds.size());
        cluster.consensusCenterU = median(centersU);
        cluster.consensusCenterV = median(centersV);
        const double upperRadiusThreshold = quantile(radii, 0.60);
        std::vector<double> upperRadii;
        std::vector<double> upperW;
        for (std::size_t i = 0; i < radii.size(); ++i) {
            if (radii[i] + 1e-9 >= upperRadiusThreshold) {
                upperRadii.push_back(radii[i]);
                upperW.push_back(layersW[i]);
            }
        }
        cluster.consensusTopRadius = median(upperRadii.empty() ? radii : upperRadii);
        cluster.consensusTopW = median(upperW.empty() ? layersW : upperW);
        cluster.id = stableClusterId(cluster.consensusCenterU, cluster.consensusCenterV,
            cluster.consensusTopW, cluster.consensusTopRadius);
        std::vector<double> centerDistances;
        centerDistances.reserve(group.size());
        for (std::size_t i = 0; i < centersU.size(); ++i) {
            centerDistances.push_back(std::hypot(
                centersU[i] - cluster.consensusCenterU,
                centersV[i] - cluster.consensusCenterV));
        }
        cluster.centerStd = robustStd(centerDistances, 0.0);
        cluster.radiusStd = robustStd(radii, median(radii));
        cluster.meanCoverage = std::accumulate(coverages.begin(), coverages.end(), 0.0)
            / static_cast<double>(coverages.size());
        cluster.meanResidual = std::accumulate(residuals.begin(), residuals.end(), 0.0)
            / static_cast<double>(residuals.size());
        if (!interiorCleanRatios.empty()) {
            cluster.meanInteriorCleanRatio = std::accumulate(
                interiorCleanRatios.begin(), interiorCleanRatios.end(), 0.0)
                / static_cast<double>(interiorCleanRatios.size());
            cluster.meanFirstEdgeRatio = std::accumulate(
                firstEdgeRatios.begin(), firstEdgeRatios.end(), 0.0)
                / static_cast<double>(firstEdgeRatios.size());
            cluster.meanContinuousArcCoverage = std::accumulate(
                continuousArcCoverages.begin(), continuousArcCoverages.end(), 0.0)
                / static_cast<double>(continuousArcCoverages.size());
        }
        // 孔口法向必须由“真正属于上口/口沿”的椭圆弧负责反推。
        // 同一稳定簇里的更深层小圆弧仍可用于孔壁连续性审核，但不能和上口弧等权平均法向，
        // 否则锥孔、局部遮挡或深层次级边缘会把上口的椭圆主轴方向拉偏。
        std::vector<const HouXuan*> mouthEllipseCandidates;
        for (const HouXuan* candidate : openArcCandidates) {
            if (!candidate || !candidate->ellipseValid) continue;
            const double radiusRatio = candidate->radius
                / std::max(cluster.consensusTopRadius, 1e-9);
            const double layerDistance = std::abs(candidate->layerW - cluster.consensusTopW);
            const double centerDistance = std::hypot(
                candidate->centerU - cluster.consensusCenterU,
                candidate->centerV - cluster.consensusCenterV);
            if (radiusRatio < 0.78 || radiusRatio > 1.22 || layerDistance > 0.80
                || centerDistance > std::max(1.15, 0.30 * cluster.consensusTopRadius))
                continue;

            // 同一层可能因残缺弧产生多个近邻圆心，只保留最像“口沿”的那一个，
            // 避免同一层多个相似候选在人为投票时被重复计算。
            auto quality = [&](const HouXuan* value) {
                return 2.4 * value->coverage
                    + 1.5 * value->continuousArcCoverage
                    + 0.9 * value->interiorCleanRatio
                    - 2.2 * value->ellipseModelRmse
                    - 0.8 * value->circleFitRmse
                    - 0.25 * std::abs(value->radius - cluster.consensusTopRadius)
                    - 0.15 * std::hypot(
                        value->centerU - cluster.consensusCenterU,
                        value->centerV - cluster.consensusCenterV);
            };
            auto sameLayer = std::find_if(
                mouthEllipseCandidates.begin(), mouthEllipseCandidates.end(),
                [&](const HouXuan* existing) {
                    return existing && existing->layerIndex == candidate->layerIndex;
                });
            if (sameLayer == mouthEllipseCandidates.end()) {
                mouthEllipseCandidates.push_back(candidate);
            } else if (quality(candidate) > quality(*sameLayer)) {
                *sameLayer = candidate;
            }
        }

        if (!mouthEllipseCandidates.empty()) {
            std::vector<double> mouthRatios;
            std::vector<double> mouthRmses;
            double mouthCos2 = 0.0;
            double mouthSin2 = 0.0;
            double mouthWeightSum = 0.0;
            mouthRatios.reserve(mouthEllipseCandidates.size());
            mouthRmses.reserve(mouthEllipseCandidates.size());
            for (const HouXuan* candidate : mouthEllipseCandidates) {
                mouthRatios.push_back(candidate->ellipseAxisRatio);
                mouthRmses.push_back(candidate->ellipseModelRmse);
                const double angle = std::atan2(candidate->ellipseTiltV, candidate->ellipseTiltU);
                const double normalizedRmse = candidate->ellipseModelRmse
                    / std::max(candidate->radius, 1e-9);
                const double weight = std::max(0.20, candidate->coverage)
                    * (0.65 + candidate->continuousArcCoverage)
                    * std::clamp(1.15 - 2.5 * normalizedRmse, 0.35, 1.15);
                mouthCos2 += weight * std::cos(2.0 * angle);
                mouthSin2 += weight * std::sin(2.0 * angle);
                mouthWeightSum += weight;
            }
            cluster.openArcEllipseSupportCount = static_cast<int>(mouthRatios.size());
            cluster.openArcEllipseAxisRatio = median(mouthRatios);
            cluster.openArcEllipseAxisRatioStd = robustStd(
                mouthRatios, cluster.openArcEllipseAxisRatio);
            cluster.openArcEllipseModelRmse = std::accumulate(
                mouthRmses.begin(), mouthRmses.end(), 0.0)
                / static_cast<double>(mouthRmses.size());
            if (std::hypot(mouthCos2, mouthSin2) > 1e-9 && mouthWeightSum > 1e-9) {
                const double direction = 0.5 * std::atan2(mouthSin2, mouthCos2);
                cluster.openArcEllipseTiltU = std::cos(direction);
                cluster.openArcEllipseTiltV = std::sin(direction);
                cluster.openArcEllipseDirectionCoherence = std::clamp(
                    std::hypot(mouthCos2, mouthSin2) / mouthWeightSum, 0.0, 1.0);
            }

            // 一段足够长、内部干净、模型残差小的上口弧本身就能直接确定椭圆二次型，
            // 不强迫必须有第二层重复“投票”。多层存在时再额外要求方向/轴比一致。
            if (mouthEllipseCandidates.size() == 1U) {
                const HouXuan& mouth = *mouthEllipseCandidates.front();
                const bool strongSingleMouthArc = mouth.ellipseSectorCount >= 12
                    && mouth.coverage >= 0.34
                    && mouth.continuousArcCoverage >= 0.20
                    && mouth.interiorCleanRatio >= 0.86
                    && mouth.ellipseModelRmse <= std::max(0.28, 0.11 * mouth.radius)
                    && mouth.circleFitRmse <= std::max(0.32, 0.12 * mouth.radius);
                cluster.openArcEllipseValid = strongSingleMouthArc;
            } else {
                cluster.openArcEllipseValid = cluster.openArcEllipseDirectionCoherence >= 0.70
                    && cluster.openArcEllipseAxisRatioStd <= 0.10
                    && cluster.openArcEllipseModelRmse
                        <= std::max(0.30, 0.12 * cluster.consensusTopRadius);
            }
        }

        // 多层斜切同一孔时，椭圆中心会沿真实孔轴在粗截面中的投影方向随 w 线性漂移。
        // 这条已有的多层信息正好可以消除椭圆长轴的 ± 方向二义性，不需要再试两个角度。
        if (mouthEllipseCandidates.size() >= 2U) {
            double meanW = 0.0, meanU = 0.0, meanV = 0.0;
            for (const HouXuan* candidate : mouthEllipseCandidates) {
                meanW += candidate->layerW;
                meanU += candidate->centerU;
                meanV += candidate->centerV;
            }
            const double count = static_cast<double>(mouthEllipseCandidates.size());
            meanW /= count; meanU /= count; meanV /= count;
            double varW = 0.0, covWU = 0.0, covWV = 0.0;
            for (const HouXuan* candidate : mouthEllipseCandidates) {
                const double dw = candidate->layerW - meanW;
                varW += dw * dw;
                covWU += dw * (candidate->centerU - meanU);
                covWV += dw * (candidate->centerV - meanV);
            }
            if (varW > 1e-6) {
                cluster.openArcCenterDriftU = covWU / varW;
                cluster.openArcCenterDriftV = covWV / varW;
                cluster.openArcCenterDriftMagnitude = std::hypot(
                    cluster.openArcCenterDriftU, cluster.openArcCenterDriftV);
                if (cluster.openArcEllipseValid && cluster.openArcCenterDriftMagnitude > 1e-9) {
                    cluster.openArcCenterDriftAlignment = std::abs(
                        (cluster.openArcCenterDriftU * cluster.openArcEllipseTiltU
                            + cluster.openArcCenterDriftV * cluster.openArcEllipseTiltV)
                        / cluster.openArcCenterDriftMagnitude);
                }
            }
        }

        std::vector<HouXuan> ordered;
        if (!openArcMouthRepresentatives.empty()) {
            ordered.reserve(openArcMouthRepresentatives.size());
            for (const HouXuan* candidate : openArcMouthRepresentatives)
                if (candidate) ordered.push_back(*candidate);
        } else {
            ordered.reserve(group.size());
            for (int index : group) ordered.push_back(
                result.candidates[static_cast<std::size_t>(index)]);
        }
        std::sort(ordered.begin(), ordered.end(), [](const HouXuan& left, const HouXuan& right) {
            if (left.layerW != right.layerW) return left.layerW < right.layerW;
            return left.score > right.score;
        });
        int continuousLinks = 0;
        int possibleLinks = 0;
        for (std::size_t i = 1; i < ordered.size(); ++i) {
            if (ordered[i].layerIndex == ordered[i - 1].layerIndex) continue;
            ++possibleLinks;
            const double centerDistance = std::hypot(
                ordered[i].centerU - ordered[i - 1].centerU,
                ordered[i].centerV - ordered[i - 1].centerV);
            // 部分圆弧的圆心会因缺口方向和切层位置产生与孔径相关的轻微漂移。
            // 这里不能使用固定 1.2 mm 把直径 10~12 mm、仅约 35%~45% 可见的真实孔拆断；
            // 与前面的跨层聚类保持同一尺度语义，但仍要求半径连续。
            const double continuityCenterLimit = std::max(
                1.20, 0.28 * std::max(ordered[i].radius, ordered[i - 1].radius));
            if (centerDistance <= continuityCenterLimit
                && std::abs(ordered[i].radius - ordered[i - 1].radius) <= 2.0)
                ++continuousLinks;
        }
        cluster.trajectoryContinuity = possibleLinks > 0
            ? static_cast<double>(continuousLinks) / static_cast<double>(possibleLinks) : 1.0;
        const BanJingQuShi trend = fitRadiusTrend(ordered);
        cluster.radiusTrendSlope = trend.slope;
        cluster.radiusTrendRmse = trend.rmse;
        cluster.radiusTrendMonotonicity = trend.monotonicity;
        cluster.radiusTrendSpan = trend.span;
        cluster.radiusTrendLayers = trend.layers;

        const bool compactRadiusStable = cluster.supportLayers >= 2
            && cluster.candidateCount >= 2
            && cluster.centerStd <= 0.90
            && cluster.radiusStd <= 1.50
            && cluster.meanCoverage >= 0.28
            && cluster.meanResidual <= 1.10;

        const double trendRmseLimit = std::max(0.50, 0.075 * cluster.consensusTopRadius);
        const bool coherentTaperStable = trend.valid
            && cluster.supportLayers >= 3
            && cluster.candidateCount >= 3
            && cluster.centerStd <= 1.10
            && cluster.meanCoverage >= 0.24
            && cluster.meanResidual <= 1.15
            && cluster.trajectoryContinuity >= 0.45
            && trend.monotonicity >= 0.60
            && trend.rmse <= trendRmseLimit
            && trend.span >= 0.60;
        const bool containsOpenArc = cluster.openArcCandidateCount > 0;
        const bool multiLayerOpenArcStable = containsOpenArc
            && cluster.openArcCandidateCount >= 2
            && cluster.supportLayers >= 2
            && cluster.centerStd <= 1.10
            && cluster.radiusStd <= 1.15
            // 每一层开放候选在生成时已经满足“单段>=90° 或 多段连续合计>=180°”。
            // 90° 单段对应 25% 总覆盖，因此这里仅把旧 30% 门同步为 25%；
            // 不再重复使用旧的 20% 最长连续弧门，否则会误伤 60°+60°+60° 这类多段 180°。
            && cluster.meanCoverage >= 0.25
            && cluster.meanResidual <= std::max(0.34, 0.08 * cluster.consensusTopRadius)
            && cluster.meanInteriorCleanRatio >= 0.80
            && cluster.trajectoryContinuity >= 0.45;

        // 严重残缺孔的扫描可能只有机械上表面上的一段真实孔口弧，几乎没有任何孔壁，
        // 因而同一孔口无法在第二个真实高度层再次出现。前端候选本身已经通过：
        // 90°连续圆弧、第一边缘、核心空腔、真实环带点数与圆拟合残差审核。
        // 对这种“单层但很强”的机械口沿，不再人为要求第二层孔壁；这里只允许一个严格
        // mouth-only fallback，并保持完整/普通残缺孔仍优先走原来的跨层稳定链。
        const bool strongSingleLayerOpenArcStable = containsOpenArc
            && cluster.openArcCandidateCount >= 1
            && cluster.supportLayers == 1
            && cluster.meanCoverage >= 0.25
            && cluster.meanContinuousArcCoverage >= 0.25
            && cluster.meanInteriorCleanRatio >= 0.90
            && cluster.meanFirstEdgeRatio >= 0.88
            && cluster.meanResidual <= std::max(0.30, 0.07 * cluster.consensusTopRadius)
            && cluster.centerStd <= 0.80
            && cluster.radiusStd <= 0.90
            && (cluster.openArcEllipseValid || cluster.meanCoverage >= 0.30);

        const bool openArcStable = multiLayerOpenArcStable || strongSingleLayerOpenArcStable;

        // 开放圆弧和闭合圆最终进入同一个孔口簇审核；开放弧因为证据更少，
        // 仍需通过自己的稳定门，避免仅凭一段随机边界绕过几何质量审核。
        cluster.openArcStable = openArcStable;
        cluster.taperTrendStable = !containsOpenArc && coherentTaperStable;
        if (containsOpenArc) {
            cluster.stabilityMode = openArcStable
                ? (strongSingleLayerOpenArcStable ? "OPEN_FIRST_EDGE_ARC_SINGLE_LAYER_STRONG"
                                                   : "OPEN_FIRST_EDGE_ARC_CLUSTER")
                : "UNSTABLE_OPEN_ARC";
        } else {
            cluster.stabilityMode = compactRadiusStable
                ? "COMPACT_RADIUS_CLUSTER"
                : (coherentTaperStable ? "COHERENT_TAPER_TREND" : "UNSTABLE");
        }
        cluster.stable = (containsOpenArc
                ? openArcStable
                : (compactRadiusStable || coherentTaperStable))
            && cluster.consensusTopRadius >= 1.0
            && cluster.consensusTopRadius <= std::clamp(
                input.maxHoleRadius * 1.12, 2.0, 33.6);
        cluster.score = 1.25 * static_cast<double>(cluster.supportLayers)
            + 0.20 * static_cast<double>(cluster.candidateCount)
            + 3.0 * cluster.meanCoverage
            + 1.8 * cluster.trajectoryContinuity
            - 1.4 * cluster.centerStd
            - 0.7 * std::min(cluster.radiusStd, 1.50)
            - 0.8 * cluster.meanResidual
            + (cluster.taperTrendStable ? 0.8 : 0.0)
            + (cluster.openArcStable ? (1.2 * cluster.meanInteriorCleanRatio
                + 0.8 * cluster.meanContinuousArcCoverage) : 0.0);

        result.clusters.push_back(cluster);
    }

    result.clusterCount = static_cast<int>(result.clusters.size());
    result.stableClusterCount = static_cast<int>(std::count_if(
        result.clusters.begin(), result.clusters.end(), [](const Cluster& cluster) {
            return cluster.stable;
        }));
    if (result.stableClusterCount == 0) {
        result.exitCode = "CanonicalSearch_NO_STABLE_CLUSTER";
        result.reason = "candidates were found but none formed a stable cross-height cluster";
        return result;
    }

    // 若本次根本没有生成开放圆弧，严格执行原有闭合孔支撑面、角色与点击排序逻辑。
    // 完整孔证据充分时保持闭合几何主链；一旦存在开放证据，
    // 再使用统一的点击归属与支撑面语义，让完整圆和部分圆弧在同一候选空间比较。
    if (!runOpenArc) {
        double outerClusterW = -std::numeric_limits<double>::infinity();
        for (const Cluster& cluster : result.clusters) {
            if (cluster.stable && finite(cluster.consensusTopW))
                outerClusterW = std::max(outerClusterW, cluster.consensusTopW);
        }
        if (!finite(outerClusterW)) outerClusterW = histogramSupportPlaneW;
        result.supportPlaneOuterClusterW = outerClusterW;
        // 完整闭合孔：外表面锚点只负责排除明显深层截面，不直接抬高支撑面。
        // 倾斜直孔的真实圆口在粗法向坐标下可能比外表面切面略深；若强制 supportPlaneW=seedW，
        // 会偏向覆盖更高但半径过大的外层椭圆/假圆；该分支必须优先保护真实 Hole 口尺度。
        result.supportPlaneW = std::max(histogramSupportPlaneW, outerClusterW);
        result.supportPlaneMode = outerClusterW > histogramSupportPlaneW + 0.50
            ? "OUTERMOST_STABLE_CLUSTER_OVERRIDES_DENSE_DEEP_BAND"
            : "DENSITY_AND_OUTERMOST_CLUSTER_CONSENSUS";

        for (Cluster& cluster : result.clusters) {
            cluster.mouthPlaneDistance = cluster.consensusTopW - result.supportPlaneW;
            const double mouthLimit = std::max(1.50, 0.30 * cluster.consensusTopRadius);
            const double deepLimit = std::max(3.00, 0.60 * cluster.consensusTopRadius);
            const double anchorDistance = input.seedWIsOuterSurfaceAnchor
                ? cluster.consensusTopW - input.seedW : 0.0;
            const double anchorMouthLimit = std::max(
                1.75, 0.35 * cluster.consensusTopRadius);
            const bool anchorRejectsDeep = input.seedWIsOuterSurfaceAnchor
                && anchorDistance < -anchorMouthLimit;
            if (cluster.stable && !anchorRejectsDeep
                && cluster.mouthPlaneDistance >= -mouthLimit
                && cluster.mouthPlaneDistance <= 0.75) {
                cluster.role = "MOUTH_ATTACHED";
                cluster.mouthAttachmentValid = true;
                cluster.roleReason = input.seedWIsOuterSurfaceAnchor
                    ? "CONSENSUS_TOP_NEAR_DATA_SUPPORT_AND_OUTER_SURFACE_ANCHOR"
                    : "CONSENSUS_TOP_ON_OUTER_SUPPORT_ENVELOPE";
            } else if (cluster.stable
                && (anchorRejectsDeep || cluster.mouthPlaneDistance <= -deepLimit)) {
                cluster.role = "DEEP_CONTINUATION";
                cluster.deepContinuation = true;
                cluster.roleReason = "CONSENSUS_TOP_BELOW_OUTER_SUPPORT_ENVELOPE";
            } else if (cluster.stable && cluster.mouthPlaneDistance > 0.75) {
                cluster.role = "SURFACE_OUTER";
                cluster.roleReason = "CONSENSUS_TOP_ABOVE_SUPPORT_ENVELOPE";
            } else {
                cluster.role = "UNRESOLVED";
                cluster.roleReason = "BETWEEN_MOUTH_AND_DEEP_ROLE_LIMITS";
            }
        }

        result.mouthAttachedClusterCount = static_cast<int>(std::count_if(
            result.clusters.begin(), result.clusters.end(), [](const Cluster& cluster) {
                return cluster.stable && cluster.mouthAttachmentValid;
            }));
        if (result.mouthAttachedClusterCount == 0) {
            result.exitCode = "SurfaceCrossShift_NO_MOUTH_ATTACHED_CLUSTER";
            result.reason = "stable clusters exist but none attach to the dominant support plane";
            return result;
        }
        std::vector<std::pair<double, int>> ranking;
        int persistentBestIndex = -1;
        double persistentBestScore = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < result.clusters.size(); ++index) {
            Cluster& cluster = result.clusters[index];
            if (!cluster.stable || !cluster.mouthAttachmentValid) continue;
            const double radialDistance = std::hypot(
                input.seedU - cluster.consensusCenterU,
                input.seedV - cluster.consensusCenterV);
            cluster.seedBoundaryDistance = std::abs(radialDistance - cluster.consensusTopRadius);
            cluster.seedAxialDistance = std::abs(input.seedW - cluster.consensusTopW);

            if (persistentFamilyEligible(cluster, radialDistance, input)) {
                const double pScore = persistentFamilyScore(cluster, radialDistance);
                if (pScore > persistentBestScore) {
                    persistentBestScore = pScore;
                    persistentBestIndex = static_cast<int>(index);
                }
            }

            if (!seedBelongsToMouth(
                    radialDistance, cluster.consensusTopRadius,
                    input.cellSize, input.maxHoleRadius)) {
                continue;
            }

            // 完整闭合孔真正以“点击点到机械孔口边缘的距离”为主排序。
            // 上一版将孔内点击的边缘距离缩成 22%，同时把 cluster.score 权重放大，
            // 会让覆盖更高但半径偏大的外层假圆压过真实孔口。深层小圆已由上面的外表面资格门排除，
            // 所以这里恢复孔沿距离的主证据权；几何分数只作很小的稳定性修正。
            const double clickCost = cluster.seedBoundaryDistance;
            cluster.selectionCost = clickCost - 0.015 * cluster.score;
            ranking.emplace_back(cluster.selectionCost, static_cast<int>(index));
        }
        // 单阶段孔沿归属：使用统一的局部归属范围，避免重复二次筛选。
        // 就是唯一捕获范围，避免“一级失败 -> 二级另换规则”造成结果不连续。
        if (ranking.empty() && persistentBestIndex < 0) {
            result.exitCode = "MouthSearch_NO_CLICK_COMPATIBLE_MOUTH_CLUSTER";
            result.reason = "stable mouth clusters were found but none is within the configured seed-to-edge or persistent-family capture distance";
            return result;
        }

        std::sort(ranking.begin(), ranking.end(), [&](const auto& left, const auto& right) {
            if (std::abs(left.first - right.first) > 1e-12) return left.first < right.first;
            const Cluster& a = result.clusters[static_cast<std::size_t>(left.second)];
            const Cluster& b = result.clusters[static_cast<std::size_t>(right.second)];
            if (a.score != b.score) return a.score > b.score;
            if (a.consensusCenterU != b.consensusCenterU)
                return a.consensusCenterU < b.consensusCenterU;
            return a.consensusCenterV < b.consensusCenterV;
        });
        const int edgeBestIndex = ranking.empty() ? -1 : ranking.front().second;
        bool persistentOverride = edgeBestIndex < 0 && persistentBestIndex >= 0;
        if (edgeBestIndex >= 0 && persistentBestIndex >= 0
            && persistentBestIndex != edgeBestIndex) {
            const Cluster& edgeBest = result.clusters[static_cast<std::size_t>(edgeBestIndex)];
            const Cluster& persistentBest = result.clusters[static_cast<std::size_t>(persistentBestIndex)];
            persistentOverride = persistentBest.score >= edgeBest.score + 0.55;
        }
        result.selectedClusterIndex = persistentOverride ? persistentBestIndex : edgeBestIndex;
        result.selectedClusterId = result.clusters[
            static_cast<std::size_t>(result.selectedClusterIndex)].id;
        result.clusterMargin = ranking.size() > 1 ? ranking[1].first - ranking[0].first
                                                 : std::numeric_limits<double>::infinity();
        result.selectMode = persistentOverride
            ? "PERSISTENT_FAMILY_WEAK_SEED_OVERRIDE"
            : "SEED_EDGE_DISTANCE_SINGLE_STAGE_MAX_RADIUS_LINKED";
        result.exitCode = "SurfaceCrossShift_OK";
        result.reason = persistentOverride
            ? "dominant persistent mouth family selected with weak seed-center association inside fixed ROI"
            : "stable closed-mouth cluster selected by single-stage max-radius-linked seed-to-edge ownership";
        result.valid = true;

        return result;
    }

    double outerClusterW = -std::numeric_limits<double>::infinity();
    for (const Cluster& cluster : result.clusters) {
        // 开放弧候选本身可能来自支撑面上下的容差层，不能单独把“最外层”定义往外推。
        // 有闭合稳定簇时仍沿用原逻辑；纯开放簇时支撑面以密度直方图为基准。
        if (cluster.stable && !cluster.openArcStable && finite(cluster.consensusTopW)) {
            const double radialDistance = std::hypot(
                input.seedU - cluster.consensusCenterU,
                input.seedV - cluster.consensusCenterV);
            const bool seedCompatible = seedBelongsToMouth(
                radialDistance, cluster.consensusTopRadius,
                input.cellSize, input.maxHoleRadius);
            // 支撑面属于“当前点击目标”，不是整个 ROI 的全局最大 W。
            // 无论是否进入开放弧回退，都只有与点击几何兼容的闭合稳定簇才允许覆盖密度支撑面；
            // 这样既不会被旁孔抢走，也不依赖扫描坐标的深度正负方向。
            if (seedCompatible)
                outerClusterW = std::max(outerClusterW, cluster.consensusTopW);
        }
    }
    const double localOuterSupportW = input.seedWIsOuterSurfaceAnchor
        ? std::max({histogramSupportPlaneW, seedLocalSupportPlaneW, input.seedW})
        : std::max(histogramSupportPlaneW, seedLocalSupportPlaneW);
    if (!finite(outerClusterW)) outerClusterW = localOuterSupportW;
    result.supportPlaneOuterClusterW = outerClusterW;
    result.supportPlaneW = std::max(localOuterSupportW, outerClusterW);
    if (seedLocalSupportPlaneW > histogramSupportPlaneW + 0.50)
        result.supportPlaneMode = "SEED_LOCAL_OUTER_SUPPORT_OVERRIDES_DENSE_DEEP_BAND";
    else if (outerClusterW > localOuterSupportW + 0.50)
        result.supportPlaneMode = "OUTERMOST_STABLE_CLUSTER_OVERRIDES_DENSE_DEEP_BAND";
    else
        result.supportPlaneMode = "DENSITY_LOCAL_AND_CLUSTER_CONSENSUS";

    for (Cluster& cluster : result.clusters) {
        cluster.mouthPlaneDistance = cluster.consensusTopW - result.supportPlaneW;
        const double mouthLimit = std::max(1.50, 0.30 * cluster.consensusTopRadius);
        const double deepLimit = std::max(3.00, 0.60 * cluster.consensusTopRadius);
        if (cluster.stable && cluster.mouthPlaneDistance >= -mouthLimit
            && cluster.mouthPlaneDistance <= 0.75) {
            cluster.role = "MOUTH_ATTACHED";
            cluster.mouthAttachmentValid = true;
            cluster.roleReason = "CONSENSUS_TOP_ON_OUTER_SUPPORT_ENVELOPE";
        } else if (cluster.stable && cluster.mouthPlaneDistance <= -deepLimit) {
            cluster.role = "DEEP_CONTINUATION";
            cluster.deepContinuation = true;
            cluster.roleReason = "CONSENSUS_TOP_BELOW_OUTER_SUPPORT_ENVELOPE";
        } else if (cluster.stable && cluster.mouthPlaneDistance > 0.75) {
            cluster.role = "SURFACE_OUTER";
            cluster.roleReason = "CONSENSUS_TOP_ABOVE_SUPPORT_ENVELOPE";
        } else {
            cluster.role = "UNRESOLVED";
            cluster.roleReason = "BETWEEN_MOUTH_AND_DEEP_ROLE_LIMITS";
        }
    }

    result.mouthAttachedClusterCount = static_cast<int>(std::count_if(
        result.clusters.begin(), result.clusters.end(), [](const Cluster& cluster) {
            return cluster.stable && cluster.mouthAttachmentValid;
        }));
    if (result.mouthAttachedClusterCount == 0) {
        result.exitCode = "SurfaceCrossShift_NO_MOUTH_ATTACHED_CLUSTER";
        result.reason = "stable clusters exist but none attach to the dominant support plane";
        return result;
    }
    std::vector<std::pair<double, int>> ranking;
    int persistentBestIndex = -1;
    double persistentBestScore = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < result.clusters.size(); ++index) {
        Cluster& cluster = result.clusters[index];
        if (!cluster.stable || !cluster.mouthAttachmentValid) continue;

        const double radialDistance = std::hypot(
            input.seedU - cluster.consensusCenterU,
            input.seedV - cluster.consensusCenterV);
        cluster.seedBoundaryDistance = std::abs(
            radialDistance - cluster.consensusTopRadius);
        cluster.seedAxialDistance = std::abs(input.seedW - cluster.consensusTopW);

        if (persistentFamilyEligible(cluster, radialDistance, input)) {
            const double pScore = persistentFamilyScore(cluster, radialDistance);
            if (pScore > persistentBestScore) {
                persistentBestScore = pScore;
                persistentBestIndex = static_cast<int>(index);
            }
        }

        // 统一的单阶段点击归属：一次完成局部归属，避免重复搜索。
        // 所有完整圆和部分圆弧从一开始就使用“点击点到真实孔口边缘的距离”。
        // 点击落在孔内时仍认为属于该孔，但边缘距离继续作为软排序量；这样既允许点孔内，
        // 又能在密集多孔区域优先选择孔沿更接近点击位置的物理孔。
        const bool seedInside = radialDistance <= cluster.consensusTopRadius;
        if (!seedBelongsToMouth(
                radialDistance, cluster.consensusTopRadius,
                input.cellSize, input.maxHoleRadius)) {
            continue;
        }

        // 孔内点击的边缘距离只给较小权重，避免点击孔心时大孔天然吃亏；
        // 孔外点击按真实孔沿距离完整计入。几何稳定度仍参与排序，防止假圆只因靠近点击点取胜。
        // 这里刻意不使用 seedW：左右孔壁或锥壁拾取深度不能改变“选哪个孔”的结果。
        const double clickCost = seedInside
            ? 0.22 * cluster.seedBoundaryDistance
            : cluster.seedBoundaryDistance;
        const double geometryWeight = cluster.openArcStable ? 0.10 : 0.045;
        cluster.selectionCost = clickCost - geometryWeight * cluster.score;
        ranking.emplace_back(cluster.selectionCost, static_cast<int>(index));
    }
    if (ranking.empty() && persistentBestIndex < 0) {
        result.exitCode = "MouthSearch_NO_CLICK_COMPATIBLE_MOUTH_CLUSTER";
        result.reason = "stable mouth clusters were found but none is compatible with the clicked seed or dominant persistent family";
        return result;
    }

    std::sort(ranking.begin(), ranking.end(), [&](const auto& left, const auto& right) {
        if (std::abs(left.first - right.first) > 1e-12) return left.first < right.first;
        const Cluster& a = result.clusters[static_cast<std::size_t>(left.second)];
        const Cluster& b = result.clusters[static_cast<std::size_t>(right.second)];
        if (a.score != b.score) return a.score > b.score;
        if (a.consensusCenterU != b.consensusCenterU)
            return a.consensusCenterU < b.consensusCenterU;
        return a.consensusCenterV < b.consensusCenterV;
    });
    const int edgeBestIndex = ranking.empty() ? -1 : ranking.front().second;
    bool persistentOverride = edgeBestIndex < 0 && persistentBestIndex >= 0;
    if (edgeBestIndex >= 0 && persistentBestIndex >= 0
        && persistentBestIndex != edgeBestIndex) {
        const Cluster& edgeBest = result.clusters[static_cast<std::size_t>(edgeBestIndex)];
        const Cluster& persistentBest = result.clusters[static_cast<std::size_t>(persistentBestIndex)];
        persistentOverride = persistentBest.score >= edgeBest.score + 0.55;
    }
    result.selectedClusterIndex = persistentOverride ? persistentBestIndex : edgeBestIndex;
    result.selectedClusterId = result.clusters[
        static_cast<std::size_t>(result.selectedClusterIndex)].id;
    result.clusterMargin = ranking.size() > 1 ? ranking[1].first - ranking[0].first
                                             : std::numeric_limits<double>::infinity();
    result.selectMode = persistentOverride
        ? "PERSISTENT_FAMILY_WEAK_SEED_OVERRIDE"
        : "SEED_EDGE_DISTANCE_SINGLE_STAGE";
    result.exitCode = "SurfaceCrossShift_OK";
    result.reason = persistentOverride
        ? "dominant persistent mouth family selected with weak seed-center association inside fixed ROI"
        : "stable mouth cluster selected by single-stage physical mouth-edge ownership";
    result.valid = true;

    return result;
}


/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    return evaluateImpl(samples, input, kUseSparseLayers);
}

}

// ============================================================================
// 功能分区：孔形与深度分析实现
// ============================================================================
/*
模块职责：
孔形分类实现。

维护说明：
本文件按功能整合生产实现。各分区通过明确职责组织，算法阈值和候选顺序集中在对应分区维护。
*/

// ============================================================================
// 功能分区：孔形基础截面分类
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/
#include <vector>

namespace HoleLeixingPouMianBase {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLayerStart = 0.05;
constexpr double kLayerStep = 0.25;
constexpr double kLayerHalf = 0.14;
constexpr double kMaxDepth = 6.00;
constexpr double kRadialBin = 0.15;
constexpr int kSectors = 36;
constexpr int kLayerCount = 24;
constexpr int kMaxRadialBins = 80;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“popcount64”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int popcount64(std::uint64_t value) noexcept
{
#if defined(_MSC_VER)
    int count = 0;
    while (value != 0) {
        value &= value - 1;
        ++count;
    }
    return count;
#else
    return __builtin_popcountll(value);
#endif
}

double median(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 != 0
        ? values[middle]
        : 0.5 * (values[middle - 1] + values[middle]);
}

/** 【类型导航注释】
 * JiZuoBiaoPoint：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JiZuoBiaoPoint {
    double depth = 0.0;
    double radius = 0.0;
    int sector = 0;
    int radialBin = 0;
};

/** 【类型导航注释】
 * HouXuan：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HouXuan {
    double radius = 0.0;
    int points = 0;
    int sectors = 0;
    double span = 0.0;
};

/** 【类型导航注释】
 * State：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct State {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    std::vector<std::pair<int, int>> path;
};

/** 【类型导航注释】
 * PouMian：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PouMian {
    bool valid = false;
    bool cone = false;
    bool stable = false;
    bool chamferPlatformStraight = false;
    bool segmentedModelValid = false;
    int polarity = 0;
    int layers = 0;
    int distributedDrops = 0;
    int breakLayer = -1;
    int platformLayers = 0;
    double depthSpan = 0.0;
    double firstRadius = 0.0;
    double lastRadius = 0.0;
    double slope = 0.0;
    double shrink = 0.0;
    double monotonic = 0.0;
    double maxStepDrop = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    double platformDepthSpan = 0.0;
    double coneWeightedRmse = 0.0;
    double coneBic = 0.0;
    double segmentedWeightedRmse = 0.0;
    double segmentedBic = 0.0;
    double bicImprovement = 0.0;
    double chamferSlope = 0.0;
    double plateauRadius = 0.0;
    double plateauSlope = 0.0;
    double plateauStd = 0.0;
    double plateauCoverage = 0.0;
    double platformEntranceDrop = 0.0;
    double platformConfidence = 0.0;
    std::vector<double> depths;
    std::vector<double> radii;
    std::vector<int> points;
    std::vector<int> sectors;
};

/** 【类型导航注释】
 * XianXingNiHe：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct XianXingNiHe {
    bool valid = false;
    double intercept = 0.0;
    double slope = 0.0;
    double sse = 0.0;
    double rmse = 0.0;
};

/** 【函数导航】
 * 作用：拟合/求解“fitLinear”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XianXingNiHe fitLinear(const std::vector<double>& x, const std::vector<double>& y,
                    const std::vector<double>& weights, std::size_t begin, std::size_t end)
{
    XianXingNiHe fit;
    if (end <= begin + 1 || end > x.size() || end > y.size() || end > weights.size())
        return fit;
    double sw = 0.0, sx = 0.0, sy = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double w = std::max(1e-6, weights[i]);
        sw += w; sx += w * x[i]; sy += w * y[i];
    }
    if (sw <= 1e-9) return fit;
    const double mx = sx / sw;
    const double my = sy / sw;
    double num = 0.0, den = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double w = std::max(1e-6, weights[i]);
        num += w * (x[i] - mx) * (y[i] - my);
        den += w * (x[i] - mx) * (x[i] - mx);
    }
    fit.slope = den > 1e-12 ? num / den : 0.0;
    fit.intercept = my - fit.slope * mx;
    for (std::size_t i = begin; i < end; ++i) {
        const double error = y[i] - (fit.intercept + fit.slope * x[i]);
        fit.sse += std::max(1e-6, weights[i]) * error * error;
    }
    fit.rmse = std::sqrt(fit.sse / sw);
    fit.valid = true;
    return fit;
}

/** 【函数导航】
 * 作用：执行“weightedBic”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double weightedBic(double sse, std::size_t n, int parameters)
{
    if (n == 0) return std::numeric_limits<double>::infinity();
    const double variance = std::max(1e-12, sse / static_cast<double>(n));
    return static_cast<double>(n) * std::log(variance)
        + static_cast<double>(parameters) * std::log(static_cast<double>(n));
}

/** 【类型导航注释】
 * PingTaiZhengJu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingTaiZhengJu {
    bool valid = false;
    int breakLayer = -1;
    int layers = 0;
    double depthSpan = 0.0;
    double radius = 0.0;
    double slope = 0.0;
    double stddev = 0.0;
    double coverage = 0.0;
    double entranceDrop = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：检测/搜索“findStablePlatform”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
PingTaiZhengJu findStablePlatform(
    const std::vector<std::vector<HouXuan>>& candidates, double topRadius)
{
    PingTaiZhengJu best;
    const double tolerance = std::max(0.20, 0.035 * topRadius);
    const double requiredDrop = std::max(0.65, 0.10 * topRadius);
    for (std::size_t start = 1; start + 4 < candidates.size(); ++start) {
        const double startDepth = kLayerStart + static_cast<double>(start) * kLayerStep;
        if (startDepth > 1.55) break;
        for (const HouXuan& anchor : candidates[start]) {
            if (anchor.radius < std::max(0.50, 0.25 * topRadius)
                || anchor.radius > 0.96 * topRadius) continue;
            std::vector<double> depths;
            std::vector<double> radii;
            std::vector<double> weights;
            std::vector<int> sectors;
            std::size_t lastLayer = start;
            double runningRadius = anchor.radius;
            for (std::size_t layer = start; layer < candidates.size(); ++layer) {
                if (layer > lastLayer + 2) break;
                const HouXuan* selected = nullptr;
                double bestCost = std::numeric_limits<double>::infinity();
                for (const HouXuan& candidate : candidates[layer]) {
                    const double delta = std::abs(candidate.radius - runningRadius);
                    if (delta > tolerance) continue;
                    const double support = candidate.points + 2.0 * candidate.sectors;
                    const double cost = delta - 0.002 * support;
                    if (cost < bestCost) { bestCost = cost; selected = &candidate; }
                }
                if (!selected) continue;
                const double depth = kLayerStart + static_cast<double>(layer) * kLayerStep;
                depths.push_back(depth);
                radii.push_back(selected->radius);
                weights.push_back(std::max(1.0, static_cast<double>(selected->sectors)
                    + 0.05 * static_cast<double>(selected->points)));
                sectors.push_back(selected->sectors);
                runningRadius = median(radii);
                lastLayer = layer;
            }
            if (radii.size() < 5) continue;
            const double span = depths.back() - depths.front();
            if (span < 1.0) continue;
            const XianXingNiHe fit = fitLinear(depths, radii, weights, 0, radii.size());
            if (!fit.valid) continue;
            const double center = median(radii);
            double variance = 0.0;
            for (double radius : radii) variance += (radius - center) * (radius - center);
            const double stddev = std::sqrt(variance / static_cast<double>(radii.size()));
            double entranceRadius = 0.0;
            for (std::size_t layer = 0; layer < start; ++layer) {
                for (const HouXuan& candidate : candidates[layer]) {
                    if (candidate.radius <= topRadius + 2.0)
                        entranceRadius = std::max(entranceRadius, candidate.radius);
                }
            }
            const double drop = entranceRadius - center;
            const double meanSectors = std::accumulate(sectors.begin(), sectors.end(), 0.0)
                / static_cast<double>(sectors.size());
            const bool stable = std::abs(fit.slope) <= 0.18
                && stddev <= std::max(0.22, 0.04 * topRadius)
                && drop >= requiredDrop
                && meanSectors >= 4.0;
            if (!stable) continue;
            const double score = 1.8 * static_cast<double>(radii.size())
                + 1.2 * span + 0.12 * meanSectors + 1.5 * drop
                - 8.0 * stddev - 4.0 * std::abs(fit.slope);
            if (!best.valid || score > best.score) {
                best.valid = true;
                best.breakLayer = static_cast<int>(start);
                best.layers = static_cast<int>(radii.size());
                best.depthSpan = span;
                best.radius = center;
                best.slope = fit.slope;
                best.stddev = stddev;
                best.coverage = meanSectors / static_cast<double>(kSectors);
                best.entranceDrop = drop;
                best.score = score;
            }
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“platformConfidenceScore”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double platformConfidenceScore(int layers, double span, double slope,
                               double stddev, double coverage, double entranceDrop,
                               double platformRadius, double topRadius, int breakLayer)
{
    if (!(topRadius > 0.0) || layers < 4 || span <= 0.0) return 0.0;
    const double requiredDrop = std::max(0.55, 0.08 * topRadius);
    const double spreadLimit = std::max(0.22, 0.04 * topRadius);
    const double radiusRatio = platformRadius / topRadius;
    const double layerScore = std::clamp((static_cast<double>(layers) - 4.0) / 3.0, 0.0, 1.0);
    const double spanScore = std::clamp((span - 0.75) / 0.75, 0.0, 1.0);
    const double slopeScore = std::clamp((0.24 - std::abs(slope)) / 0.18, 0.0, 1.0);
    const double spreadScore = std::clamp((spreadLimit - stddev) / std::max(0.08, spreadLimit), 0.0, 1.0);
    const double coverageScore = std::clamp((coverage - 0.12) / 0.28, 0.0, 1.0);
    const double dropScore = std::clamp((entranceDrop - requiredDrop) / std::max(0.45, 0.10 * topRadius), 0.0, 1.0);
    const double radiusScore = radiusRatio >= 0.35 && radiusRatio <= 0.92
        ? std::clamp((0.92 - radiusRatio) / 0.30, 0.25, 1.0) : 0.0;
    const double breakScore = breakLayer >= 1 && breakLayer <= 7 ? 1.0 : 0.0;
    return 0.18 * layerScore + 0.16 * spanScore + 0.16 * slopeScore
        + 0.16 * spreadScore + 0.10 * coverageScore + 0.14 * dropScore
        + 0.06 * radiusScore + 0.04 * breakScore;
}

/** 【函数导航】
 * 作用：构建“buildCandidates”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<std::vector<HouXuan>> buildCandidates(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const HoleJiheFinal::Result& geometry,
    int polarity,
    double maxRadius)
{
    std::vector<JiZuoBiaoPoint> points;
    points.reserve(samples.size() / 4 + 32);
    const int radialBins = std::clamp(
        static_cast<int>(std::ceil(maxRadius / kRadialBin)), 1, kMaxRadialBins);

    for (const auto& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double du = sample.u - geometry.centerU;
        const double dv = sample.v - geometry.centerV;
        const double depth = static_cast<double>(polarity) * (sample.w - geometry.topW);
        if (depth < 0.05 || depth > kMaxDepth + kLayerHalf) continue;
        const double radius = std::hypot(du, dv);
        if (radius < 0.25 || radius > maxRadius) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(
            static_cast<int>(std::floor(angle * static_cast<double>(kSectors)
                                        / (2.0 * kPi))),
            0, kSectors - 1);
        const int radialBin = std::clamp(
            static_cast<int>(std::floor(radius / kRadialBin)),
            0, radialBins - 1);
        points.push_back({depth, radius, sector, radialBin});
    }

    std::vector<std::vector<HouXuan>> layers(static_cast<std::size_t>(kLayerCount));
    for (int layer = 0; layer < kLayerCount; ++layer) {
        const double layerDepth = kLayerStart + static_cast<double>(layer) * kLayerStep;
        std::array<int, kMaxRadialBins> counts{};
        std::array<std::uint64_t, kMaxRadialBins> sectorMasks{};
        std::vector<const JiZuoBiaoPoint*> layerPoints;
        layerPoints.reserve(160);

        for (const JiZuoBiaoPoint& point : points) {
            if (std::abs(point.depth - layerDepth) > kLayerHalf) continue;
            ++counts[static_cast<std::size_t>(point.radialBin)];
            sectorMasks[static_cast<std::size_t>(point.radialBin)]
                |= (std::uint64_t{1} << static_cast<unsigned>(point.sector));
            layerPoints.push_back(&point);
        }
        if (layerPoints.size() < 5) continue;

        std::vector<HouXuan> raw;
        raw.reserve(static_cast<std::size_t>(radialBins));
        for (int bin = 0; bin < radialBins; ++bin) {
            int count = 0;
            std::uint64_t mask = 0;
            for (int offset = -1; offset <= 1; ++offset) {
                const int index = bin + offset;
                if (index < 0 || index >= radialBins) continue;
                count += counts[static_cast<std::size_t>(index)];
                mask |= sectorMasks[static_cast<std::size_t>(index)];
            }
            const int sectors = popcount64(mask);
            if (count < 5 || sectors < 3) continue;

            std::vector<double> radii;
            radii.reserve(static_cast<std::size_t>(count));
            double minimum = std::numeric_limits<double>::infinity();
            double maximum = -std::numeric_limits<double>::infinity();
            for (const JiZuoBiaoPoint* point : layerPoints) {
                if (std::abs(point->radialBin - bin) > 1) continue;
                radii.push_back(point->radius);
                minimum = std::min(minimum, point->radius);
                maximum = std::max(maximum, point->radius);
            }
            const double radius = median(std::move(radii));
            if (!finite(radius)) continue;
            raw.push_back({radius, count, sectors, maximum - minimum});
        }

        std::sort(raw.begin(), raw.end(), [](const HouXuan& lhs, const HouXuan& rhs) {
            return lhs.radius < rhs.radius;
        });
        for (const HouXuan& candidate : raw) {
            if (!layers[static_cast<std::size_t>(layer)].empty()
                && std::abs(candidate.radius
                            - layers[static_cast<std::size_t>(layer)].back().radius) < 0.18) {
                HouXuan& previous = layers[static_cast<std::size_t>(layer)].back();
                const double candidateSupport = candidate.points + 2.0 * candidate.sectors;
                const double previousSupport = previous.points + 2.0 * previous.sectors;
                if (candidateSupport > previousSupport) previous = candidate;
            } else {
                layers[static_cast<std::size_t>(layer)].push_back(candidate);
            }
        }
    }
    return layers;
}

/** 【函数导航】
 * 作用：评估/审核“evaluatePolarity”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
PouMian evaluatePolarity(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const HoleJiheFinal::Result& geometry,
    int polarity)
{
    PouMian profile;
    profile.polarity = polarity;
    const double maxRadius = std::min(12.0, std::max(geometry.topRadius + 1.8, 5.5));
    const auto candidates = buildCandidates(samples, geometry, polarity, maxRadius);
    const PingTaiZhengJu platformEvidence = findStablePlatform(candidates, geometry.topRadius);

    std::vector<std::vector<State>> states(candidates.size());
    State best;
    for (std::size_t layer = 0; layer < candidates.size(); ++layer) {
        states[layer].resize(candidates[layer].size());
        for (std::size_t index = 0; index < candidates[layer].size(); ++index) {
            const HouXuan& candidate = candidates[layer][index];
            const double support = 1.5 * static_cast<double>(candidate.sectors)
                + 0.08 * static_cast<double>(candidate.points)
                - 0.60 * candidate.span;
            State state;
            state.valid = true;
            state.score = support - 1.8 * std::abs(candidate.radius - geometry.topRadius);
            state.path.push_back({static_cast<int>(layer), static_cast<int>(index)});

            for (int gap = 1; gap <= 2; ++gap) {
                if (static_cast<int>(layer) - gap < 0) continue;
                const std::size_t previousLayer = layer - static_cast<std::size_t>(gap);
                for (std::size_t previousIndex = 0;
                     previousIndex < states[previousLayer].size(); ++previousIndex) {
                    const State& previous = states[previousLayer][previousIndex];
                    if (!previous.valid) continue;
                    const double previousRadius = candidates[previousLayer][previousIndex].radius;
                    const double delta = candidate.radius - previousRadius;
                    if (delta > 0.45 + 0.12 * static_cast<double>(gap - 1)) continue;
                    if (delta < -1.25 * static_cast<double>(gap)) continue;
                    const double transition = -1.4 * std::abs(delta)
                        - (gap == 2 ? 1.0 : 0.0);
                    const double score = previous.score + support + transition;
                    if (score > state.score) {
                        state.score = score;
                        state.path = previous.path;
                        state.path.push_back({static_cast<int>(layer), static_cast<int>(index)});
                    }
                }
            }
            states[layer][index] = state;
            if (!best.valid || state.score > best.score) best = state;
        }
    }

    if (!best.valid || best.path.empty()) return profile;

    std::vector<double> depths;
    std::vector<double> radii;
    std::vector<int> pathPoints;
    std::vector<int> pathSectors;
    depths.reserve(best.path.size());
    radii.reserve(best.path.size());
    pathPoints.reserve(best.path.size());
    pathSectors.reserve(best.path.size());
    for (const auto& item : best.path) {
        const int layer = item.first;
        const int index = item.second;
        depths.push_back(kLayerStart + static_cast<double>(layer) * kLayerStep);
        const HouXuan& selectedCandidate = candidates[static_cast<std::size_t>(layer)]
            [static_cast<std::size_t>(index)];
        radii.push_back(selectedCandidate.radius);
        pathPoints.push_back(selectedCandidate.points);
        pathSectors.push_back(selectedCandidate.sectors);
    }

    profile.valid = radii.size() >= 2;
    profile.layers = static_cast<int>(radii.size());
    profile.score = best.score;
    profile.depths = depths;
    profile.radii = radii;
    profile.points = pathPoints;
    profile.sectors = pathSectors;
    if (!profile.valid) return profile;

    const double meanDepth = std::accumulate(depths.begin(), depths.end(), 0.0)
        / static_cast<double>(depths.size());
    const double meanRadius = std::accumulate(radii.begin(), radii.end(), 0.0)
        / static_cast<double>(radii.size());
    double numerator = 0.0;
    double denominator = 0.0;
    for (std::size_t index = 0; index < radii.size(); ++index) {
        numerator += (depths[index] - meanDepth) * (radii[index] - meanRadius);
        denominator += (depths[index] - meanDepth) * (depths[index] - meanDepth);
    }
    profile.slope = denominator > 1e-9 ? numerator / denominator : 0.0;
    profile.depthSpan = depths.back() - depths.front();

    const std::size_t edgeCount = std::min<std::size_t>(2, radii.size());
    std::vector<double> first(radii.begin(), radii.begin() + edgeCount);
    std::vector<double> last(radii.end() - edgeCount, radii.end());
    profile.firstRadius = median(std::move(first));
    profile.lastRadius = median(std::move(last));
    profile.shrink = profile.firstRadius - profile.lastRadius;

    int monotonic = 0;
    profile.maxStepDrop = 0.0;
    for (std::size_t index = 1; index < radii.size(); ++index) {
        const double delta = radii[index] - radii[index - 1];
        if (delta <= 0.12) ++monotonic;
        if (delta < -0.12) ++profile.distributedDrops;
        profile.maxStepDrop = std::max(profile.maxStepDrop, -delta);
    }
    profile.monotonic = radii.size() > 1
        ? static_cast<double>(monotonic) / static_cast<double>(radii.size() - 1)
        : 0.0;

    const double requiredShrink = std::max(0.45, 0.12 * geometry.topRadius);
    profile.cone = profile.layers >= 4
        && profile.depthSpan >= 0.70
        && profile.shrink >= requiredShrink
        && profile.slope <= -0.28
        && profile.monotonic >= 0.72
        && profile.distributedDrops >= 2
        && profile.maxStepDrop <= std::max(0.75, 0.70 * profile.shrink + 0.05);

    std::vector<double> weights;
    weights.reserve(radii.size());
    for (std::size_t i = 0; i < radii.size(); ++i) {
        weights.push_back(std::max(1.0, static_cast<double>(pathSectors[i])
            + 0.05 * static_cast<double>(pathPoints[i])));
    }
    const XianXingNiHe coneFit = fitLinear(depths, radii, weights, 0, radii.size());
    if (coneFit.valid) {
        profile.coneWeightedRmse = coneFit.rmse;
        profile.coneBic = weightedBic(coneFit.sse, radii.size(), 2);
    }

    double bestSegmentedBic = std::numeric_limits<double>::infinity();
    std::size_t bestBreak = 0;
    XianXingNiHe bestFront;
    XianXingNiHe bestTail;
    double bestTailRadius = 0.0;
    double bestTailStd = 0.0;
    double bestTailCoverage = 0.0;
    double bestSegmentedSse = 0.0;
    if (radii.size() >= 7) {
        for (std::size_t split = 2; split + 4 <= radii.size(); ++split) {
            const XianXingNiHe front = fitLinear(depths, radii, weights, 0, split);
            const XianXingNiHe tail = fitLinear(depths, radii, weights, split, radii.size());
            if (!front.valid || !tail.valid) continue;
            std::vector<double> tailValues(radii.begin() + static_cast<std::ptrdiff_t>(split), radii.end());
            const double tailRadius = median(tailValues);
            double tailSse = 0.0;
            double tailVariance = 0.0;
            double tailWeight = 0.0;
            double sectorSum = 0.0;
            for (std::size_t i = split; i < radii.size(); ++i) {
                const double error = radii[i] - tailRadius;
                tailSse += weights[i] * error * error;
                tailVariance += error * error;
                tailWeight += weights[i];
                sectorSum += static_cast<double>(pathSectors[i]);
            }
            const double segmentedSse = front.sse + tailSse;
            const double bic = weightedBic(segmentedSse, radii.size(), 4);
            if (bic < bestSegmentedBic) {
                bestSegmentedBic = bic;
                bestBreak = split;
                bestFront = front;
                bestTail = tail;
                bestTailRadius = tailRadius;
                bestTailStd = std::sqrt(tailVariance / static_cast<double>(radii.size() - split));
                bestTailCoverage = sectorSum
                    / (static_cast<double>(radii.size() - split) * static_cast<double>(kSectors));
                bestSegmentedSse = segmentedSse;
                (void)tailWeight;
            }
        }
    }

    if (bestBreak > 0 && finite(bestSegmentedBic)) {
        profile.segmentedModelValid = true;
        profile.breakLayer = static_cast<int>(bestBreak);
        profile.platformLayers = static_cast<int>(radii.size() - bestBreak);
        profile.platformDepthSpan = depths.back() - depths[bestBreak];
        profile.segmentedBic = bestSegmentedBic;
        profile.segmentedWeightedRmse = std::sqrt(
            bestSegmentedSse / std::max(1.0, std::accumulate(weights.begin(), weights.end(), 0.0)));
        profile.bicImprovement = profile.coneBic - profile.segmentedBic;
        profile.chamferSlope = bestFront.slope;
        profile.plateauRadius = bestTailRadius;
        profile.plateauSlope = bestTail.slope;
        profile.plateauStd = bestTailStd;
        profile.plateauCoverage = bestTailCoverage;
        const double entranceRadius = median(std::vector<double>(
            radii.begin(), radii.begin() + static_cast<std::ptrdiff_t>(bestBreak)));
        const double entranceDrop = entranceRadius - bestTailRadius;
        profile.platformEntranceDrop = entranceDrop;
        profile.platformConfidence = platformConfidenceScore(
            profile.platformLayers, profile.platformDepthSpan, profile.plateauSlope,
            profile.plateauStd, profile.plateauCoverage, entranceDrop,
            profile.plateauRadius, geometry.topRadius, profile.breakLayer);
        const bool segmentedPlatform = profile.platformLayers >= 5
            && profile.platformDepthSpan >= 1.00
            && std::abs(profile.plateauSlope) <= 0.20
            && profile.plateauStd <= std::max(0.22, 0.04 * geometry.topRadius)
            && profile.chamferSlope <= -0.45
            && entranceDrop >= std::max(0.55, 0.08 * geometry.topRadius)
            && profile.plateauCoverage >= 0.12
            && profile.bicImprovement >= 2.0
            && profile.platformConfidence >= 0.52;
        profile.chamferPlatformStraight = segmentedPlatform;
    }

    if (platformEvidence.valid) {
        const bool strongerPlatform = !profile.chamferPlatformStraight
            || platformEvidence.score > 0.0;
        if (strongerPlatform) {
            profile.chamferPlatformStraight = true;
            profile.segmentedModelValid = true;
            profile.breakLayer = platformEvidence.breakLayer;
            profile.platformLayers = platformEvidence.layers;
            profile.platformDepthSpan = platformEvidence.depthSpan;
            profile.plateauRadius = platformEvidence.radius;
            profile.plateauSlope = platformEvidence.slope;
            profile.plateauStd = platformEvidence.stddev;
            profile.plateauCoverage = platformEvidence.coverage;
            profile.platformEntranceDrop = platformEvidence.entranceDrop;
            profile.platformConfidence = platformConfidenceScore(
                profile.platformLayers, profile.platformDepthSpan, profile.plateauSlope,
                profile.plateauStd, profile.plateauCoverage, profile.platformEntranceDrop,
                profile.plateauRadius, geometry.topRadius, profile.breakLayer);
            if (profile.platformConfidence < 0.52)
                profile.chamferPlatformStraight = false;
        }
    }

    if (profile.chamferPlatformStraight
        && profile.layers >= 8
        && profile.shrink >= 0.30 * geometry.topRadius) {
        profile.chamferPlatformStraight = false;
    }
    if (profile.chamferPlatformStraight) profile.cone = false;

    const std::size_t tailCount = std::min<std::size_t>(4, radii.size());
    const auto tailBegin = radii.end() - static_cast<std::ptrdiff_t>(tailCount);
    const auto minMax = std::minmax_element(tailBegin, radii.end());
    profile.stable = tailCount >= 2
        && (*minMax.second - *minMax.first) <= 0.28
        && std::abs(profile.slope) <= 0.20;
    return profile;
}

}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry)
{
    Result result;
    result.inputType = geometry.holeType;
    result.holeType = geometry.holeType;

    if (!geometry.valid || (geometry.holeType != 1 && geometry.holeType != 2)
        || !finite(geometry.centerU) || !finite(geometry.centerV)
        || !finite(geometry.topW) || !finite(geometry.topRadius)
        || geometry.topRadius <= 0.0 || samples.size() < 20) {
        result.reason = "MultiSectionType_INVALID_INPUT";
        return result;
    }

    const PouMian positive = evaluatePolarity(samples, geometry, 1);
    const PouMian negative = evaluatePolarity(samples, geometry, -1);

    const PouMian* selected = &positive;

    if (positive.chamferPlatformStraight != negative.chamferPlatformStraight) {
        const PouMian& chamferSide =
            positive.chamferPlatformStraight ? positive : negative;
        const PouMian& otherSide =
            positive.chamferPlatformStraight ? negative : positive;

        const bool strongOppositeCone = otherSide.cone
            && otherSide.layers >= 6
            && otherSide.shrink >= 0.25 * geometry.topRadius
            && geometry.holeType == 2
            && geometry.bottomValid
            && (geometry.topRadius - geometry.bottomRadius) >= 0.25;
        selected = strongOppositeCone ? &otherSide : &chamferSide;
    } else if (positive.cone != negative.cone) {
        selected = positive.cone ? &positive : &negative;
    } else if (!positive.valid || (negative.valid && negative.score > positive.score)) {
        selected = &negative;
    }
    const PouMian& profile = *selected;

    result.valid = true;
    result.profileValid = profile.valid;
    result.sustainedCone = profile.cone;
    result.straightByProfile = !profile.cone;
    result.polarity = profile.polarity;
    result.layers = profile.layers;
    result.distributedDrops = profile.distributedDrops;
    result.depthSpan = profile.depthSpan;
    result.firstRadius = profile.firstRadius;
    result.lastRadius = profile.lastRadius;
    result.slope = profile.slope;
    result.shrink = profile.shrink;
    result.shrinkRatio = geometry.topRadius > 1e-9
        ? profile.shrink / geometry.topRadius : 0.0;
    result.monotonicRatio = profile.monotonic;
    result.maxStepDrop = profile.maxStepDrop;
    result.requiredShrink = std::max(0.45, 0.12 * geometry.topRadius);
    result.chamferPlatformStraight = profile.chamferPlatformStraight;
    result.segmentedModelValid = profile.segmentedModelValid;
    result.breakLayer = profile.breakLayer;
    result.platformLayers = profile.platformLayers;
    result.platformDepthSpan = profile.platformDepthSpan;
    result.coneWeightedRmse = profile.coneWeightedRmse;
    result.coneBic = profile.coneBic;
    result.segmentedWeightedRmse = profile.segmentedWeightedRmse;
    result.segmentedBic = profile.segmentedBic;
    result.bicImprovement = profile.bicImprovement;
    result.chamferSlope = profile.chamferSlope;
    result.plateauRadius = profile.plateauRadius;
    result.plateauSlope = profile.plateauSlope;
    result.plateauStd = profile.plateauStd;
    result.plateauCoverage = profile.plateauCoverage;
    result.platformEntranceDrop = profile.platformEntranceDrop;
    result.platformConfidence = profile.platformConfidence;
    result.profileDepths = profile.depths;
    result.profileRadii = profile.radii;
    result.profilePoints = profile.points;
    result.profileSectors = profile.sectors;
    result.robustBaseCone = geometry.base.valid
        && geometry.base.holeType == 2
        && geometry.base.profileValid
        && geometry.base.validLayers >= 4
        && geometry.base.radiusSlope < -0.25
        && geometry.base.radiusShrink >= 0.50
        && geometry.base.monotonicRatio >= 0.75;
    result.robustRecoveredWall = geometry.takeoverMode
            == "FinalGeometry_STRONG_WALL_CONE_BEFORE_VOID_STRAIGHT"
        && geometry.wallEvidenceValid
        && geometry.wallEvidenceCone
        && geometry.wallEvidenceLayers >= 4
        && geometry.wallEvidenceSlope < -0.30
        && geometry.wallEvidenceShrink >= 0.45
        && geometry.wallEvidenceMonotonic >= 0.75;

    result.holeType = result.chamferPlatformStraight
        ? 1
        : ((profile.cone || result.robustBaseCone || result.robustRecoveredWall) ? 2 : 1);
    result.typeChanged = result.holeType != result.inputType;
    if (result.chamferPlatformStraight) {
        result.mode = "MultiSectionType_CHAMFER_PLATFORM_TO_STRAIGHT";
        result.reason = "MultiSectionType_ENTRANCE_TAPER_FOLLOWED_BY_STABLE_INNER_WALL";
    } else if (result.holeType == 2) {
        if (profile.cone) {
            result.mode = result.inputType == 1
                ? "MultiSectionType_SUSTAINED_PROFILE_STRAIGHT_TO_CONE"
                : "MultiSectionType_KEEP_SUSTAINED_CONE";
            result.reason = "MultiSectionType_DISTRIBUTED_MULTI_LAYER_TAPER";
        } else if (result.robustRecoveredWall) {
            result.mode = "MultiSectionType_KEEP_STRONG_RECOVERED_WALL_CONE";
            result.reason = "MultiSectionType_RECOVERED_WALL_MULTI_LAYER_TAPER";
        } else {
            result.mode = "MultiSectionType_KEEP_ROBUST_BASE_CONE";
            result.reason = "MultiSectionType_ROBUST_BASE_MULTI_LAYER_TAPER";
        }
    } else {
        result.mode = result.inputType == 2
            ? "MultiSectionType_UNSUPPORTED_CONE_TO_STRAIGHT"
            : "MultiSectionType_KEEP_PROFILE_STRAIGHT";
        result.reason = profile.valid
            ? "MultiSectionType_NO_SUSTAINED_MULTI_LAYER_TAPER"
            : "MultiSectionType_INSUFFICIENT_TAPER_LAYERS";
    }
    return result;
}

}


// ============================================================================
// 功能分区：孔形多截面一致性分类
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/

namespace HoleLeixingPouMianGongShi {
namespace {

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“isShortStrongBase”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isShortStrongBase(const HoleLeixingPouMianBase::Result& base,
                       double topRadius) noexcept
{
    if (!base.valid || !base.profileValid || !finite(topRadius) || topRadius <= 0.0)
        return false;

    const double requiredShrink = std::max(0.70, 0.18 * topRadius);
    return base.layers >= 4
        && base.depthSpan >= 0.70
        && base.slope <= -1.00
        && base.shrink >= requiredShrink
        && base.distributedDrops >= 2
        && base.monotonicRatio >= 0.60;
}

/** 【函数导航】
 * 作用：执行“isDirectionalBaseTaper”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isDirectionalBaseTaper(const HoleLeixingPouMianBase::Result& base,
                            double topRadius) noexcept
{
    if (!base.valid || !base.profileValid || !finite(topRadius) || topRadius <= 0.0)
        return false;

    const double requiredShrink = std::max(0.30, 0.07 * topRadius);
    const double allowedMaxStep = std::max(1.10, 0.90 * base.shrink + 0.10);
    return base.layers >= 4
        && base.depthSpan >= 0.70
        && base.slope <= -0.35
        && base.shrink >= requiredShrink
        && base.distributedDrops >= 1
        && base.monotonicRatio >= 0.55
        && base.maxStepDrop <= allowedMaxStep;
}

/** 【函数导航】
 * 作用：执行“isStrongPlatform”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isStrongPlatform(const HoleLeixingPouMianBase::Result& profile,
                      double topRadius) noexcept
{
    if (!profile.valid || !profile.profileValid || !profile.segmentedModelValid
        || !finite(topRadius) || topRadius <= 0.0) return false;
    const double ratio = profile.plateauRadius / topRadius;
    return profile.platformLayers >= 5
        && profile.platformDepthSpan >= 1.0
        && std::abs(profile.plateauSlope) <= 0.22
        && profile.plateauStd <= std::max(0.24, 0.045 * topRadius)
        && profile.plateauCoverage >= 0.10
        && profile.platformEntranceDrop >= std::max(0.50, 0.075 * topRadius)
        && profile.platformConfidence >= 0.50
        && ratio >= 0.32 && ratio <= 0.94;
}

/** 【函数导航】
 * 作用：执行“interpolateRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double interpolateRadius(const HoleLeixingPouMianBase::Result& profile,
                         double physicalDepth,
                         double depthOffset,
                         bool* validOut) noexcept
{
    if (validOut) *validOut = false;
    if (profile.profileDepths.size() != profile.profileRadii.size()
        || profile.profileDepths.size() < 2) return 0.0;
    const double localDepth = physicalDepth - depthOffset;
    if (localDepth < profile.profileDepths.front() - 1e-9
        || localDepth > profile.profileDepths.back() + 1e-9) return 0.0;
    for (std::size_t i = 1; i < profile.profileDepths.size(); ++i) {
        const double d0 = profile.profileDepths[i - 1];
        const double d1 = profile.profileDepths[i];
        if (localDepth < d0 - 1e-9 || localDepth > d1 + 1e-9) continue;
        const double t = d1 > d0 + 1e-12 ? (localDepth - d0) / (d1 - d0) : 0.0;
        if (validOut) *validOut = true;
        return profile.profileRadii[i - 1] * (1.0 - t) + profile.profileRadii[i] * t;
    }
    if (std::abs(localDepth - profile.profileDepths.back()) <= 1e-9) {
        if (validOut) *validOut = true;
        return profile.profileRadii.back();
    }
    return 0.0;
}

/** 【类型导航注释】
 * XianXingNiHe：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct XianXingNiHe {
    bool valid = false;
    double slope = 0.0;
    double intercept = 0.0;
    double rmse = 0.0;
};

/** 【函数导航】
 * 作用：拟合/求解“fitLineRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XianXingNiHe fitLineRange(const std::vector<double>& x,
                       const std::vector<double>& y,
                       std::size_t begin,
                       std::size_t end) noexcept
{
    XianXingNiHe out;
    if (x.size() != y.size() || begin >= end || end > x.size()
        || end - begin < 2) return out;
    const std::size_t count = end - begin;
    double meanX = 0.0;
    double meanY = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        meanX += x[i];
        meanY += y[i];
    }
    meanX /= static_cast<double>(count);
    meanY /= static_cast<double>(count);
    double varX = 0.0;
    double covXY = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double dx = x[i] - meanX;
        varX += dx * dx;
        covXY += dx * (y[i] - meanY);
    }
    if (varX <= 1e-12) return out;
    out.slope = covXY / varX;
    out.intercept = meanY - out.slope * meanX;
    double squared = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double error = y[i] - (out.intercept + out.slope * x[i]);
        squared += error * error;
    }
    out.rmse = std::sqrt(squared / static_cast<double>(count));
    out.valid = finite(out.slope) && finite(out.intercept) && finite(out.rmse);
    return out;
}

/** 【函数导航】
 * 作用：拟合/求解“fitLine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XianXingNiHe fitLine(const std::vector<double>& x,
                  const std::vector<double>& y) noexcept
{
    return fitLineRange(x, y, 0, x.size());
}

double median(std::vector<double> values) noexcept
{
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if ((values.size() & 1U) != 0U) return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

/** 【类型导航注释】
 * ZhuiShiftAgreement：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhuiShiftAgreement {
    bool valid = false;
    int layers = 0;
    double span = 0.0;
    double rawRmse = 0.0;
    double alignedRmse = 0.0;
    double radiusOffset = 0.0;
    double slopeDelta = 0.0;
    double slopeRelativeDifference = 0.0;
    double baseSlope = 0.0;
    double axialSlope = 0.0;
    double sharedSlope = 0.0;
    double baseIntercept = 0.0;
    double axialIntercept = 0.0;
    double sharedRmse = 0.0;
};

/** 【函数导航】
 * 作用：评估/审核“evaluateConeShiftAgreement”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ZhuiShiftAgreement evaluateConeShiftAgreement(
    const HoleLeixingPouMianBase::Result& base,
    const HoleLeixingPouMianBase::Result& axial,
    double axialShift,
    double topRadius) noexcept
{
    ZhuiShiftAgreement out;
    if (!base.valid || !axial.valid || !base.profileValid || !axial.profileValid
        || !base.sustainedCone || !axial.sustainedCone
        || base.chamferPlatformStraight || axial.chamferPlatformStraight
        || !finite(topRadius) || topRadius <= 0.0
        || base.profileDepths.size() != base.profileRadii.size()
        || axial.profileDepths.size() != axial.profileRadii.size()
        || base.profileDepths.size() < 4 || axial.profileDepths.size() < 4) return out;

    std::vector<double> overlapDepths;
    std::vector<double> baseAtOverlap;
    std::vector<double> axialAtOverlap;
    std::vector<double> radiusDifferences;
    const std::size_t overlapCapacity = axial.profileDepths.size();
    overlapDepths.reserve(overlapCapacity);
    baseAtOverlap.reserve(overlapCapacity);
    axialAtOverlap.reserve(overlapCapacity);
    radiusDifferences.reserve(overlapCapacity);
    for (std::size_t i = 0; i < axial.profileDepths.size(); ++i) {
        const double physicalDepth = axial.profileDepths[i] + axialShift;
        bool validBase = false;
        const double baseRadius = interpolateRadius(base, physicalDepth, 0.0, &validBase);
        if (!validBase) continue;
        overlapDepths.push_back(physicalDepth);
        baseAtOverlap.push_back(baseRadius);
        axialAtOverlap.push_back(axial.profileRadii[i]);
        radiusDifferences.push_back(baseRadius - axial.profileRadii[i]);
    }
    out.layers = static_cast<int>(overlapDepths.size());
    if (out.layers < 4) return out;
    out.span = overlapDepths.back() - overlapDepths.front();
    out.radiusOffset = median(radiusDifferences);

    double rawSquared = 0.0;
    double alignedSquared = 0.0;
    for (std::size_t i = 0; i < radiusDifferences.size(); ++i) {
        const double rawError = axialAtOverlap[i] - baseAtOverlap[i];
        const double alignedError = (axialAtOverlap[i] + out.radiusOffset) - baseAtOverlap[i];
        rawSquared += rawError * rawError;
        alignedSquared += alignedError * alignedError;
    }
    out.rawRmse = std::sqrt(rawSquared / static_cast<double>(out.layers));
    out.alignedRmse = std::sqrt(alignedSquared / static_cast<double>(out.layers));

    const std::vector<double>& basePhysicalDepths = base.profileDepths;
    std::vector<double> axialPhysicalDepths;
    axialPhysicalDepths.reserve(axial.profileDepths.size());
    for (double depth : axial.profileDepths) axialPhysicalDepths.push_back(depth + axialShift);
    const XianXingNiHe baseFit = fitLine(basePhysicalDepths, base.profileRadii);
    const XianXingNiHe axialFit = fitLine(axialPhysicalDepths, axial.profileRadii);
    if (!baseFit.valid || !axialFit.valid) return out;
    out.baseSlope = baseFit.slope;
    out.axialSlope = axialFit.slope;
    out.baseIntercept = baseFit.intercept;
    out.axialIntercept = axialFit.intercept;
    out.slopeDelta = std::abs(out.baseSlope - out.axialSlope);
    const double slopeScale = std::max({0.25, std::abs(out.baseSlope), std::abs(out.axialSlope)});
    out.slopeRelativeDifference = out.slopeDelta / slopeScale;

    double numerator = 0.0;
    double denominator = 0.0;
    auto addCentered = [&](const std::vector<double>& x, const std::vector<double>& y) {
        double meanX = 0.0;
        double meanY = 0.0;
        for (std::size_t i = 0; i < x.size(); ++i) {
            meanX += x[i];
            meanY += y[i];
        }
        meanX /= static_cast<double>(x.size());
        meanY /= static_cast<double>(y.size());
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double dx = x[i] - meanX;
            numerator += dx * (y[i] - meanY);
            denominator += dx * dx;
        }
    };
    addCentered(basePhysicalDepths, base.profileRadii);
    addCentered(axialPhysicalDepths, axial.profileRadii);
    if (denominator <= 1e-12) return out;
    out.sharedSlope = numerator / denominator;
    double baseMeanX = 0.0, baseMeanY = 0.0;
    double axialMeanX = 0.0, axialMeanY = 0.0;
    for (std::size_t i = 0; i < basePhysicalDepths.size(); ++i) {
        baseMeanX += basePhysicalDepths[i];
        baseMeanY += base.profileRadii[i];
    }
    for (std::size_t i = 0; i < axialPhysicalDepths.size(); ++i) {
        axialMeanX += axialPhysicalDepths[i];
        axialMeanY += axial.profileRadii[i];
    }
    baseMeanX /= static_cast<double>(basePhysicalDepths.size());
    baseMeanY /= static_cast<double>(base.profileRadii.size());
    axialMeanX /= static_cast<double>(axialPhysicalDepths.size());
    axialMeanY /= static_cast<double>(axial.profileRadii.size());
    out.baseIntercept = baseMeanY - out.sharedSlope * baseMeanX;
    out.axialIntercept = axialMeanY - out.sharedSlope * axialMeanX;
    double jointSquared = 0.0;
    std::size_t jointCount = 0;
    for (std::size_t i = 0; i < basePhysicalDepths.size(); ++i) {
        const double error = base.profileRadii[i]
            - (out.baseIntercept + out.sharedSlope * basePhysicalDepths[i]);
        jointSquared += error * error;
        ++jointCount;
    }
    for (std::size_t i = 0; i < axialPhysicalDepths.size(); ++i) {
        const double error = axial.profileRadii[i]
            - (out.axialIntercept + out.sharedSlope * axialPhysicalDepths[i]);
        jointSquared += error * error;
        ++jointCount;
    }
    out.sharedRmse = jointCount > 0
        ? std::sqrt(jointSquared / static_cast<double>(jointCount)) : 0.0;

    const double alignedRmseLimit = std::max(0.34, 0.060 * topRadius);
    const double sharedRmseLimit = std::max(0.40, 0.075 * topRadius);
    const double slopeDeltaLimit = std::max(0.55, 0.35 * slopeScale);
    out.valid = out.span >= 0.70
        && out.baseSlope <= -0.25
        && out.axialSlope <= -0.25
        && out.sharedSlope <= -0.25
        && out.alignedRmse <= alignedRmseLimit
        && out.sharedRmse <= sharedRmseLimit
        && out.slopeDelta <= slopeDeltaLimit
        && out.slopeRelativeDifference <= 0.38
        && base.monotonicRatio >= 0.70
        && axial.monotonicRatio >= 0.70;
    return out;
}

/** 【类型导航注释】
 * FeiHoleBiZhengJu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct FeiHoleBiZhengJu {
    bool terminalCollapse = false;
    bool mouthOvershoot = false;
    bool lateSectorDrop = false;
    bool latePointDrop = false;
    bool terminalTrendDeviation = false;
    int failureCount = 0;
    double lateSectorRatio = 1.0;
    double latePointRatio = 1.0;
    double terminalTrendError = 0.0;
};

/** 【函数导航】
 * 作用：执行“meanRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double meanRange(const std::vector<int>& values,
                 std::size_t begin, std::size_t end) noexcept
{
    if (values.empty() || begin >= end || end > values.size()) return 0.0;
    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i) sum += static_cast<double>(values[i]);
    return sum / static_cast<double>(end - begin);
}

/** 【函数导航】
 * 作用：评估/审核“evaluateNonWallEvidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
FeiHoleBiZhengJu evaluateNonWallEvidence(const HoleLeixingPouMianBase::Result& axial,
                                        double topRadius) noexcept
{
    FeiHoleBiZhengJu out;
    if (!axial.valid || !axial.profileValid || !finite(topRadius) || topRadius <= 0.0)
        return out;
    const double terminalRatio = axial.lastRadius / topRadius;
    const double mouthOvershoot = axial.firstRadius - topRadius;
    out.terminalCollapse = terminalRatio <= 0.35
        && axial.lastRadius <= std::max(1.35, 0.35 * topRadius);
    out.mouthOvershoot = mouthOvershoot >= std::max(0.85, 0.18 * topRadius);

    const std::size_t n = axial.profileRadii.size();
    if (n >= 6) {
        const std::size_t headEnd = std::min<std::size_t>(3, n);
        const std::size_t tailBegin = n - std::min<std::size_t>(3, n);
        if (axial.profileSectors.size() == n) {
            const double head = meanRange(axial.profileSectors, 0, headEnd);
            const double tail = meanRange(axial.profileSectors, tailBegin, n);
            if (head > 1e-9) out.lateSectorRatio = tail / head;
            out.lateSectorDrop = head >= 12.0
                && (out.lateSectorRatio <= 0.72 || head - tail >= 8.0);
        }
        if (axial.profilePoints.size() == n) {
            const double head = meanRange(axial.profilePoints, 0, headEnd);
            const double tail = meanRange(axial.profilePoints, tailBegin, n);
            if (head > 1e-9) out.latePointRatio = tail / head;
            out.latePointDrop = head >= 20.0
                && (out.latePointRatio <= 0.48 || head - tail >= 45.0);
        }
        const XianXingNiHe prefixFit = fitLineRange(
            axial.profileDepths, axial.profileRadii, 0, n - 2);
        if (prefixFit.valid) {
            double error = 0.0;
            for (std::size_t i = n - 2; i < n; ++i) {
                const double predicted = prefixFit.intercept + prefixFit.slope * axial.profileDepths[i];
                error += std::abs(axial.profileRadii[i] - predicted);
            }
            out.terminalTrendError = error / 2.0;
            out.terminalTrendDeviation = out.terminalTrendError
                >= std::max(0.42, 0.080 * topRadius);
        }
    }
    out.failureCount = static_cast<int>(out.terminalCollapse)
        + static_cast<int>(out.mouthOvershoot)
        + static_cast<int>(out.lateSectorDrop)
        + static_cast<int>(out.latePointDrop)
        + static_cast<int>(out.terminalTrendDeviation);
    return out;
}

/** 【函数导航】
 * 作用：执行“hasCompatibleTailPlatform”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool hasCompatibleTailPlatform(const HoleLeixingPouMianBase::Result& profile,
                               double referenceRadius, double topRadius,
                               double* deltaOut) noexcept
{
    if (deltaOut) *deltaOut = 0.0;
    if (!profile.valid || profile.profileRadii.size() < 4
        || !finite(referenceRadius) || referenceRadius <= 0.0
        || !finite(topRadius) || topRadius <= 0.0) return false;
    const std::size_t count = std::min<std::size_t>(5, profile.profileRadii.size());
    const std::size_t begin = profile.profileRadii.size() - count;
    std::array<double, 5> tail{};
    for (std::size_t i = 0; i < count; ++i)
        tail[i] = profile.profileRadii[begin + i];

    for (std::size_t i = 1; i < count; ++i) {
        const double value = tail[i];
        std::size_t j = i;
        while (j > 0 && value < tail[j - 1]) {
            tail[j] = tail[j - 1];
            --j;
        }
        tail[j] = value;
    }
    const double medianRadius = tail[count / 2];
    double variance = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const double error = tail[i] - medianRadius;
        variance += error * error;
    }
    const double stddev = std::sqrt(variance / static_cast<double>(count));
    const double delta = std::abs(medianRadius - referenceRadius);
    if (deltaOut) *deltaOut = delta;
    double slope = 0.0;
    if (profile.profileDepths.size() == profile.profileRadii.size() && count >= 2) {
        const double x0 = profile.profileDepths[begin];
        const double x1 = profile.profileDepths.back();
        if (x1 > x0 + 1e-9) slope = (profile.profileRadii.back() - profile.profileRadii[begin]) / (x1 - x0);
    }
    return delta <= std::max(0.40, 0.07 * topRadius)
        && stddev <= std::max(0.28, 0.05 * topRadius)
        && std::abs(slope) <= 0.30;
}

/** 【函数导航】
 * 作用：执行“isAxialOverCollapse”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isAxialOverCollapse(const HoleLeixingPouMianBase::Result& axial,
                         double topRadius) noexcept
{
    if (!axial.valid || !axial.profileValid || !axial.sustainedCone
        || !finite(topRadius) || topRadius <= 0.0
        || !finite(axial.lastRadius) || !finite(axial.shrink)) {
        return false;
    }

    const double terminalRatio = axial.lastRadius / topRadius;
    const double shrinkRatio = axial.shrink / topRadius;
    const double mouthOvershoot = axial.firstRadius - topRadius;
    const bool deepLongTrack = axial.layers >= 8 && axial.depthSpan >= 1.75;
    const bool terminalCollapse = terminalRatio <= 0.35
        && axial.lastRadius <= std::max(1.35, 0.35 * topRadius);
    const bool startsOutsideLockedMouth = mouthOvershoot
        >= std::max(0.85, 0.18 * topRadius);

    return deepLongTrack
        && (shrinkRatio >= 0.88
            || (shrinkRatio >= 0.77
                && (terminalCollapse || startsOutsideLockedMouth)));
}

}

/** 【函数导航】
 * 作用：评估/审核“classifyProfiles”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result classifyProfiles(const HoleLeixingPouMianBase::Result& base,
                        const HoleLeixingPouMianBase::Result& axial,
                        const HoleJiheFinal::Result& geometry,
                        double axialShift)
{
    Result result;
    result.inputType = geometry.holeType;
    result.holeType = geometry.holeType;
    result.axialShift = axialShift;
    result.base = base;
    result.axial = axial;

    if (!geometry.valid || (geometry.holeType != 1 && geometry.holeType != 2)
        || !finite(geometry.topRadius) || geometry.topRadius <= 0.0
        || !base.valid || !axial.valid) {
        result.reason = "HoleTypeReview_INVALID_PROFILE_INPUT";
        return result;
    }

    result.valid = true;
    result.baseType = base.holeType;
    result.axialType = axial.holeType;
    result.shortStrongBase = isShortStrongBase(base, geometry.topRadius);
    result.baseDirectionalTaper = isDirectionalBaseTaper(base, geometry.topRadius);
    result.axialOverCollapseRaw = isAxialOverCollapse(axial, geometry.topRadius);

    result.axialOverCollapse = result.axialOverCollapseRaw;
    result.axialTerminalRadius = finite(axial.lastRadius) ? axial.lastRadius : 0.0;
    result.axialTerminalRatio = geometry.topRadius > 1e-9
        ? result.axialTerminalRadius / geometry.topRadius : 0.0;
    result.axialShrinkRatio = geometry.topRadius > 1e-9
        ? axial.shrink / geometry.topRadius : 0.0;

    result.directionalRecoveryMinShrink = std::max(0.55, 0.12 * geometry.topRadius);
    result.directionalRecoveryEvidenceStrong = axial.valid
        && axial.profileValid
        && axial.holeType == 2
        && axial.layers >= 4
        && axial.depthSpan >= 0.70
        && axial.monotonicRatio >= 0.67
        && axial.shrink >= result.directionalRecoveryMinShrink
        && result.axialShrinkRatio >= result.directionalRecoveryMinShrinkRatio
        && result.axialTerminalRatio <= result.directionalRecoveryMaxTerminalRatio;

    result.baseStrongPlatform = isStrongPlatform(base, geometry.topRadius);
    result.axialStrongPlatform = isStrongPlatform(axial, geometry.topRadius);
    result.wallStraightConsensus = geometry.wallEvidenceValid
        && geometry.wallEvidenceStable
        && !geometry.wallEvidenceCone
        && geometry.wallEvidenceLayers >= 4
        && std::abs(geometry.wallEvidenceSlope) <= 0.16
        && std::abs(geometry.wallEvidenceShrink) <= 0.28;
    result.geometryStraightConsensus = geometry.holeType == 1
        && (result.wallStraightConsensus
            || geometry.takeoverMode == "FinalGeometry_FAST_STRAIGHT_CONSENSUS"
            || geometry.takeoverMode == "FinalGeometry_VOID_STABLE_STRAIGHT"
            || geometry.takeoverMode == "FinalGeometry_VOID_STABLE_STRAIGHT_OVERRIDE_WEAK_CONE");
    double baseToAxialDelta = 0.0;
    double axialToBaseDelta = 0.0;
    const bool basePlatformConfirmed = result.baseStrongPlatform
        && hasCompatibleTailPlatform(axial, base.plateauRadius, geometry.topRadius,
                                     &baseToAxialDelta);
    const bool axialPlatformConfirmed = result.axialStrongPlatform
        && hasCompatibleTailPlatform(base, axial.plateauRadius, geometry.topRadius,
                                     &axialToBaseDelta);
    result.crossShiftPlatformStraight = basePlatformConfirmed || axialPlatformConfirmed;
    if (basePlatformConfirmed) {
        result.platformReferenceRadius = base.plateauRadius;
        result.platformCrossShiftDelta = baseToAxialDelta;
    } else if (axialPlatformConfirmed) {
        result.platformReferenceRadius = axial.plateauRadius;
        result.platformCrossShiftDelta = axialToBaseDelta;
    }

    const ZhuiShiftAgreement coneAgreement = evaluateConeShiftAgreement(
        base, axial, axialShift, geometry.topRadius);
    result.crossShiftConeConsistent = coneAgreement.valid;
    result.affineConeConsistent = coneAgreement.valid;
    result.crossShiftConeOverlapLayers = coneAgreement.layers;
    result.crossShiftConeOverlapSpan = coneAgreement.span;
    result.crossShiftConeRmse = coneAgreement.rawRmse;
    result.crossShiftConeSlopeDelta = coneAgreement.slopeDelta;
    result.affineAlignedRmse = coneAgreement.alignedRmse;
    result.affineRadiusOffset = coneAgreement.radiusOffset;
    result.affineBaseSlope = coneAgreement.baseSlope;
    result.affineAxialSlope = coneAgreement.axialSlope;
    result.affineSharedSlope = coneAgreement.sharedSlope;
    result.affineBaseIntercept = coneAgreement.baseIntercept;
    result.affineAxialIntercept = coneAgreement.axialIntercept;
    result.affineSharedRmse = coneAgreement.sharedRmse;
    result.affineSlopeRelativeDifference = coneAgreement.slopeRelativeDifference;

    const FeiHoleBiZhengJu nonWall = evaluateNonWallEvidence(axial, geometry.topRadius);
    result.terminalCollapseEvidence = nonWall.terminalCollapse;
    result.mouthOvershootEvidence = nonWall.mouthOvershoot;
    result.lateSectorDrop = nonWall.lateSectorDrop;
    result.latePointDrop = nonWall.latePointDrop;
    result.terminalTrendDeviation = nonWall.terminalTrendDeviation;
    const bool crossShiftShapeMismatch = !result.affineConeConsistent
        && coneAgreement.layers >= 4;
    result.crossShiftShapeMismatch = crossShiftShapeMismatch;
    result.nonWallQualityFailureCount = nonWall.failureCount
        + static_cast<int>(crossShiftShapeMismatch);
    result.lateSectorRatio = nonWall.lateSectorRatio;
    result.latePointRatio = nonWall.latePointRatio;
    result.terminalTrendError = nonWall.terminalTrendError;
    const bool summaryOnlyNonWallTuoDi = result.axialOverCollapseRaw
        && (axial.profileDepths.size() < 4
            || axial.profileDepths.size() != axial.profileRadii.size());
    result.summaryOnlyNonWallTuoDi = summaryOnlyNonWallTuoDi;

    constexpr double kExtremeShrinkRatio = 0.88;
    constexpr double kExtremeTerminalTrendError = 0.45;
    result.extremeShrinkThreshold = kExtremeShrinkRatio;
    result.extremeTerminalTrendErrorThreshold = kExtremeTerminalTrendError;
    result.extremeNonWallCollapseConfirmed = result.axialOverCollapseRaw
        && result.axialShrinkRatio >= kExtremeShrinkRatio
        && result.nonWallQualityFailureCount >= 4
        && result.terminalCollapseEvidence
        && result.mouthOvershootEvidence
        && result.terminalTrendError >= kExtremeTerminalTrendError;
    result.affineConeBlockedByExtremeNonWall = result.affineConeConsistent
        && result.extremeNonWallCollapseConfirmed;

    const bool ordinaryNonWallCollapse = result.axialOverCollapseRaw
        && !result.affineConeConsistent
        && (result.nonWallQualityFailureCount >= 2
            || summaryOnlyNonWallTuoDi);
    result.nonWallCollapseConfirmed = result.extremeNonWallCollapseConfirmed
        || ordinaryNonWallCollapse;

    const bool usableAffineCone = result.affineConeConsistent
        && !result.affineConeBlockedByExtremeNonWall;
    result.axialConfirmedCone = axial.holeType == 2
        && (!result.nonWallCollapseConfirmed || usableAffineCone);

    if (result.crossShiftPlatformStraight) {
        result.holeType = 1;
        result.mode = "SurfaceCrossShift_CROSS_SHIFT_PLATFORM_TO_STRAIGHT";
        result.reason = "SurfaceCrossShift_ENTRANCE_TAPER_FOLLOWED_BY_CROSS_SHIFT_STABLE_INNER_WALL";
    } else if (result.geometryStraightConsensus) {
        result.holeType = 1;
        result.mode = result.wallStraightConsensus
            ? "CanonicalExpansion_STABLE_WALL_CONSENSUS_TO_STRAIGHT"
            : "CanonicalExpansion_CLOSED_MOUTH_STRAIGHT_CONSENSUS";
        result.reason = result.wallStraightConsensus
            ? "CanonicalExpansion_MULTI_LAYER_WALL_IS_STABLE_AND_OVERRULES_WEAK_CONE_FIT"
            : "CanonicalExpansion_FinalGeometry_CLOSED_MOUTH_AND_BASE_GEOMETRY_CONFIRM_STRAIGHT";
    } else if (result.affineConeConsistent
               && !result.affineConeBlockedByExtremeNonWall
               && base.holeType == 2 && axial.holeType == 2) {
        result.holeType = 2;
        result.crossShiftConeHuiFuChengGong = result.axialOverCollapseRaw;
        result.affineConeOverruledCollapse = result.axialOverCollapseRaw;
        result.mode = result.axialOverCollapseRaw
            ? "NonWallReview_AFFINE_OVERRULES_NONEXTREME_COLLAPSE"
            : "NonWallReview_AFFINE_CROSS_SHIFT_CONE_KEEP";
        result.reason = result.axialOverCollapseRaw
            ? "NonWallReview_AFFINE_CONE_RESCUE_ALLOWED_BECAUSE_EXTREME_NONWALL_BUNDLE_IS_INCOMPLETE"
            : "NonWallReview_BASE_AND_SHIFTED_PROFILES_SHARE_ONE_AFFINE_CONE";
    } else if (base.holeType == 2) {
        if (result.nonWallCollapseConfirmed) {
            result.holeType = 1;
            result.axialUnstableConeRejected = true;
            result.mode = result.affineConeBlockedByExtremeNonWall
                ? "NonWallReview_EXTREME_NONWALL_VETOES_AFFINE_RESCUE"
                : "NonWallReview_AXIAL_NONWALL_COLLAPSE_TO_STRAIGHT";
            result.reason = result.affineConeBlockedByExtremeNonWall
                ? "NonWallReview_EXTREME_SHRINK_PLUS_FOUR_QUALITY_FAILURES_TERMINAL_COLLAPSE_MOUTH_OVERSHOOT_AND_TREND_ERROR"
                : "NonWallReview_RAW_COLLAPSE_PLUS_MULTIPLE_LATE_TRACK_QUALITY_FAILURES";
        } else if (result.axialConfirmedCone) {
            result.holeType = 2;
            result.mode = result.axialOverCollapseRaw
                ? "NonWallReview_UNCONFIRMED_COLLAPSE_KEEP_CONE"
                : "HoleTypeReview_KEEP_AXIALLY_CONFIRMED_CONE";
            result.reason = result.axialOverCollapseRaw
                ? "NonWallReview_RAW_COLLAPSE_LACKS_COMPLETE_NONWALL_EVIDENCE"
                : "HoleTypeReview_CONE_SURVIVES_SHIFT_WITH_MEASURABLE_TERMINAL_WALL";
        } else {

            const double coneBottomShrink = geometry.bottomValid
                ? geometry.topRadius - geometry.bottomRadius : 0.0;
            if (geometry.holeType == 2
                && geometry.bottomValid
                && coneBottomShrink >= 0.25
                && !result.wallStraightConsensus) {
                result.holeType = 2;
                result.axialUnstableConeRejected = false;
                result.mode = "HoleTypeReview_WEAK_AXIAL_CONE_KEPT_BY_HYSTERESIS";
                result.reason =
                    "HoleTypeReview_AXIAL_CONFIDENCE_WEAK_BUT_MultiSectionType_DUAL_CONE_AND_FinalGeometry_BOTTOM_CONTRACTION_CONSENSUS";
            } else {
                result.holeType = 1;
                result.axialUnstableConeRejected = true;
                result.mode = "HoleTypeReview_AXIAL_UNSTABLE_CONE_TO_STRAIGHT";
                result.reason = "HoleTypeReview_CONE_EXISTS_ONLY_AT_ONE_SLICE_OR_FALLBACK_PATH";
            }
        }
    } else if (result.baseDirectionalTaper && result.axialConfirmedCone
               && result.directionalRecoveryEvidenceStrong) {
        result.holeType = 2;
        result.mode = "HoleTypeReview_DIRECTIONAL_BASE_AXIAL_RECOVERY_TO_CONE";
        result.reason = "CanonicalExpansion_SHALLOW_BASE_TAPER_HAS_MEANINGFUL_SHIFTED_SHRINK_AND_TERMINAL_REDUCTION";
    } else {
        result.holeType = 1;
        result.mode = "HoleTypeReview_KEEP_PROFILE_STRAIGHT";
        if (result.nonWallCollapseConfirmed) {
            result.reason = "NonWallReview_AXIAL_TRACK_HAS_CONFIRMED_NONWALL_QUALITY_FAILURES";
        } else if (result.baseDirectionalTaper && result.axialConfirmedCone
                   && !result.directionalRecoveryEvidenceStrong) {
            result.reason = "CanonicalExpansion_SHIFTED_CONE_FIT_LACKS_MEANINGFUL_TAPER_MAGNITUDE";
        } else if (result.baseDirectionalTaper) {
            result.reason = "HoleTypeReview_DIRECTIONAL_BASE_NOT_AXIALLY_CONFIRMED";
        } else {
            result.reason = "HoleTypeReview_NO_DIRECTIONALLY_COHERENT_BASE_TAPER";
        }
    }

    result.typeChanged = result.holeType != result.inputType;
    result.changedFromMultiSectionType = result.holeType != result.baseType;
    return result;
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry)
{
    Result invalid;
    invalid.inputType = geometry.holeType;
    invalid.holeType = geometry.holeType;

    if (!geometry.valid || (geometry.holeType != 1 && geometry.holeType != 2)
        || !finite(geometry.topW) || !finite(geometry.topRadius)
        || geometry.topRadius <= 0.0 || samples.size() < 20) {
        invalid.reason = "HoleTypeReview_INVALID_INPUT";
        return invalid;
    }

    const HoleLeixingPouMianBase::Result base = HoleLeixingPouMianBase::evaluate(samples, geometry);
    if (!base.valid) {
        invalid.reason = "HoleTypeReview_BASE_PROFILE_INVALID";
        return invalid;
    }

    constexpr double kAxialShift = 0.35;
    HoleJiheFinal::Result axialGeometry = geometry;
    axialGeometry.topW += kAxialShift;
    const HoleLeixingPouMianBase::Result axial =
        HoleLeixingPouMianBase::evaluate(samples, axialGeometry);
    if (!axial.valid) {
        invalid.reason = "HoleTypeReview_AXIAL_PROFILE_INVALID";
        return invalid;
    }

    return classifyProfiles(base, axial, geometry, kAxialShift);
}

}


// ============================================================================
// 功能分区：最终孔形分类
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/

namespace HoleLeixingFinal {
namespace {

constexpr std::array<double, 5> kProfileShifts{{0.00, 0.15, 0.35, 0.55, 0.90}};
constexpr std::array<double, 3> kExtraShifts{{0.15, 0.55, 0.90}};

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“supportsConservativeVeto”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool supportsConservativeVeto(const std::string& mode)
{
    return mode == "NonWallReview_AXIAL_NONWALL_COLLAPSE_TO_STRAIGHT"
        || mode == "NonWallReview_EXTREME_NONWALL_VETOES_AFFINE_RESCUE";
}

/** 【函数导航】
 * 作用：执行“meanRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double meanRange(const std::vector<int>& values,
                 std::size_t begin, std::size_t end) noexcept
{
    if (values.empty() || begin >= end || end > values.size()) return 0.0;
    double sum = 0.0;
    for (std::size_t index = begin; index < end; ++index)
        sum += static_cast<double>(values[index]);
    return sum / static_cast<double>(end - begin);
}

/** 【函数导航】
 * 作用：执行“meanRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double meanRange(const std::vector<double>& values,
                 std::size_t begin, std::size_t end) noexcept
{
    if (values.empty() || begin >= end || end > values.size()) return 0.0;
    double sum = 0.0;
    for (std::size_t index = begin; index < end; ++index) sum += values[index];
    return sum / static_cast<double>(end - begin);
}

/** 【类型导航注释】
 * MiDuZhengJu：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct MiDuZhengJu {
    bool valid = false;
    double pointDensityRatio = 0.0;
    double headPointDensity = 0.0;
    double tailPointDensity = 0.0;
};

/** 【函数导航】
 * 作用：评估/审核“evaluateDensity”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
MiDuZhengJu evaluateDensity(const HoleLeixingPouMianBase::Result& profile) noexcept
{
    MiDuZhengJu out;
    const std::size_t count = profile.profileRadii.size();
    if (count < 6 || profile.profilePoints.size() != count) return out;
    const std::size_t edge = std::min<std::size_t>(3, count / 2);
    const double headRadius = meanRange(profile.profileRadii, 0, edge);
    const double tailRadius = meanRange(profile.profileRadii, count - edge, count);
    const double headPoints = meanRange(profile.profilePoints, 0, edge);
    const double tailPoints = meanRange(profile.profilePoints, count - edge, count);
    if (!(headRadius > 0.20) || !(tailRadius > 0.20) || !(headPoints > 0.0)) return out;

    out.headPointDensity = headPoints / headRadius;
    out.tailPointDensity = tailPoints / tailRadius;
    out.pointDensityRatio = out.headPointDensity > 1e-9
        ? out.tailPointDensity / out.headPointDensity : 0.0;
    out.valid = finite(out.pointDensityRatio)
        && finite(out.headPointDensity) && finite(out.tailPointDensity);
    return out;
}

/** 【函数导航】
 * 作用：执行“coneLike”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool coneLike(const HoleLeixingPouMianBase::Result& profile) noexcept
{
    return profile.valid
        && profile.profileValid
        && profile.holeType == 2
        && profile.sustainedCone
        && !profile.chamferPlatformStraight
        && profile.layers >= 4
        && profile.depthSpan >= 0.70
        && profile.slope <= -0.28
        && profile.monotonicRatio >= 0.65
        && profile.shrink >= profile.requiredShrink;
}

/** 【类型导航注释】
 * GongXiangNiHe：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct GongXiangNiHe {
    bool valid = false;
    double slope = 0.0;
    double rmse = 0.0;
    double slopeSpread = 0.0;
};

/** 【函数导航】
 * 作用：拟合/求解“fitSharedSlope”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
GongXiangNiHe fitSharedSlope(
    const std::array<const HoleLeixingPouMianBase::Result*, 5>& profiles,
    const std::array<bool, 5>& use) noexcept
{
    GongXiangNiHe out;
    double numerator = 0.0;
    double denominator = 0.0;
    int usedProfiles = 0;
    std::vector<double> slopes;
    slopes.reserve(profiles.size());

    for (std::size_t profileIndex = 0; profileIndex < profiles.size(); ++profileIndex) {
        if (!use[profileIndex] || profiles[profileIndex] == nullptr) continue;
        const auto& profile = *profiles[profileIndex];
        if (profile.profileDepths.size() != profile.profileRadii.size()
            || profile.profileDepths.size() < 2) continue;
        double meanDepth = 0.0;
        double meanRadius = 0.0;
        for (std::size_t index = 0; index < profile.profileDepths.size(); ++index) {
            meanDepth += profile.profileDepths[index] + kProfileShifts[profileIndex];
            meanRadius += profile.profileRadii[index];
        }
        meanDepth /= static_cast<double>(profile.profileDepths.size());
        meanRadius /= static_cast<double>(profile.profileRadii.size());
        for (std::size_t index = 0; index < profile.profileDepths.size(); ++index) {
            const double depth = profile.profileDepths[index] + kProfileShifts[profileIndex];
            const double dx = depth - meanDepth;
            numerator += dx * (profile.profileRadii[index] - meanRadius);
            denominator += dx * dx;
        }
        slopes.push_back(profile.slope);
        ++usedProfiles;
    }
    if (usedProfiles < 3 || denominator <= 1e-12) return out;
    out.slope = numerator / denominator;

    double squared = 0.0;
    std::size_t pointCount = 0;
    for (std::size_t profileIndex = 0; profileIndex < profiles.size(); ++profileIndex) {
        if (!use[profileIndex] || profiles[profileIndex] == nullptr) continue;
        const auto& profile = *profiles[profileIndex];
        double meanDepth = 0.0;
        double meanRadius = 0.0;
        for (std::size_t index = 0; index < profile.profileDepths.size(); ++index) {
            meanDepth += profile.profileDepths[index] + kProfileShifts[profileIndex];
            meanRadius += profile.profileRadii[index];
        }
        meanDepth /= static_cast<double>(profile.profileDepths.size());
        meanRadius /= static_cast<double>(profile.profileRadii.size());
        const double intercept = meanRadius - out.slope * meanDepth;
        for (std::size_t index = 0; index < profile.profileDepths.size(); ++index) {
            const double depth = profile.profileDepths[index] + kProfileShifts[profileIndex];
            const double error = profile.profileRadii[index] - (intercept + out.slope * depth);
            squared += error * error;
            ++pointCount;
        }
    }
    out.rmse = pointCount > 0
        ? std::sqrt(squared / static_cast<double>(pointCount))
        : std::numeric_limits<double>::infinity();
    if (!slopes.empty()) {
        const auto [minimum, maximum] = std::minmax_element(slopes.begin(), slopes.end());
        out.slopeSpread = *maximum - *minimum;
    }
    out.valid = finite(out.slope) && finite(out.rmse) && finite(out.slopeSpread);
    return out;
}

/** 【函数导航】
 * 作用：构建“makeDiagnostic”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShiftPouMianState makeDiagnostic(const HoleLeixingPouMianBase::Result& profile,
                                      double shift) noexcept
{
    ShiftPouMianState out;
    out.shift = shift;
    out.valid = profile.valid && profile.profileValid;
    out.coneLike = coneLike(profile);
    out.platform = profile.chamferPlatformStraight;
    out.layers = profile.layers;
    out.span = profile.depthSpan;
    out.slope = profile.slope;
    out.shrink = profile.shrink;
    out.monotonicRatio = profile.monotonicRatio;
    const MiDuZhengJu density = evaluateDensity(profile);
    out.pointDensityRatio = density.valid ? density.pointDensityRatio : 0.0;
    return out;
}

}

/** 【函数导航】
 * 作用：评估/审核“classifyProfiles”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result classifyProfiles(const HoleLeixingPouMianGongShi::Result& baseline,
                        const std::array<HoleLeixingPouMianBase::Result, 3>& extras,
                        const HoleJiheFinal::Result& geometry)
{
    Result result;
    result.valid = baseline.valid;
    result.finalType = baseline;
    result.mode = baseline.mode;
    result.reason = baseline.reason;
    if (!baseline.valid) return result;

    result.eligibleConservativeVeto = baseline.holeType == 1
        && supportsConservativeVeto(baseline.mode)
        && geometry.valid
        && geometry.holeType == 2
        && baseline.baseType == 2
        && baseline.axialType == 2
        && !baseline.geometryStraightConsensus
        && !baseline.wallStraightConsensus
        && !baseline.crossShiftPlatformStraight
        && !baseline.baseStrongPlatform
        && !baseline.axialStrongPlatform
        && !baseline.base.chamferPlatformStraight
        && !baseline.axial.chamferPlatformStraight;
    if (!result.eligibleConservativeVeto) return result;
    result.attempted = true;

    const std::array<const HoleLeixingPouMianBase::Result*, 5> profiles{{
        &baseline.base, &extras[0], &baseline.axial, &extras[1], &extras[2]}};
    std::array<bool, 5> coneUse{};
    for (std::size_t index = 0; index < profiles.size(); ++index) {
        result.profiles[index] = makeDiagnostic(*profiles[index], kProfileShifts[index]);
        if (result.profiles[index].valid) ++result.validProfileCount;
        if (result.profiles[index].platform) ++result.platformProfileCount;
        if (result.profiles[index].coneLike) {
            coneUse[index] = true;
            ++result.coneProfileCount;
            if (index == 1 || index == 3 || index == 4)
                ++result.extraConeProfileCount;
        }
    }

    const MiDuZhengJu baseDensity = evaluateDensity(baseline.base);
    const MiDuZhengJu axialDensity = evaluateDensity(baseline.axial);
    result.basePointDensityRatio = baseDensity.valid ? baseDensity.pointDensityRatio : 0.0;
    result.axialPointDensityRatio = axialDensity.valid ? axialDensity.pointDensityRatio : 0.0;
    result.normalizedSupportStrong = baseDensity.valid
        && result.basePointDensityRatio >= 0.42
        && baseDensity.tailPointDensity >= 2.5;

    result.bottomShrink = geometry.topRadius - geometry.bottomRadius;
    result.bottomRadiusRatio = geometry.topRadius > 1e-9
        ? geometry.bottomRadius / geometry.topRadius : 0.0;
    result.bottomEvidenceStrong = geometry.bottomValid
        && geometry.depth >= 0.70
        && geometry.bottomRadius > 0.05
        && result.bottomRadiusRatio <= 0.82
        && result.bottomShrink >= std::max(0.55, 0.14 * geometry.topRadius);

    const GongXiangNiHe fit = fitSharedSlope(profiles, coneUse);
    result.sharedSlope = fit.slope;
    result.sharedRmse = fit.rmse;
    result.slopeSpread = fit.slopeSpread;
    const double rmseLimit = std::max(0.55, 0.10 * geometry.topRadius);
    const double spreadLimit = std::max(1.05, 0.65 * std::abs(result.sharedSlope));
    result.multiShiftConeConsistent = fit.valid
        && result.validProfileCount >= 4
        && result.coneProfileCount >= 3
        && result.extraConeProfileCount >= 1
        && result.platformProfileCount == 0
        && result.sharedSlope <= -0.35
        && result.sharedRmse <= rmseLimit
        && result.slopeSpread <= spreadLimit;

    result.huiFuChengGong = result.bottomEvidenceStrong
        && result.multiShiftConeConsistent
        && (result.normalizedSupportStrong || result.coneProfileCount >= 4);
    if (result.huiFuChengGong) {
        result.finalType.holeType = 2;
        result.finalType.typeChanged = result.finalType.holeType != result.finalType.inputType;
        result.finalType.changedFromMultiSectionType = result.finalType.holeType != result.finalType.baseType;
        result.finalType.axialConfirmedCone = true;
        result.finalType.axialUnstableConeRejected = false;
        result.finalType.crossShiftConeHuiFuChengGong = true;
        result.finalType.affineConeOverruledCollapse = baseline.axialOverCollapseRaw;
        result.finalType.mode = "ConeRescueReview_MULTI_SHIFT_NORMALIZED_SUPPORT_CONE_RESCUE";
        result.finalType.reason = "ConeRescueReview_FinalGeometry_BOTTOM_PLUS_RADIUS_NORMALIZED_WALL_SUPPORT_AND_MULTI_SHIFT_CONE_CONSENSUS";
        result.mode = result.finalType.mode;
        result.reason = result.finalType.reason;
    } else {
        result.mode = "ConeRescueReview_KEEP_NonWallReview_VETO";
        if (!result.bottomEvidenceStrong)
            result.reason = "ConeRescueReview_RESCUE_REJECTED_NO_INDEPENDENT_FinalGeometry_BOTTOM";
        else if (!result.normalizedSupportStrong)
            result.reason = "ConeRescueReview_RESCUE_REJECTED_RADIUS_NORMALIZED_SUPPORT_COLLAPSES";
        else
            result.reason = "ConeRescueReview_RESCUE_REJECTED_MULTI_SHIFT_CONE_INCONSISTENT";
    }
    return result;
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry,
                const HoleLeixingPouMianGongShi::Result& baseline)
{
    std::array<HoleLeixingPouMianBase::Result, 3> extras{};
    if (!baseline.valid || baseline.holeType != 1 || !supportsConservativeVeto(baseline.mode)) {
        Result result;
        result.valid = baseline.valid;
        result.finalType = baseline;
        result.mode = baseline.mode;
        result.reason = baseline.reason;
        return result;
    }

    for (std::size_t index = 0; index < extras.size(); ++index) {
        HoleJiheFinal::Result shifted = geometry;
        shifted.topW += kExtraShifts[index];
        extras[index] = HoleLeixingPouMianBase::evaluate(samples, shifted);
    }
    return classifyProfiles(baseline, extras, geometry);
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry)
{
    const HoleLeixingPouMianGongShi::Result baseline = HoleLeixingPouMianGongShi::evaluate(samples, geometry);
    return evaluate(samples, geometry, baseline);
}

}

// ============================================================================
// 孔深与下口计算实现
// ============================================================================
/*
模块职责：
孔深度估计子模块。

主要调用位置：
由正式孔识别链在几何候选稳定后调用，估计可观测深度或下口证据。

维护说明：
深度步长、层数和下口门限直接影响输出“可测/不可测”，修改必须专项验证。
*/

namespace HoleShenduGuJi {
namespace {

constexpr int kSectors = 48;
constexpr double kLayerStart = 0.10;
constexpr double kLayerStep = 0.20;
constexpr double kLayerHalf = 0.16;
constexpr double kPi = 3.14159265358979323846;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double v) noexcept { return std::isfinite(v); }

double median(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    return (values.size() & 1U) ? values[mid]
                               : 0.5 * (values[mid - 1] + values[mid]);
}

/** 【函数导航】
 * 作用：执行“medianWithScratch”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double medianWithScratch(const std::vector<double>& values,
                         std::vector<double>& scratch)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    scratch.assign(values.begin(), values.end());
    std::sort(scratch.begin(), scratch.end());
    const std::size_t mid = scratch.size() / 2;
    return (scratch.size() & 1U) ? scratch[mid]
                                 : 0.5 * (scratch[mid - 1] + scratch[mid]);
}

/** 【函数导航】
 * 作用：执行“quantileInPlace”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double quantileInPlace(std::vector<double>& values, double q)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    q = std::clamp(q, 0.0, 1.0);
    const double pos = q * static_cast<double>(values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double t = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - t) + values[hi] * t;
}

/** 【类型导航注释】
 * Fit：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Fit {
    bool valid = false;
    double intercept = 0.0;
    double slope = 0.0;
    double rmse = 0.0;
    double sse = 0.0;
};

/** 【函数导航】
 * 作用：拟合/求解“fitLine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Fit fitLine(const std::vector<Layer>& layers, std::size_t begin, std::size_t end)
{
    Fit out;
    if (end <= begin + 1 || end > layers.size()) return out;
    double sx = 0.0, sy = 0.0, sw = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double w = std::max(1.0, static_cast<double>(layers[i].sectors)
            + 0.025 * static_cast<double>(layers[i].points));
        sw += w;
        sx += w * layers[i].depth;
        sy += w * layers[i].radius;
    }
    if (sw <= 1e-12) return out;
    const double mx = sx / sw;
    const double my = sy / sw;
    double num = 0.0, den = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double w = std::max(1.0, static_cast<double>(layers[i].sectors)
            + 0.025 * static_cast<double>(layers[i].points));
        const double dx = layers[i].depth - mx;
        num += w * dx * (layers[i].radius - my);
        den += w * dx * dx;
    }
    if (den <= 1e-12) return out;
    out.slope = num / den;
    out.intercept = my - out.slope * mx;
    double sse = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double w = std::max(1.0, static_cast<double>(layers[i].sectors)
            + 0.025 * static_cast<double>(layers[i].points));
        const double e = layers[i].radius - (out.intercept + out.slope * layers[i].depth);
        sse += w * e * e;
    }
    out.sse = sse;
    out.rmse = std::sqrt(sse / sw);
    out.valid = finite(out.slope) && finite(out.intercept) && finite(out.rmse);
    return out;
}

/** 【函数导航】
 * 作用：构建“buildProfile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<Layer> buildProfile(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const HoleJiheFinal::Result& geometry,
    int polarity)
{
    std::vector<Layer> layers;
    if (polarity != -1 && polarity != 1) return layers;
    const double maxDepth = std::min(10.0, std::max(6.40, 1.15 * geometry.topRadius + 1.0));
    const double maxRadius = std::min(13.0, geometry.topRadius + 2.0);
    const double madLimit = std::max(0.35, 0.08 * geometry.topRadius);

    /** 【类型导航注释】
     * YuChuLiYangBen：Hole 分析实现中的自定义 结构体。
     * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct YuChuLiYangBen {
        double w = 0.0;
        double radius = 0.0;
        int sector = 0;
    };
    std::vector<YuChuLiYangBen> prepared;
    prepared.reserve(samples.size());
    for (const auto& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double du = sample.u - geometry.centerU;
        const double dv = sample.v - geometry.centerV;
        const double radius = std::hypot(du, dv);
        if (radius < 0.30 || radius > maxRadius) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * kSectors));
        sector = std::clamp(sector, 0, kSectors - 1);
        prepared.push_back({sample.w, radius, sector});
    }

    std::vector<Layer> raw;
    raw.reserve(static_cast<std::size_t>(std::ceil(maxDepth / kLayerStep)));
    std::array<std::vector<double>, kSectors> sectorRadii;
    for (auto& values : sectorRadii) values.reserve(16);
    std::vector<double> sectorValues;
    std::vector<double> deviations;
    std::vector<double> medianScratch;
    sectorValues.reserve(kSectors);
    deviations.reserve(kSectors);
    medianScratch.reserve(kSectors);

    for (double layerDepth = kLayerStart; layerDepth <= maxDepth + 1e-9;
         layerDepth += kLayerStep) {
        for (auto& values : sectorRadii) values.clear();
        sectorValues.clear();
        deviations.clear();
        int layerPoints = 0;
        for (const YuChuLiYangBen& sample : prepared) {
            const double depth = static_cast<double>(polarity) * (sample.w - geometry.topW);
            if (std::abs(depth - layerDepth) > kLayerHalf) continue;
            sectorRadii[static_cast<std::size_t>(sample.sector)].push_back(sample.radius);
            ++layerPoints;
        }
        for (auto& values : sectorRadii) {
            if (values.empty()) continue;
            sectorValues.push_back(quantileInPlace(values, 0.20));
        }
        if (sectorValues.size() < 8 || layerPoints < 10) continue;
        const double radius = medianWithScratch(sectorValues, medianScratch);
        for (double value : sectorValues) deviations.push_back(std::abs(value - radius));
        const double mad = median(std::move(deviations));
        if (!finite(radius) || !finite(mad) || mad > madLimit) continue;
        raw.push_back({layerDepth, radius, layerPoints,
            static_cast<int>(sectorValues.size()), mad});
    }

    double previousDepth = -1.0;
    for (const Layer& layer : raw) {
        if (layers.empty()) {
            if (layer.depth > 0.50) continue;
        } else if (layer.depth - previousDepth > 0.45) {
            break;
        }
        layers.push_back(layer);
        previousDepth = layer.depth;
    }
    return layers;
}

/** 【类型导航注释】
 * Platform：Hole 分析实现中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Platform {
    bool valid = false;
    int breakLayer = -1;
    double radius = 0.0;
    double slope = 0.0;
    double stddev = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：检测/搜索“findShortBottomPlatform”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Platform findShortBottomPlatform(const std::vector<Layer>& layers, double topRadius)
{
    Platform best;
    if (layers.size() < 7) return best;
    std::vector<double> tailRadii;
    std::vector<double> medianScratch;
    tailRadii.reserve(layers.size());
    medianScratch.reserve(layers.size());
    for (std::size_t split = 3; split + 3 <= layers.size(); ++split) {
        const Fit cone = fitLine(layers, 0, split);
        const Fit tail = fitLine(layers, split, layers.size());
        if (!cone.valid || !tail.valid || cone.slope > -0.18 || std::abs(tail.slope) > 0.18)
            continue;
        tailRadii.clear();
        for (std::size_t i = split; i < layers.size(); ++i) tailRadii.push_back(layers[i].radius);
        const double center = medianWithScratch(tailRadii, medianScratch);
        double variance = 0.0;
        for (double radius : tailRadii) variance += (radius - center) * (radius - center);
        const double stddev = std::sqrt(variance / static_cast<double>(tailRadii.size()));
        const double span = layers.back().depth - layers[split].depth;
        const double entranceDrop = layers.front().radius - center;
        const double stdLimit = std::max(0.18, 0.035 * topRadius);
        if (span < 0.38 || stddev > stdLimit
            || entranceDrop < std::max(0.35, 0.06 * topRadius)
            || center <= 0.20 || center >= 0.96 * topRadius) continue;
        const double score = 2.0 * static_cast<double>(layers.size() - split)
            + 2.0 * span + 1.5 * entranceDrop
            - 8.0 * stddev - 4.0 * std::abs(tail.slope) - 2.0 * cone.rmse;
        if (!best.valid || score > best.score) {
            best.valid = true;
            best.breakLayer = static_cast<int>(split);
            best.radius = center;
            best.slope = tail.slope;
            best.stddev = stddev;
            best.score = score;
        }
    }
    return best;
}

}

/** 【函数导航】
 * 作用：评估/审核“reviewMeasuredCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
WuLiReview reviewMeasuredCone(double topRadius,
                                  double rawBottomRadius,
                                  double depth)
{
    WuLiReview out;
    if (!finite(topRadius) || !finite(rawBottomRadius) || !finite(depth)
        || topRadius <= 0.0 || rawBottomRadius <= 0.0 || depth <= 0.0) {
        out.reason = "DepthBottomReview_PHYSICAL_REVIEW_NOT_EVALUATED";
        return out;
    }

    out.evaluated = true;
    out.rawBottomRadius = rawBottomRadius;

    // 本项目只定义由大口向小口收缩的锥孔。上下口顺序违反物理定义时，
    // 不能通过数值裁剪把结果“修”成合法锥孔，而应直接拒绝该孔结果。
    if (!(topRadius > rawBottomRadius)) {
        out.rejectHole = true;
        out.reason = "DepthBottomReview_PHYSICAL_REVIEW_REJECT_RTOP_NOT_GREATER_THAN_RBOTTOM";
        return out;
    }

    const double shrink = topRadius - rawBottomRadius;
    out.slopeDeg = std::atan2(shrink, depth) * 180.0 / kPi;
    out.depthRadiusRatio = depth / topRadius;

    constexpr double kMinConeSlopeDeg = 25.0;
    constexpr double kMinConeAxialDepthMm = 1.50;
    constexpr double kMinConeDepthRadiusRatio = 0.20;

    if (!finite(out.slopeDeg) || out.slopeDeg < kMinConeSlopeDeg) {
        out.classifyStraight = true;
        out.reason = "DepthBottomReview_PHYSICAL_REVIEW_SLOPE_LT_25_DEG_TO_STRAIGHT";
    } else if (depth < kMinConeAxialDepthMm) {
        out.classifyStraight = true;
        out.reason = "DepthBottomReview_PHYSICAL_REVIEW_CONE_DEPTH_LT_1HoleTypeReview_MM_TO_STRAIGHT";
    } else if (!finite(out.depthRadiusRatio)
               || out.depthRadiusRatio < kMinConeDepthRadiusRatio) {
        out.classifyStraight = true;
        out.reason = "DEPTH_BOTTOM_REVIEW_DEPTH_RADIUS_RATIO_BELOW_LIMIT_TO_STRAIGHT";
    } else {
        out.reason = "DepthBottomReview_PHYSICAL_REVIEW_CONE_ACCEPTED";
    }
    return out;
}

/** 【函数导航】
 * 作用：评估/审核“reviewProfileConeWithoutReliableBottom”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
WuLiReview reviewProfileConeWithoutReliableBottom(
    const HoleLeixingPouMianBase::Result& profile,
    double topRadius)
{
    WuLiReview out;
    out.evaluated = true;
    if (!profile.valid || !profile.profileValid || !finite(topRadius) || topRadius <= 0.0) {
        out.classifyStraight = true;
        out.reason = "DepthBottomReview_PROFILE_ONLY_CONE_EVIDENCE_INVALID_TO_STRAIGHT";
        return out;
    }

    const double effectiveDepth = profile.depthSpan;
    const double rawBottomRadius = profile.lastRadius;
    const double shrink = profile.firstRadius - profile.lastRadius;
    out.rawBottomRadius = rawBottomRadius;
    out.depthRadiusRatio = effectiveDepth / topRadius;
    out.slopeDeg = effectiveDepth > 0.0
        ? std::atan2(shrink, effectiveDepth) * 180.0 / kPi : 0.0;

    // 没有可靠下口时，锥孔仍必须由已有多层孔壁形成完整的物理证据链。
    // 任一条件不足都回到直孔，禁止输出“锥孔但下口/深度/坡角均不可测”的半成品状态。
    const bool profileConeSupported = profile.layers >= 4
        && profile.monotonicRatio >= 0.70
        && effectiveDepth >= 1.50
        && finite(profile.firstRadius) && finite(profile.lastRadius)
        && profile.firstRadius > profile.lastRadius
        && shrink > 0.0
        && finite(out.depthRadiusRatio) && out.depthRadiusRatio >= 0.20
        && finite(out.slopeDeg) && out.slopeDeg >= 25.0;
    if (!profileConeSupported) {
        out.classifyStraight = true;
        out.reason = "DepthBottomReview_PROFILE_ONLY_CONE_EVIDENCE_INSUFFICIENT_TO_STRAIGHT";
    } else {
        out.reason = "DepthBottomReview_PROFILE_ONLY_CONE_EVIDENCE_ACCEPTED";
    }
    return out;
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry,
                const HoleLeixingPouMianGongShi::Result& typeResult)
{
    Result out;
    if (!geometry.valid || !typeResult.valid || typeResult.holeType != 2
        || !finite(geometry.topRadius) || geometry.topRadius <= 0.0 || samples.size() < 20) {
        out.reason = "DepthBottomReview_INVALID_OR_NONCONE_INPUT";
        return out;
    }

    int polarity = typeResult.base.polarity;
    if (polarity != -1 && polarity != 1) polarity = geometry.inwardPolarity;
    if (polarity != -1 && polarity != 1) {
        out.reason = "DepthBottomReview_NO_INWARD_POLARITY";
        return out;
    }
    out.polarity = polarity;
    out.profile = buildProfile(samples, geometry, polarity);
    out.layers = static_cast<int>(out.profile.size());
    if (out.profile.size() < 4) {
        out.reason = "DepthBottomReview_INSUFFICIENT_CONTIGUOUS_LAYERS";
        return out;
    }

    const Fit fullFit = fitLine(out.profile, 0, out.profile.size());
    if (!fullFit.valid) {
        out.reason = "DepthBottomReview_PROFILE_FIT_FAILED";
        return out;
    }

    const Platform platform = findShortBottomPlatform(out.profile, geometry.topRadius);
    const std::size_t coneEnd = platform.valid
        ? static_cast<std::size_t>(platform.breakLayer)
        : out.profile.size();
    const Fit coneFit = fitLine(out.profile, 0, coneEnd);
    if (!coneFit.valid) {
        out.reason = "DepthBottomReview_CONE_SEGMENT_FIT_FAILED";
        return out;
    }
    out.slope = coneFit.slope;
    out.rmse = coneFit.rmse;
    out.shrink = out.profile.front().radius - out.profile[coneEnd - 1].radius;
    int monotonic = 0;
    for (std::size_t i = 1; i < coneEnd; ++i) {
        if (out.profile[i].radius <= out.profile[i - 1].radius + 0.12) ++monotonic;
    }
    out.monotonicRatio = coneEnd > 1
        ? static_cast<double>(monotonic) / static_cast<double>(coneEnd - 1) : 0.0;
    out.terminalDepth = out.profile.back().depth;
    out.terminalRadius = out.profile.back().radius;
    out.terminalCoverage = static_cast<double>(out.profile.back().sectors) / kSectors;

    const bool coherentCone = coneFit.slope <= -0.15
        && out.shrink >= std::max(0.25, 0.06 * geometry.topRadius)
        && out.monotonicRatio >= 0.65
        && coneFit.rmse <= std::max(0.32, 0.055 * geometry.topRadius);
    if (!coherentCone) {
        out.reason = "DepthBottomReview_NO_COHERENT_CONE_WALL";
        return out;
    }

    if (platform.valid) {
        out.platformDetected = true;
        out.breakLayer = platform.breakLayer;
        const std::size_t split = static_cast<std::size_t>(platform.breakLayer);
        const double splitMidpoint = 0.5 * (out.profile[split - 1].depth
            + out.profile[split].depth);
        const double intersectionDepth = std::abs(coneFit.slope) > 1e-9
            ? (platform.radius - coneFit.intercept) / coneFit.slope
            : splitMidpoint;
        out.depth = std::clamp(intersectionDepth, splitMidpoint - 0.45, splitMidpoint + 0.45);
        out.bottomRadius = platform.radius;
        out.mode = "DepthBottomReview_CONE_TO_INNER_BORE_PLATFORM";
    } else {
        const std::size_t tailCount = std::min<std::size_t>(2, out.profile.size());
        out.bottomRadius = tailCount == 1
            ? out.profile.back().radius
            : 0.5 * (out.profile[out.profile.size() - 2].radius
                     + out.profile.back().radius);
        out.depth = out.profile.back().depth + 0.5 * kLayerStep;
        out.mode = "DepthBottomReview_TERMINAL_WALL_HALF_LAYER_EXTENSION";
    }

    // 原始下口半径必须在任何上限裁剪之前保存。后面的普通数值裁剪只服务于
    // DepthBottomReview 置信度/末端点统计，最终物理审核始终使用 rawBottomRadius。
    out.rawBottomRadius = out.bottomRadius;
    out.depth = std::clamp(out.depth, 0.35, 10.0);
    out.bottomRadius = std::clamp(out.bottomRadius, 0.20, 0.96 * geometry.topRadius);

    constexpr int capSectorTotal = 24;
    std::array<unsigned char, capSectorTotal> capSectorHit{};
    const double capRadius = std::max(0.35, 0.78 * out.bottomRadius);
    for (const auto& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double sampleDepth = static_cast<double>(polarity) * (sample.w - geometry.topW);
        if (sampleDepth < out.depth - 0.35 || sampleDepth > out.depth + 0.40) continue;
        const double du = sample.u - geometry.centerU;
        const double dv = sample.v - geometry.centerV;
        const double radial = std::hypot(du, dv);
        if (radial > capRadius) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * capSectorTotal));
        sector = std::clamp(sector, 0, capSectorTotal - 1);
        capSectorHit[static_cast<std::size_t>(sector)] = 1;
        ++out.terminalCapPoints;
    }
    out.terminalCapSectors = static_cast<int>(std::count(
        capSectorHit.begin(), capSectorHit.end(), static_cast<unsigned char>(1)));
    out.terminalCapEvidence = out.terminalCapPoints >= 8 && out.terminalCapSectors >= 4;

    const double layerScore = std::clamp((static_cast<double>(out.layers) - 4.0) / 12.0, 0.0, 1.0);
    const double spanScore = std::clamp((out.depth - 0.50) / 3.5, 0.0, 1.0);
    const double coverageScore = std::clamp((out.terminalCoverage - 0.16) / 0.50, 0.0, 1.0);
    const double fitScore = std::clamp((std::max(0.32, 0.055 * geometry.topRadius) - coneFit.rmse)
        / std::max(0.20, 0.055 * geometry.topRadius), 0.0, 1.0);
    const double confidenceTerminalRatio = out.bottomRadius / geometry.topRadius;
    const double taperScore = std::clamp((0.90 - confidenceTerminalRatio) / 0.55, 0.0, 1.0);
    out.confidence = 0.20 * layerScore + 0.18 * spanScore + 0.18 * coverageScore
        + 0.18 * fitScore + 0.16 * out.monotonicRatio + 0.10 * taperScore
        + (out.platformDetected ? 0.06 : 0.0);

    const double terminalRatio = out.bottomRadius / geometry.topRadius;
    const bool strongMissingBottomRecovery = platform.valid || (
        out.layers >= 8
        && out.depth >= 1.20
        && terminalRatio <= 0.38
        && out.terminalCoverage >= 0.28
        && out.monotonicRatio >= 0.80
        && out.rmse <= std::max(0.24, 0.040 * geometry.topRadius)
        && out.terminalCapEvidence);
    out.bottomValid = out.depth >= 0.40 && out.bottomRadius > 0.20
        && out.bottomRadius < 0.98 * geometry.topRadius
        && out.confidence >= (geometry.bottomValid ? 0.42 : 0.62)
        && out.profile.back().sectors >= 8 && out.profile.back().points >= 10
        && (geometry.bottomValid || strongMissingBottomRecovery);
    out.valid = out.bottomValid;
    if (!out.bottomValid) {
        out.reason = "DepthBottomReview_BOTTOM_CONFIDENCE_TOO_LOW";
        return out;
    }

    out.finalGeometryDepthDelta = geometry.bottomValid ? out.depth - geometry.depth : 0.0;
    out.finalGeometryBottomRadiusDelta = geometry.bottomValid
        ? out.bottomRadius - geometry.bottomRadius : 0.0;

    if (geometry.bottomValid) {
        const double depthLimit = std::max(0.80, 0.20 * std::max(geometry.depth, out.depth));
        const double radiusLimit = std::max(0.75, 0.16 * geometry.topRadius);
        if (std::abs(out.finalGeometryDepthDelta) > depthLimit
            || std::abs(out.finalGeometryBottomRadiusDelta) > radiusLimit) {
            out.used = false;
            out.reason = "DepthBottomReview_REJECTED_BY_FinalGeometry_CONSISTENCY_GUARD";
            return out;
        }
        out.replacedFinalGeometryBottom = true;
    } else {
        out.recoveredMissingBottom = true;
    }
    out.used = true;

    // DepthBottomReview 的深度、下口和 FinalGeometry 一致性审核全部通过后，再执行同一模块内的
    // 锥孔物理审核统一使用几何证据，角度、尺度和深径比都在同一审核链中处理，
    // 而是“可靠深度已经形成以后”的正常孔型审核条件。
    out.physicalReview = reviewMeasuredCone(
        geometry.topRadius, out.rawBottomRadius, out.depth);
    if (out.physicalReview.rejectHole || out.physicalReview.classifyStraight) {
        out.reason = out.physicalReview.reason;
    } else {
        out.reason = out.mode;
    }
    return out;
}

}
