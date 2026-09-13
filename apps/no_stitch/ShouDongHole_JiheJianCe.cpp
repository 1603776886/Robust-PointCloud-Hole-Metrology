/*
================================================================================
文件：ShouDongHole_JiheJianCe.cpp
模块：手动 Hole 几何检测

【主要职责】
实现 seed 邻域栅格/mask、轮廓候选、模板评分、Hole 口及局部几何检测。

【主要调用关系】
由 shouDongHole 命名空间公开函数被 HoleShibie 总编排调用。

【线程与状态】
纯计算；候选顺序与评分逻辑属于生产行为。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
集中实现手动选孔的孔口检测与局部几何计算，包括点级几何、二值掩膜、形态学、轮廓候选、模板评分、多高度搜索和孔口几何细化。
这些步骤属于同一条“从局部点集找到孔口”的计算链，合并后避免维护者在多个小文件之间反复跳转。

主要调用位置：
ShouDongHole_WeiziZhicheng.cpp 准备三维局部坐标与支撑面后，通过 ShouDongHole_Manual.h 的公开接口调用本文件；
HoleShibie_Recognition.cpp 负责更上层的识别编排。

维护说明：
搜索半径、模板半径、栅格分辨率、形态学邻域、候选数量和多高度偏移都会影响孔口粗定位与精修。
这些量若需要调整，应优先看相邻中文注释中的单位和增减影响；不要为了减少代码量把候选顺序或比较规则改写。
*/
#include "ShouDongHole_Manual.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

// ============================================================================
// 功能分区：点级几何、掩膜与轮廓基础
// ============================================================================
namespace shouDongHole {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

/** 【函数导航】
 * 作用：拟合/求解“solve3x3Point”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool solve3x3Point(double a[3][4], double out[3]) {
    for (int c = 0; c < 3; ++c) {
        int pivot = c;
        for (int r = c + 1; r < 3; ++r) {
            if (std::abs(a[r][c]) > std::abs(a[pivot][c])) pivot = r;
        }
        if (std::abs(a[pivot][c]) < 1e-14) return false;
        if (pivot != c) {
            for (int j = c; j < 4; ++j) std::swap(a[pivot][j], a[c][j]);
        }
        const double d = a[c][c];
        for (int j = c; j < 4; ++j) a[c][j] /= d;
        for (int r = 0; r < 3; ++r) {
            if (r == c) continue;
            const double f = a[r][c];
            for (int j = c; j < 4; ++j) a[r][j] -= f * a[c][j];
        }
    }
    for (int i = 0; i < 3; ++i) out[i] = a[i][3];
    return true;
}

/** 【函数导航】
 * 作用：拟合/求解“fitPlaneLeastSquares”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool fitPlaneLeastSquares(
    const std::vector<Point3d>& points,
    const std::vector<std::size_t>& ids,
    const std::vector<double>* weights,
    PingMianModel& plane)
{
    if (ids.size() < 3) return false;
    double normal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    for (std::size_t j = 0; j < ids.size(); ++j) {
        const Point3d& p = points[ids[j]];
        const double w = weights ? (*weights)[j] : 1.0;
        const double row[3] = {p.x, p.y, 1.0};
        for (int r = 0; r < 3; ++r) {
            rhs[r] += w * row[r] * p.z;
            for (int c = 0; c < 3; ++c) normal[r][c] += w * row[r] * row[c];
        }
    }
    double aug[3][4];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) aug[r][c] = normal[r][c];
        aug[r][3] = rhs[r];
    }
    double sol[3];
    if (!solve3x3Point(aug, sol)) return false;
    plane = {sol[0], sol[1], sol[2]};
    return true;
}

/** 【函数导航】
 * 作用：拟合/求解“fitCircleLeastSquares”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool fitCircleLeastSquares(
    const std::vector<std::pair<double, double>>& xy,
    double& centerX,
    double& centerY,
    double& radius)
{
    if (xy.size() < 3) return false;
    double normal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    for (const auto& p : xy) {
        const double row[3] = {2.0 * p.first, 2.0 * p.second, 1.0};
        const double b = p.first * p.first + p.second * p.second;
        for (int r = 0; r < 3; ++r) {
            rhs[r] += row[r] * b;
            for (int c = 0; c < 3; ++c) normal[r][c] += row[r] * row[c];
        }
    }
    double aug[3][4];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) aug[r][c] = normal[r][c];
        aug[r][3] = rhs[r];
    }
    double sol[3];
    if (!solve3x3Point(aug, sol)) return false;
    centerX = sol[0];
    centerY = sol[1];
    const double r2 = sol[2] + centerX * centerX + centerY * centerY;
    if (!(r2 > 0.0) || !std::isfinite(r2)) return false;
    radius = std::sqrt(r2);
    return true;
}

/** 【函数导航】
 * 作用：执行“planeDistance”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double planeDistance(const Point3d& p, const PingMianModel& plane) noexcept {
    return std::abs(p.z - (plane.a * p.x + plane.b * p.y + plane.c))
        / std::sqrt(1.0 + plane.a * plane.a + plane.b * plane.b);
}

/** 【函数导航】
 * 作用：执行“normalFromPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d normalFromPlane(const PingMianModel& plane) noexcept {
    Vec3d n{-plane.a, -plane.b, 1.0};
    const double length = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (length > 0.0) {
        n.x /= length;
        n.y /= length;
        n.z /= length;
    }
    if (n.z < 0.0) {
        n.x = -n.x;
        n.y = -n.y;
        n.z = -n.z;
    }
    return n;
}

double dot(const Vec3d& a, const Vec3d& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** 【函数导航】
 * 作用：执行“angleDegrees”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double angleDegrees(const Vec3d& a, const Vec3d& b) noexcept {
    const double value = std::clamp(std::abs(dot(a, b)), 0.0, 1.0);
    return std::acos(value) * 180.0 / kPi;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
    const double hi = values[mid];
    if (values.size() % 2 != 0) return hi;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid - 1), values.begin() + static_cast<std::ptrdiff_t>(mid));
    return 0.5 * (values[mid - 1] + hi);
}

/** 【函数导航】
 * 作用：执行“dilate3x3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ErZhiMask dilate3x3(const ErZhiMask& input) {
    ErZhiMask output = input;
    std::fill(output.data.begin(), output.data.end(), 0);
    for (int y = 0; y < input.height; ++y) {
        for (int x = 0; x < input.width; ++x) {
            std::uint8_t value = 0;
            for (int dy = -1; dy <= 1 && !value; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    const int yy = y + dy;
                    if (xx >= 0 && yy >= 0 && xx < input.width && yy < input.height
                        && input.at(xx, yy)) {
                        value = 1;
                        break;
                    }
                }
            }
            output.data[static_cast<std::size_t>(y * input.width + x)] = value;
        }
    }
    return output;
}

/** 【函数导航】
 * 作用：执行“erode3x3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ErZhiMask erode3x3(const ErZhiMask& input) {
    ErZhiMask output = input;
    std::fill(output.data.begin(), output.data.end(), 0);
    for (int y = 0; y < input.height; ++y) {
        for (int x = 0; x < input.width; ++x) {
            std::uint8_t value = 1;
            for (int dy = -1; dy <= 1 && value; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    const int yy = y + dy;

                    if (xx >= 0 && yy >= 0 && xx < input.width && yy < input.height
                        && !input.at(xx, yy)) {
                        value = 0;
                        break;
                    }
                }
            }
            output.data[static_cast<std::size_t>(y * input.width + x)] = value;
        }
    }
    return output;
}

/** 【函数导航】
 * 作用：执行“sectorIndex”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int sectorIndex(double y, double x) noexcept {
    const double angle = std::atan2(y, x);
    int sector = static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * 12.0));
    return std::clamp(sector, 0, 11);
}

}

/** 【函数导航】
 * 作用：构建“makeSurfaceMask”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ErZhiMask makeSurfaceMask(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& plane,
    double halfWidth,
    double resolution,
    double planeTolerance,
    int* surfacePointCount)
{
    if (!(halfWidth > 0.0) || !(resolution > 0.0)) {
        throw std::invalid_argument("makeSurfaceMask: invalid dimensions");
    }
    const int size = static_cast<int>(std::ceil(2.0 * halfWidth / resolution)) + 1;
    ErZhiMask mask;
    mask.width = size;
    mask.height = size;
    mask.resolution = resolution;
    mask.originX = seed.x - halfWidth;
    mask.originY = seed.y - halfWidth;
    mask.data.assign(static_cast<std::size_t>(size * size), 0);
    int acceptedPointCount = 0;
    for (const Point3d& p : points) {
        if (std::abs(p.x - seed.x) > halfWidth || std::abs(p.y - seed.y) > halfWidth) continue;
        if (planeDistance(p, plane) > planeTolerance) continue;
        ++acceptedPointCount;
        const long ix = std::lrint((p.x - mask.originX) / resolution);
        const long iy = std::lrint((p.y - mask.originY) / resolution);
        if (ix >= 0 && iy >= 0 && ix < size && iy < size) {
            mask.data[static_cast<std::size_t>(iy * size + ix)] = 1;
        }
    }

    mask = dilate3x3(mask);
    mask = dilate3x3(mask);
    mask = erode3x3(mask);
    if (surfacePointCount) *surfacePointCount = acceptedPointCount;
    return mask;
}

/** 【函数导航】
 * 作用：精修“refineCircleSurface”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CircleJihe refineCircleSurface(
    const std::vector<Point3d>& points,
    double centerX,
    double centerY,
    double radius,
    const PingMianModel& plane,
    double planeTolerance)
{
    std::vector<const Point3d*> support;
    support.reserve(4096);
    const double minRadius = std::max(0.8, radius - 2.0);
    const double maxRadius = radius + 2.5;
    for (const Point3d& p : points) {
        const double d = std::hypot(p.x - centerX, p.y - centerY);
        if (d <= minRadius || d >= maxRadius) continue;
        if (planeDistance(p, plane) < planeTolerance) support.push_back(&p);
    }
    CircleJihe result{centerX, centerY, radius, support.size()};
    if (support.size() < 25) return result;

    for (int iteration = 0; iteration < 5; ++iteration) {
        std::array<const Point3d*, 72> bestPoint{};
        std::array<double, 72> bestDifference{};
        bestDifference.fill(std::numeric_limits<double>::infinity());
        for (const Point3d* p : support) {
            const double radial = std::hypot(p->x - result.centerX, p->y - result.centerY);
            if (std::abs(radial - result.radius) >= 1.2) continue;
            double angle = std::atan2(p->y - result.centerY, p->x - result.centerX);
            int bin = static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * 72.0));
            bin = std::clamp(bin, 0, 71);
            const double difference = std::abs(radial - result.radius);
            if (difference < bestDifference[static_cast<std::size_t>(bin)]) {
                bestDifference[static_cast<std::size_t>(bin)] = difference;
                bestPoint[static_cast<std::size_t>(bin)] = p;
            }
        }
        std::vector<std::pair<double, double>> xy;
        xy.reserve(72);
        for (const Point3d* p : bestPoint) if (p) xy.emplace_back(p->x, p->y);
        if (xy.size() < 12) break;
        double nextX = 0.0;
        double nextY = 0.0;
        double nextRadius = 0.0;
        if (!fitCircleLeastSquares(xy, nextX, nextY, nextRadius)) break;
        if (std::hypot(nextX - result.centerX, nextY - result.centerY) > 1.5
            || std::abs(nextRadius - result.radius) > 1.5) {
            break;
        }
        result.centerX = 0.5 * (result.centerX + nextX);
        result.centerY = 0.5 * (result.centerY + nextY);
        result.radius = 0.5 * (result.radius + nextRadius);
    }
    return result;
}

/** 【函数导航】
 * 作用：精修“refineCenterAtFirstDepth”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CircleJihe refineCenterAtFirstDepth(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& mouthPlane,
    double centerX,
    double centerY,
    double radius)
{
    PingMianModel depthPlane = mouthPlane;
    depthPlane.c -= 0.5;
    const ErZhiMask mask = makeSurfaceMask(points, seed, depthPlane, 21.0, 0.25, 0.35);
    const std::vector<LunKuoHouXuan> candidates = extractLunKuoHouXuan(mask, seed.x, seed.y);
    const LunKuoHouXuan* best = nullptr;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (const LunKuoHouXuan& c : candidates) {
        const double centerDelta = std::hypot(c.centerX - centerX, c.centerY - centerY);
        if (centerDelta > 1.5 || c.radius < 0.72 * radius || c.radius > 1.30 * radius
            || c.coverage < 0.52 || c.rmse > 1.0) {
            continue;
        }
        const double score = 1.5 * c.coverage - 0.45 * c.rmse - 0.18 * centerDelta
            - 0.35 * std::abs(c.radius / radius - 1.0);
        if (!best || score > bestScore) {
            best = &c;
            bestScore = score;
        }
    }
    CircleJihe result{centerX, centerY, radius, 0};
    if (best) {
        result.centerX = best->centerX;
        result.centerY = best->centerY;
        result.supportPoints = 1;
    }
    return result;
}

/** 【函数导航】
 * 作用：拟合/求解“fitSectorBalancedSurfaceNormal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
NormalJihe fitSectorBalancedSurfaceNormal(
    const std::vector<Point3d>& points,
    double centerX,
    double centerY,
    double radius,
    const Vec3d& globalNormal,
    const PingMianModel& mouthPlane,
    double planeTolerance,
    double maxDeltaDegrees)
{
    /** 【类型导航注释】
     * Sample：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h、HoleJihe_Geometry.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Sample {
        Point3d point;
        double planeResidual = 0.0;
        int sector = 0;
    };
    std::vector<Sample> samples;
    samples.reserve(6000);
    const double denominator = std::sqrt(1.0 + mouthPlane.a * mouthPlane.a + mouthPlane.b * mouthPlane.b);
    for (const Point3d& p : points) {
        const double radial = std::hypot(p.x - centerX, p.y - centerY);
        if (radial < radius + 0.8 || radial > radius + 5.0) continue;
        const double residual = std::abs(p.z - (mouthPlane.a * p.x + mouthPlane.b * p.y + mouthPlane.c)) / denominator;
        if (residual >= planeTolerance) continue;
        samples.push_back({p, residual, sectorIndex(p.y - centerY, p.x - centerX)});
    }
    NormalJihe result;
    result.normal = globalNormal;
    result.supportPoints = samples.size();
    result.tiltDegrees = std::acos(std::clamp(std::abs(globalNormal.z), 0.0, 1.0)) * 180.0 / kPi;
    result.deltaFromGlobalDegrees = 0.0;
    if (samples.size() < 30) return result;

    std::vector<Point3d> localPoints;
    localPoints.reserve(samples.size());
    for (const Sample& s : samples) localPoints.push_back(s.point);

    auto balancedSelection = [&](const std::vector<double>& metric) {
        std::vector<std::size_t> selected;
        selected.reserve(720);
        for (int sector = 0; sector < 12; ++sector) {
            std::vector<std::size_t> ids;
            for (std::size_t i = 0; i < samples.size(); ++i) {
                if (samples[i].sector == sector) ids.push_back(i);
            }
            std::sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
                if (metric[a] != metric[b]) return metric[a] < metric[b];
                return a < b;
            });
            if (ids.size() > 60) ids.resize(60);
            selected.insert(selected.end(), ids.begin(), ids.end());
        }
        return selected;
    };

    std::vector<double> initialMetric(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) initialMetric[i] = samples[i].planeResidual;
    std::vector<std::size_t> selected = balancedSelection(initialMetric);
    PingMianModel fitted;
    if (selected.size() < 20 || !fitPlaneLeastSquares(localPoints, selected, nullptr, fitted)) return result;

    const double minCosine = std::cos(maxDeltaDegrees * kPi / 180.0);
    for (int iteration = 0; iteration < 8; ++iteration) {
        const double den = std::sqrt(1.0 + fitted.a * fitted.a + fitted.b * fitted.b);
        std::vector<double> residuals(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const Point3d& p = samples[i].point;
            residuals[i] = std::abs(p.z - (fitted.a * p.x + fitted.b * p.y + fitted.c)) / den;
        }
        selected = balancedSelection(residuals);
        if (selected.size() < 20) break;
        std::vector<double> selectedResiduals;
        selectedResiduals.reserve(selected.size());
        for (std::size_t id : selected) selectedResiduals.push_back(residuals[id]);
        const double centerResidual = median(selectedResiduals);
        std::vector<double> absoluteDeviation;
        absoluteDeviation.reserve(selectedResiduals.size());
        for (double value : selectedResiduals) absoluteDeviation.push_back(std::abs(value - centerResidual));
        const double scale = std::max(0.05, 1.4826 * median(std::move(absoluteDeviation)));
        std::vector<double> weights;
        weights.reserve(selected.size());
        for (std::size_t id : selected) {
            weights.push_back(1.0 / std::max(1.0, residuals[id] / (2.5 * scale)));
        }
        PingMianModel next;
        if (!fitPlaneLeastSquares(localPoints, selected, &weights, next)) break;
        const Vec3d nextNormal = normalFromPlane(next);
        if (std::abs(dot(nextNormal, globalNormal)) < minCosine) break;
        fitted = next;
    }

    result.normal = normalFromPlane(fitted);
    result.tiltDegrees = std::acos(std::clamp(std::abs(result.normal.z), 0.0, 1.0)) * 180.0 / kPi;
    result.deltaFromGlobalDegrees = angleDegrees(result.normal, globalNormal);
    return result;
}

/** 【函数导航】
 * 作用：精修“refineMouthGeometry”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
JingXiuHoleKouJihe refineMouthGeometry(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const PingMianModel& mouthPlane,
    const Vec3d& globalNormal,
    double mouthCenterX,
    double mouthCenterY,
    double mouthRadius,
    const std::string& mouthSource)
{
    JingXiuHoleKouJihe result;
    result.surfaceFit = refineCircleSurface(points, mouthCenterX, mouthCenterY, mouthRadius, mouthPlane);
    result.acceptedSurface = {mouthCenterX, mouthCenterY, mouthRadius, result.surfaceFit.supportPoints};
    const double centerMove = std::hypot(result.surfaceFit.centerX - mouthCenterX, result.surfaceFit.centerY - mouthCenterY);
    if (mouthSource.rfind("contour", 0) == 0) {
        if (centerMove <= 0.55) {
            result.acceptedSurface.centerX = result.surfaceFit.centerX;
            result.acceptedSurface.centerY = result.surfaceFit.centerY;
        }
    } else if (centerMove <= 0.8 && std::abs(result.surfaceFit.radius - mouthRadius) <= 0.8) {
        result.acceptedSurface.centerX = result.surfaceFit.centerX;
        result.acceptedSurface.centerY = result.surfaceFit.centerY;
        result.acceptedSurface.radius = result.surfaceFit.radius;
    }

    result.finalCircle = result.acceptedSurface;
    const CircleJihe depth = refineCenterAtFirstDepth(
        points, seed, mouthPlane,
        result.finalCircle.centerX, result.finalCircle.centerY, result.finalCircle.radius);
    if (depth.supportPoints > 0
        && std::hypot(depth.centerX - result.finalCircle.centerX, depth.centerY - result.finalCircle.centerY) <= 0.6) {
        result.finalCircle.centerX = depth.centerX;
        result.finalCircle.centerY = depth.centerY;
        result.depthCenterUsed = true;
    }
    result.normalFit = fitSectorBalancedSurfaceNormal(
        points, result.finalCircle.centerX, result.finalCircle.centerY,
        result.finalCircle.radius, globalNormal, mouthPlane);
    return result;
}

namespace {

/** 【类型导航注释】
 * WenDingSaoMiaoZhou：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct WenDingSaoMiaoZhou {
    bool valid = false;
    double x = 1.0;
    double y = 0.0;
};

/** 【函数导航】
 * 作用：估计“estimateStableScanAxis”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
WenDingSaoMiaoZhou estimateStableScanAxis(
    const std::vector<Point3d>& points,
    const std::vector<const Point3d*>& boreWindow)
{

    double mxx = 0.0, mxy = 0.0, myy = 0.0;
    int scanSamples = 0;
    for (std::size_t q = 1; q < points.size(); ++q) {
        const double dx = points[q].x - points[q - 1].x;
        const double dy = points[q].y - points[q - 1].y;
        const double distance = std::hypot(dx, dy);
        if (distance < 0.05 || distance > 0.80) continue;
        const double ux = dx / distance, uy = dy / distance;
        mxx += ux * ux; mxy += ux * uy; myy += uy * uy; ++scanSamples;
    }
    if (scanSamples >= 100) {
        const double angle = 0.5 * std::atan2(2.0 * mxy, mxx - myy);
        return {true, std::cos(angle), std::sin(angle)};
    }

    if (boreWindow.size() < 40) return {};
    constexpr double cellSize = 0.80;
    constexpr double minDistance = 0.05;
    constexpr double maxDistance = 0.80;
    double minX = boreWindow.front()->x;
    double minY = boreWindow.front()->y;
    for (const Point3d* point : boreWindow) {
        minX = std::min(minX, point->x);
        minY = std::min(minY, point->y);
    }
    auto cellCoordinate = [&](double value, double origin) {
        return static_cast<int>(std::floor((value - origin) / cellSize));
    };
    auto cellKey = [](int x, int y) {
        const std::uint64_t ux = static_cast<std::uint32_t>(x);
        const std::uint64_t uy = static_cast<std::uint32_t>(y);
        return (ux << 32) | uy;
    };
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells;
    cells.reserve(boreWindow.size() * 2);
    for (std::size_t i = 0; i < boreWindow.size(); ++i) {
        const int ix = cellCoordinate(boreWindow[i]->x, minX);
        const int iy = cellCoordinate(boreWindow[i]->y, minY);
        cells[cellKey(ix, iy)].push_back(i);
    }

    mxx = 0.0; mxy = 0.0; myy = 0.0; scanSamples = 0;
    constexpr std::size_t neighbourCount = 8;
    const double minDistance2 = minDistance * minDistance;
    const double maxDistance2 = maxDistance * maxDistance;
    for (std::size_t i = 0; i < boreWindow.size(); ++i) {
        const Point3d& p = *boreWindow[i];
        const int ix = cellCoordinate(p.x, minX);
        const int iy = cellCoordinate(p.y, minY);
        std::array<double, neighbourCount> bestDistance2;
        std::array<std::size_t, neighbourCount> bestIndex;
        bestDistance2.fill(std::numeric_limits<double>::infinity());
        bestIndex.fill(i);
        for (int dyCell = -1; dyCell <= 1; ++dyCell) {
            for (int dxCell = -1; dxCell <= 1; ++dxCell) {
                const auto found = cells.find(cellKey(ix + dxCell, iy + dyCell));
                if (found == cells.end()) continue;
                for (std::size_t j : found->second) {
                    if (j == i) continue;
                    const double dx = boreWindow[j]->x - p.x;
                    const double dy = boreWindow[j]->y - p.y;
                    const double distance2 = dx * dx + dy * dy;
                    if (distance2 < minDistance2 || distance2 > maxDistance2) continue;
                    if (distance2 >= bestDistance2.back()) continue;
                    std::size_t position = neighbourCount - 1;
                    while (position > 0 && distance2 < bestDistance2[position - 1]) {
                        bestDistance2[position] = bestDistance2[position - 1];
                        bestIndex[position] = bestIndex[position - 1];
                        --position;
                    }
                    bestDistance2[position] = distance2;
                    bestIndex[position] = j;
                }
            }
        }
        for (std::size_t k = 0; k < neighbourCount; ++k) {
            if (!std::isfinite(bestDistance2[k])) break;
            const double distance = std::sqrt(bestDistance2[k]);
            const double ux = (boreWindow[bestIndex[k]]->x - p.x) / distance;
            const double uy = (boreWindow[bestIndex[k]]->y - p.y) / distance;
            mxx += ux * ux; mxy += ux * uy; myy += uy * uy; ++scanSamples;
        }
    }
    if (scanSamples < 100) return {};
    const double anisotropy = std::hypot(mxx - myy, 2.0 * mxy);
    if (anisotropy < 0.02 * (mxx + myy)) return {};
    const double angle = 0.5 * std::atan2(2.0 * mxy, mxx - myy);
    return {true, std::cos(angle), std::sin(angle)};
}

/** 【函数导航】
 * 作用：精修“refineChamferedStraightBoreOrdered”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleJingXiu refineChamferedStraightBoreOrdered(
    const std::vector<Point3d>& points,
    const PingMianModel& mouthPlane,
    double mouthCenterX,
    double mouthCenterY,
    double mouthRadius,
    int stableOrderMode)
{
    HoleJingXiu output;

    if (!(mouthRadius >= 2.45 && mouthRadius <= 5.20)) return output;

    /** 【类型导航注释】
     * CengYuan：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct CengYuan {
        double depth = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double radius = 0.0;
        double medianResidual = 0.0;
        int inliers = 0;
        int sectors = 0;
        int outerPoints = 0;
        int outerSectors = 0;
    };

    const double denominator = std::sqrt(1.0 + mouthPlane.a * mouthPlane.a + mouthPlane.b * mouthPlane.b);
    auto signedResidual = [&](const Point3d& p) noexcept {
        return (p.z - (mouthPlane.a * p.x + mouthPlane.b * p.y + mouthPlane.c)) / denominator;
    };
    auto circleFromThree = [](const std::pair<double,double>& p1,
                              const std::pair<double,double>& p2,
                              const std::pair<double,double>& p3,
                              double& cx,double& cy,double& r) noexcept {
        const double x1=p1.first,y1=p1.second,x2=p2.first,y2=p2.second,x3=p3.first,y3=p3.second;
        const double d=2.0*(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2));
        if(std::abs(d)<1e-10) return false;
        cx=((x1*x1+y1*y1)*(y2-y3)+(x2*x2+y2*y2)*(y3-y1)+(x3*x3+y3*y3)*(y1-y2))/d;
        cy=((x1*x1+y1*y1)*(x3-x2)+(x2*x2+y2*y2)*(x1-x3)+(x3*x3+y3*y3)*(x2-x1))/d;
        r=std::hypot(cx-x1,cy-y1);
        return std::isfinite(cx)&&std::isfinite(cy)&&std::isfinite(r);
    };
    auto sectorCount = [](const std::vector<std::pair<double,double>>& xy,
                          const std::vector<std::size_t>& ids,double cx,double cy) {
        std::array<bool,16> used{};
        for(std::size_t id:ids){
            double a=std::atan2(xy[id].second-cy,xy[id].first-cx);
            if(a<0.0)a+=2.0*kPi;
            int sec=std::clamp(static_cast<int>(std::floor(a/(2.0*kPi)*16.0)),0,15);
            used[static_cast<std::size_t>(sec)]=true;
        }
        return static_cast<int>(std::count(used.begin(),used.end(),true));
    };

    std::vector<CengYuan> layers;
    const double searchRadius = mouthRadius + 2.0;
    std::vector<const Point3d*> boreWindow;
    boreWindow.reserve(4096);
    for (const Point3d& p : points) {
        if (std::hypot(p.x - mouthCenterX, p.y - mouthCenterY) <= searchRadius) {
            boreWindow.push_back(&p);
        }
    }
    const double depths[] = {0.75,1.05,1.35,1.65,1.95,2.25};
    for(double depth:depths){
        std::vector<std::pair<double,double>> xy;
        xy.reserve(256);
        for(const Point3d* point:boreWindow){
            const Point3d& p = *point;
            const double d=-signedResidual(p);
            if(std::abs(d-depth)>0.22) continue;
            xy.emplace_back(p.x,p.y);
        }
        if(xy.size()<10 || xy.size()>450) continue;
        if (stableOrderMode == 1) {
            std::sort(xy.begin(), xy.end(), [](const auto& left, const auto& right) {
                if (left.first != right.first) return left.first < right.first;
                return left.second < right.second;
            });
        } else if (stableOrderMode == 2) {
            std::sort(xy.begin(), xy.end(), [](const auto& left, const auto& right) {
                if (left.second != right.second) return left.second < right.second;
                return left.first < right.first;
            });
        }

        std::vector<std::size_t> outerIds;
        outerIds.reserve(xy.size());
        for (std::size_t q = 0; q < xy.size(); ++q) {
            const double radial = std::hypot(xy[q].first - mouthCenterX,
                                             xy[q].second - mouthCenterY);
            if (std::abs(radial - mouthRadius) <= 0.40) outerIds.push_back(q);
        }
        const int outerSectors = sectorCount(xy, outerIds, mouthCenterX, mouthCenterY);

        CengYuan best;
        double bestScore=-1e30;
        const std::size_t n=xy.size();
        std::uint64_t state=1469598103934665603ULL ^ static_cast<std::uint64_t>(n*131+std::llround(depth*100));
        const int trials=std::min<int>(900,std::max<int>(150,static_cast<int>(n*n)));
        for(int t=0;t<trials;++t){
            auto next=[&](){state=state*6364136223846793005ULL+1442695040888963407ULL;return state;};
            const std::size_t i=static_cast<std::size_t>(next()%n);
            const std::size_t j=static_cast<std::size_t>(next()%n);
            const std::size_t k=static_cast<std::size_t>(next()%n);
            if(i==j||i==k||j==k) continue;
            double cx=0.0,cy=0.0,r=0.0;
            if(!circleFromThree(xy[i],xy[j],xy[k],cx,cy,r)) continue;
            if(r<1.35||r>3.35) continue;
            if(std::hypot(cx-mouthCenterX,cy-mouthCenterY)>2.25) continue;
            std::vector<std::size_t> inliers;
            std::vector<double> residuals;
            inliers.reserve(n);residuals.reserve(n);
            for(std::size_t q=0;q<n;++q){
                const double e=std::abs(std::hypot(xy[q].first-cx,xy[q].second-cy)-r);
                if(e<0.19){inliers.push_back(q);residuals.push_back(e);}
            }
            if(inliers.size()<9) continue;
            const int sectors=sectorCount(xy,inliers,cx,cy);
            if(sectors<6) continue;
            const double med=median(residuals);
            const double score=static_cast<double>(inliers.size())+1.8*sectors-8.0*med;
            if(score<=bestScore) continue;
            std::vector<std::pair<double,double>> fitPoints;
            fitPoints.reserve(inliers.size());
            for(std::size_t id:inliers) fitPoints.push_back(xy[id]);
            double fx=0.0,fy=0.0,fr=0.0;
            if(!fitCircleLeastSquares(fitPoints,fx,fy,fr)) continue;
            if(fr<1.35||fr>3.35||std::hypot(fx-mouthCenterX,fy-mouthCenterY)>2.25) continue;
            std::vector<std::size_t> refinedIds;
            std::vector<double> refinedResiduals;
            for(std::size_t q=0;q<n;++q){
                const double e=std::abs(std::hypot(xy[q].first-fx,xy[q].second-fy)-fr);
                if(e<0.20){refinedIds.push_back(q);refinedResiduals.push_back(e);}
            }
            const int refinedSectors=sectorCount(xy,refinedIds,fx,fy);
            if(refinedIds.size()<9||refinedSectors<6) continue;
            const double refinedMed=median(refinedResiduals);
            if(refinedMed>0.16) continue;
            bestScore=score;
            best={depth,fx,fy,fr,refinedMed,static_cast<int>(refinedIds.size()),refinedSectors,
                static_cast<int>(outerIds.size()),outerSectors};
        }
        if(bestScore>-1e20) layers.push_back(best);
    }
    output.detectedLayerCount = static_cast<int>(layers.size());
    for (const CengYuan& layer : layers) {
        if (layer.depth >= 1.25
            && layer.radius <= std::min(2.75, mouthRadius * 0.86)) {
            ++output.deepSmallLayerCount;
        }
        if (layer.depth <= 1.35 + 1e-6
            && layer.outerSectors >= 7 && layer.outerPoints >= 15) {
            ++output.shallowOuterWallLayerCount;
        }
    }
    output.stableRetryEligible = layers.size() >= 3
        && output.deepSmallLayerCount >= 2
        && output.shallowOuterWallLayerCount < 2;
    if(layers.size()<3) return output;
    WenDingSaoMiaoZhou scanAxis;
    bool scanAxisComputed = false;
    auto getScanAxis = [&]() -> const WenDingSaoMiaoZhou& {
        if (!scanAxisComputed) {
            scanAxis = estimateStableScanAxis(points, boreWindow);
            scanAxisComputed = true;
        }
        return scanAxis;
    };

    double bestPairScore=-1e30;
    for(std::size_t i=0;i<layers.size();++i){
        for(std::size_t j=i+1;j<layers.size();++j){
            const CengYuan& a=layers[i];const CengYuan& b=layers[j];
            if(a.depth<1.55) continue;
            const double dd=b.depth-a.depth;
            if(dd<0.20||dd>0.45) continue;
            if(std::abs(a.radius-b.radius)>0.13) continue;
            if(std::hypot(a.cx-b.cx,a.cy-b.cy)>0.48) continue;
            const double radius=0.5*(a.radius+b.radius);
            const double shrink=mouthRadius-radius;
            if(shrink<0.48||radius>2.65||radius>mouthRadius*0.81||radius<mouthRadius*0.42) continue;
            const double centerX=0.5*(a.cx+b.cx),centerY=0.5*(a.cy+b.cy);
            if(std::hypot(centerX-mouthCenterX,centerY-mouthCenterY)>2.25) continue;

            bool hasIntermediateTransition = false;
            double maximumConsecutiveDrop = 0.0;
            const CengYuan* previousLayer = nullptr;
            for (const CengYuan& q : layers) {
                if (q.depth > b.depth + 1e-6) continue;
                if (q.depth < a.depth - 0.10
                    && q.radius >= radius + 0.20 && q.radius <= radius + 0.90) {
                    hasIntermediateTransition = true;
                }
                if (previousLayer && q.depth - previousLayer->depth <= 0.45) {
                    maximumConsecutiveDrop = std::max(maximumConsecutiveDrop,
                        previousLayer->radius - q.radius);
                }
                previousLayer = &q;
            }
            if (!hasIntermediateTransition || maximumConsecutiveDrop > 0.82) continue;

            int shallowOuterWallLayers = 0;
            for (const CengYuan& q : layers) {
                if (q.depth > 1.35 + 1e-6) continue;
                if (q.outerSectors >= 7 && q.outerPoints >= 15) ++shallowOuterWallLayers;
            }
            if (shallowOuterWallLayers >= 2) continue;

            int checked=0, violations=0;
            double previous=mouthRadius;
            for(const CengYuan& q:layers){
                if(q.depth>b.depth+1e-6) continue;
                ++checked;
                if(q.radius>previous+0.22) ++violations;
                previous=std::min(previous,q.radius+0.12);
            }
            if(checked<4||violations>1) continue;
            const double score=4.0*(a.depth+b.depth)+0.25*(a.sectors+b.sectors)
                +0.08*(a.inliers+b.inliers)-8.0*(a.medianResidual+b.medianResidual)
                -0.5*radius-1.5*violations;
            if(score>bestPairScore){
                bestPairScore=score;output.valid=true;

                double finalX=centerX,finalY=centerY;
                const WenDingSaoMiaoZhou& selectedScanAxis = getScanAxis();
                if(selectedScanAxis.valid){
                    const double along=(mouthCenterX-centerX)*selectedScanAxis.x
                        +(mouthCenterY-centerY)*selectedScanAxis.y;
                    finalX+=along*selectedScanAxis.x;finalY+=along*selectedScanAxis.y;
                }
                output.centerX=finalX;output.centerY=finalY;output.radius=radius;
                output.layerCount=2;output.depthBegin=a.depth;output.depthEnd=b.depth;
                output.residual=0.5*(a.medianResidual+b.medianResidual);
            }
        }
    }

    if (!output.valid && layers.size() >= 4) {
        double bestTerminalScore = -1e30;
        for (std::size_t j = 1; j < layers.size(); ++j) {
            const CengYuan& a = layers[j - 1];
            const CengYuan& b = layers[j];
            if (a.depth < 1.25) continue;
            const double dd = b.depth - a.depth;
            if (dd < 0.20 || dd > 0.45) continue;
            if (std::abs(a.radius - b.radius) > 0.25) continue;
            if (std::hypot(a.cx - b.cx, a.cy - b.cy) > 0.55) continue;
            const double radius = 0.5 * (a.radius + b.radius);
            const bool deepestPair = (j + 1 == layers.size());
            bool terminalShoulderPair = false;
            if (!deepestPair && j + 2 == layers.size()) {
                const CengYuan& tail = layers[j + 1];
                terminalShoulderPair = (b.radius - tail.radius >= 0.25)
                    && tail.inliers <= b.inliers
                    && radius >= 0.70 * mouthRadius;
            }
            if (!deepestPair && !terminalShoulderPair) continue;
            if (deepestPair && a.depth < 1.55) continue;
            const double shrink = mouthRadius - radius;
            if (shrink < 0.48 || radius > 2.65 || radius > mouthRadius * 0.83
                || radius < mouthRadius * 0.42) continue;
            if (a.sectors < 6 || b.sectors < 6 || a.inliers < 10 || b.inliers < 9) continue;

            int sequenceCount = 0;
            int expansionViolations = 0;
            double maximumDrop = 0.0;
            double previousRadius = mouthRadius;
            bool hasIntermediateTransition = false;
            for (const CengYuan& q : layers) {
                if (q.depth > b.depth + 1e-6) continue;
                ++sequenceCount;
                const double drop = previousRadius - q.radius;
                maximumDrop = std::max(maximumDrop, drop);
                if (q.radius > previousRadius + 0.16) ++expansionViolations;
                if (q.depth < a.depth - 0.10
                    && q.radius >= radius + 0.18 && q.radius <= radius + 0.95) {
                    hasIntermediateTransition = true;
                }
                previousRadius = q.radius;
            }
            if (sequenceCount < 4 || expansionViolations > 0
                || maximumDrop > 0.48 || !hasIntermediateTransition) continue;

            int shallowOuterWallLayers = 0;
            for (const CengYuan& q : layers) {
                if (q.depth > 1.35 + 1e-6) continue;
                if (q.outerSectors >= 7 && q.outerPoints >= 15) ++shallowOuterWallLayers;
            }
            if (shallowOuterWallLayers >= 2) continue;

            const double centerX = 0.5 * (a.cx + b.cx);
            const double centerY = 0.5 * (a.cy + b.cy);
            if (std::hypot(centerX - mouthCenterX, centerY - mouthCenterY) > 2.25) continue;
            const double score = 3.0 * (a.depth + b.depth)
                + 0.20 * (a.sectors + b.sectors)
                + 0.06 * (a.inliers + b.inliers)
                - 8.0 * (a.medianResidual + b.medianResidual)
                - 1.5 * std::abs(a.radius - b.radius);
            if (score <= bestTerminalScore) continue;

            double finalX = centerX, finalY = centerY;
            const WenDingSaoMiaoZhou& selectedScanAxis = getScanAxis();
            if (selectedScanAxis.valid) {
                const double along = (mouthCenterX - centerX) * selectedScanAxis.x
                    + (mouthCenterY - centerY) * selectedScanAxis.y;
                finalX += along * selectedScanAxis.x; finalY += along * selectedScanAxis.y;
            }
            bestTerminalScore = score;
            output.valid = true;
            output.centerX = finalX;
            output.centerY = finalY;
            output.radius = radius;
            output.layerCount = 2;
            output.depthBegin = a.depth;
            output.depthEnd = b.depth;
            output.residual = 0.5 * (a.medianResidual + b.medianResidual);
        }
    }
    return output;
}

}

/** 【函数导航】
 * 作用：精修“refineChamferedStraightBore”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleJingXiu refineChamferedStraightBore(
    const std::vector<Point3d>& points,
    const PingMianModel& mouthPlane,
    double mouthCenterX,
    double mouthCenterY,
    double mouthRadius)
{
    HoleJingXiu primary = refineChamferedStraightBoreOrdered(
        points, mouthPlane, mouthCenterX, mouthCenterY, mouthRadius, 0);
    if (primary.valid || !primary.stableRetryEligible) return primary;
    HoleJingXiu xyOrdered = refineChamferedStraightBoreOrdered(
        points, mouthPlane, mouthCenterX, mouthCenterY, mouthRadius, 1);
    HoleJingXiu yxOrdered = refineChamferedStraightBoreOrdered(
        points, mouthPlane, mouthCenterX, mouthCenterY, mouthRadius, 2);
    HoleJingXiu selected;
    int selectedMode = 0;
    if (xyOrdered.valid && yxOrdered.valid) {
        const bool agree = std::abs(xyOrdered.radius - yxOrdered.radius) <= 0.22
            && std::hypot(xyOrdered.centerX - yxOrdered.centerX,
                          xyOrdered.centerY - yxOrdered.centerY) <= 0.65;
        if (agree) {
            selected = (xyOrdered.residual <= yxOrdered.residual) ? xyOrdered : yxOrdered;
            selectedMode = (xyOrdered.residual <= yxOrdered.residual) ? 1 : 2;
        }
    } else if (xyOrdered.valid) {
        selected = xyOrdered;
        selectedMode = 1;
    } else if (yxOrdered.valid) {
        selected = yxOrdered;
        selectedMode = 2;
    }
    if (selected.valid) {
        selected.stableOrderRetry = true;
        selected.stableOrderMode = selectedMode;
    }
    return selected;
}

/** 【函数导航】
 * 作用：读取/解析“loadXyz64DianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<Point3d> loadXyz64DianYun(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("loadXyz64DianYun: cannot open " + path);
    std::uint64_t count = 0;
    input.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!input) throw std::runtime_error("loadXyz64DianYun: truncated header " + path);
    if (count > 100000000ULL) throw std::runtime_error("loadXyz64DianYun: unreasonable point count");
    std::vector<Point3d> points(static_cast<std::size_t>(count));
    input.read(reinterpret_cast<char*>(points.data()), static_cast<std::streamsize>(points.size() * sizeof(Point3d)));
    if (!input) throw std::runtime_error("loadXyz64DianYun: truncated data " + path);
    return points;
}

}


// ============================================================================
// 功能分区：孔口候选、模板评分与多高度检测
// ============================================================================
namespace shouDongHole {
namespace {

/** 【类型导航注释】
 * JuBuScore：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JuBuScore {
    double score = -1e30;
    double ring = 0.0;
    double inner = 1.0;
    double core = 1.0;
    double ratio = 0.0;
};

/** 【函数导航】
 * 作用：评估/审核“scoreAt”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
JuBuScore scoreAt(
    const ErZhiMask& mask,
    double cx,
    double cy,
    double radius,
    double seedPx,
    double seedPy)
{
    const double res = mask.resolution;
    const int rad = static_cast<int>(std::ceil((radius + 1.6) / res)) + 1;
    const int x0 = std::max(0, static_cast<int>(std::floor(cx - rad)));
    const int x1 = std::min(mask.width, static_cast<int>(std::ceil(cx + rad + 1.0)));
    const int y0 = std::max(0, static_cast<int>(std::floor(cy - rad)));
    const int y1 = std::min(mask.height, static_cast<int>(std::ceil(cy + rad + 1.0)));

    double innerSum = 0.0, ringSum = 0.0, coreSum = 0.0;
    std::size_t innerN = 0, ringN = 0, coreN = 0;
    const double innerLimit = std::max(0.4, radius - 0.8);
    const double ringLo = std::max(0.0, radius - 0.25);
    const double ringHi = radius + 1.6;
    const double coreLimit = std::max(0.25, radius * 0.55);

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const double d = std::hypot(static_cast<double>(x) - cx, static_cast<double>(y) - cy) * res;
            const double f = static_cast<double>(mask.at(x, y));
            if (d <= innerLimit) {
                innerSum += f;
                ++innerN;
            }
            if (d >= ringLo && d <= ringHi) {
                ringSum += f;
                ++ringN;
            }
            if (d <= coreLimit) {
                coreSum += f;
                ++coreN;
            }
        }
    }

    const double inner = innerN ? innerSum / static_cast<double>(innerN) : 1.0;
    const double ring = ringN ? ringSum / static_cast<double>(ringN) : 0.0;
    const double core = coreN ? coreSum / static_cast<double>(coreN) : 1.0;
    const double ratio = std::hypot(cx - seedPx, cy - seedPy) * res / std::max(radius, 1e-6);
    const double z = (ratio - 1.15) / 0.55;
    const double seedConsistency = std::exp(-0.5 * z * z);
    const double score = 0.58 * ring + 0.34 * (1.0 - inner) + 0.16 * (1.0 - core)
        - 0.18 * std::max(0.0, 0.5 - ring) + 0.30 * seedConsistency;
    return {score, ring, inner, core, ratio};
}

/** 【函数导航】
 * 作用：执行“better”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool better(double value, double best) noexcept {
    return value > best;
}

/** 【函数导航】
 * 作用：执行“distanceTransformExact”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<double> distanceTransformExact(const ErZhiMask& mask) {

    const int w = mask.width, h = mask.height;
    const double inf = 1e18;
    std::vector<double> tmp(static_cast<std::size_t>(w*h), inf);
    std::vector<double> out(static_cast<std::size_t>(w*h), inf);
    auto dt1d = [inf](const std::vector<double>& f) {
        const int n = static_cast<int>(f.size());
        std::vector<double> d(static_cast<std::size_t>(n), inf);
        std::vector<int> v(static_cast<std::size_t>(n), 0);
        std::vector<double> z(static_cast<std::size_t>(n+1), 0.0);
        int k = 0;
        v[0] = 0;
        z[0] = -inf;
        z[1] = inf;
        for (int q = 1; q < n; ++q) {
            double sep = 0.0;
            while (true) {
                const int vk = v[k];
                sep = ((f[q] + static_cast<double>(q*q))
                    - (f[vk] + static_cast<double>(vk*vk))) / (2.0 * (q-vk));
                if (sep > z[k] || k == 0) break;
                --k;
            }
            ++k;
            v[k] = q;
            z[k] = sep;
            z[k+1] = inf;
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (z[k+1] < q) ++k;
            const double dq = static_cast<double>(q-v[k]);
            d[q] = dq*dq + f[v[k]];
        }
        return d;
    };
    for (int y=0;y<h;++y) {
        std::vector<double> f(static_cast<std::size_t>(w),inf);
        for (int x=0;x<w;++x) if (mask.at(x,y)) f[x]=0.0;
        const auto d=dt1d(f);
        for (int x=0;x<w;++x) tmp[static_cast<std::size_t>(y*w+x)]=d[x];
    }
    for (int x=0;x<w;++x) {
        std::vector<double> f(static_cast<std::size_t>(h),inf);
        for (int y=0;y<h;++y) f[y]=tmp[static_cast<std::size_t>(y*w+x)];
        const auto d=dt1d(f);
        for (int y=0;y<h;++y) out[static_cast<std::size_t>(y*w+x)]=std::sqrt(std::max(0.0,d[y]));
    }
    return out;
}

}

namespace {

/** 【类型导航注释】
 * BianYuanJilu：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct BianYuanJilu {
    int sx=0,sy=0,ex=0,ey=0;
    int fgx=0,fgy=0;
    int dir=0;
    bool used=false;
};

/** 【函数导航】
 * 作用：执行“vertexKey”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
long long vertexKey(int x,int y) {
    return (static_cast<long long>(y) << 32) ^ static_cast<unsigned int>(x);
}

/** 【函数导航】
 * 作用：拟合/求解“solve3x3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool solve3x3(double a[3][4], double out[3]) {
    for (int c=0;c<3;++c) {
        int piv=c;
        for (int r=c+1;r<3;++r) if (std::abs(a[r][c])>std::abs(a[piv][c])) piv=r;
        if (std::abs(a[piv][c])<1e-12) return false;
        if (piv!=c) for (int j=c;j<4;++j) std::swap(a[piv][j],a[c][j]);
        const double d=a[c][c]; for (int j=c;j<4;++j) a[c][j]/=d;
        for (int r=0;r<3;++r) if (r!=c) {
            const double f=a[r][c]; for (int j=c;j<4;++j) a[r][j]-=f*a[c][j];
        }
    }
    for (int i=0;i<3;++i) out[i]=a[i][3];
    return true;
}

/** 【函数导航】
 * 作用：拟合/求解“fitCircleLs”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool fitCircleLs(const std::vector<std::pair<double,double>>& pts,double& cx,double& cy,double& radius) {
    if (pts.size()<8) return false;
    double ata[3][3]={{0,0,0},{0,0,0},{0,0,0}};
    double atb[3]={0,0,0};
    for (const auto& p:pts) {
        const double row[3]={2.0*p.first,2.0*p.second,1.0};
        const double b=p.first*p.first+p.second*p.second;
        for(int i=0;i<3;++i){atb[i]+=row[i]*b;for(int j=0;j<3;++j)ata[i][j]+=row[i]*row[j];}
    }
    double aug[3][4];for(int i=0;i<3;++i){for(int j=0;j<3;++j)aug[i][j]=ata[i][j];aug[i][3]=atb[i];}
    double sol[3];if(!solve3x3(aug,sol))return false;
    cx = sol[0];
    cy = sol[1];
    const double rr = sol[2] + cx * cx + cy * cy;
    if (!(rr > 0.0) || !std::isfinite(rr)) {
        return false;
    }
    radius = std::sqrt(rr);
    return true;
}

/** 【函数导航】
 * 作用：执行“traceBoundaryLoops”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<std::vector<std::pair<double,double>>> traceBoundaryLoops(const ErZhiMask& mask,std::uint8_t target) {
    std::vector<BianYuanJilu> edges;edges.reserve(mask.data.size());
    auto isFg=[&](int x,int y){return x>=0&&y>=0&&x<mask.width&&y<mask.height&&mask.at(x,y)==target;};
    for(int y=0;y<mask.height;++y)for(int x=0;x<mask.width;++x){
        if(!isFg(x,y))continue;
        if(!isFg(x,y-1))edges.push_back({x,y,x+1,y,x,y,0,false});
        if(!isFg(x+1,y))edges.push_back({x+1,y,x+1,y+1,x,y,1,false});
        if(!isFg(x,y+1))edges.push_back({x+1,y+1,x,y+1,x,y,2,false});
        if(!isFg(x-1,y))edges.push_back({x,y+1,x,y,x,y,3,false});
    }
    std::unordered_map<long long,std::vector<int>> outgoing;
    outgoing.reserve(edges.size()*2);
    for(int i=0;i<(int)edges.size();++i)outgoing[vertexKey(edges[i].sx,edges[i].sy)].push_back(i);
    std::vector<std::vector<std::pair<double,double>>> loops;
    for(int start=0;start<(int)edges.size();++start){
        if(edges[start].used)continue;
        std::vector<std::pair<double,double>> pts;int cur=start;const int startX=edges[start].sx,startY=edges[start].sy;
        int guard=0;
        while(cur>=0&&!edges[cur].used&&guard++<(int)edges.size()+4){
            auto&e=edges[cur];e.used=true;pts.emplace_back((double)e.fgx,(double)e.fgy);
            const int vx=e.ex,vy=e.ey;
            if(vx==startX&&vy==startY)break;
            auto it=outgoing.find(vertexKey(vx,vy));if(it==outgoing.end()){cur=-1;break;}
            int next=-1;

            const int pref[4]={(e.dir+1)%4,e.dir,(e.dir+3)%4,(e.dir+2)%4};
            for(int pd:pref){for(int idx:it->second)if(!edges[idx].used&&edges[idx].dir==pd){next=idx;break;}if(next>=0)break;}
            cur=next;
        }
        if(pts.size()>=4)loops.push_back(std::move(pts));
    }
    return loops;
}

/** 【函数导航】
 * 作用：执行“occupancyMean”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double occupancyMean(const ErZhiMask& mask,double cx,double cy,double lo,double hi) {
    double sum=0.0;std::size_t n=0;
    const int rad=(int)std::ceil(hi/mask.resolution)+2;
    const int x0=std::max(0,(int)std::floor(cx-rad)),x1=std::min(mask.width,(int)std::ceil(cx+rad+1));
    const int y0=std::max(0,(int)std::floor(cy-rad)),y1=std::min(mask.height,(int)std::ceil(cy+rad+1));
    for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){const double d=std::hypot(x-cx,y-cy)*mask.resolution;if(d>=lo&&d<=hi){sum+=mask.at(x,y);++n;}}
    return n?sum/n:0.0;
}

}

/** 【函数导航】
 * 作用：执行“extractLunKuoHouXuan”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<LunKuoHouXuan> extractLunKuoHouXuan(
    const ErZhiMask& mask,
    double seedX,
    double seedY)
{
    if(!mask.valid())throw std::invalid_argument("extractLunKuoHouXuan: invalid mask");
    std::vector<LunKuoHouXuan> out;
    for(const auto target:{std::uint8_t(1),std::uint8_t(0)}){
        const auto loops=traceBoundaryLoops(mask,target);
        for(const auto& pts:loops){
            if(pts.size()<18)continue;
            double cx=0,cy=0,rpx=0;if(!fitCircleLs(pts,cx,cy,rpx))continue;
            const double r=rpx*mask.resolution;if(r<1.25||r>10.0)continue;
            const double worldX=mask.originX+cx*mask.resolution,worldY=mask.originY+cy*mask.resolution;
            const double dist=std::hypot(worldX-seedX,worldY-seedY);const double ratio=dist/std::max(r,1e-6);
            if(dist>14.0)continue;
            double sse=0;std::array<std::uint8_t,72> bins{};
            for(const auto&p:pts){const double rr=std::hypot(p.first-cx,p.second-cy);const double e=rr-rpx;sse+=e*e;
                double a=std::atan2(p.second-cy,p.first-cx);int b=(int)std::floor((a+3.14159265358979323846)/(2*3.14159265358979323846)*72.0);b=std::clamp(b,0,71);bins[(std::size_t)b]=1;}
            const double rmse=std::sqrt(sse/pts.size())*mask.resolution;
            int bc=0;for(auto b:bins)bc+=b?1:0;const double cov=bc/72.0;
            double area2=0,per=0;for(std::size_t i=0;i<pts.size();++i){const auto&a=pts[i];const auto&b=pts[(i+1)%pts.size()];area2+=a.first*b.second-b.first*a.second;per+=std::hypot(a.first-b.first,a.second-b.second);}
            const double area=std::abs(area2)*0.5*mask.resolution*mask.resolution;per*=mask.resolution;
            const double circ=per>0?4.0*3.14159265358979323846*area/(per*per):0.0;
            const double inocc=occupancyMean(mask,cx,cy,std::max(0.0,r-1.4),std::max(0.0,r-0.35));
            const double outocc=occupancyMean(mask,cx,cy,r+0.35,r+1.4);
            const double polarity=outocc-inocc;
            const double z=(ratio-1.15)/0.7;const double seedcons=std::exp(-0.5*z*z);
            const double score=1.3*cov+0.8*std::max(0.0,1.0-rmse/1.2)+0.5*seedcons+0.2*circ+1.1*polarity;
            LunKuoHouXuan c;c.kind=target?"surf":"void";c.centerX=worldX;c.centerY=worldY;c.radius=r;c.rmse=rmse;c.coverage=cov;c.circularity=circ;c.ratio=ratio;c.polarity=polarity;c.score=score;
            out.push_back(std::move(c));
        }
    }
    return out;
}

/** 【函数导航】
 * 作用：选择“choosePositiveJiXingLunKuo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool choosePositiveJiXingLunKuo(
    const std::vector<LunKuoHouXuan>& candidates,
    double resolution,
    LunKuoHouXuan& chosen)
{
    std::vector<const LunKuoHouXuan*> good;
    double top=-1e30;
    for(const auto&c:candidates){
        if(c.polarity>=0.35&&c.coverage>=0.60&&c.rmse<=1.05&&c.ratio>=0.75&&c.ratio<=2.30){good.push_back(&c);top=std::max(top,c.score);}
    }
    if(good.empty())return false;
    const LunKuoHouXuan* best=nullptr;
    for(const auto*c:good){if(c->score<top-0.45)continue;if(!best||c->radius>best->radius||(c->radius==best->radius&&(c->coverage>best->coverage||(c->coverage==best->coverage&&c->score>best->score))))best=c;}
    if (!best) {
        return false;
    }
    chosen = *best;

    chosen.correctedRadius = chosen.radius + 2.28 * resolution;
    return true;
}

/** 【函数导航】
 * 作用：评估/审核“scoreTemplateExhaustive”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
MuBanJieGuo scoreTemplateExhaustive(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    double searchRadius,
    double minRadius,
    double maxRadius)
{
    if (!mask.valid()) {
        throw std::invalid_argument("scoreTemplateExhaustive: invalid mask");
    }
    const int width = mask.width;
    const int height = mask.height;
    const double res = mask.resolution;
    const double seedPx = (seedX - mask.originX) / res;
    const double seedPy = (seedY - mask.originY) / res;
    const double searchPx = searchRadius / res;

    std::vector<int> prefix(static_cast<std::size_t>(height * (width + 1)), 0);
    for (int y = 0; y < height; ++y) {
        int acc = 0;
        const std::size_t base = static_cast<std::size_t>(y * (width + 1));
        for (int x = 0; x < width; ++x) {
            acc += mask.at(x, y) ? 1 : 0;
            prefix[base + static_cast<std::size_t>(x + 1)] = acc;
        }
    }
    auto rowSum = [&](int y, int x0, int x1) noexcept -> int {
        const std::size_t base = static_cast<std::size_t>(y * (width + 1));
        return prefix[base + static_cast<std::size_t>(x1 + 1)]
            - prefix[base + static_cast<std::size_t>(x0)];
    };

    /** 【类型导航注释】
     * KuaDuHe：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct KuaDuHe {
        std::vector<int> diskDx;
        std::vector<int> ringOuterDx;
        std::vector<int> ringStrictInnerDx;
        int diskCount = 0;
        int ringCount = 0;
    };
    auto makeDisk = [](double limitPx) {
        const int maxDy = static_cast<int>(std::floor(limitPx + 1e-12));
        std::vector<int> spans(static_cast<std::size_t>(2 * maxDy + 1), -1);
        const double lim2 = limitPx * limitPx;
        for (int dy = -maxDy; dy <= maxDy; ++dy) {
            const double rem = lim2 - static_cast<double>(dy * dy);
            if (rem < -1e-12) continue;
            spans[static_cast<std::size_t>(dy + maxDy)] =
                static_cast<int>(std::floor(std::sqrt(std::max(0.0, rem)) + 1e-12));
        }
        return spans;
    };
    auto makeStrictDisk = [](double limitPx, int maxDy) {
        std::vector<int> spans(static_cast<std::size_t>(2 * maxDy + 1), -1);
        if (limitPx <= 0.0) return spans;
        const double lim2 = limitPx * limitPx;
        for (int dy = -maxDy; dy <= maxDy; ++dy) {
            const double rem = lim2 - static_cast<double>(dy * dy);
            if (rem <= 0.0) continue;

            spans[static_cast<std::size_t>(dy + maxDy)] =
                static_cast<int>(std::ceil(std::sqrt(rem) - 1e-12)) - 1;
        }
        return spans;
    };
    auto countSpans = [](const std::vector<int>& spans) {
        int n = 0;
        for (int dx : spans) if (dx >= 0) n += 2 * dx + 1;
        return n;
    };

    MuBanJieGuo best;
    int bestX = 0;
    int bestY = 0;
    for (double radius = minRadius; radius <= maxRadius + 1e-9; radius += 0.25) {
        const double radiusPx = radius / res;
        const double innerPx = std::max(0.4, radius - 0.8) / res;
        const double outerPx = (radius + 1.6) / res;
        const double ringLoPx = std::max(0.0, radiusPx - 1.0);
        const double corePx = std::max(1.0, radiusPx * 0.55);

        const std::vector<int> innerSpan = makeDisk(innerPx);
        const std::vector<int> outerSpan = makeDisk(outerPx);
        const int outerMaxDy = static_cast<int>((outerSpan.size() - 1) / 2);
        const std::vector<int> strictLoSpan = makeStrictDisk(ringLoPx, outerMaxDy);
        const std::vector<int> coreSpan = makeDisk(corePx);
        const int innerMaxDy = static_cast<int>((innerSpan.size() - 1) / 2);
        const int coreMaxDy = static_cast<int>((coreSpan.size() - 1) / 2);
        const int innerCount = countSpans(innerSpan);
        const int outerCount = countSpans(outerSpan);
        const int strictLoCount = countSpans(strictLoSpan);
        const int ringCount = outerCount - strictLoCount;
        const int coreCount = countSpans(coreSpan);
        if (innerCount <= 0 || ringCount <= 0 || coreCount <= 0) continue;

        const int margin = static_cast<int>(std::ceil((radius + 2.0) / res));
        const int xBegin = margin;
        const int xEnd = width - margin;
        const int yBegin = margin;
        const int yEnd = height - margin;
        if (xBegin >= xEnd || yBegin >= yEnd) continue;

        for (int y = yBegin; y < yEnd; ++y) {
            const double dyp = static_cast<double>(y) - seedPy;
            for (int x = xBegin; x < xEnd; ++x) {
                const double dxp = static_cast<double>(x) - seedPx;
                const double seedDistancePx = std::hypot(dxp, dyp);
                if (seedDistancePx > searchPx) continue;
                const double ratio = seedDistancePx * res / radius;
                if (ratio < 0.75 || ratio > 2.20) continue;

                int innerSum = 0;
                for (int dy = -innerMaxDy; dy <= innerMaxDy; ++dy) {
                    const int dx = innerSpan[static_cast<std::size_t>(dy + innerMaxDy)];
                    if (dx >= 0) innerSum += rowSum(y + dy, x - dx, x + dx);
                }
                int outerSum = 0;
                int strictLoSum = 0;
                for (int dy = -outerMaxDy; dy <= outerMaxDy; ++dy) {
                    const int dxOuter = outerSpan[static_cast<std::size_t>(dy + outerMaxDy)];
                    if (dxOuter >= 0) outerSum += rowSum(y + dy, x - dxOuter, x + dxOuter);
                    const int dxLo = strictLoSpan[static_cast<std::size_t>(dy + outerMaxDy)];
                    if (dxLo >= 0) strictLoSum += rowSum(y + dy, x - dxLo, x + dxLo);
                }
                int coreSum = 0;
                for (int dy = -coreMaxDy; dy <= coreMaxDy; ++dy) {
                    const int dx = coreSpan[static_cast<std::size_t>(dy + coreMaxDy)];
                    if (dx >= 0) coreSum += rowSum(y + dy, x - dx, x + dx);
                }

                const double inner = static_cast<double>(innerSum) / innerCount;
                const double ring = static_cast<double>(outerSum - strictLoSum) / ringCount;
                const double core = static_cast<double>(coreSum) / coreCount;
                const double z = (ratio - 1.15) / 0.55;
                const double seedConsistency = std::exp(-0.5 * z * z);
                const double score = 0.58 * ring + 0.34 * (1.0 - inner)
                    + 0.16 * (1.0 - core)
                    - 0.18 * std::max(0.0, 0.5 - ring)
                    + 0.30 * seedConsistency;
                if (!best.valid || score > best.score) {
                    best.valid = true;
                    bestX = x;
                    bestY = y;
                    best.radius = radius;
                    best.score = score;
                    best.ring = ring;
                    best.inner = inner;
                    best.core = core;
                    best.ratio = ratio;
                    best.proposalSource = "exhaustive_template";
                }
            }
        }
    }
    if (best.valid) {
        best.centerX = mask.originX + static_cast<double>(bestX) * res;
        best.centerY = mask.originY + static_cast<double>(bestY) * res;
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“crosscheckIncompleteLunKuo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
MuBanJieGuo crosscheckIncompleteLunKuo(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    const LunKuoHouXuan& /*contour*/)
{
    return scoreTemplateExhaustive(mask, seedX, seedY);
}

/** 【函数导航】
 * 作用：检测/搜索“detectHoleKouAtLevel”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleKouLevelResult detectHoleKouAtLevel(
    const ErZhiMask& mask,
    double seedX,
    double seedY)
{
    HoleKouLevelResult out;
    const auto contourCandidates=extractLunKuoHouXuan(mask,seedX,seedY);
    LunKuoHouXuan contour;
    const bool hasContour=choosePositiveJiXingLunKuo(contourCandidates,mask.resolution,contour);
    if(hasContour&&contour.score>=2.05){
        out.valid=true;out.centerX=contour.centerX;out.centerY=contour.centerY;
        out.radius=contour.correctedRadius;out.source="contour";out.contour=contour;

        const bool needsCrosscheck = contour.coverage < 0.83
            && !(contour.coverage >= 0.81 && contour.polarity >= 0.85);
        if(needsCrosscheck){
            const MuBanJieGuo t=crosscheckIncompleteLunKuo(mask,seedX,seedY,contour);
            if(t.valid&&t.ring>=0.83&&t.inner>=0.15&&t.score>=1.10&&t.ratio>=0.75&&t.ratio<=2.20){
                out.centerX=t.centerX;out.centerY=t.centerY;out.radius=t.radius;
                out.source="crosscheck";out.templateResult=t;
            }
        }
        return out;
    }
    const auto candidates=proposeHouXuanCenters(mask,seedX,seedY);
    const MuBanJieGuo t=scoreSparseTemplate(mask,seedX,seedY,candidates);
    if(t.valid){out.valid=true;out.centerX=t.centerX;out.centerY=t.centerY;out.radius=t.radius;out.source="template_fast";out.templateResult=t;}
    return out;
}

/** 【函数导航】
 * 作用：执行“proposeHouXuanCenters”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HouXuanCenter> proposeHouXuanCenters(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    double searchRadius,
    int maxCandidates)
{
    if (!mask.valid()) throw std::invalid_argument("proposeHouXuanCenters: invalid mask");
    const auto dt=distanceTransformExact(mask);
    const double sx=(seedX-mask.originX)/mask.resolution;
    const double sy=(seedY-mask.originY)/mask.resolution;
    /** 【类型导航注释】
     * Raw：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Raw { double value,x,y; std::string source; };
    std::vector<Raw> raw;
    const double searchPx=searchRadius/mask.resolution;

    for (int y=2;y+2<mask.height;++y) {
        for (int x=2;x+2<mask.width;++x) {
            const double v=dt[static_cast<std::size_t>(y*mask.width+x)];
            if (v<4.0 || std::hypot(x-sx,y-sy)>searchPx) continue;
            double mx=v;
            for (int yy=y-2;yy<=y+2;++yy) for (int xx=x-2;xx<=x+2;++xx)
                mx=std::max(mx,dt[static_cast<std::size_t>(yy*mask.width+xx)]);
            if (v>=mx-1e-9) raw.push_back({v,static_cast<double>(x),static_cast<double>(y),"dt_exact"});
        }
    }

    std::vector<std::uint8_t> seen(mask.data.size(),0);
    const int dirs[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    std::vector<int> queue; queue.reserve(mask.data.size());
    for (int y=0;y<mask.height;++y) for (int x=0;x<mask.width;++x) {
        const std::size_t id=static_cast<std::size_t>(y*mask.width+x);
        if (mask.data[id] || seen[id]) continue;
        queue.clear(); queue.push_back(static_cast<int>(id)); seen[id]=1;
        std::size_t head=0,count=0; double sumx=0,sumy=0;
        while (head<queue.size()) {
            const int cur=queue[head++]; const int cx=cur%mask.width, cy=cur/mask.width;
            ++count; sumx+=cx; sumy+=cy;
            for (const auto& d:dirs) { const int nx=cx+d[0],ny=cy+d[1];
                if (nx<0||ny<0||nx>=mask.width||ny>=mask.height) continue;
                const std::size_t ni=static_cast<std::size_t>(ny*mask.width+nx);
                if (mask.data[ni] || seen[ni]) {
                    continue;
                }
                seen[ni] = 1;
                queue.push_back(static_cast<int>(ni));
            }
        }
        if (count<12) continue;
        const double cx=sumx/count,cy=sumy/count;
        if (std::hypot(cx-sx,cy-sy)>searchPx) continue;
        const int ix=std::clamp(static_cast<int>(std::llround(cx)),0,mask.width-1);
        const int iy=std::clamp(static_cast<int>(std::llround(cy)),0,mask.height-1);
        raw.push_back({dt[static_cast<std::size_t>(iy*mask.width+ix)],cx,cy,"component"});
    }
    std::stable_sort(raw.begin(),raw.end(),[](const Raw&a,const Raw&b){return a.value>b.value;});
    const int guard=static_cast<int>(std::ceil((9.2+2.0)/mask.resolution));
    std::vector<HouXuanCenter> out;
    for (const auto& r:raw) {
        if (r.x<guard||r.x>mask.width-1-guard||r.y<guard||r.y>mask.height-1-guard) continue;
        bool separate=true; for (const auto& a:out) if ((r.x-a.px)*(r.x-a.px)+(r.y-a.py)*(r.y-a.py)<=4.0){separate=false;break;}
        if (!separate) continue;
        out.push_back({r.x,r.y,r.source,r.value});
        if (static_cast<int>(out.size())>=maxCandidates) break;
    }
    return out;
}

/** 【函数导航】
 * 作用：评估/审核“scoreSparseTemplate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
MuBanJieGuo scoreSparseTemplate(
    const ErZhiMask& mask,
    double seedX,
    double seedY,
    const std::vector<HouXuanCenter>& candidates,
    double ,
    double minRadius,
    double maxRadius)
{
    if (!mask.valid()) {
        throw std::invalid_argument("scoreSparseTemplate: invalid mask");
    }
    if (candidates.empty()) {
        return {};
    }

    const double seedPx = (seedX - mask.originX) / mask.resolution;
    const double seedPy = (seedY - mask.originY) / mask.resolution;
    MuBanJieGuo best;
    double bestPx = 0.0, bestPy = 0.0;

    for (const auto& c : candidates) {
        for (double r = minRadius; r <= maxRadius + 1e-9; r += 0.5) {
            const JuBuScore s = scoreAt(mask, c.px, c.py, r, seedPx, seedPy);
            if (s.ratio < 0.75 || s.ratio > 2.20) {
                continue;
            }
            if (!best.valid || better(s.score, best.score)) {
                best.valid = true;
                bestPx = c.px;
                bestPy = c.py;
                best.radius = r;
                best.score = s.score;
                best.ring = s.ring;
                best.inner = s.inner;
                best.core = s.core;
                best.ratio = s.ratio;
                best.proposalSource = c.source;
            }
        }
    }
    if (!best.valid) {
        return best;
    }

    const double baseX = bestPx;
    const double baseY = bestPy;
    const double baseR = best.radius;
    for (double cy = baseY - 2.0; cy <= baseY + 2.0 + 1e-9; cy += 1.0) {
        for (double cx = baseX - 2.0; cx <= baseX + 2.0 + 1e-9; cx += 1.0) {
            const double r0 = std::max(minRadius, baseR - 0.75);
            const double r1 = std::min(maxRadius, baseR + 0.75);
            for (double r = r0; r <= r1 + 1e-9; r += 0.25) {
                const JuBuScore s = scoreAt(mask, cx, cy, r, seedPx, seedPy);
                if (s.ratio < 0.75 || s.ratio > 2.20) {
                    continue;
                }
                if (better(s.score, best.score)) {
                    bestPx = cx;
                    bestPy = cy;
                    best.radius = r;
                    best.score = s.score;
                    best.ring = s.ring;
                    best.inner = s.inner;
                    best.core = s.core;
                    best.ratio = s.ratio;
                    best.proposalSource = "refine";
                }
            }
        }
    }

    best.centerX = mask.originX + bestPx * mask.resolution;
    best.centerY = mask.originY + bestPy * mask.resolution;
    return best;
}

/** 【函数导航】
 * 作用：执行“cheapGaoduLevelScore”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double cheapGaoduLevelScore(int surfacePointCount, double shift) noexcept {
    return 0.65 * std::log1p(static_cast<double>(std::max(0, surfacePointCount)))
        - 0.45 * std::abs(shift);
}

/** 【函数导航】
 * 作用：执行“attachGaoduScore”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
GaoduLevelJieGuo attachGaoduScore(
    const MuBanJieGuo& circle,
    double shift,
    int surfacePointCount,
    double /*seedX*/,
    double /*seedY*/) noexcept
{
    GaoduLevelJieGuo out;
    out.circle = circle;
    out.shift = shift;
    out.surfacePointCount = surfacePointCount;
    if (!circle.valid) {
        return out;
    }
    double raw = circle.score;
    if (circle.ratio < 0.65) {
        raw -= 0.25 * (0.65 - circle.ratio);
    }
    if (circle.ratio > 2.4) {
        raw -= 0.12 * (circle.ratio - 2.4);
    }
    out.rawScore = raw;
    out.levelScore = raw + cheapGaoduLevelScore(surfacePointCount, shift);
    return out;
}

/** 【函数导航】
 * 作用：选择“chooseBestLevel”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
const GaoduLevelJieGuo* chooseBestLevel(const std::vector<GaoduLevelJieGuo>& levels) noexcept {
    const GaoduLevelJieGuo* best = nullptr;
    for (const auto& level : levels) {
        if (!level.circle.valid) {
            continue;
        }
        if (!best || level.levelScore > best->levelScore) {
            best = &level;
        }
    }
    return best;
}

}


// ============================================================================
// 功能分区：多高度检测编排
// ============================================================================
/*
模块职责：
手动选孔识别子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的手动选孔识别链调用，把用户种子转换为局部候选、几何或姿态证据。

维护说明：
ROI 半径、采样步长、迭代次数和内点距离均可能影响首次识别结果；默认值保持生产基线。
*/
// 与上方基础检测实现共用 ShouDongHole_Manual.h。


namespace shouDongHole {
namespace {

/** 【类型导航注释】
 * Trial：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Trial {
    double shift = 0.0;
    PingMianModel plane;
    ErZhiMask mask;
    int surfacePointCount = 0;
    bool hasContour = false;
    LunKuoHouXuan contour;
    std::vector<LunKuoHouXuan> contourCandidates;
    double contourLevelScore = -1e30;
};

/** 【类型导航注释】
 * MuBanShiSuan：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct MuBanShiSuan {
    bool valid = false;
    double levelScore = -1e30;
    double rawScore = -1e30;
    std::size_t trialIndex = 0;
    MuBanJieGuo circle;
};

/** 【函数导航】
 * 作用：执行“planeFromNormalSeed”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
PingMianModel planeFromNormalSeed(const Vec3d& normal, const Point3d& seed, double shift) {
    if (std::abs(normal.z) < 1e-9) {
        throw std::invalid_argument("planeFromNormalSeed: normal.z too small");
    }
    const double a = -normal.x / normal.z;
    const double b = -normal.y / normal.z;
    const double c = seed.z - a * seed.x - b * seed.y + shift;
    return {a, b, c};
}

/** 【函数导航】
 * 作用：执行“collectTrialWindowPoints”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<Point3d> collectTrialWindowPoints(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    double halfWidth = 21.0)
{
    std::vector<Point3d> window;
    window.reserve(std::min<std::size_t>(points.size(), 50000));
    for (const Point3d& p : points) {
        if (std::abs(p.x - seed.x) > halfWidth || std::abs(p.y - seed.y) > halfWidth) continue;
        window.push_back(p);
    }
    return window;
}

/** 【函数导航】
 * 作用：构建“buildTrial”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Trial buildTrial(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal,
    double shift)
{
    Trial trial;
    trial.shift = shift;
    trial.plane = planeFromNormalSeed(globalNormal, seed, shift);
    trial.mask = makeSurfaceMask(points, seed, trial.plane, 21.0, 0.25, 0.42,
        &trial.surfacePointCount);
    trial.contourCandidates = extractLunKuoHouXuan(trial.mask, seed.x, seed.y);
    trial.hasContour = choosePositiveJiXingLunKuo(trial.contourCandidates, trial.mask.resolution, trial.contour);
    if (trial.hasContour) {
        trial.contourLevelScore = trial.contour.score
            + cheapGaoduLevelScore(trial.surfacePointCount, trial.shift);
    }
    return trial;
}

/** 【函数导航】
 * 作用：评估/审核“evaluateTemplate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
MuBanShiSuan evaluateTemplate(const Trial& trial, std::size_t index, const Point3d& seed) {
    MuBanShiSuan result;
    const auto candidates = proposeHouXuanCenters(trial.mask, seed.x, seed.y);
    MuBanJieGuo circle = scoreSparseTemplate(trial.mask, seed.x, seed.y, candidates);
    if (!circle.valid) return result;
    double raw = circle.score;
    if (circle.ratio < 0.65) raw -= 0.25 * (0.65 - circle.ratio);
    if (circle.ratio > 2.4) raw -= 0.12 * (circle.ratio - 2.4);
    result.valid = true;
    result.rawScore = raw;
    result.levelScore = raw + cheapGaoduLevelScore(trial.surfacePointCount, trial.shift);
    result.trialIndex = index;
    result.circle = circle;
    return result;
}

/** 【函数导航】
 * 作用：执行“betterTemplate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool betterTemplate(const MuBanShiSuan& a, const MuBanShiSuan& b) {
    return a.valid && (!b.valid || a.levelScore > b.levelScore);
}

/** 【类型导航注释】
 * ChiXuLunKuoJieGuo：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ChiXuLunKuoJieGuo {
    bool valid = false;
    std::size_t trialIndex = 0;
    LunKuoHouXuan contour;
    double rawScore = -1e30;
    double levelScore = -1e30;
    int distinctLevels = 0;
    bool splitLobeRecovered = false;
    bool upperMouthMode = false;
    bool largeRingTuoDi = false;
    bool compactOuterMode = false;
};

/** 【函数导航】
 * 作用：选择“choosePersistentContourFamily”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ChiXuLunKuoJieGuo choosePersistentContourFamily(
    const std::vector<Trial>& trials,
    const Point3d& seed)
{
    /** 【类型导航注释】
     * Ref：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Ref {
        std::size_t trialIndex = 0;
        const LunKuoHouXuan* candidate = nullptr;
    };
    std::vector<Ref> refs;
    for (std::size_t ti = 0; ti < trials.size(); ++ti) {
        for (const LunKuoHouXuan& c : trials[ti].contourCandidates) {
            const double distance = std::hypot(c.centerX - seed.x, c.centerY - seed.y);

            if (distance > 14.0) continue;
            if (c.radius < 1.25 || c.radius > 9.2) continue;
            if (c.polarity < 0.35 || c.coverage < 0.46 || c.rmse > 1.10) continue;
            refs.push_back({ti, &c});
        }
    }
    if (refs.size() < 2) return {};

    std::vector<std::size_t> parent(refs.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    auto findRoot = [&](std::size_t x) {
        std::size_t root = x;
        while (parent[root] != root) root = parent[root];
        while (parent[x] != x) {
            const std::size_t next = parent[x];
            parent[x] = root;
            x = next;
        }
        return root;
    };
    auto unite = [&](std::size_t a, std::size_t b) {
        a = findRoot(a);
        b = findRoot(b);
        if (a != b) parent[b] = a;
    };
    for (std::size_t i = 0; i < refs.size(); ++i) {
        for (std::size_t j = i + 1; j < refs.size(); ++j) {
            if (refs[i].trialIndex == refs[j].trialIndex) continue;
            const LunKuoHouXuan& a = *refs[i].candidate;
            const LunKuoHouXuan& b = *refs[j].candidate;
            const double maxRadius = std::max(a.radius, b.radius);
            const double centerTolerance = std::max(0.90, 0.24 * maxRadius + 0.30);
            const double radiusTolerance = std::max(0.85, 0.38 * maxRadius);
            if (std::hypot(a.centerX - b.centerX, a.centerY - b.centerY) <= centerTolerance
                && std::abs(a.radius - b.radius) <= radiusTolerance) {
                unite(i, j);
            }
        }
    }

    /** 【类型导航注释】
     * Family：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：HoleFenxi_Analysis.h。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Family {
        std::vector<Ref> members;
        LunKuoHouXuan representative;
        std::size_t representativeTrial = 0;
        int distinctLevels = 0;
        double score = -1e30;
        double medianPolarity = 0.0;
        double medianCoverage = 0.0;
        bool upperMouthMode = false;
    };
    std::vector<Family> families;
    std::vector<std::size_t> roots;
    for (std::size_t i = 0; i < refs.size(); ++i) {
        const std::size_t root = findRoot(i);
        auto it = std::find(roots.begin(), roots.end(), root);
        if (it == roots.end()) {
            roots.push_back(root);
            families.push_back({});
            it = roots.end() - 1;
        }
        families[static_cast<std::size_t>(it - roots.begin())].members.push_back(refs[i]);
    }

    auto medianValue = [](std::vector<double> values) {
        if (values.empty()) return 0.0;
        const std::size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
        double value = values[mid];
        if ((values.size() & 1U) == 0U) {
            const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
            value = 0.5 * (value + *lower);
        }
        return value;
    };

    for (Family& family : families) {

        std::vector<Ref> perLevel;
        for (const Ref& ref : family.members) {
            auto it = std::find_if(perLevel.begin(), perLevel.end(), [&](const Ref& existing) {
                return existing.trialIndex == ref.trialIndex;
            });
            if (it == perLevel.end()) {
                perLevel.push_back(ref);
            } else if (ref.candidate->score > it->candidate->score) {
                *it = ref;
            }
        }
        family.distinctLevels = static_cast<int>(perLevel.size());
        if (family.distinctLevels < 2) continue;

        auto correctedRadius = [&](const Ref& ref) {
            return ref.candidate->radius + 2.28 * trials[ref.trialIndex].mask.resolution;
        };

        std::vector<Ref> representativeRefs = perLevel;
        if (perLevel.size() >= 4) {
            double meanShift = 0.0, meanRadius = 0.0;
            double minRadius = std::numeric_limits<double>::infinity();
            double maxRadius = -std::numeric_limits<double>::infinity();
            for (const Ref& ref : perLevel) {
                meanShift += trials[ref.trialIndex].shift;
                const double radius = correctedRadius(ref);
                meanRadius += radius;
                minRadius = std::min(minRadius, radius);
                maxRadius = std::max(maxRadius, radius);
            }
            meanShift /= static_cast<double>(perLevel.size());
            meanRadius /= static_cast<double>(perLevel.size());
            double covariance = 0.0, shiftVariance = 0.0, radiusVariance = 0.0;
            for (const Ref& ref : perLevel) {
                const double ds = trials[ref.trialIndex].shift - meanShift;
                const double dr = correctedRadius(ref) - meanRadius;
                covariance += ds * dr;
                shiftVariance += ds * ds;
                radiusVariance += dr * dr;
            }
            const double slope = shiftVariance > 1e-9 ? covariance / shiftVariance : 0.0;
            const double correlation = (shiftVariance > 1e-9 && radiusVariance > 1e-9)
                ? covariance / std::sqrt(shiftVariance * radiusVariance) : 0.0;
            if (maxRadius - minRadius >= 0.80 && slope >= 0.42 && correlation >= 0.78) {
                std::sort(representativeRefs.begin(), representativeRefs.end(), [&](const Ref& left, const Ref& right) {
                    return correctedRadius(left) > correctedRadius(right);
                });
                const std::size_t upperCount = std::max<std::size_t>(2,
                    static_cast<std::size_t>(std::ceil(0.40 * static_cast<double>(representativeRefs.size()))));
                representativeRefs.resize(std::min(upperCount, representativeRefs.size()));
                family.upperMouthMode = true;
            }
        }

        std::vector<double> xs, ys, radii, polarities, coverages, scores;
        xs.reserve(representativeRefs.size()); ys.reserve(representativeRefs.size());
        radii.reserve(representativeRefs.size()); polarities.reserve(representativeRefs.size());
        coverages.reserve(representativeRefs.size()); scores.reserve(representativeRefs.size());
        for (const Ref& ref : representativeRefs) {
            xs.push_back(ref.candidate->centerX);
            ys.push_back(ref.candidate->centerY);
            radii.push_back(correctedRadius(ref));
            polarities.push_back(ref.candidate->polarity);
            coverages.push_back(ref.candidate->coverage);
            scores.push_back(ref.candidate->score);
        }
        family.representative.centerX = medianValue(xs);
        family.representative.centerY = medianValue(ys);
        family.representative.radius = medianValue(radii);
        family.representative.correctedRadius = family.representative.radius;
        family.representative.polarity = medianValue(polarities);
        family.representative.coverage = medianValue(coverages);
        family.representative.score = medianValue(scores);
        family.representative.kind = family.upperMouthMode ? "persistent_upper_mouth" : "persistent";
        family.medianPolarity = family.representative.polarity;
        family.medianCoverage = family.representative.coverage;

        double nearest = std::numeric_limits<double>::infinity();
        for (const Ref& ref : representativeRefs) {
            const double delta = std::hypot(ref.candidate->centerX - family.representative.centerX,
                                            ref.candidate->centerY - family.representative.centerY)
                + 0.4 * std::abs(correctedRadius(ref) - family.representative.radius);
            if (delta < nearest) {
                nearest = delta;
                family.representativeTrial = ref.trialIndex;
            }
        }
        const double seedDistance = std::hypot(family.representative.centerX - seed.x,
                                               family.representative.centerY - seed.y);
        family.score = family.representative.score
            + 0.22 * std::min(3, family.distinctLevels - 1)
            + 0.38 * family.medianPolarity
            + 0.22 * family.medianCoverage
            + (family.upperMouthMode ? 0.08 : 0.0)
            - 0.018 * seedDistance;
    }

    Family* best = nullptr;
    for (Family& family : families) {
        if (family.distinctLevels < 2) continue;
        if (family.medianPolarity < 0.40 || family.medianCoverage < 0.50) continue;
        if (!best || family.score > best->score) best = &family;
    }
    if (!best) return {};

    bool splitLobeRecovered = false;
    for (Family& parentFamily : families) {
        if (&parentFamily == best || parentFamily.distinctLevels < 2) continue;
        const double centerDistance = std::hypot(
            parentFamily.representative.centerX - best->representative.centerX,
            parentFamily.representative.centerY - best->representative.centerY);
        const double radiusGap = parentFamily.representative.radius - best->representative.radius;
        const bool enclosingFamily =
            parentFamily.medianPolarity >= 0.40 && parentFamily.medianCoverage >= 0.56
            && radiusGap >= 0.70
            && centerDistance <= parentFamily.representative.radius + 0.35
            && parentFamily.score >= best->score - 0.48;

        const bool splitLobeFamily =
            best->representative.radius <= 2.45
            && parentFamily.representative.radius >= 2.65
            && parentFamily.representative.radius <= 5.20
            && parentFamily.medianPolarity >= 0.34
            && parentFamily.medianCoverage >= 0.50
            && radiusGap >= 0.55
            && centerDistance >= 1.15
            && centerDistance <= parentFamily.representative.radius + 0.65
            && parentFamily.score >= best->score - 0.95;
        if (enclosingFamily || splitLobeFamily) {

            if (parentFamily.representative.radius < 7.0) {
                best = &parentFamily;
                splitLobeRecovered = splitLobeFamily;
            }
        }
    }

    if (best->representative.radius <= 2.45) {
        const Ref* singleParent = nullptr;
        double singleParentScore = -1e30;
        for (const Ref& ref : refs) {
            const double radius = ref.candidate->radius + 2.28 * trials[ref.trialIndex].mask.resolution;
            const double centerDistance = std::hypot(
                ref.candidate->centerX - best->representative.centerX,
                ref.candidate->centerY - best->representative.centerY);
            if (radius < best->representative.radius + 0.50 || radius > 5.20) continue;
            if (centerDistance < 1.25 || centerDistance > radius + 0.65) continue;
            if (ref.candidate->polarity < 0.45 || ref.candidate->coverage < 0.60
                || ref.candidate->rmse > 0.98 || ref.candidate->score < 1.65) continue;
            const double score = ref.candidate->score + 0.30 * ref.candidate->polarity
                + 0.18 * ref.candidate->coverage - 0.10 * ref.candidate->rmse;
            if (!singleParent || score > singleParentScore) {
                singleParent = &ref;
                singleParentScore = score;
            }
        }
        if (singleParent) {
            best->representative = *singleParent->candidate;
            best->representative.radius += 2.28 * trials[singleParent->trialIndex].mask.resolution;
            best->representative.correctedRadius = best->representative.radius;
            best->representative.kind = "persistent_single_parent";
            best->representativeTrial = singleParent->trialIndex;
            best->score = std::max(best->score - 0.10, singleParentScore);
            splitLobeRecovered = true;
        }
    }

    bool largeRingTuoDi = false;
    if (best->representative.radius >= 7.50) {
        Family* inner = nullptr;
        for (Family& candidate : families) {
            if (&candidate == best || candidate.distinctLevels < 2) continue;
            if (candidate.medianPolarity < 0.38 || candidate.medianCoverage < 0.48) continue;
            const double radius = candidate.representative.radius;
            if (radius < 2.00 || radius > 6.50) continue;
            if (radius < 0.35 * best->representative.radius) continue;
            if (best->representative.radius < 1.30 * radius) continue;
            const double centerDistance = std::hypot(
                candidate.representative.centerX - best->representative.centerX,
                candidate.representative.centerY - best->representative.centerY);
            if (centerDistance > 2.80 || candidate.score < best->score - 1.80) continue;
            if (!inner || radius > inner->representative.radius + 0.30
                || (std::abs(radius - inner->representative.radius) <= 0.30
                    && (candidate.score > inner->score
                        || (std::abs(candidate.score - inner->score) < 0.15
                            && candidate.distinctLevels > inner->distinctLevels)))) {
                inner = &candidate;
            }
        }
        if (inner) {
            best = inner;
            largeRingTuoDi = true;
        }
    }

    ChiXuLunKuoJieGuo output;
    output.valid = true;
    output.trialIndex = best->representativeTrial;
    output.contour = best->representative;
    output.rawScore = best->score;
    output.levelScore = best->score
        + cheapGaoduLevelScore(trials[best->representativeTrial].surfacePointCount,
                                trials[best->representativeTrial].shift);
    output.distinctLevels = best->distinctLevels;
    output.splitLobeRecovered = splitLobeRecovered;
    output.upperMouthMode = best->upperMouthMode;
    output.largeRingTuoDi = largeRingTuoDi;
    return output;
}

/** 【函数导航】
 * 作用：选择“chooseNestedOuterMouthFamily”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ChiXuLunKuoJieGuo chooseNestedOuterMouthFamily(
    const std::vector<Trial>& trials,
    const Point3d& seed)
{
    /** 【类型导航注释】
     * PairRef：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct PairRef {
        std::size_t trialIndex = 0;
        const LunKuoHouXuan* inner = nullptr;
        const LunKuoHouXuan* outer = nullptr;
    };
    std::vector<PairRef> pairs;
    for (std::size_t trialIndex = 0; trialIndex < trials.size(); ++trialIndex) {
        PairRef bestPair;
        double bestPairScore = -1e30;
        const auto& candidates = trials[trialIndex].contourCandidates;
        for (const LunKuoHouXuan& inner : candidates) {
            if (inner.polarity < 0.55 || inner.coverage < 0.88 || inner.rmse > 0.65) continue;
            if (inner.radius < 2.8 || inner.radius > 7.2) continue;
            for (const LunKuoHouXuan& outer : candidates) {
                if (outer.polarity > -0.55 || outer.coverage < 0.90 || outer.rmse > 0.38) continue;
                const double radiusGap = outer.radius - inner.radius;
                if (radiusGap < 0.65 || radiusGap > 2.20) continue;
                const double centerDistance = std::hypot(
                    outer.centerX - inner.centerX, outer.centerY - inner.centerY);
                if (centerDistance > 0.48) continue;
                const double seedDistance = std::hypot(outer.centerX - seed.x, outer.centerY - seed.y);
                if (seedDistance > 14.0) continue;
                const double score = inner.score + 0.30 * inner.polarity
                    + 0.20 * inner.coverage - 0.22 * inner.rmse
                    + 0.18 * (-outer.polarity) + 0.12 * outer.coverage
                    - 0.15 * outer.rmse - 0.08 * centerDistance;
                if (score > bestPairScore) {
                    bestPairScore = score;
                    bestPair = {trialIndex, &inner, &outer};
                }
            }
        }
        if (bestPair.inner && bestPair.outer) pairs.push_back(bestPair);
    }
    if (pairs.size() < 2) return {};

    auto medianValue = [](std::vector<double> values) {
        if (values.empty()) return 0.0;
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
        double value = values[middle];
        if ((values.size() & 1U) == 0U) {
            const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
            value = 0.5 * (value + *lower);
        }
        return value;
    };
    auto correctedOuterRadius = [&](const PairRef& pair) {
        return pair.outer->radius + 2.28 * trials[pair.trialIndex].mask.resolution;
    };

    std::vector<double> allInnerRadii;
    allInnerRadii.reserve(pairs.size());
    for (const PairRef& pair : pairs) {
        allInnerRadii.push_back(pair.inner->radius + 2.28 * trials[pair.trialIndex].mask.resolution);
    }
    const double allMedianInner = medianValue(allInnerRadii);
    const bool compactInnerTrace = allMedianInner < 5.00;
    if (compactInnerTrace) {
        auto pairSupportScore = [&](const PairRef& pair) {
            const Trial& trial = trials[pair.trialIndex];
            return cheapGaoduLevelScore(trial.surfacePointCount, trial.shift)
                + 0.16 * pair.inner->score
                + 0.10 * pair.inner->polarity
                + 0.06 * (-pair.outer->polarity)
                - 0.08 * pair.inner->rmse
                - 0.06 * pair.outer->rmse;
        };
        std::sort(pairs.begin(), pairs.end(), [&](const PairRef& left, const PairRef& right) {
            const double leftScore = pairSupportScore(left);
            const double rightScore = pairSupportScore(right);
            if (std::abs(leftScore - rightScore) > 1e-9) return leftScore > rightScore;
            return correctedOuterRadius(left) > correctedOuterRadius(right);
        });
    } else {
        std::sort(pairs.begin(), pairs.end(), [&](const PairRef& left, const PairRef& right) {
            return correctedOuterRadius(left) > correctedOuterRadius(right);
        });
    }
    const std::size_t upperCount = std::max<std::size_t>(2,
        static_cast<std::size_t>(std::ceil(0.50 * static_cast<double>(pairs.size()))));
    pairs.resize(std::min(upperCount, pairs.size()));

    std::vector<double> xs, ys, radii, scores, selectedGaps;
    xs.reserve(pairs.size()); ys.reserve(pairs.size()); radii.reserve(pairs.size()); scores.reserve(pairs.size());
    for (const PairRef& pair : pairs) {
        xs.push_back(pair.outer->centerX);
        ys.push_back(pair.outer->centerY);
        const double correctedInner = pair.inner->radius + 2.28 * trials[pair.trialIndex].mask.resolution;
        const double correctedOuter = correctedOuterRadius(pair);
        radii.push_back(correctedOuter);
        selectedGaps.push_back(correctedOuter - correctedInner);
        scores.push_back(pair.inner->score);
    }
    const double centerX = medianValue(xs);
    const double centerY = medianValue(ys);
    double radius = medianValue(radii);
    const double medianGap = medianValue(selectedGaps);
    if (compactInnerTrace && medianGap >= 1.45) {
        radius -= 0.15 * medianGap;
    }
    double maximumCenterDeviation = 0.0;
    for (const PairRef& pair : pairs) {
        maximumCenterDeviation = std::max(maximumCenterDeviation,
            std::hypot(pair.outer->centerX - centerX, pair.outer->centerY - centerY));
    }
    if (maximumCenterDeviation > 0.80 || radius < 3.5 || radius > 8.0) return {};

    std::size_t representativeTrial = pairs.front().trialIndex;
    double nearest = std::numeric_limits<double>::infinity();
    for (const PairRef& pair : pairs) {
        const double delta = std::hypot(pair.outer->centerX - centerX, pair.outer->centerY - centerY)
            + 0.35 * std::abs(correctedOuterRadius(pair) - radius);
        if (delta < nearest) { nearest = delta; representativeTrial = pair.trialIndex; }
    }

    ChiXuLunKuoJieGuo output;
    output.valid = true;
    output.trialIndex = representativeTrial;
    output.contour.kind = "nested_outer_mouth";
    output.contour.centerX = centerX;
    output.contour.centerY = centerY;
    output.contour.radius = radius;
    output.contour.correctedRadius = radius;
    output.contour.coverage = 1.0;
    output.contour.polarity = 1.0;
    output.contour.score = medianValue(scores);
    output.rawScore = output.contour.score + 0.28 * std::min<std::size_t>(4, pairs.size());
    output.levelScore = output.rawScore + cheapGaoduLevelScore(
        trials[representativeTrial].surfacePointCount, trials[representativeTrial].shift);
    output.distinctLevels = static_cast<int>(pairs.size());
    output.upperMouthMode = true;
    output.compactOuterMode = compactInnerTrace;
    return output;
}

/** 【函数导航】
 * 作用：选择“chooseReversePolarityContourFamily”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ChiXuLunKuoJieGuo chooseReversePolarityContourFamily(
    const std::vector<Trial>& trials,
    const Point3d& seed)
{
    /** 【类型导航注释】
     * Ref：手动 Hole 几何检测中的自定义 结构体。
     * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Ref {
        std::size_t trialIndex = 0;
        const LunKuoHouXuan* candidate = nullptr;
    };
    std::vector<Ref> refs;
    for (std::size_t trialIndex = 0; trialIndex < trials.size(); ++trialIndex) {
        for (const LunKuoHouXuan& candidate : trials[trialIndex].contourCandidates) {
            const double distance = std::hypot(candidate.centerX - seed.x, candidate.centerY - seed.y);
            if (distance > 14.0 || candidate.radius < 3.0 || candidate.radius > 8.6) continue;
            if (candidate.polarity < -0.42 || candidate.polarity > 0.25) continue;
            if (candidate.coverage < 0.72 || candidate.rmse > 1.15 || candidate.score < 1.02) continue;
            refs.push_back({trialIndex, &candidate});
        }
    }
    if (refs.size() < 2) return {};

    auto medianValue = [](std::vector<double> values) {
        if (values.empty()) return 0.0;
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
        double value = values[middle];
        if ((values.size() & 1U) == 0U) {
            const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
            value = 0.5 * (value + *lower);
        }
        return value;
    };
    auto correctedRadius = [&](const Ref& ref) {
        return ref.candidate->radius + 2.28 * trials[ref.trialIndex].mask.resolution;
    };

    ChiXuLunKuoJieGuo best;
    double bestScore = -1e30;
    for (const Ref& anchor : refs) {
        std::vector<Ref> members;
        for (const Ref& candidate : refs) {
            if (candidate.trialIndex == anchor.trialIndex && candidate.candidate != anchor.candidate) continue;
            const double maximumRadius = std::max(anchor.candidate->radius, candidate.candidate->radius);
            const double radiusTolerance = std::max(0.90, 0.35 * maximumRadius);
            if (std::hypot(anchor.candidate->centerX - candidate.candidate->centerX,
                           anchor.candidate->centerY - candidate.candidate->centerY) <= 1.05
                && std::abs(anchor.candidate->radius - candidate.candidate->radius) <= radiusTolerance) {
                auto existing = std::find_if(members.begin(), members.end(), [&](const Ref& value) {
                    return value.trialIndex == candidate.trialIndex;
                });
                if (existing == members.end()) members.push_back(candidate);
                else if (candidate.candidate->score > existing->candidate->score) *existing = candidate;
            }
        }
        if (members.size() < 2) continue;

        std::vector<Ref> representativeMembers = members;
        if (members.size() >= 4) {
            double meanShift = 0.0, meanRadius = 0.0;
            double minimumRadius = std::numeric_limits<double>::infinity();
            double maximumRadius = -std::numeric_limits<double>::infinity();
            for (const Ref& member : members) {
                meanShift += trials[member.trialIndex].shift;
                const double radius = correctedRadius(member);
                meanRadius += radius;
                minimumRadius = std::min(minimumRadius, radius);
                maximumRadius = std::max(maximumRadius, radius);
            }
            meanShift /= static_cast<double>(members.size());
            meanRadius /= static_cast<double>(members.size());
            double covariance = 0.0, shiftVariance = 0.0, radiusVariance = 0.0;
            for (const Ref& member : members) {
                const double ds = trials[member.trialIndex].shift - meanShift;
                const double dr = correctedRadius(member) - meanRadius;
                covariance += ds * dr;
                shiftVariance += ds * ds;
                radiusVariance += dr * dr;
            }
            const double slope = shiftVariance > 1e-9 ? covariance / shiftVariance : 0.0;
            const double correlation = (shiftVariance > 1e-9 && radiusVariance > 1e-9)
                ? covariance / std::sqrt(shiftVariance * radiusVariance) : 0.0;
            if (maximumRadius - minimumRadius >= 0.70 && slope >= 0.35 && correlation >= 0.72) {
                std::sort(representativeMembers.begin(), representativeMembers.end(), [&](const Ref& left, const Ref& right) {
                    return correctedRadius(left) > correctedRadius(right);
                });
                const std::size_t count = std::max<std::size_t>(2,
                    static_cast<std::size_t>(std::ceil(0.40 * static_cast<double>(representativeMembers.size()))));
                representativeMembers.resize(std::min(count, representativeMembers.size()));
            }
        }

        std::vector<double> xs, ys, radii, coverages, rmses, scores;
        for (const Ref& member : representativeMembers) {
            xs.push_back(member.candidate->centerX);
            ys.push_back(member.candidate->centerY);
            radii.push_back(correctedRadius(member));
            coverages.push_back(member.candidate->coverage);
            rmses.push_back(member.candidate->rmse);
            scores.push_back(member.candidate->score);
        }
        const double centerX = medianValue(xs);
        const double centerY = medianValue(ys);
        double maximumCenterDeviation = 0.0;
        for (const Ref& member : members) {
            maximumCenterDeviation = std::max(maximumCenterDeviation,
                std::hypot(member.candidate->centerX - centerX, member.candidate->centerY - centerY));
        }
        if (maximumCenterDeviation > 1.20) continue;
        double radius = medianValue(radii);
        const double coverage = medianValue(coverages);
        const double rmse = medianValue(rmses);
        const double familyScore = medianValue(scores)
            + 0.32 * std::min<std::size_t>(4, members.size() - 1)
            + 0.35 * coverage - 0.20 * rmse
            - 0.015 * std::hypot(centerX - seed.x, centerY - seed.y);

        if (members.size() == 2 && familyScore < 1.80 && radius >= 6.50) {
            radius -= 0.35;
        }
        if (familyScore <= bestScore) continue;

        std::size_t representativeTrial = representativeMembers.front().trialIndex;
        double nearest = std::numeric_limits<double>::infinity();
        for (const Ref& member : representativeMembers) {
            const double delta = std::hypot(member.candidate->centerX - centerX,
                                            member.candidate->centerY - centerY)
                + 0.35 * std::abs(correctedRadius(member) - radius);
            if (delta < nearest) { nearest = delta; representativeTrial = member.trialIndex; }
        }
        bestScore = familyScore;
        best.valid = true;
        best.trialIndex = representativeTrial;
        best.contour.kind = "reverse_persistent";
        best.contour.centerX = centerX;
        best.contour.centerY = centerY;
        best.contour.radius = radius;
        best.contour.correctedRadius = radius;
        best.contour.coverage = coverage;
        best.contour.rmse = rmse;
        best.contour.score = medianValue(scores);
        best.rawScore = familyScore;
        best.levelScore = familyScore + cheapGaoduLevelScore(
            trials[representativeTrial].surfacePointCount, trials[representativeTrial].shift);
        best.distinctLevels = static_cast<int>(members.size());
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“gaussianSmoothSigmaOneReflect”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<double> gaussianSmoothSigmaOneReflect(const std::vector<double>& values) {
    static const double rawWeights[9] = {
        std::exp(-8.0), std::exp(-4.5), std::exp(-2.0), std::exp(-0.5), 1.0,
        std::exp(-0.5), std::exp(-2.0), std::exp(-4.5), std::exp(-8.0)};
    double weightSum = 0.0;
    for (double w : rawWeights) weightSum += w;
    auto reflectHalfSample = [&](int index) {
        const int n = static_cast<int>(values.size());
        while (index < 0 || index >= n) {
            if (index < 0) index = -index - 1;
            if (index >= n) index = 2 * n - index - 1;
        }
        return index;
    };
    std::vector<double> output(values.size(), 0.0);
    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        double sum = 0.0;
        for (int k = -4; k <= 4; ++k) {
            sum += values[static_cast<std::size_t>(reflectHalfSample(i + k))]
                * rawWeights[static_cast<std::size_t>(k + 4)];
        }
        output[static_cast<std::size_t>(i)] = sum / weightSum;
    }
    return output;
}

/** 【函数导航】
 * 作用：执行“peakProminence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double peakProminence(const std::vector<double>& values, int index) {
    const double peak = values[static_cast<std::size_t>(index)];
    double leftMinimum = peak;
    for (int i = index - 1; i >= 0; --i) {
        leftMinimum = std::min(leftMinimum, values[static_cast<std::size_t>(i)]);
        if (values[static_cast<std::size_t>(i)] > peak) break;
    }
    double rightMinimum = peak;
    for (int i = index + 1; i < static_cast<int>(values.size()); ++i) {
        rightMinimum = std::min(rightMinimum, values[static_cast<std::size_t>(i)]);
        if (values[static_cast<std::size_t>(i)] > peak) break;
    }
    return peak - std::max(leftMinimum, rightMinimum);
}

/** 【函数导航】
 * 作用：执行“appendUniqueShift”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void appendUniqueShift(std::vector<double>& shifts, double value) {
    for (double existing : shifts) {
        if (std::abs(existing - value) <= 0.18) return;
    }
    shifts.push_back(value);
}

/** 【类型导航注释】
 * SurfaceShiftJingXiu：手动 Hole 几何检测中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct SurfaceShiftJingXiu {
    bool valid = false;
    double shift = 0.0;
    int pointCount = 0;
};

/** 【函数导航】
 * 作用：精修“refineSurfaceShiftFromOuterAnnulus”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
SurfaceShiftJingXiu refineSurfaceShiftFromOuterAnnulus(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal,
    double centerX,
    double centerY,
    double radius,
    double currentShift)
{
    SurfaceShiftJingXiu output;
    if (std::abs(globalNormal.z) < 1e-9 || radius < 1.0) return output;
    const double a = -globalNormal.x / globalNormal.z;
    const double b = -globalNormal.y / globalNormal.z;
    const double c = seed.z - a * seed.x - b * seed.y;
    constexpr double low = -3.0;
    constexpr double high = 3.0;
    constexpr double binWidth = 0.10;
    constexpr int binCount = 60;
    std::array<int, binCount> histogram{};
    const double inner = radius + 0.40;
    const double outer = radius + 3.00;
    const double inner2 = inner * inner;
    const double outer2 = outer * outer;
    int count = 0;
    for (const Point3d& p : points) {
        const double dx = p.x - centerX;
        const double dy = p.y - centerY;
        const double rr = dx * dx + dy * dy;
        if (rr < inner2 || rr > outer2) continue;
        const double residual = p.z - (a * p.x + b * p.y + c);
        if (residual < low || residual > high) continue;
        int index = static_cast<int>(std::floor((residual - low) / binWidth));
        if (index == binCount) index = binCount - 1;
        if (index < 0 || index >= binCount) continue;
        ++histogram[static_cast<std::size_t>(index)];
        ++count;
    }
    output.pointCount = count;
    if (count < 500) return output;
    std::array<int, binCount> smooth{};
    for (int i = 0; i < binCount; ++i) {
        int value = 0;
        for (int k = -2; k <= 2; ++k) {
            const int j = i + k;
            if (j < 0 || j >= binCount) continue;
            const int weight = (k == 0) ? 3 : (std::abs(k) == 1 ? 2 : 1);
            value += weight * histogram[static_cast<std::size_t>(j)];
        }
        smooth[static_cast<std::size_t>(i)] = value;
    }
    const auto best = std::max_element(smooth.begin(), smooth.end());
    if (best == smooth.end() || *best <= 0) return output;
    const int index = static_cast<int>(best - smooth.begin());
    const double modeShift = low + (static_cast<double>(index) + 0.5) * binWidth;
    if (std::abs(modeShift - currentShift) < 0.20) return output;
    output.valid = true;
    output.shift = modeShift;
    return output;
}

}

/** 【函数导航】
 * 作用：执行“generateHeightHypotheses”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<double> generateHeightHypotheses(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal,
    double radialDistance,
    double low,
    double high,
    double binWidth,
    int maxPeaks)
{
    if (std::abs(globalNormal.z) < 1e-9 || !(binWidth > 0.0)) return {0.0};
    const double a = -globalNormal.x / globalNormal.z;
    const double b = -globalNormal.y / globalNormal.z;
    const double c = seed.z - a * seed.x - b * seed.y;
    std::vector<double> residuals;
    residuals.reserve(12000);
    for (const Point3d& p : points) {
        if (std::hypot(p.x - seed.x, p.y - seed.y) >= radialDistance) continue;
        const double residual = p.z - (a * p.x + b * p.y + c);
        if (residual >= low && residual <= high) residuals.push_back(residual);
    }

    std::vector<double> shifts{0.0};
    if (residuals.size() >= 50) {
        std::vector<double> edges;
        for (double edge = low; edge <= high + binWidth * 1.01 + 1e-12; edge += binWidth) {
            edges.push_back(edge);
        }
        std::vector<double> histogram(edges.size() - 1, 0.0);
        for (double value : residuals) {
            auto it = std::upper_bound(edges.begin(), edges.end(), value);
            int index = static_cast<int>(it - edges.begin()) - 1;
            if (value == edges.back()) index = static_cast<int>(histogram.size()) - 1;
            if (index >= 0 && index < static_cast<int>(histogram.size())) {
                histogram[static_cast<std::size_t>(index)] += 1.0;
            }
        }
        const std::vector<double> smooth = gaussianSmoothSigmaOneReflect(histogram);
        const double maxValue = *std::max_element(smooth.begin(), smooth.end());
        const double minimumProminence = std::max(3.0, maxValue * 0.025);
        std::vector<int> peaks;
        for (int i = 1; i + 1 < static_cast<int>(smooth.size()); ++i) {
            if (smooth[static_cast<std::size_t>(i)] > smooth[static_cast<std::size_t>(i - 1)]
                && smooth[static_cast<std::size_t>(i)] >= smooth[static_cast<std::size_t>(i + 1)]
                && peakProminence(smooth, i) >= minimumProminence) {
                peaks.push_back(i);
            }
        }
        std::sort(peaks.begin(), peaks.end(), [&](int left, int right) {
            const double aValue = smooth[static_cast<std::size_t>(left)];
            const double bValue = smooth[static_cast<std::size_t>(right)];
            if (aValue != bValue) return aValue > bValue;
            return left > right;
        });
        if (static_cast<int>(peaks.size()) > maxPeaks) peaks.resize(static_cast<std::size_t>(maxPeaks));
        for (int index : peaks) {
            appendUniqueShift(shifts, 0.5 * (edges[static_cast<std::size_t>(index)]
                + edges[static_cast<std::size_t>(index + 1)]));
        }
    }
    appendUniqueShift(shifts, -1.0);
    appendUniqueShift(shifts, -0.5);
    appendUniqueShift(shifts, 0.5);
    if (shifts.size() > 7) shifts.resize(7);
    return shifts;
}

/** 【函数导航】
 * 作用：检测/搜索“detectShouDongHoleJihe”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 几何检测。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
FullJianCeResult detectShouDongHoleJihe(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormal)
{
    FullJianCeResult output;
    const std::vector<Point3d> trialWindowPoints = collectTrialWindowPoints(points, seed);

    output.heightHypotheses = generateHeightHypotheses(trialWindowPoints, seed, globalNormal);
    std::vector<Trial> trials;
    trials.reserve(16);
    for (double shift : output.heightHypotheses) {
        trials.push_back(buildTrial(trialWindowPoints, seed, globalNormal, shift));
    }

    bool anyContour = false;
    for (const Trial& trial : trials) anyContour = anyContour || trial.hasContour;
    if (!anyContour) {
        std::vector<double> sortedShifts = output.heightHypotheses;
        std::sort(sortedShifts.begin(), sortedShifts.end());
        std::vector<double> midpoints;
        for (std::size_t i = 0; i + 1 < sortedShifts.size(); ++i) {
            midpoints.push_back(0.5 * (sortedShifts[i] + sortedShifts[i + 1]));
        }
        std::sort(midpoints.begin(), midpoints.end(), [](double a, double b) {
            if (std::abs(a) != std::abs(b)) return std::abs(a) < std::abs(b);
            return a < b;
        });
        int inspected = 0;
        for (double shift : midpoints) {
            if (inspected >= 4) break;
            if (std::abs(shift) > 1.25) continue;
            bool duplicate = false;
            for (const Trial& trial : trials) {
                if (std::abs(shift - trial.shift) < 0.12) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            ++inspected;
            Trial trial = buildTrial(trialWindowPoints, seed, globalNormal, shift);
            if (trial.hasContour) {
                const bool strong = trial.contour.score >= 2.05;
                trials.push_back(std::move(trial));
                if (strong) break;
            }
        }
    }

    {
        std::vector<std::size_t> supportedOrder(trials.size());
        for (std::size_t i = 0; i < supportedOrder.size(); ++i) supportedOrder[i] = i;
        std::sort(supportedOrder.begin(), supportedOrder.end(), [&](std::size_t left, std::size_t right) {
            if (trials[left].surfacePointCount != trials[right].surfacePointCount)
                return trials[left].surfacePointCount > trials[right].surfacePointCount;
            return std::abs(trials[left].shift) < std::abs(trials[right].shift);
        });
        const std::size_t baseCount = std::min<std::size_t>(3, supportedOrder.size());
        std::vector<double> neighbourShifts;
        neighbourShifts.reserve(baseCount * 2);
        for (std::size_t k = 0; k < baseCount; ++k) {
            const double baseShift = trials[supportedOrder[k]].shift;
            neighbourShifts.push_back(baseShift - 0.25);
            neighbourShifts.push_back(baseShift + 0.25);
        }
        for (double shift : neighbourShifts) {
            if (std::abs(shift) > 14.0) continue;
            bool duplicate = false;
            for (const Trial& trial : trials) {
                if (std::abs(shift - trial.shift) < 0.12) { duplicate = true; break; }
            }
            if (!duplicate) trials.push_back(buildTrial(trialWindowPoints, seed, globalNormal, shift));
        }
    }

    bool strongUnassociatedPositive = false;
    double maximumShift = -std::numeric_limits<double>::infinity();
    for (const Trial& trial : trials) {
        maximumShift = std::max(maximumShift, trial.shift);
        for (const LunKuoHouXuan& candidate : trial.contourCandidates) {
            if (candidate.ratio < 0.72 && candidate.polarity >= 0.60
                && candidate.coverage >= 0.80 && candidate.rmse <= 0.60
                && candidate.score >= 2.75) {
                strongUnassociatedPositive = true;
            }
        }
    }
    if (!anyContour && strongUnassociatedPositive) {
        for (double shift : {maximumShift + 0.50, maximumShift + 1.00}) {
            if (shift > 3.50) continue;
            bool duplicate = false;
            for (const Trial& trial : trials) {
                if (std::abs(shift - trial.shift) < 0.12) { duplicate = true; break; }
            }
            if (!duplicate) trials.push_back(buildTrial(trialWindowPoints, seed, globalNormal, shift));
        }
    }

    const ChiXuLunKuoJieGuo persistent = choosePersistentContourFamily(trials, seed);
    const ChiXuLunKuoJieGuo nestedOuter = chooseNestedOuterMouthFamily(trials, seed);
    const ChiXuLunKuoJieGuo reversePersistent = persistent.valid
        ? ChiXuLunKuoJieGuo{} : chooseReversePolarityContourFamily(trials, seed);

    const Trial* bestContourTrial = nullptr;
    std::size_t bestContourIndex = 0;
    for (std::size_t i = 0; i < trials.size(); ++i) {
        if (!trials[i].hasContour) continue;
        if (!bestContourTrial || trials[i].contourLevelScore > bestContourTrial->contourLevelScore) {
            bestContourTrial = &trials[i];
            bestContourIndex = i;
        }
    }

    std::size_t chosenIndex = 0;
    HoleKouLevelResult mouth;
    double rawScore = -1e30;
    double levelScore = -1e30;
    bool standardStrongContour = false;
    if (bestContourTrial && bestContourTrial->contour.score >= 2.05) {
        standardStrongContour = true;
        chosenIndex = bestContourIndex;
        mouth = detectHoleKouAtLevel(trials[chosenIndex].mask, seed.x, seed.y);
        rawScore = trials[chosenIndex].contour.score;
        levelScore = trials[chosenIndex].contourLevelScore;
    } else {
        std::vector<std::size_t> order(trials.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
            return cheapGaoduLevelScore(trials[left].surfacePointCount, trials[left].shift)
                > cheapGaoduLevelScore(trials[right].surfacePointCount, trials[right].shift);
        });
        std::vector<MuBanShiSuan> evaluated;
        for (std::size_t k = 0; k < std::min<std::size_t>(2, order.size()); ++k) {
            evaluated.push_back(evaluateTemplate(trials[order[k]], order[k], seed));
        }
        double initialRaw = -1e30;
        for (const MuBanShiSuan& value : evaluated) if (value.valid) initialRaw = std::max(initialRaw, value.rawScore);
        if (initialRaw >= 1.05 && initialRaw < 1.15) {
            for (std::size_t k = 2; k < order.size(); ++k) {
                evaluated.push_back(evaluateTemplate(trials[order[k]], order[k], seed));
            }
        }
        if (initialRaw < 1.05) {
            const double deepShifts[] = {-1.5, -2.0, -2.5, -3.0, -3.5, -4.0};
            std::vector<std::size_t> deepIndices;
            for (double shift : deepShifts) {
                bool duplicate = false;
                for (const Trial& trial : trials) {
                    if (std::abs(shift - trial.shift) < 0.18) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) continue;
                trials.push_back(buildTrial(trialWindowPoints, seed, globalNormal, shift));
                deepIndices.push_back(trials.size() - 1);
            }
            std::sort(deepIndices.begin(), deepIndices.end(), [&](std::size_t left, std::size_t right) {
                return cheapGaoduLevelScore(trials[left].surfacePointCount, trials[left].shift)
                    > cheapGaoduLevelScore(trials[right].surfacePointCount, trials[right].shift);
            });
            std::vector<MuBanShiSuan> deepEvaluated;
            for (std::size_t k = 0; k < std::min<std::size_t>(2, deepIndices.size()); ++k) {
                deepEvaluated.push_back(evaluateTemplate(trials[deepIndices[k]], deepIndices[k], seed));
            }
            evaluated.insert(evaluated.end(), deepEvaluated.begin(), deepEvaluated.end());
            double bestDeepRaw = -1e30;
            for (const MuBanShiSuan& value : deepEvaluated) if (value.valid) bestDeepRaw = std::max(bestDeepRaw, value.rawScore);
            if (!deepEvaluated.empty() && bestDeepRaw < 1.15) {
                for (std::size_t k = 2; k < deepIndices.size(); ++k) {
                    evaluated.push_back(evaluateTemplate(trials[deepIndices[k]], deepIndices[k], seed));
                }
            }
        }
        MuBanShiSuan best;
        for (const MuBanShiSuan& value : evaluated) if (betterTemplate(value, best)) best = value;
        if (initialRaw < 1.05) {
            MuBanShiSuan deepRawBest;
            for (const MuBanShiSuan& value : evaluated) {
                if (!value.valid || trials[value.trialIndex].shift > -3.0) continue;
                if (value.circle.radius < 4.0 || value.rawScore < 1.10) continue;
                if (!deepRawBest.valid || value.rawScore > deepRawBest.rawScore) deepRawBest = value;
            }
            if (deepRawBest.valid && (!best.valid || deepRawBest.rawScore >= best.rawScore + 0.12)) {
                best = deepRawBest;
            }
        }
        if (!best.valid) return output;
        chosenIndex = best.trialIndex;
        mouth.valid = true;
        mouth.centerX = best.circle.centerX;
        mouth.centerY = best.circle.centerY;
        mouth.radius = best.circle.radius;
        mouth.source = "template_fast";
        mouth.templateResult = best.circle;
        rawScore = best.rawScore;
        levelScore = best.levelScore;
    }

    if (!standardStrongContour && !persistent.valid && rawScore < 1.05) {
        MuBanShiSuan rawDominant;
        MuBanShiSuan supportedBest;
        for (std::size_t trialIndex = 0; trialIndex < trials.size(); ++trialIndex) {
            const MuBanShiSuan candidateTemplate = evaluateTemplate(trials[trialIndex], trialIndex, seed);
            if (!candidateTemplate.valid || candidateTemplate.rawScore < rawScore + 0.10) continue;
            if (candidateTemplate.rawScore >= 1.15
                && (!rawDominant.valid || candidateTemplate.rawScore > rawDominant.rawScore)) {
                rawDominant = candidateTemplate;
            }
            bool weakContourSupport = false;
            for (const LunKuoHouXuan& contourCandidate : trials[trialIndex].contourCandidates) {
                const double corrected = contourCandidate.radius + 2.28 * trials[trialIndex].mask.resolution;
                if (contourCandidate.coverage < 0.70 || contourCandidate.rmse > 1.30) continue;
                if (corrected < 3.0 || corrected > 8.8) continue;
                if (std::hypot(contourCandidate.centerX - candidateTemplate.circle.centerX,
                               contourCandidate.centerY - candidateTemplate.circle.centerY) > 1.60) continue;
                if (std::abs(corrected - candidateTemplate.circle.radius) > 1.80) continue;
                weakContourSupport = true;
                break;
            }
            if (weakContourSupport
                && (!supportedBest.valid || candidateTemplate.rawScore > supportedBest.rawScore
                    || (std::abs(candidateTemplate.rawScore - supportedBest.rawScore) < 1e-9
                        && candidateTemplate.levelScore > supportedBest.levelScore))) {
                supportedBest = candidateTemplate;
            }
        }
        const MuBanShiSuan& replacement = rawDominant.valid ? rawDominant : supportedBest;
        if (replacement.valid) {
            chosenIndex = replacement.trialIndex;
            mouth.valid = true;
            mouth.centerX = replacement.circle.centerX;
            mouth.centerY = replacement.circle.centerY;
            mouth.radius = replacement.circle.radius;
            mouth.source = rawDominant.valid ? "template_raw_rescue" : "template_contour_anchor";
            mouth.templateResult = replacement.circle;
            rawScore = replacement.rawScore;
            levelScore = replacement.levelScore;
        }
    }

    if (!persistent.valid && reversePersistent.valid && !standardStrongContour && rawScore < 1.10) {
        chosenIndex = reversePersistent.trialIndex;
        mouth.valid = true;
        mouth.centerX = reversePersistent.contour.centerX;
        mouth.centerY = reversePersistent.contour.centerY;
        mouth.radius = reversePersistent.contour.correctedRadius;
        mouth.source = "contour_reverse_persistent";
        mouth.contour = reversePersistent.contour;
        rawScore = reversePersistent.rawScore;
        levelScore = reversePersistent.levelScore;
    }

    if (!persistent.valid && reversePersistent.valid && standardStrongContour && mouth.valid) {
        const double reverseCenterGap = std::hypot(
            mouth.centerX - reversePersistent.contour.centerX,
            mouth.centerY - reversePersistent.contour.centerY);
        if (mouth.radius >= 8.0
            && reversePersistent.contour.correctedRadius >= 3.0
            && reversePersistent.contour.correctedRadius <= 8.0
            && reversePersistent.distinctLevels >= 3
            && reverseCenterGap <= 1.0
            && reversePersistent.rawScore >= rawScore - 0.75) {
            chosenIndex = reversePersistent.trialIndex;
            mouth.valid = true;
            mouth.centerX = reversePersistent.contour.centerX;
            mouth.centerY = reversePersistent.contour.centerY;
            mouth.radius = reversePersistent.contour.correctedRadius;
            mouth.source = "contour_reverse_persistent";
            mouth.contour = reversePersistent.contour;
            rawScore = reversePersistent.rawScore;
            levelScore = reversePersistent.levelScore;
        }
    }

    // ActivePath 内部证据：在 persistent-family 接管前捕获精确活动路径证据。
    // 这些变量只读，不参与几何选择。
    const double primaryMouthRadius = mouth.valid ? mouth.radius : 0.0;
    const double primaryMouthRmse = mouth.valid ? mouth.contour.rmse : 0.0;
    const double primaryMouthPolarity = mouth.valid ? mouth.contour.polarity : 0.0;
    const double primaryMouthCoverage = mouth.valid ? mouth.contour.coverage : 0.0;
    const double persistentFamilyRadius = persistent.valid
        ? persistent.contour.correctedRadius : 0.0;
    const double persistentFamilyCenterX = persistent.valid ? persistent.contour.centerX : 0.0;
    const double persistentFamilyCenterY = persistent.valid ? persistent.contour.centerY : 0.0;
    const double persistentFamilyCenterZ = persistent.valid ? trials[persistent.trialIndex].shift : 0.0;
    const double persistentFamilyCenterGap = (persistent.valid && mouth.valid)
        ? std::hypot(mouth.centerX - persistent.contour.centerX,
                     mouth.centerY - persistent.contour.centerY)
        : 0.0;
    int outerHuiFuGateMask = 0;
    if (persistent.valid) outerHuiFuGateMask |= (1 << 0);
    if (standardStrongContour) outerHuiFuGateMask |= (1 << 1);
    if (persistentFamilyRadius >= 3.00 && persistentFamilyRadius <= 6.50)
        outerHuiFuGateMask |= (1 << 2);
    if (persistent.valid && persistent.distinctLevels >= 3)
        outerHuiFuGateMask |= (1 << 3);
    if (persistent.valid && mouth.valid
        && mouth.radius >= persistentFamilyRadius + 1.00
        && mouth.radius <= persistentFamilyRadius + 1.80)
        outerHuiFuGateMask |= (1 << 4);
    if (persistent.valid && mouth.valid && persistentFamilyCenterGap <= 1.50)
        outerHuiFuGateMask |= (1 << 5);
    if (mouth.valid && mouth.contour.rmse >= 0.90)
        outerHuiFuGateMask |= (1 << 6);
    if (mouth.valid && mouth.contour.polarity <= 0.62)
        outerHuiFuGateMask |= (1 << 7);
    if (persistent.valid && persistent.rawScore >= rawScore + 0.75)
        outerHuiFuGateMask |= (1 << 8);
    bool boreEvidenceEvaluated = false;
    bool boreEvidenceValid = false;

    if (persistent.valid) {
        bool usePersistent = !mouth.valid;
        const double persistentRadius = persistent.contour.correctedRadius;
        const double centerGap = mouth.valid
            ? std::hypot(mouth.centerX - persistent.contour.centerX,
                         mouth.centerY - persistent.contour.centerY)
            : std::numeric_limits<double>::infinity();
        if (persistent.splitLobeRecovered && persistentRadius <= 5.20) {
            usePersistent = true;
        } else if (!standardStrongContour
                   && persistentRadius <= 5.20
                   && persistent.distinctLevels >= 2
                   && (persistent.rawScore > rawScore + 0.80
                       || mouth.radius > persistentRadius + 1.15
                       || (centerGap > 1.50 && persistent.rawScore > rawScore + 0.25))) {

            usePersistent = true;
        } else if (!standardStrongContour
                   && persistentRadius >= 4.25
                   && persistentRadius <= 7.60
                   && persistent.distinctLevels >= 2
                   && persistent.rawScore >= 3.15
                   && (rawScore < 1.15
                       || persistent.rawScore > rawScore + 0.75
                       || (centerGap > 1.50 && persistent.rawScore > rawScore + 0.25))) {

            usePersistent = true;
        } else if (standardStrongContour
                   && persistentRadius >= 2.00
                   && persistentRadius <= 4.50
                   && persistent.distinctLevels >= 3
                   && persistent.rawScore >= rawScore + 0.55
                   && centerGap >= 2.00) {

            usePersistent = true;
        } else if (standardStrongContour
                   && persistentRadius >= 3.00
                   && persistentRadius <= 6.50
                   && persistent.distinctLevels >= 3
                   && mouth.radius >= persistentRadius + 1.00
                   && mouth.radius <= persistentRadius + 1.80
                   && centerGap <= 1.50
                   && mouth.contour.rmse >= 0.90
                   && mouth.contour.polarity <= 0.62
                   && persistent.rawScore >= rawScore + 0.75) {

            const JingXiuHoleKouJihe currentRefined = refineMouthGeometry(
                points, seed, trials[chosenIndex].plane, globalNormal,
                mouth.centerX, mouth.centerY, mouth.radius, mouth.source);
            const HoleJingXiu currentBore = refineChamferedStraightBore(
                points, trials[chosenIndex].plane,
                currentRefined.finalCircle.centerX,
                currentRefined.finalCircle.centerY,
                currentRefined.finalCircle.radius);
            if (!currentBore.valid) usePersistent = true;
        } else if (standardStrongContour
                   && centerGap >= 4.0
                   && persistentRadius >= 2.45
                   && persistentRadius <= 6.50
                   && persistent.distinctLevels >= 2
                   && persistent.rawScore >= rawScore - 0.35
                   && (mouth.radius >= 7.50
                       || persistent.rawScore >= rawScore + 0.35
                       || mouth.radius >= persistentRadius + 0.90)) {

            usePersistent = true;
        } else if (standardStrongContour
                   && persistent.upperMouthMode
                   && persistentRadius <= 7.80
                   && persistentRadius > mouth.radius + 0.45
                   && persistent.distinctLevels >= 4
                   && centerGap <= 1.40
                   && persistent.rawScore >= rawScore - 0.60) {

            usePersistent = true;
        } else if (standardStrongContour
                   && persistentRadius <= 5.20
                   && mouth.radius > persistentRadius + 1.50
                   && centerGap > 2.00
                   && persistent.rawScore >= rawScore - 0.20) {
            usePersistent = true;
        } else if (standardStrongContour
                   && mouth.radius >= 7.50
                   && persistentRadius >= 2.00
                   && persistentRadius <= 6.50
                   && persistent.distinctLevels >= 2
                   && mouth.radius >= 1.30 * persistentRadius
                   && centerGap <= 2.80
                   && (persistent.largeRingTuoDi || persistent.rawScore >= rawScore - 1.25)) {

            usePersistent = true;
        }
        if (usePersistent) {
            chosenIndex = persistent.trialIndex;
            mouth.valid = true;
            mouth.centerX = persistent.contour.centerX;
            mouth.centerY = persistent.contour.centerY;
            mouth.radius = persistentRadius;
            mouth.source = "contour_persistent";
            mouth.contour = persistent.contour;
            rawScore = persistent.rawScore;
            levelScore = persistent.levelScore;
        }
    }

    if (mouth.valid && nestedOuter.valid) {
        const double centerGap = std::hypot(
            mouth.centerX - nestedOuter.contour.centerX,
            mouth.centerY - nestedOuter.contour.centerY);
        const double outerRadius = nestedOuter.contour.correctedRadius;
        const double radiusGap = outerRadius - mouth.radius;
        const bool standardNestedGap = radiusGap <= 1.30;
        const bool strongWideNestedGap = radiusGap <= 1.65
            && nestedOuter.distinctLevels >= 4
            && centerGap <= 0.25
            && nestedOuter.rawScore >= rawScore + 0.80;
        if (centerGap <= 0.85
            && nestedOuter.distinctLevels >= 2
            && mouth.radius >= 4.40
            && radiusGap >= 0.35
            && (standardNestedGap || strongWideNestedGap)
            && outerRadius <= 8.0) {
            mouth.radius = outerRadius;
            mouth.source += "_outer_radius";
        }
    }
    if (!mouth.valid) return output;

    const JingXiuHoleKouJihe refined = refineMouthGeometry(
        points, seed, trials[chosenIndex].plane, globalNormal,
        mouth.centerX, mouth.centerY, mouth.radius, mouth.source);
    const HoleJingXiu bore = refineChamferedStraightBore(
        points, trials[chosenIndex].plane,
        refined.finalCircle.centerX, refined.finalCircle.centerY, refined.finalCircle.radius);

    bool useBore = bore.valid;
    if (useBore && persistent.valid && !persistent.upperMouthMode
        && persistent.distinctLevels >= 3
        && persistent.contour.correctedRadius >= 3.60
        && persistent.contour.correctedRadius <= 5.20
        && std::abs(persistent.contour.correctedRadius - refined.finalCircle.radius) <= 0.55
        && std::hypot(persistent.contour.centerX - refined.finalCircle.centerX,
                      persistent.contour.centerY - refined.finalCircle.centerY) <= 0.90) {
        useBore = false;
    }

    const double finalCenterX = useBore ? bore.centerX : refined.finalCircle.centerX;
    const double finalCenterY = useBore ? bore.centerY : refined.finalCircle.centerY;
    output.valid = true;
    output.centerX = finalCenterX;
    output.centerY = finalCenterY;
    output.radius = useBore ? bore.radius : refined.finalCircle.radius;
    output.normal = refined.normalFit.normal;
    output.tiltDegrees = refined.normalFit.tiltDegrees;
    const double originalShift = trials[chosenIndex].shift;
    double finalShift = originalShift;
    bool surfaceShiftUsed = false;
    const bool outerSurfaceEligible = refined.depthCenterUsed
        && (mouth.source.find("outer_radius") != std::string::npos
            || mouth.source.find("reverse_persistent") != std::string::npos);
    if (outerSurfaceEligible) {
        const SurfaceShiftJingXiu surfaceShift = refineSurfaceShiftFromOuterAnnulus(
            points, seed, globalNormal, finalCenterX, finalCenterY,
            useBore ? bore.radius : refined.finalCircle.radius, finalShift);
        if (surfaceShift.valid
            && surfaceShift.shift >= finalShift + 0.55
            && surfaceShift.shift <= originalShift + 1.85) {
            finalShift = surfaceShift.shift;
            surfaceShiftUsed = true;
        }

        if (!surfaceShiftUsed) {
            int maximumSurfacePoints = 0;
            for (const Trial& trial : trials) maximumSurfacePoints = std::max(maximumSurfacePoints, trial.surfacePointCount);
            if (maximumSurfacePoints >= 1000) {
                double upperSupportedShift = originalShift;
                for (const Trial& trial : trials) {
                    if (trial.surfacePointCount >= static_cast<int>(0.65 * maximumSurfacePoints)
                        && trial.shift <= originalShift + 1.10) {
                        upperSupportedShift = std::max(upperSupportedShift, trial.shift);
                    }
                }
                const double upperEdgeShift = upperSupportedShift + 0.15;
                if (upperEdgeShift >= originalShift + 0.20
                    && upperEdgeShift <= originalShift + 1.05) {
                    finalShift = upperEdgeShift;
                    surfaceShiftUsed = true;
                }
            }
        }
    }

    if (!surfaceShiftUsed && refined.depthCenterUsed
        && mouth.source.find("template_fast") != std::string::npos) {
        const SurfaceShiftJingXiu surfaceShift = refineSurfaceShiftFromOuterAnnulus(
            points, seed, globalNormal, finalCenterX, finalCenterY,
            useBore ? bore.radius : refined.finalCircle.radius, finalShift);
        if (surfaceShift.valid
            && surfaceShift.shift >= finalShift + 0.20
            && surfaceShift.shift <= originalShift + 0.80) {
            finalShift = surfaceShift.shift;
            surfaceShiftUsed = true;
        }
    }

    if (useBore && bore.radius <= 2.25 && !trials.empty()) {
        int maximumSurfacePoints = 0;
        for (const Trial& trial : trials) maximumSurfacePoints = std::max(maximumSurfacePoints, trial.surfacePointCount);
        if (maximumSurfacePoints >= 1000) {
            double upperSupportedShift = originalShift;
            for (const Trial& trial : trials) {
                if (trial.surfacePointCount >= static_cast<int>(0.65 * maximumSurfacePoints)
                    && trial.shift <= originalShift + 1.25) {
                    upperSupportedShift = std::max(upperSupportedShift, trial.shift);
                }
            }
            const double upperEdgeShift = upperSupportedShift + 0.15;
            if (upperEdgeShift >= originalShift + 0.20
                && upperEdgeShift <= originalShift + 1.05) {
                finalShift = upperEdgeShift;
                surfaceShiftUsed = true;
            }
        }
    }
    output.selectedShift = finalShift;
    output.rawScore = rawScore;
    output.levelScore = levelScore;
    output.surfacePointCount = trials[chosenIndex].surfacePointCount;
    output.source = useBore ? (mouth.source + "_deep_bore") : mouth.source;
    if (useBore && bore.stableOrderRetry) {
        output.source += bore.stableOrderMode == 1
            ? "_stable_xy_retry" : "_stable_yx_retry";
    }
    if (surfaceShiftUsed) output.source += "_surface_shift";
    output.depthCenterUsed = refined.depthCenterUsed;
    output.persistentFamilyValid = persistent.valid;
    output.persistentFamilyRadius = persistentFamilyRadius;
    output.persistentFamilyCenterX = persistentFamilyCenterX;
    output.persistentFamilyCenterY = persistentFamilyCenterY;
    output.persistentFamilyCenterZ = persistentFamilyCenterZ;
    output.persistentFamilyLevels = persistent.valid ? persistent.distinctLevels : 0;
    output.persistentFamilyRawScore = persistent.valid ? persistent.rawScore : -1e30;
    output.primaryMouthRadius = primaryMouthRadius;
    output.primaryMouthRmse = primaryMouthRmse;
    output.primaryMouthPolarity = primaryMouthPolarity;
    output.primaryMouthCoverage = primaryMouthCoverage;
    output.persistentFamilyCenterGap = persistentFamilyCenterGap;
    output.outerHuiFuGateMask = outerHuiFuGateMask;
    output.boreEvidenceEvaluated = boreEvidenceEvaluated;
    output.boreEvidenceValid = boreEvidenceValid;
    for (const Trial& trial : trials) {
        output.trials.push_back({trial.shift, trial.surfacePointCount, trial.hasContour,
            trial.hasContour ? trial.contour.score : -1e30, trial.contourLevelScore});
    }
    return output;
}

}
