/*
================================================================================
文件：ShouDongHole_WeiziZhicheng.cpp
模块：手动 Hole 位姿支撑

【主要职责】
实现局部支撑面、坐标系、RANSAC/最小二乘平面和 PCL 位姿辅助。

【主要调用关系】
由 HoleShibie 与 ShouDongHole 几何链调用。

【线程与状态】
纯计算。

【维护边界】
1. 当前 GUI 正式识别从本文件主要使用“整云缓存 + 全局粗法线”能力；全局粗法线每份点云只建立一次并缓存。
2. estimateLocalSupportPlane()/detectShouDongHoleWeizi()/detectShouDongHoleJihePcl() 属于兼容的旧位姿入口，
   当前 GUI 的 direct-cluster-first 生产路径不经过它们；不要把这里的 RANSAC 误认为每个 seed 都会额外执行。
3. 当前生产 mouth-search 的每 seed 初始法线算法已经集中到 HoleFaXian_Normal.h，并由 HoleShibie_Recognition.cpp 编排。
4. 本轮只整理结构、命名与注释，不改变任何数值参数或调用顺序。
================================================================================
*/
/*
模块职责：
集中实现手动选孔的三维坐标准备、PCL 适配、整云参考法向、局部位姿和孔口外环支撑面估计。
这些步骤共同负责“把原始三维点转换为稳定的孔局部坐标系”，因此统一放在一个实现文件中。

主要调用位置：
HoleShibie_Recognition.cpp 在用户给出种子点后进入本模块；本模块随后调用 ShouDongHole_JiheJianCe.cpp 完成孔口检测。

维护说明：
全局法向采样上限、近邻数、局部保留半径、外环范围、扇区数量和平面残差阈值会影响法向与孔轴稳定性。
参数旁的中文注释说明单位及调大/调小影响；显示坐标变换和 GUI 状态不得混入这里。
*/
#include "ShouDongHole_Manual.h"
#include "HoleFenxi_Analysis.h"

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>
#include <array>
#include <cstdint>
#include <numeric>
#include <random>
#include <cstddef>
#include <utility>

// ============================================================================
// 功能分区：PCL 适配、参考法向与孔位姿
// ============================================================================
namespace shouDongHole {
namespace {

constexpr double kPiPcl = 3.141592653589793238462643383279502884;

/** 【类型导航注释】
 * HuanCunShangXiaWen：手动 Hole 位姿支撑中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HuanCunShangXiaWen {
    DianYunBiaoshi::ZhiWen fingerprint;
    std::vector<Point3d> points;
    Vec3d globalNormal;
    bool valid = false;
};

std::mutex gContextMutex;
std::shared_ptr<const HuanCunShangXiaWen> gContext;

/** 【函数导航】
 * 作用：执行“finitePoint”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finitePoint(const pcl::PointXYZRGB& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/** 【函数导航】
 * 作用：执行“dotPcl”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double dotPcl(const Vec3d& a, const Vec3d& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** 【函数导航】
 * 作用：执行“normalized”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp、ZhuChuangKou_Window.cpp、DianYunXianshi_View.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d normalized(Vec3d value) {
    const double length = std::sqrt(dotPcl(value, value));
    if (!(length > 1e-12) || !std::isfinite(length)) return {0.0, 0.0, 1.0};
    value.x /= length;
    value.y /= length;
    value.z /= length;
    if (value.z < 0.0) {
        value.x = -value.x;
        value.y = -value.y;
        value.z = -value.z;
    }
    return value;
}

/**
 * 【整云全局粗法线：当前 GUI 正式路径会使用】
 * 作用：给整份点云建立一个稳定但不追求亚角度精度的参考方向。
 * 方法：最多均匀抽取 800 个点；每个点用 25-NN PCA 得到局部法线，过滤明显非平面的邻域后，
 *       在 8° 主方向内做加权投票/平均。结果保存在 HuanCunShangXiaWen 中，同一份点云只计算一次。
 * 调用：analysisCacheAcquireThreadDianYunFenxi()/预热阶段建立；正式每个 seed 直接复用缓存。
 * 与椭圆法线的关系：这里只负责“搜索朝向基准”，绝不拥有最终 Hole 法线解释权。
 */
Vec3d estimateGlobalNormalInternal(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZRGB>* existingTree = nullptr)
{
    if (!cloud || cloud->empty()) return {0.0, 0.0, 1.0};

    pcl::KdTreeFLANN<pcl::PointXYZRGB> ownedTree;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>* tree = existingTree;
    if (!tree) {
        ownedTree.setInputCloud(cloud);
        tree = &ownedTree;
    }

    constexpr std::size_t kSampleCount = 800;
    constexpr int kNeighbours = 25;
    const std::size_t sampleCount = std::min<std::size_t>(kSampleCount, cloud->size());

    /** 【类型导航注释】
     * FaXianYangBen：手动 Hole 位姿支撑中的自定义 结构体。
     * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct FaXianYangBen {
        Vec3d normal;
        double weight = 1.0;
    };
    std::vector<FaXianYangBen> samples;
    samples.reserve(sampleCount);

    std::vector<int> indices;
    std::vector<float> squaredDistances;
    indices.reserve(kNeighbours);
    squaredDistances.reserve(kNeighbours);

    for (std::size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
        const double position = sampleCount <= 1
            ? 0.0
            : static_cast<double>(sampleIndex) * static_cast<double>(cloud->size() - 1)
                / static_cast<double>(sampleCount - 1);
        const std::size_t pointIndex = static_cast<std::size_t>(std::llround(position));
        if (pointIndex >= cloud->size() || !finitePoint((*cloud)[pointIndex])) continue;

        indices.clear();
        squaredDistances.clear();
        const int found = tree->nearestKSearch(
            (*cloud)[pointIndex], std::min<int>(kNeighbours, static_cast<int>(cloud->size())),
            indices, squaredDistances);
        if (found < 8) continue;

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        int finiteCount = 0;
        for (int index : indices) {
            if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) continue;
            const auto& p = (*cloud)[static_cast<std::size_t>(index)];
            if (!finitePoint(p)) continue;
            centroid += Eigen::Vector3d(p.x, p.y, p.z);
            ++finiteCount;
        }
        if (finiteCount < 8) continue;
        centroid /= static_cast<double>(finiteCount);

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (int index : indices) {
            if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) continue;
            const auto& p = (*cloud)[static_cast<std::size_t>(index)];
            if (!finitePoint(p)) continue;
            const Eigen::Vector3d delta = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
            covariance.noalias() += delta * delta.transpose();
        }
        covariance /= static_cast<double>(finiteCount);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) continue;
        const Eigen::Vector3d eigenvalues = solver.eigenvalues();
        if (!(eigenvalues[1] > 1e-12)) continue;
        const double planarityRatio = eigenvalues[0] / eigenvalues[1];
        if (!std::isfinite(planarityRatio) || planarityRatio >= 0.18) continue;

        Eigen::Vector3d eigenNormal = solver.eigenvectors().col(0).normalized();
        if (!eigenNormal.allFinite()) continue;
        if (eigenNormal.z() < 0.0) eigenNormal = -eigenNormal;
        samples.push_back({{eigenNormal.x(), eigenNormal.y(), eigenNormal.z()},
            1.0 / (planarityRatio + 0.03)});
    }

    if (samples.empty()) return {0.0, 0.0, 1.0};

    const double cosineEightDegrees = std::cos(8.0 * kPiPcl / 180.0);
    const std::size_t candidateStep = std::max<std::size_t>(1, samples.size() / 200);
    Vec3d bestNormal = samples.front().normal;
    std::size_t bestSupport = 0;
    for (std::size_t i = 0; i < samples.size(); i += candidateStep) {
        std::size_t support = 0;
        for (const FaXianYangBen& sample : samples) {
            if (std::abs(dotPcl(sample.normal, samples[i].normal)) > cosineEightDegrees) ++support;
        }
        if (support > bestSupport) {
            bestSupport = support;
            bestNormal = samples[i].normal;
        }
    }

    Vec3d weightedSum{0.0, 0.0, 0.0};
    double weightSum = 0.0;
    for (const FaXianYangBen& sample : samples) {
        double alignment = dotPcl(sample.normal, bestNormal);
        if (std::abs(alignment) <= cosineEightDegrees) continue;
        Vec3d oriented = sample.normal;
        if (alignment < 0.0) {
            oriented.x = -oriented.x;
            oriented.y = -oriented.y;
            oriented.z = -oriented.z;
        }
        weightedSum.x += oriented.x * sample.weight;
        weightedSum.y += oriented.y * sample.weight;
        weightedSum.z += oriented.z * sample.weight;
        weightSum += sample.weight;
    }
    if (!(weightSum > 0.0)) return {0.0, 0.0, 1.0};
    weightedSum.x /= weightSum;
    weightedSum.y /= weightSum;
    weightedSum.z /= weightSum;
    return normalized(weightedSum);
}

/** 【函数导航】
 * 作用：执行“rebuildContext”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void rebuildContext(
    HuanCunShangXiaWen& context,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const DianYunBiaoshi::ZhiWen& fingerprint)
{
    context = HuanCunShangXiaWen{};
    if (!cloud || cloud->empty()) return;
    context.fingerprint = fingerprint;
    context.points.reserve(cloud->size());
    for (const auto& p : *cloud) {
        if (!finitePoint(p)) continue;
        context.points.push_back({p.x, p.y, p.z});
    }
    context.globalNormal = estimateGlobalNormalInternal(cloud);
    context.valid = !context.points.empty();}

/** 【函数导航】
 * 作用：执行“acquireContext”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::shared_ptr<const HuanCunShangXiaWen> acquireContext(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    if (!cloud || cloud->empty()) return {};

    const auto fingerprint = DianYunBiaoshi::fingerprintDianYun(cloud);
    std::lock_guard<std::mutex> lock(gContextMutex);
    if (gContext && gContext->valid
        && DianYunBiaoshi::sameFingerprint(gContext->fingerprint, fingerprint)) return gContext;

    auto next = std::make_shared<HuanCunShangXiaWen>();
    rebuildContext(*next, cloud, fingerprint);
    if (!next->valid) return {};
    gContext = next;
    return gContext;
}

}

/** 【函数导航】
 * 作用：估计“estimateGlobalNormalDeterministicPcl”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d estimateGlobalNormalDeterministicPcl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    const std::shared_ptr<const HuanCunShangXiaWen> context = acquireContext(cloud);
    return context && context->valid ? context->globalNormal : Vec3d{0.0, 0.0, 1.0};
}

/** 【函数导航】
 * 作用：估计“estimateGlobalNormalDeterministicPclWithTreeUncached”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d estimateGlobalNormalDeterministicPclWithTreeUncached(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZRGB>& existingTree)
{
    return estimateGlobalNormalInternal(cloud, &existingTree);
}

/** 【函数导航】
 * 作用：估计“estimateGlobalNormalDeterministicPclWithTree”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_Manual.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d estimateGlobalNormalDeterministicPclWithTree(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZRGB>& existingTree)
{
    if (!cloud || cloud->empty()) return {0.0, 0.0, 1.0};

    /** 【类型导航注释】
     * KuaiSuFaXianHuanCun：手动 Hole 位姿支撑中的自定义 结构体。
     * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct KuaiSuFaXianHuanCun {
        DianYunBiaoshi::ZhiWen fingerprint;
        Vec3d normal{0.0, 0.0, 1.0};
        bool valid = false;
    };
    thread_local KuaiSuFaXianHuanCun cache;
    const auto fingerprint = DianYunBiaoshi::fingerprintDianYun(cloud);
    if (cache.valid && DianYunBiaoshi::sameFingerprint(cache.fingerprint, fingerprint))
        return cache.normal;

    cache.fingerprint = fingerprint;
    cache.normal = estimateGlobalNormalInternal(cloud, &existingTree);
    cache.valid = true;
    return cache.normal;
}

/**
 * 【兼容入口，不是当前 GUI 正式识别主链】
 * detectShouDongHoleJihePcl() 保留给旧接口/兼容调用：它会先取得整云缓存，再进入 detectShouDongHoleWeizi()。
 * 当前 GUI 的 shouDongJuBuHoleShibieProductionImpl() 使用 direct-cluster-first 路由，直接在
 * HoleShibie_Recognition.cpp 中建立 mouth-search frame，因此正常 GUI 识别不会额外执行这里的局部 RANSAC。
 * 保留原因：仍有公开声明与旧调用契约；删除前必须先确认外部调用方，不应为了“看起来重复”直接移除。
 */
PclJianCeResult detectShouDongHoleJihePcl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& seedPoint)
{
    PclJianCeResult result;
    if (!cloud || cloud->empty()) {
        result.error = "empty point cloud";
        return result;
    }
    if (!seedPoint.allFinite()) {
        result.error = "non-finite seed";
        return result;
    }

    const std::shared_ptr<const HuanCunShangXiaWen> context = acquireContext(cloud);
    if (!context || !context->valid) {
        result.error = "failed to build finite point cache";
        return result;
    }

    result.globalNormal = context->globalNormal;
    result.finitePointCount = context->points.size();

    const WeiziJianCeResult pose = detectShouDongHoleWeizi(
        context->points,
        {seedPoint.x(), seedPoint.y(), seedPoint.z()},
        context->globalNormal);
    result.geometry = pose.geometry;
    result.centerZ = pose.centerZ;
    result.initialCenterZ = pose.initialCenterZ;
    result.usedLocalPose = pose.usedLocalPose;
    result.usedMouthSupportPlane = pose.usedMouthSupportPlane;
    result.localPlane = pose.localPlane;
    result.mouthSupportPlane = pose.mouthSupportPlane;    if (!result.geometry.valid) result.error = "mouth geometry not found";
    return result;
}

}


// ============================================================================
// 功能分区：局部平面与孔位姿拟合
// ============================================================================
/*
模块职责：
根据用户种子附近的局部点集拟合参考平面、建立局部坐标系并估计手动孔的姿态。

主要调用位置：
HoleShibie_Recognition.cpp 的手动选孔路径。

维护说明：
平面内点阈值、RANSAC/迭代次数和局部保留半径都会影响法向与孔轴；这些数值属于识别参数，不应因格式整理而修改。
*/
// 与上方 PCL 适配实现共用 ShouDongHole_Manual.h。


namespace shouDongHole {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

double dot(const Vec3d& a, const Vec3d& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3d cross(const Vec3d& a, const Vec3d& b) noexcept {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
/** 【函数导航】
 * 作用：执行“normalize”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp、ZhuChuangKou_Window.cpp、HoleLeixing_Types.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d normalize(Vec3d v) noexcept {
    const double n = std::sqrt(dot(v, v));
    if (!(n > 1e-12) || !std::isfinite(n)) return {0.0, 0.0, 1.0};
    v.x /= n; v.y /= n; v.z /= n;
    return v;
}
/** 【函数导航】
 * 作用：执行“angleDegrees”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double angleDegrees(const Vec3d& a, const Vec3d& b) noexcept {
    return std::acos(std::clamp(std::abs(dot(normalize(a), normalize(b))), 0.0, 1.0)) * 180.0 / kPi;
}

/** 【函数导航】
 * 作用：拟合/求解“solve3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool solve3(double a[3][4], double out[3]) noexcept {
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
 * 作用：执行“planeFromThree”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool planeFromThree(const Point3d& p0, const Point3d& p1, const Point3d& p2, PingMianModel& plane) noexcept {
    double aug[3][4] = {
        {p0.x, p0.y, 1.0, p0.z},
        {p1.x, p1.y, 1.0, p1.z},
        {p2.x, p2.y, 1.0, p2.z}};
    double s[3];
    if (!solve3(aug, s)) return false;
    plane = {s[0], s[1], s[2]};
    return std::isfinite(plane.a) && std::isfinite(plane.b) && std::isfinite(plane.c);
}

/** 【函数导航】
 * 作用：拟合/求解“fitPlaneLeastSquares”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool fitPlaneLeastSquares(const std::vector<Point3d>& q, const std::vector<std::size_t>& ids, PingMianModel& plane) noexcept {
    if (ids.size() < 3) return false;
    double n[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    double rhs[3] = {0.0,0.0,0.0};
    for (std::size_t id : ids) {
        const auto& p = q[id];
        const double r[3] = {p.x,p.y,1.0};
        for (int i=0;i<3;++i) {
            rhs[i] += r[i]*p.z;
            for (int j=0;j<3;++j) n[i][j] += r[i]*r[j];
        }
    }
    double aug[3][4];
    for (int i=0;i<3;++i) {
        for (int j=0;j<3;++j) aug[i][j]=n[i][j];
        aug[i][3]=rhs[i];
    }
    double s[3]; if(!solve3(aug,s)) return false;
    plane={s[0],s[1],s[2]}; return true;
}

/** 【函数导航】
 * 作用：执行“normalFromPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_JiheJianCe.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d normalFromPlane(const PingMianModel& p, const Vec3d& reference) noexcept {
    Vec3d n = normalize({-p.a,-p.b,1.0});
    if (dot(n, reference) < 0.0) {n.x=-n.x;n.y=-n.y;n.z=-n.z;}
    return n;
}

/** 【函数导航】
 * 作用：执行“signedPlaneResidual”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double signedPlaneResidual(const Point3d& p, const PingMianModel& plane) noexcept {
    return (p.z - (plane.a*p.x + plane.b*p.y + plane.c)) /
           std::sqrt(1.0 + plane.a*plane.a + plane.b*plane.b);
}

double median(std::vector<double> x) {
    if (x.empty()) return std::numeric_limits<double>::infinity();
    const std::size_t m=x.size()/2;
    std::nth_element(x.begin(),x.begin()+static_cast<std::ptrdiff_t>(m),x.end());
    const double hi=x[m];
    if (x.size()%2) return hi;
    std::nth_element(x.begin(),x.begin()+static_cast<std::ptrdiff_t>(m-1),x.end());
    return 0.5*(hi+x[m-1]);
}

/** 【函数导航】
 * 作用：执行“seedValue”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::uint32_t seedValue(const Point3d& seed) noexcept {
    const double v=std::abs(seed.x*1009.0+seed.y*9176.0+seed.z*31337.0)*1000.0;
    const auto u=static_cast<std::uint64_t>(std::llround(v));
    return static_cast<std::uint32_t>((u^(u>>32))&0xffffffffu);
}

/** 【函数导航】
 * 作用：执行“planeBasis”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void planeBasis(const Vec3d& normal, Vec3d& u, Vec3d& v, Vec3d& w) noexcept {
    w=normalize(normal);
    Vec3d ref{0.0,1.0,0.0};
    if (std::abs(dot(ref,w))>0.92) ref={1.0,0.0,0.0};
    u=normalize(cross(ref,w));
    v=normalize(cross(w,u));
}

/** 【函数导航】
 * 作用：执行“toLocal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Point3d toLocal(const Point3d& p,const Point3d& seed,const Vec3d& u,const Vec3d& v,const Vec3d& w) noexcept {
    const Vec3d d{p.x-seed.x,p.y-seed.y,p.z-seed.z};
    return {dot(d,u),dot(d,v),dot(d,w)};
}

/** 【函数导航】
 * 作用：执行“fromLocal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Point3d fromLocal(const Point3d& p,const Point3d& seed,const Vec3d& u,const Vec3d& v,const Vec3d& w) noexcept {
    return {seed.x+u.x*p.x+v.x*p.y+w.x*p.z,
            seed.y+u.y*p.x+v.y*p.y+w.y*p.z,
            seed.z+u.z*p.x+v.z*p.y+w.z*p.z};
}

/** 【函数导航】
 * 作用：执行“vectorFromLocal”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d vectorFromLocal(const Vec3d& p,const Vec3d& u,const Vec3d& v,const Vec3d& w) noexcept {
    return normalize({u.x*p.x+v.x*p.y+w.x*p.z,
                      u.y*p.x+v.y*p.y+w.y*p.z,
                      u.z*p.x+v.z*p.y+w.z*p.z});
}

}

/**
 * 【兼容位姿入口中的局部支撑平面】
 * 这是旧 detectShouDongHoleWeizi() 链使用的 RANSAC + 最小二乘平面，不是当前 GUI 正式 mouth-search
 * 的 guJiChuShiJuBuFaXian()。它们历史上服务不同入口，因此当前源码中同时存在并不代表每个 seed 会算两遍。
 * 若未来确认所有外部调用都已迁移，可单独删除整条兼容链；在未确认前只标清职责，不改变行为。
 */
LocalPlaneState estimateLocalSupportPlane(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormalInput,
    double radius,
    double threshold,
    int iterations,
    std::size_t maxPoints)
{
    LocalPlaneState out;
    const Vec3d globalNormal=normalize(globalNormalInput);
    std::vector<Point3d> q;
    q.reserve(std::min<std::size_t>(points.size(),maxPoints*2));
    const double r2=radius*radius;
    for(const auto& p:points){
        const double dx=p.x-seed.x,dy=p.y-seed.y,dz=p.z-seed.z;
        if(dx*dx+dy*dy+dz*dz<=r2) q.push_back(p);
    }
    if(q.size()<100){out.reason="few_points";out.candidatePoints=static_cast<int>(q.size());return out;}
    std::mt19937 rng(seedValue(seed));
    if(q.size()>maxPoints){
        std::shuffle(q.begin(),q.end(),rng);q.resize(maxPoints);
    }
    out.candidatePoints=static_cast<int>(q.size());
    std::uniform_int_distribution<std::size_t> pick(0,q.size()-1);
    double bestScore=-1e30;PingMianModel bestPlane;std::vector<std::size_t> bestIds;double bestMedian=0.0;
    const double minAlignment=std::cos(50.0*kPi/180.0);
    for(int it=0;it<iterations;++it){
        std::size_t i0=pick(rng),i1=pick(rng),i2=pick(rng);
        if(i0==i1||i0==i2||i1==i2) continue;
        PingMianModel plane;if(!planeFromThree(q[i0],q[i1],q[i2],plane))continue;
        const Vec3d n=normalFromPlane(plane,globalNormal);
        if(std::abs(dot(n,globalNormal))<minAlignment)continue;
        if(std::abs(signedPlaneResidual(seed,plane))>0.80)continue;
        std::vector<std::size_t> ids;ids.reserve(q.size());std::vector<double> res;res.reserve(q.size());
        for(std::size_t i=0;i<q.size();++i){const double r=std::abs(signedPlaneResidual(q[i],plane));if(r<threshold){ids.push_back(i);res.push_back(r);}}
        if(ids.size()<80)continue;
        const double med=median(std::move(res));
        const double score=static_cast<double>(ids.size())-80.0*med;
        if(score>bestScore){bestScore=score;bestPlane=plane;bestIds=std::move(ids);bestMedian=med;}
    }
    if(bestIds.empty()){out.reason="no_plane";return out;}
    PingMianModel refined=bestPlane;
    if(!fitPlaneLeastSquares(q,bestIds,refined)){out.reason="refine_failed";return out;}
    std::vector<std::size_t> inliers;inliers.reserve(q.size());std::vector<double> residuals;residuals.reserve(q.size());
    for(std::size_t i=0;i<q.size();++i){const double r=std::abs(signedPlaneResidual(q[i],refined));if(r<threshold){inliers.push_back(i);residuals.push_back(r);}}
    if(inliers.size()<80){out.reason="few_refined_inliers";return out;}
    const Vec3d n=normalFromPlane(refined,globalNormal);
    Vec3d u,v,w;planeBasis(n,u,v,w);
    double mu=0.0,mv=0.0,mw=0.0;
    for(std::size_t id:inliers){const Vec3d d{q[id].x-seed.x,q[id].y-seed.y,q[id].z-seed.z};mu+=dot(d,u);mv+=dot(d,v);mw+=dot(d,w);}
    const double inv=1.0/static_cast<double>(inliers.size());mu*=inv;mv*=inv;mw*=inv;
    double suu=0.0,svv=0.0,suv=0.0,sww=0.0;
    for(std::size_t id:inliers){const Vec3d d{q[id].x-seed.x,q[id].y-seed.y,q[id].z-seed.z};const double du=dot(d,u)-mu,dv=dot(d,v)-mv,dw=dot(d,w)-mw;suu+=du*du;svv+=dv*dv;suv+=du*dv;sww+=dw*dw;}
    suu*=inv;svv*=inv;suv*=inv;sww*=inv;
    const double trace=suu+svv;const double disc=std::sqrt(std::max(0.0,(suu-svv)*(suu-svv)+4.0*suv*suv));const double smallInPlane=0.5*(trace-disc);
    out.valid=true;out.inlierPoints=static_cast<int>(inliers.size());out.inlierRatio=static_cast<double>(inliers.size())/q.size();
    out.medianResidual=median(std::move(residuals));out.planeRatio=sww/std::max(smallInPlane,1e-12);
    out.deltaFromGlobalDegrees=angleDegrees(n,globalNormal);out.tiltDegrees=angleDegrees(n,{0.0,0.0,1.0});out.normal=n;out.reason="ok";
    (void)bestMedian;
    return out;
}

/**
 * 【兼容位姿入口，不是当前 GUI direct-cluster-first 主链】
 * 该函数会在旧链中比较“全局粗法线”和 estimateLocalSupportPlane() 的 RANSAC 局部法线，再决定
 * 是否把点云转进局部 frame。当前 GUI 正式 Hole 识别不经过这里；保留它是为了兼容旧公开接口。
 */
WeiziJianCeResult detectShouDongHoleWeizi(
    const std::vector<Point3d>& points,
    const Point3d& seed,
    const Vec3d& globalNormalInput)
{
    WeiziJianCeResult output;
    const Vec3d globalNormal=normalize(globalNormalInput);
    output.localPlane=estimateLocalSupportPlane(points,seed,globalNormal);
    const auto& d=output.localPlane;

    const bool useLocal=d.valid && d.deltaFromGlobalDegrees>=8.0 && d.inlierRatio>=0.38 &&
        d.inlierPoints>=300 && d.medianResidual<=0.12 && d.planeRatio<=0.02;

    auto applyMouthSupportPlane = [&]() {
        if (!output.valid || !output.geometry.valid) return;
        output.initialCenterZ = output.centerZ;
        const Point3d candidate{
            output.geometry.centerX, output.geometry.centerY, output.centerZ};
        Vec3d normalHint = output.geometry.normal;
        const double normalLength = std::sqrt(dot(normalHint, normalHint));
        if (!(normalLength > 1e-12) || !std::isfinite(normalLength)) normalHint = globalNormal;
        output.mouthSupportPlane = MouthSupportPlane::estimate(
            points, candidate, normalHint, output.geometry.radius);
        if (!output.mouthSupportPlane.valid) return;

        output.centerZ = output.mouthSupportPlane.correctedCenter.z;
        output.geometry.normal = output.mouthSupportPlane.normal;
        output.geometry.tiltDegrees = angleDegrees(output.geometry.normal, {0.0, 0.0, 1.0});
        output.geometry.source += "_mouth_support_plane_mouthSupportPlane";
        output.usedMouthSupportPlane = true;

        if (output.geometry.persistentFamilyValid
            && std::isfinite(output.geometry.persistentFamilyCenterX)
            && std::isfinite(output.geometry.persistentFamilyCenterY)
            && std::isfinite(output.geometry.persistentFamilyCenterZ)
            && output.geometry.persistentFamilyRadius > 0.5) {
            const Point3d persistentCandidate{
                output.geometry.persistentFamilyCenterX,
                output.geometry.persistentFamilyCenterY,
                output.geometry.persistentFamilyCenterZ};
            const MouthSupportPlane::Result persistentPlane = MouthSupportPlane::estimate(
                points, persistentCandidate, output.geometry.normal,
                output.geometry.persistentFamilyRadius);
            if (persistentPlane.valid) {
                output.geometry.persistentFamilyCenterZ = persistentPlane.correctedCenter.z;
            }
        }
    };
    if(!useLocal){
        output.geometry=detectShouDongHoleJihe(points,seed,globalNormal);
        output.valid=output.geometry.valid;output.usedLocalPose=false;
        if(output.valid){
            const double a=-globalNormal.x/globalNormal.z,b=-globalNormal.y/globalNormal.z;
            const double c=seed.z-a*seed.x-b*seed.y+output.geometry.selectedShift;
            output.centerZ=a*output.geometry.centerX+b*output.geometry.centerY+c;
            if(output.geometry.persistentFamilyValid){
                const double pc=seed.z-a*seed.x-b*seed.y+output.geometry.persistentFamilyCenterZ;
                output.geometry.persistentFamilyCenterZ=a*output.geometry.persistentFamilyCenterX+b*output.geometry.persistentFamilyCenterY+pc;
            }
        }
        applyMouthSupportPlane();
        return output;
    }
    Vec3d u,v,w;planeBasis(d.normal,u,v,w);
    std::vector<Point3d> local;local.reserve(std::min<std::size_t>(points.size(),50000));
    constexpr double keepRadius=35.0;const double kr2=keepRadius*keepRadius;
    for(const auto& p:points){const double dx=p.x-seed.x,dy=p.y-seed.y,dz=p.z-seed.z;if(dx*dx+dy*dy+dz*dz<=kr2)local.push_back(toLocal(p,seed,u,v,w));}
    const Point3d localSeed{0.0,0.0,0.0};
    FullJianCeResult z=detectShouDongHoleJihe(local,localSeed,{0.0,0.0,1.0});
    if(!z.valid){output.geometry=z;return output;}
    const Point3d world=fromLocal({z.centerX,z.centerY,z.selectedShift},seed,u,v,w);
    if(z.persistentFamilyValid){
        const Point3d pw=fromLocal({z.persistentFamilyCenterX,z.persistentFamilyCenterY,z.persistentFamilyCenterZ},seed,u,v,w);
        z.persistentFamilyCenterX=pw.x;z.persistentFamilyCenterY=pw.y;z.persistentFamilyCenterZ=pw.z;
    }
    z.centerX=world.x;z.centerY=world.y;z.normal=vectorFromLocal(z.normal,u,v,w);
    z.tiltDegrees=angleDegrees(z.normal,{0.0,0.0,1.0});z.source="pose_local_"+z.source;
    output.geometry=z;output.centerZ=world.z;output.valid=true;output.usedLocalPose=true;
    applyMouthSupportPlane();
    return output;
}

}


// ============================================================================
// 功能分区：孔口外环支撑面
// ============================================================================
namespace shouDongHole {
namespace MouthSupportPlane {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kSectorCount = 24;

/** 【类型导航注释】
 * JuBuPoint：手动 Hole 位姿支撑中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JuBuPoint {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    double radius = 0.0;
};

/** 【类型导航注释】
 * PingMianNiHe：手动 Hole 位姿支撑中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PingMianNiHe {
    bool valid = false;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double rmse = 0.0;
    double mad = 0.0;
    std::vector<std::size_t> inliers;
};

/** 【类型导航注释】
 * HouXuan：手动 Hole 位姿支撑中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HouXuan {
    PingMianNiHe fit;
    Vec3d normal;
    double coverage = 0.0;
    double radialSpan = 0.0;
    double normalDeltaDegrees = 0.0;
    double innerBelow = 0.0;
    double innerNear = 0.0;
    double innerAbove = 0.0;
    double outerNear = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    int band = -1;
};

double dot(const Vec3d& a, const Vec3d& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d cross(const Vec3d& a, const Vec3d& b) noexcept
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

/** 【函数导航】
 * 作用：执行“normalize”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp、ZhuChuangKou_Window.cpp、HoleLeixing_Types.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Vec3d normalize(Vec3d v) noexcept
{
    const double length = std::sqrt(dot(v, v));
    if (!(length > 1e-12) || !std::isfinite(length)) return {0.0, 0.0, 1.0};
    v.x /= length;
    v.y /= length;
    v.z /= length;
    return v;
}

/** 【函数导航】
 * 作用：执行“finitePoint”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finitePoint(const Point3d& p) noexcept
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/** 【函数导航】
 * 作用：构建“makeBasis”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void makeBasis(const Vec3d& normal, Vec3d& u, Vec3d& v, Vec3d& w) noexcept
{
    w = normalize(normal);
    Vec3d reference{0.0, 1.0, 0.0};
    if (std::abs(dot(reference, w)) > 0.92) reference = {1.0, 0.0, 0.0};
    u = normalize(cross(reference, w));
    v = normalize(cross(w, u));
}

double median(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    const double high = values[middle];
    if ((values.size() & 1U) != 0U) return high;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle - 1), values.end());
    return 0.5 * (values[middle - 1] + high);
}

/** 【函数导航】
 * 作用：执行“percentile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double percentile(std::vector<double> values, double q)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    q = std::clamp(q, 0.0, 1.0);
    std::sort(values.begin(), values.end());
    const double position = q * static_cast<double>(values.size() - 1);
    const std::size_t low = static_cast<std::size_t>(std::floor(position));
    const std::size_t high = static_cast<std::size_t>(std::ceil(position));
    const double t = position - static_cast<double>(low);
    return values[low] * (1.0 - t) + values[high] * t;
}

/** 【函数导航】
 * 作用：拟合/求解“solve3”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool solve3(double augmented[3][4], double output[3]) noexcept
{
    for (int column = 0; column < 3; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 3; ++row) {
            if (std::abs(augmented[row][column]) > std::abs(augmented[pivot][column])) pivot = row;
        }
        if (std::abs(augmented[pivot][column]) < 1e-14) return false;
        if (pivot != column) {
            for (int j = column; j < 4; ++j) std::swap(augmented[pivot][j], augmented[column][j]);
        }
        const double divisor = augmented[column][column];
        for (int j = column; j < 4; ++j) augmented[column][j] /= divisor;
        for (int row = 0; row < 3; ++row) {
            if (row == column) continue;
            const double factor = augmented[row][column];
            for (int j = column; j < 4; ++j) augmented[row][j] -= factor * augmented[column][j];
        }
    }
    for (int i = 0; i < 3; ++i) output[i] = augmented[i][3];
    return true;
}

/** 【函数导航】
 * 作用：执行“leastSquaresPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool leastSquaresPlane(
    const std::vector<JuBuPoint>& points,
    const std::vector<std::size_t>& ids,
    double& a,
    double& b,
    double& c) noexcept
{
    if (ids.size() < 3) return false;
    double normal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    for (std::size_t id : ids) {
        const JuBuPoint& p = points[id];
        const double row[3] = {p.u, p.v, 1.0};
        for (int i = 0; i < 3; ++i) {
            rhs[i] += row[i] * p.w;
            for (int j = 0; j < 3; ++j) normal[i][j] += row[i] * row[j];
        }
    }
    double augmented[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) augmented[i][j] = normal[i][j];
        augmented[i][3] = rhs[i];
    }
    double solution[3];
    if (!solve3(augmented, solution)) return false;
    a = solution[0];
    b = solution[1];
    c = solution[2];
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

/** 【函数导航】
 * 作用：执行“robustPlaneFit”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
PingMianNiHe robustPlaneFit(
    const std::vector<JuBuPoint>& points,
    const std::vector<std::size_t>& initialIds)
{
    PingMianNiHe output;
    if (initialIds.size() < 24) return output;

    std::vector<std::size_t> ids = initialIds;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        if (!leastSquaresPlane(points, ids, a, b, c)) return output;
        std::vector<double> residuals;
        residuals.reserve(ids.size());
        for (std::size_t id : ids) {
            const JuBuPoint& p = points[id];
            residuals.push_back(p.w - (a * p.u + b * p.v + c));
        }
        const double residualMedian = median(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (double residual : residuals) deviations.push_back(std::abs(residual - residualMedian));
        const double mad = median(std::move(deviations));
        const double threshold = std::max(0.28, std::min(0.60, 3.5 * 1.4826 * mad + 0.05));

        std::vector<std::size_t> next;
        next.reserve(initialIds.size());
        for (std::size_t id : initialIds) {
            const JuBuPoint& p = points[id];
            const double residual = p.w - (a * p.u + b * p.v + c);
            if (std::abs(residual - residualMedian) <= threshold) next.push_back(id);
        }
        if (next.size() < 24) return output;
        ids.swap(next);
    }

    if (!leastSquaresPlane(points, ids, a, b, c)) return output;
    std::vector<double> residuals;
    residuals.reserve(ids.size());
    double sumSquared = 0.0;
    for (std::size_t id : ids) {
        const JuBuPoint& p = points[id];
        const double residual = p.w - (a * p.u + b * p.v + c);
        residuals.push_back(residual);
        sumSquared += residual * residual;
    }
    const double residualMedian = median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (double residual : residuals) deviations.push_back(std::abs(residual - residualMedian));

    output.valid = true;
    output.a = a;
    output.b = b;
    output.c = c;
    output.rmse = std::sqrt(sumSquared / static_cast<double>(ids.size()));
    output.mad = median(std::move(deviations));
    output.inliers = std::move(ids);
    return output;
}

/** 【函数导航】
 * 作用：执行“angularCoverage”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double angularCoverage(
    const std::vector<JuBuPoint>& points,
    const std::vector<std::size_t>& ids)
{
    std::array<unsigned char, kSectorCount> occupied{};
    for (std::size_t id : ids) {
        const JuBuPoint& p = points[id];
        double angle = std::atan2(p.v, p.u);
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * kSectorCount));
        sector = std::clamp(sector, 0, kSectorCount - 1);
        occupied[static_cast<std::size_t>(sector)] = 1;
    }
    int count = 0;
    for (unsigned char value : occupied) count += value != 0 ? 1 : 0;
    return static_cast<double>(count) / static_cast<double>(kSectorCount);
}

/** 【函数导航】
 * 作用：执行“angleDegrees”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double angleDegrees(const Vec3d& a, const Vec3d& b) noexcept
{
    return std::acos(std::clamp(std::abs(dot(normalize(a), normalize(b))), 0.0, 1.0)) * 180.0 / kPi;
}

/** 【函数导航】
 * 作用：执行“histogramPeaks”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<int> histogramPeaks(const std::vector<int>& histogram)
{
    std::vector<int> indices(histogram.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::stable_sort(indices.begin(), indices.end(), [&](int lhs, int rhs) {
        return histogram[static_cast<std::size_t>(lhs)] > histogram[static_cast<std::size_t>(rhs)];
    });
    if (indices.size() > 20) indices.resize(20);
    for (std::size_t i = 1; i + 1 < histogram.size(); ++i) {
        if (histogram[i] >= 10 && histogram[i] >= histogram[i - 1] && histogram[i] >= histogram[i + 1]) {
            const int index = static_cast<int>(i);
            if (std::find(indices.begin(), indices.end(), index) == indices.end()) indices.push_back(index);
        }
    }
    return indices;
}

/** 【函数导航】
 * 作用：构建“buildCandidate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HouXuan buildCandidate(
    const std::vector<JuBuPoint>& points,
    const std::vector<std::size_t>& bandIds,
    double mode,
    double bandMin,
    double bandMax,
    double mouthRadius,
    int bandIndex,
    const Vec3d& u,
    const Vec3d& v,
    const Vec3d& w)
{
    HouXuan output;
    std::vector<std::size_t> initial;
    initial.reserve(bandIds.size());
    for (std::size_t id : bandIds) {
        if (std::abs(points[id].w - mode) <= 0.55) initial.push_back(id);
    }
    output.fit = robustPlaneFit(points, initial);
    if (!output.fit.valid) return output;

    output.coverage = angularCoverage(points, output.fit.inliers);
    std::vector<double> radii;
    radii.reserve(output.fit.inliers.size());
    for (std::size_t id : output.fit.inliers) radii.push_back(points[id].radius);
    const double radialLow = percentile(radii, 0.10);
    const double radialHigh = percentile(std::move(radii), 0.90);
    output.radialSpan = (radialHigh - radialLow) / std::max(0.1, bandMax - bandMin);

    output.normal = normalize({
        -output.fit.a * u.x - output.fit.b * v.x + w.x,
        -output.fit.a * u.y - output.fit.b * v.y + w.y,
        -output.fit.a * u.z - output.fit.b * v.z + w.z
    });
    if (dot(output.normal, w) < 0.0) {
        output.normal.x = -output.normal.x;
        output.normal.y = -output.normal.y;
        output.normal.z = -output.normal.z;
    }
    output.normalDeltaDegrees = angleDegrees(output.normal, w);

    const double innerRadius = std::max(0.8, 0.85 * mouthRadius);
    int innerCount = 0;
    int innerBelow = 0;
    int innerNear = 0;
    int innerAbove = 0;
    int outerCount = 0;
    int outerNear = 0;
    for (const JuBuPoint& p : points) {
        const double planeW = output.fit.a * p.u + output.fit.b * p.v + output.fit.c;
        const double residual = p.w - planeW;
        if (p.radius <= innerRadius) {
            ++innerCount;
            if (residual < -0.40) ++innerBelow;
            else if (std::abs(residual) <= 0.30) ++innerNear;
            else if (residual > 0.40) ++innerAbove;
        }
        if (p.radius >= bandMin && p.radius <= bandMax) {
            ++outerCount;
            if (std::abs(residual) <= 0.30) ++outerNear;
        }
    }
    if (innerCount >= 8) {
        const double inverse = 1.0 / static_cast<double>(innerCount);
        output.innerBelow = static_cast<double>(innerBelow) * inverse;
        output.innerNear = static_cast<double>(innerNear) * inverse;
        output.innerAbove = static_cast<double>(innerAbove) * inverse;
    }
    if (outerCount > 0) output.outerNear = static_cast<double>(outerNear) / static_cast<double>(outerCount);

    const double geometricScore = 4.0 * output.coverage
        + 0.002 * static_cast<double>(std::min<std::size_t>(output.fit.inliers.size(), 1500))
        + output.radialSpan
        - 3.0 * output.fit.rmse
        - 0.03 * output.normalDeltaDegrees;
    const double topologyScore = 5.0 * output.innerBelow
        + 1.5 * output.innerNear
        - 7.0 * output.innerAbove
        + output.outerNear;
    output.score = geometricScore + topologyScore;
    output.band = bandIndex;
    return output;
}

}

/** 【函数导航】
 * 作用：估计“estimate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：手动 Hole 位姿支撑。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_Manual.h、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result estimate(
    const std::vector<Point3d>& points,
    const Point3d& candidateCenter,
    const Vec3d& referenceNormal,
    double mouthRadius)
{
    Result output;
    output.correctedCenter = candidateCenter;
    output.normal = normalize(referenceNormal);

    if (!finitePoint(candidateCenter) || !std::isfinite(mouthRadius) || mouthRadius <= 0.5
        || points.size() < 80 || std::abs(output.normal.z) < 0.35) {
        output.reason = "MouthSupportPlane_INVALID_INPUT";
        return output;
    }

    Vec3d u;
    Vec3d v;
    Vec3d w;
    makeBasis(output.normal, u, v, w);

    const std::array<std::pair<double, double>, 4> bands{{

        {std::max(0.9, 0.78 * mouthRadius),
         std::max(std::max(0.9, 0.78 * mouthRadius) + 2.0,
                  std::min(1.35 * mouthRadius, mouthRadius + 3.5))},
        {std::max(0.9, 1.03 * mouthRadius),
         std::max(std::max(0.9, 1.03 * mouthRadius) + 2.0,
                  std::min(1.70 * mouthRadius, mouthRadius + 4.0))},
        {std::max(1.0, 1.10 * mouthRadius),
         std::max(std::max(1.0, 1.10 * mouthRadius) + 3.0,
                  std::min(2.20 * mouthRadius, mouthRadius + 7.0))},
        {std::max(1.2, 1.20 * mouthRadius),
         std::max(std::max(1.2, 1.20 * mouthRadius) + 4.0,
                  std::min(2.60 * mouthRadius, mouthRadius + 9.0))}
    }};
    double maximumRadius = 0.0;
    for (const auto& band : bands) maximumRadius = std::max(maximumRadius, band.second);

    std::vector<JuBuPoint> local;
    local.reserve(std::min<std::size_t>(points.size(), 50000));
    for (const Point3d& point : points) {
        if (!finitePoint(point)) continue;
        const Vec3d delta{
            point.x - candidateCenter.x,
            point.y - candidateCenter.y,
            point.z - candidateCenter.z
        };
        const double localU = dot(delta, u);
        const double localV = dot(delta, v);
        const double radius = std::hypot(localU, localV);
        if (radius > maximumRadius + 0.5) continue;
        local.push_back({localU, localV, dot(delta, w), radius});
    }
    if (local.size() < 80) {
        output.reason = "MouthSupportPlane_TOO_FEW_LOCAL_POINTS";
        return output;
    }

    std::vector<HouXuan> candidates;
    for (int bandIndex = 0; bandIndex < static_cast<int>(bands.size()); ++bandIndex) {
        const double bandMin = bands[static_cast<std::size_t>(bandIndex)].first;
        const double bandMax = bands[static_cast<std::size_t>(bandIndex)].second;
        std::vector<std::size_t> bandIds;
        std::vector<double> axial;
        bandIds.reserve(local.size());
        axial.reserve(local.size());
        for (std::size_t i = 0; i < local.size(); ++i) {
            if (local[i].radius < bandMin || local[i].radius > bandMax) continue;
            bandIds.push_back(i);
            axial.push_back(local[i].w);
        }
        if (bandIds.size() < 30) continue;

        const double low = percentile(axial, 0.002);
        const double high = percentile(axial, 0.998);
        if (!std::isfinite(low) || !std::isfinite(high) || high <= low) continue;
        constexpr double binWidth = 0.20;
        const int binCount = std::max(2, static_cast<int>(std::ceil((high - low) / binWidth)) + 1);
        std::vector<int> histogram(static_cast<std::size_t>(binCount), 0);
        for (double value : axial) {
            int bin = static_cast<int>(std::floor((value - low) / binWidth));
            bin = std::clamp(bin, 0, binCount - 1);
            ++histogram[static_cast<std::size_t>(bin)];
        }
        const std::vector<int> peaks = histogramPeaks(histogram);
        for (int peak : peaks) {
            if (histogram[static_cast<std::size_t>(peak)] < 10) continue;
            const double mode = low + (static_cast<double>(peak) + 0.5) * binWidth;
            HouXuan candidate = buildCandidate(
                local, bandIds, mode, bandMin, bandMax, mouthRadius, bandIndex, u, v, w);
            if (!candidate.fit.valid) continue;
            candidates.push_back(std::move(candidate));
        }
    }

    if (candidates.empty()) {
        output.reason = "MouthSupportPlane_NO_PLANE_CANDIDATE";
        return output;
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const HouXuan& lhs, const HouXuan& rhs) {
        return lhs.score > rhs.score;
    });

    std::vector<const HouXuan*> physicalCandidates;
    physicalCandidates.reserve(candidates.size());
    for (const HouXuan& candidate : candidates) {
        if (candidate.fit.inliers.size() < 24) continue;
        if (candidate.coverage < 0.25) continue;
        if (candidate.fit.rmse > 0.35) continue;
        if (candidate.normalDeltaDegrees > 12.0) continue;
        if (candidate.outerNear < 0.10) continue;

        if (candidate.innerAbove > 0.45 && candidate.innerAbove > candidate.innerBelow + candidate.innerNear) continue;
        physicalCandidates.push_back(&candidate);
    }

    const HouXuan* selected = physicalCandidates.empty() ? nullptr : physicalCandidates.front();

    if (selected != nullptr && selected->fit.c < -3.0) {
        for (const HouXuan* candidate : physicalCandidates) {
            if (std::abs(candidate->fit.c) > 1.5) continue;
            if (candidate->score < selected->score - 1.25) continue;
            selected = candidate;
            break;
        }
    }
    if (selected == nullptr) {
        output.reason = "MouthSupportPlane_NO_PHYSICAL_MOUTH_PLANE";
        return output;
    }

    const Point3d planePoint{
        candidateCenter.x + w.x * selected->fit.c,
        candidateCenter.y + w.y * selected->fit.c,
        candidateCenter.z + w.z * selected->fit.c
    };
    if (std::abs(selected->normal.z) < 0.35) {
        output.reason = "MouthSupportPlane_SELECTED_PLANE_NEAR_VERTICAL";
        return output;
    }
    const double correctedZ = planePoint.z
        - (selected->normal.x * (candidateCenter.x - planePoint.x)
           + selected->normal.y * (candidateCenter.y - planePoint.y)) / selected->normal.z;
    if (!std::isfinite(correctedZ)) {
        output.reason = "MouthSupportPlane_NONFINITE_CORRECTED_CENTER";
        return output;
    }

    output.valid = true;
    output.correctedCenter = {candidateCenter.x, candidateCenter.y, correctedZ};
    output.normal = selected->normal;
    output.axialShift = dot({
        output.correctedCenter.x - candidateCenter.x,
        output.correctedCenter.y - candidateCenter.y,
        output.correctedCenter.z - candidateCenter.z
    }, w);
    output.planeRmse = selected->fit.rmse;
    output.planeMad = selected->fit.mad;
    output.planeCoverage = selected->coverage;
    output.planeSupport = static_cast<int>(selected->fit.inliers.size());
    output.normalDeltaDegrees = selected->normalDeltaDegrees;
    output.innerBelowFraction = selected->innerBelow;
    output.innerNearFraction = selected->innerNear;
    output.innerAboveFraction = selected->innerAbove;
    output.outerNearFraction = selected->outerNear;
    output.selectedBand = selected->band;
    output.selectedScore = selected->score;
    output.reason = "MouthSupportPlane_MOUTH_SUPPORT_PLANE_PROJECTED";
    return output;
}

}
}
