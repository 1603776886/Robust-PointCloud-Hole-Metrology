/*
================================================================================
文件：HoleShibie_Recognition.cpp
模块：Hole 识别总编排

【主要职责】
串联 seed 局部 ROI、支撑面、Hole 口候选、规范几何、类型/深度、位姿和最终审核。

【主要调用关系】
由 HoleShibie_Recognition.h 的公开函数进入；下调 Fenxi/Jihe/Weizi/ShouDongHole 模块。

【线程与状态】
后台 worker；候选顺序、阈值和浮点计算顺序属于生产行为。

【维护边界】
1. 本文件只做识别流程编排；法线数学集中在 HoleFaXian_Normal.h，几何拟合集中在 HoleJihe_Geometry.*。
2. “初始粗法线”只服务 mouth-search；“椭圆精确法线”只在 Hole 口已经确定后接管角度精修，两者不得互相覆盖职责。
3. 候选排序、ROI、阈值、迭代次数和浮点表达式属于识别行为；结构整理时不得顺手改动。
4. 第三方 Qt/PCL/VTK/Eigen 的类型、函数和命名保持原库形式。
================================================================================
*/
/*
模块职责：
实现正式手动选孔识别的总编排，并把支撑面、孔口、深度、孔形和姿态等子模块组合成最终 HoleShibieResult。

主要调用位置：
ZhuChuangKouWindow 的孔识别入口调用这里，负责把各几何子模块组合成 GUI 使用的正式识别结果。

维护说明：
本文件负责 GUI 正式孔识别流程编排；具体几何计算、孔型分析、位姿估计和缓存策略由对应功能模块承担。
重要迭代次数和距离阈值旁保留中文说明；修改这些数值会影响精度或速度，应按用途单独评估。
*/

#include "HoleShibie_Recognition.h"
#include "HoleFenxi_Analysis.h"
#include "HoleWeizi_Pose.h"
#include <future>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include "ShouDongHole_Manual.h"
#include "HoleJihe_Geometry.h"
#include "HoleLeixing_Types.h"
// 备忘：这个函数只负责把孔识别结果画成检查用点云，我用它核对孔心、上口圆、下口圆和深度线。

/** 【函数导航】
 * 作用：执行“keShiHuaHole”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr keShiHuaHole(const CloudPtr& edgeCloud, const HoleMiaoshu& desc)
{
    CloudPtr colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (edgeCloud) {
        for (const auto& p : *edgeCloud) {
            pcl::PointXYZRGB q = p;
            q.r = 255; q.g = 255; q.b = 0;
            colored->push_back(q);
        }
    }

    const Eigen::Vector3f cTop = (desc.rTop > 0.0f) ? desc.centerTop : desc.center;
    const float rTop = (desc.rTop > 0.0f) ? desc.rTop : desc.radius;
    const float rBot = (desc.rBot > 0.0f) ? desc.rBot : 0.0f;

    Eigen::Vector3f inwardAxis(
        desc.holeAxisInNx, desc.holeAxisInNy, desc.holeAxisInNz);
    const bool inwardAxisValid = desc.holeAxisInValid && inwardAxis.allFinite()
        && inwardAxis.norm() > 0.5f;
    if (inwardAxisValid) inwardAxis.normalize();
    else inwardAxis = Eigen::Vector3f::Zero();

    Eigen::Vector3f normal(desc.localPlaneNx, desc.localPlaneNy, desc.localPlaneNz);
    if (inwardAxisValid && HoleJiXing::known(desc.inwardPolarity)) {
        normal = inwardAxis
            / static_cast<float>(HoleJiXing::signOrZero(desc.inwardPolarity));
    } else if ((!normal.allFinite() || normal.norm() < 0.5f)
        && desc.refinedSurfaceNormalValid) {
        normal = Eigen::Vector3f(
            desc.refinedSurfaceNx, desc.refinedSurfaceNy, desc.refinedSurfaceNz);
    }
    if (!normal.allFinite() || normal.norm() < 0.5f)
        normal = Eigen::Vector3f::UnitZ();
    else
        normal.normalize();
    const Eigen::Vector3f cBot = (inwardAxisValid && desc.depth > 0.0f)
        ? (cTop + inwardAxis * desc.depth)
        : ((desc.rBot > 0.0f) ? desc.centerBot : desc.center);
    Eigen::Vector3f axisU = normal.unitOrthogonal().normalized();
    Eigen::Vector3f axisV = normal.cross(axisU).normalized();

    pcl::PointXYZRGB c;
    c.x = desc.center.x(); c.y = desc.center.y(); c.z = desc.center.z();
    c.r = 255; c.g = 0; c.b = 0;
    for (int i = 0; i < 20; ++i) colored->push_back(c);

    auto addRing = [&](const Eigen::Vector3f& center, float radius,
                       std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
        if (!(radius > 0.0f) || !std::isfinite(radius) || !center.allFinite()) return;
        const int N = 72;
        for (int i = 0; i < N; ++i) {
            const float ang = static_cast<float>(2.0 * M_PI * i / N);
            const Eigen::Vector3f w = center + radius
                * (std::cos(ang) * axisU + std::sin(ang) * axisV);
            pcl::PointXYZRGB p;
            p.x = w.x(); p.y = w.y(); p.z = w.z();
            p.r = red; p.g = green; p.b = blue;
            colored->push_back(p);
        }
    };

    // 识别结果中的主示意圆始终按规范 rTop 绘制；真实孔口交线仅用于诊断数据，
    // 不参与最终 GUI 的半径示意，确保数值与图形表达同一个几何量。
    addRing(cTop, rTop, 255, 0, 255);
    if (desc.type != 1 && rBot > 0.0f) addRing(cBot, rBot, 0, 255, 0);

    if (desc.type != 1 && desc.depth > 0.0f && std::isfinite(desc.depth)
        && inwardAxisValid) {
        Eigen::Vector3f lineEnd = cBot;
        if (!lineEnd.allFinite() || (lineEnd - cTop).norm() < 0.1f)
            lineEnd = cTop + inwardAxis * desc.depth;
        const int N = 20;
        for (int i = 0; i < N; ++i) {
            const float t = static_cast<float>(i) / (N - 1);
            const Eigen::Vector3f w = cTop + (lineEnd - cTop) * t;
            pcl::PointXYZRGB p;
            p.x = w.x(); p.y = w.y(); p.z = w.z();
            p.r = 0; p.g = 255; p.b = 255;
            colored->push_back(p);
        }
    }

    return colored;
}

#include <QDateTime>
#include <unordered_map>
#include <QDir>
#include <QStringLiteral>
#include <algorithm>
#include <array>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <utility>
#include <vector>

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudPtr = Cloud::Ptr;

struct FenxiCacheBalancedNormalGuardResult
{
    holeNormalPingHeng::Result ng;
    bool guardApplies = false;
    Eigen::Vector3f guardSurface{0.0f, 0.0f, 1.0f};
    double wallLimit = 0.0;
};

struct FenxiCacheWeiziPairCacheEntry
{
    std::size_t sampleCount = 0;
    std::uint64_t sampleIndexHash = 0;
    std::uint64_t mouthXBits = 0;
    std::uint64_t mouthYBits = 0;
    std::uint64_t mouthZBits = 0;
    std::uint64_t surfaceXBits = 0;
    std::uint64_t surfaceYBits = 0;
    std::uint64_t surfaceZBits = 0;
    std::uint64_t topRadiusBits = 0;
    std::uint64_t bottomRadiusBits = 0;
    std::uint64_t depthBits = 0;
    int polarity = 0;
    int holeType = 0;
    std::vector<int> originalIndices;
    HoleWeiziFinal::Result basePose;
    LianXuZhuiWeizi::Result continuousConePose;
    bool hasBalancedNormalGuard = false;
    FenxiCacheBalancedNormalGuardResult balancedNormalGuard;
};

namespace {
using holeJihe::kasaZhongWeiShu;
using holeJihe::kasaNiHeYuan;
using holeJihe::kasaJiaoJunBianJie;
using holeJihe::kasaJieDuanNiHe;
using holeJihe::ransacNiHeYuanSanDian;
using holeJihe::ransacNiHeSanWeiYuan;
using holeJihe::shouDongZhongWeiShu;
using holeJihe::shouDongFenWeiShu;
using holeZhicheng::shouDongZhuZaiFaXiangGao;
using holeZhicheng::shouDongJinLinZhuZaiFaXiangGao;
using holeZhicheng::shouDongJinLinZhiChengMianFaXiangGao;
using holeShendu::ShouDongFenCengTongJi;
using holeShendu::shouDongFenCengBanJing;
using holeShendu::shouDongZhongWeiBanJing;
using holeShendu::shouDongJinLinShenDuBanJing;
using holeShendu::ShouDongJingXiangShenDu;
using holeShendu::shouDongJingXiangShenDu;
using holeKou::ShouDongZhuiHoleKouBaoLuoJingXiu;
using holeKou::ShouDongZhuiBiKouBuWaiTui;
using holeKou::shouDongGuJiZhuiHoleKouBuBaoLuo;
using holeKou::shouDongGuJiZhuiBiKouBuWaiTui;
using holeKou::ShouDongWangGeHoleKou;
using holeKou::guJiWangGeHoleKouYinYing;
using holeKou::guJiDaBanJingHoleKouWaiBianJie;
// 备忘：这个函数算一组数的中位数，主要用来减少飞点和局部毛刺对半径的影响。

// 备忘：这个结构体保存手动识别用的局部坐标系，origin 是原点，u/v/n 是三个局部方向。
struct ShouDongJuBuZuoBiao
{
    Eigen::Vector3f origin{0, 0, 0};
    Eigen::Vector3f u{1, 0, 0};
    Eigen::Vector3f v{0, 1, 0};
    Eigen::Vector3f n{0, 0, 1};
    bool valid = false;
};
// 备忘：这个函数根据原点和法向搭局部坐标系，后面切片和孔口计算都按这个坐标系走。


// 锥孔的上下口顺序、25°坡角、1.50 mm有效锥深和 depth/rTop 审核
// 已并入 HoleFenxi_Analysis.cpp 的 DepthBottomReview 深度/下口正常审核。
// 本编排层只读取审核结果，不再维护一套独立的末端“最终守门”。

/** 【函数导航】
 * 作用：构建“gouJianShouDongJuBuZuoBiao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static ShouDongJuBuZuoBiao gouJianShouDongJuBuZuoBiao(const Eigen::Vector3f& origin, Eigen::Vector3f n)
{
    ShouDongJuBuZuoBiao frame;
    frame.origin = origin;
    if (n.norm() <= 1e-6f) return frame;
    n.normalize();
    if (!std::isfinite(n.x()) || !std::isfinite(n.y()) || !std::isfinite(n.z())) return frame;
    if (n.dot(Eigen::Vector3f::UnitZ()) < 0.0f) n = -n;
    frame.n = n;
    frame.u = n.unitOrthogonal().normalized();
    frame.v = n.cross(frame.u).normalized();
    frame.valid = true;
    return frame;
}
// 备忘：这个函数把局部坐标里的孔心和孔底还原到原始点云坐标，方便显示和导出。

/** 【函数导航】
 * 作用：执行“juBuZuoBiaoDaoShiJie”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static Eigen::Vector3f juBuZuoBiaoDaoShiJie(const ShouDongJuBuZuoBiao& f, const Eigen::Vector3f& p)
{
    return f.origin + f.u * p.x() + f.v * p.y() + f.n * p.z();
}
// 备忘：这个函数把种子周围的点裁成局部 ROI，并把点转换到局部坐标系。

/** 【函数导航】
 * 作用：构建“gouJianShouDongJuBuDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static CloudPtr gouJianShouDongJuBuDianYun(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const ShouDongHoleSeed& seed,
    const ShouDongJuBuZuoBiao& frame,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    CloudPtr local(new Cloud);
    if (!cloud || !frame.valid) return local;
    const float r = std::max(12.0f, seed.searchRadiusMm);
    const float r2 = r * r;
    local->reserve(4096);

    auto tianJiaJuBuDian = [&](const pcl::PointXYZRGB& p) {
        Eigen::Vector3f w(p.x, p.y, p.z);
        Eigen::Vector3f d = w - frame.origin;
        if (d.squaredNorm() > r2) return;
        pcl::PointXYZRGB q = p;
        q.x = d.dot(frame.u);
        q.y = d.dot(frame.v);
        q.z = d.dot(frame.n);
        local->push_back(q);
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = frame.origin.x();
        query.y = frame.origin.y();
        query.z = frame.origin.z();
        std::vector<int> indices;
        std::vector<float> sqrDists;
        if (kdtree->radiusSearch(query, r, indices, sqrDists) > 0) {
            local->reserve(indices.size());
            for (int suoYin : indices) {
                if ((unsigned)suoYin >= cloud->size()) continue;
                tianJiaJuBuDian((*cloud)[(size_t)suoYin]);
            }
        }
    } else {
        for (const auto& p : *cloud) tianJiaJuBuDian(p);
    }
    return local;
}

static void canonicalRoiHashBytes(std::uint64_t& hash, const void* data, std::size_t size);

namespace RoiRuntime {

/** 【类型导航注释】
 * PaiXuYangBenJinCou：Hole 识别总编排中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PaiXuYangBenJinCou {
    float u = 0.0f;
    float v = 0.0f;
    float w = 0.0f;
    int originalIndex = -1;
    long long quantizedW = 0;
    std::int32_t quantizedRadial = 0;
    std::int32_t quantizedAngle = 0;
};
static_assert(sizeof(PaiXuYangBenJinCou) <= 32,
    "局部 ROI 排序记录不应意外膨胀");

// 正式生产路径关闭 FLANN 半径搜索结果的距离排序：后续会按几何键做确定性排序，
// 因此这里无需先按距离排序，可减少首次孔识别的 ROI 查询开销。
/** 【类型导航注释】
 * WuPaiXuKdShouHu：Hole 识别总编排中的自定义 类。
 * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
class WuPaiXuKdShouHu {
public:
    explicit WuPaiXuKdShouHu(const pcl::KdTreeFLANN<pcl::PointXYZRGB>* tree)
        : tree_(const_cast<pcl::KdTreeFLANN<pcl::PointXYZRGB>*>(tree))
    {
        if (!tree_) return;
        oldSorted_ = tree_->getSortedResults();
        if (oldSorted_) tree_->setSortedResults(false);
    }

    ~WuPaiXuKdShouHu()
    {
        if (tree_ && oldSorted_) tree_->setSortedResults(true);
    }

    WuPaiXuKdShouHu(const WuPaiXuKdShouHu&) = delete;
    WuPaiXuKdShouHu& operator=(const WuPaiXuKdShouHu&) = delete;

private:
    pcl::KdTreeFLANN<pcl::PointXYZRGB>* tree_ = nullptr;
    bool oldSorted_ = false;
};

}

struct GuiFanSearchCuRoi
{
    std::vector<HoleHouXuanJuLei::Sample> samples;
    std::size_t pointCount = 0;
    std::uint64_t hash = 1469598103934665603ULL;
    float radialLimit = 25.0f;
    float axialLimit = 15.0f;
    float sphereRadius = 0.0f;
};

/** 【函数导航】
 * 作用：构建“gouJianCanonicalSearchCoarseRoi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static GuiFanSearchCuRoi gouJianCanonicalSearchCoarseRoi(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const ShouDongJuBuZuoBiao& stableFrame,
    const Eigen::Vector3f& seedWorld,
    float radialLimit,
    float axialLimit,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree,
    std::vector<HoleHouXuanJuLei::Sample>* reusableSamples = nullptr)
{
    GuiFanSearchCuRoi out;
    out.radialLimit = std::clamp(radialLimit, 20.0f, 52.0f);
    out.axialLimit = std::clamp(axialLimit, 12.0f, 22.0f);
    if (!cloud || cloud->empty() || !stableFrame.valid || !seedWorld.allFinite()) return out;

    const Eigen::Vector3f seedDelta = seedWorld - stableFrame.origin;
    const float seedU = seedDelta.dot(stableFrame.u);
    const float seedV = seedDelta.dot(stableFrame.v);
    const float seedW = seedDelta.dot(stableFrame.n);
    const float sphereRadius = std::sqrt(
        out.radialLimit * out.radialLimit + out.axialLimit * out.axialLimit) + 0.5f;
    out.sphereRadius = sphereRadius;

    using PaiXuYangBenJinCou = RoiRuntime::PaiXuYangBenJinCou;
    static thread_local std::vector<PaiXuYangBenJinCou> accepted;
    accepted.clear();
    if (accepted.capacity() < 16000) accepted.reserve(16000);

    auto consider = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const Eigen::Vector3f world(p.x, p.y, p.z);
        const Eigen::Vector3f delta = world - stableFrame.origin;
        const float u = delta.dot(stableFrame.u);
        const float v = delta.dot(stableFrame.v);
        const float w = delta.dot(stableFrame.n);
        const float du = u - seedU;
        const float dv = v - seedV;
        if (std::abs(w - seedW) > out.axialLimit) return;
        const float radial = std::hypot(du, dv);
        if (radial > out.radialLimit) return;
        float angle = std::atan2(dv, du);
        if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
        accepted.push_back({
            u, v, w, index,
            HoleFastPath::quantize1e4(w),
            static_cast<std::int32_t>(HoleFastPath::quantize1e4(radial)),
            static_cast<std::int32_t>(HoleFastPath::quantize1e4(angle))});
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = seedWorld.x();
        query.y = seedWorld.y();
        query.z = seedWorld.z();
        static thread_local std::vector<int> indices;
        static thread_local std::vector<float> squaredDistances;
        indices.clear();
        squaredDistances.clear();

        RoiRuntime::WuPaiXuKdShouHu unsortedGuard(kdtree);
        if (kdtree->radiusSearch(query, sphereRadius, indices, squaredDistances) > 0) {
            if (accepted.capacity() < indices.size()) accepted.reserve(indices.size());
            for (int index : indices) consider(index);
        }
    } else {
        for (std::size_t index = 0; index < cloud->size(); ++index)
            consider(static_cast<int>(index));
    }

    std::sort(accepted.begin(), accepted.end(),
        [](const RoiRuntime::PaiXuYangBenJinCou& a,
           const RoiRuntime::PaiXuYangBenJinCou& b) {
            if (a.quantizedW != b.quantizedW) return a.quantizedW < b.quantizedW;
            if (a.quantizedRadial != b.quantizedRadial) return a.quantizedRadial < b.quantizedRadial;
            if (a.quantizedAngle != b.quantizedAngle) return a.quantizedAngle < b.quantizedAngle;
            return a.originalIndex < b.originalIndex;
        });

    std::vector<HoleHouXuanJuLei::Sample>& targetSamples =
        reusableSamples ? *reusableSamples : out.samples;
    targetSamples.clear();
    if (targetSamples.capacity() < accepted.size()) targetSamples.reserve(accepted.size());
    for (const RoiRuntime::PaiXuYangBenJinCou& item : accepted) {
        HoleHouXuanJuLei::Sample sample;
        sample.u = item.u;
        sample.v = item.v;
        sample.w = item.w;
        sample.originalIndex = item.originalIndex;
        targetSamples.push_back(sample);
        canonicalRoiHashBytes(out.hash, &sample.originalIndex, sizeof(sample.originalIndex));
        canonicalRoiHashBytes(out.hash, &sample.u, sizeof(sample.u));
        canonicalRoiHashBytes(out.hash, &sample.v, sizeof(sample.v));
        canonicalRoiHashBytes(out.hash, &sample.w, sizeof(sample.w));
    }
    out.pointCount = targetSamples.size();
    return out;
}

// 直孔最终物理空腔审核：候选阶段的二维“空白连通域”只能证明某个薄层存在采样空斑，
// 不能单独证明实体内部真的为空。这里直接回到原始点云，检查机械上口内侧核心柱体是否
// 被点持续占据。真实直孔允许少量离群点，但若核心在多个深度带持续有点，同时孔壁/内喉
// 又完全不可观测，则该候选更符合“实心表面采样空斑/纹理小环”，不能输出为孔。
struct StraightWuLiVoidAudit
{
    bool executed = false;
    bool rejectSolidCore = false;
    int corePoints = 0;
    int coreOccupiedDepthBins = 0;
    int annulusPoints = 0;
    int annulusSectors = 0;
    // 浅层孔壁：只统计已经从机械上口向孔内进入一定深度的环带点。
    // 这组量和深层 wall/inner/observed-bore 互补，用来描述“孔还在，但降噪或遮挡
    // 只留下前几百微米孔壁”的情况；它不是特殊补丁，而是直孔物理证据的一部分。
    int qianCengAnnulusPoints = 0;
    int qianCengAnnulusSectors = 0;
    float qianCengAnnulusMaxDepth = 0.0f;
    float coreRadius = 0.0f;
    float depthMin = 0.0f;
    float depthMax = 0.0f;
    float coreToAnnulusRatio = 0.0f;
    std::string reason = "NOT_EXECUTED";
};

/** 【函数导航】
 * 作用：评估/审核“auditStraightPhysicalVoid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static StraightWuLiVoidAudit auditStraightPhysicalVoid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree,
    const HoleMiaoshu& d)
{
    StraightWuLiVoidAudit out;
    if (!cloud || cloud->empty() || d.type != 1 || !(d.rTop > 0.5f)
        || !d.centerTop.allFinite()) {
        out.reason = "SKIP_NOT_STRAIGHT_OR_INVALID_GEOMETRY";
        return out;
    }

    Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    if (!d.holeAxisInValid || !axis.allFinite() || axis.norm() < 0.5f) {
        Eigen::Vector3f surface(d.localPlaneNx, d.localPlaneNy, d.localPlaneNz);
        if (!surface.allFinite() || surface.norm() < 0.5f
            || !HoleJiXing::known(d.inwardPolarity)) {
            out.reason = "SKIP_NO_RELIABLE_INWARD_AXIS";
            return out;
        }
        surface.normalize();
        axis = surface * static_cast<float>(HoleJiXing::signOrZero(d.inwardPolarity));
    }
    axis.normalize();

    out.executed = true;
    out.coreRadius = std::max(0.35f, 0.78f * d.rTop);
    out.depthMin = -0.15f;
    out.depthMax = std::clamp(0.30f * d.rTop + 0.55f, 0.80f, 1.35f);
    const float annulusInner = 0.84f * d.rTop;
    const float annulusOuter = 1.20f * d.rTop;
    const float queryRadius = std::sqrt(
        annulusOuter * annulusOuter + out.depthMax * out.depthMax) + 0.25f;

    std::array<unsigned char, 8> coreDepthHit{};
    std::array<unsigned char, 36> annulusSectorHit{};
    std::array<unsigned char, 36> qianCengAnnulusSectorHit{};
    const Eigen::Vector3f annulusU = axis.unitOrthogonal().normalized();
    const Eigen::Vector3f annulusV = axis.cross(annulusU).normalized();
    auto inspectPoint = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const Eigen::Vector3f delta = Eigen::Vector3f(p.x, p.y, p.z) - d.centerTop;
        const float depth = delta.dot(axis);
        if (depth < out.depthMin || depth > out.depthMax) return;
        const float radial2 = std::max(0.0f, delta.squaredNorm() - depth * depth);
        const float radial = std::sqrt(radial2);

        if (radial <= out.coreRadius) {
            ++out.corePoints;
            const float t = (depth - out.depthMin)
                / std::max(1e-6f, out.depthMax - out.depthMin);
            const int bin = std::clamp(static_cast<int>(std::floor(t * 8.0f)), 0, 7);
            coreDepthHit[static_cast<std::size_t>(bin)] = 1U;
        }
        if (radial >= annulusInner && radial <= annulusOuter) {
            ++out.annulusPoints;
            const Eigen::Vector3f radialVec = delta - axis * depth;
            float angle = std::atan2(radialVec.dot(annulusV), radialVec.dot(annulusU));
            if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
            const int sector = std::clamp(
                static_cast<int>(std::floor(angle / static_cast<float>(2.0 * M_PI) * 36.0f)),
                0, 35);
            annulusSectorHit[static_cast<std::size_t>(sector)] = 1U;

            // 0.18 mm 以内很容易混入外表面粗糙度、口沿倒角和拟合误差。只有真正
            // 进入孔内以后才记为浅层孔壁。上限仍由 depthMax 约束，因此这里只描述
            // 机械上口后面的短孔壁，不会把远处其他结构当成当前 Hole 的内部证据。
            if (depth >= 0.18f) {
                ++out.qianCengAnnulusPoints;
                qianCengAnnulusSectorHit[static_cast<std::size_t>(sector)] = 1U;
                out.qianCengAnnulusMaxDepth = std::max(out.qianCengAnnulusMaxDepth, depth);
            }
        }
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = d.centerTop.x();
        query.y = d.centerTop.y();
        query.z = d.centerTop.z();
        static thread_local std::vector<int> indices;
        static thread_local std::vector<float> squaredDistances;
        indices.clear();
        squaredDistances.clear();
        RoiRuntime::WuPaiXuKdShouHu unsortedGuard(kdtree);
        if (kdtree->radiusSearch(query, queryRadius, indices, squaredDistances) > 0) {
            for (int index : indices) inspectPoint(index);
        }
    } else {
        for (std::size_t index = 0; index < cloud->size(); ++index)
            inspectPoint(static_cast<int>(index));
    }

    out.coreOccupiedDepthBins = static_cast<int>(std::count(
        coreDepthHit.begin(), coreDepthHit.end(), static_cast<unsigned char>(1U)));
    out.annulusSectors = static_cast<int>(std::count(
        annulusSectorHit.begin(), annulusSectorHit.end(), static_cast<unsigned char>(1U)));
    out.qianCengAnnulusSectors = static_cast<int>(std::count(
        qianCengAnnulusSectorHit.begin(), qianCengAnnulusSectorHit.end(),
        static_cast<unsigned char>(1U)));
    out.coreToAnnulusRatio = static_cast<float>(out.corePoints)
        / static_cast<float>(std::max(1, out.annulusPoints));

    const bool persistentCoreOccupation = out.corePoints >= 4
        && out.coreOccupiedDepthBins >= 2
        && out.coreToAnnulusRatio >= 0.10f;
    out.rejectSolidCore = persistentCoreOccupation;
    out.reason = out.rejectSolidCore
        ? "REJECT_PERSISTENT_SOLID_CORE_OCCUPANCY"
        : "PASS_CORE_VOID_OR_ONLY_ISOLATED_INTERIOR_POINTS";
    return out;
}


// 统一直孔 Hole 族证据。它只负责“这里是否仍然有足够物理证据说明是同一个 Hole”，
// 不负责重新拟合半径、法向、孔型或深度。完整圆与开放圆弧共享同一套字段和评分方式，
// 区别仅由 visibleFraction 连续改变所需环带点数/扇区数。
struct StraightHoleZuEvidence
{
    bool deepEvidence = false;
    bool shallowEvidence = false;
    bool valid = false;
    bool candidateStable = false;
    bool arcQualityValid = false;
    bool coreEmpty = false;
    bool visibleRingValid = false;
    bool qianCengWallValid = false;
    double visibleFraction = 0.0;
    double radiusAgreement = 0.0;
    double radiusAgreementLimit = 0.0;
    int annulusPointsMin = 0;
    int annulusSectorsMin = 0;
    int qianCengPointsMin = 0;
    int qianCengSectorsMin = 0;
    float qianCengDepthMin = 0.0f;
};

/**
 * @brief 统一评估直孔 Hole 族的深层与浅层物理证据。
 *
 * 调用位置：HoleShibie_Recognition.cpp 的最终直孔物理审核。
 * 输入含义：
 * - audit：围绕前端锁定 Hole 口锚点、直接从原始点云得到的 core/annulus/浅层孔壁统计；
 * - cluster：mouth-search 锁定的同一个物理候选簇；
 * - resolvedRadius：Hole 族口沿半径；
 * - d：后续正式几何/孔型/孔壁测量结果；
 * - mouthSearch...：候选阶段已经计算过的核心占据证据。
 *
 * 该函数不会启动新的搜索、不会重新拟合中心，也不会修改 d。它只把“深层孔壁足够”与
 * “深层被降噪/遮挡，但口沿 + 空洞 + 真实浅层孔壁足够”合成一个正式直孔存在性结论。
 */
static StraightHoleZuEvidence pingGuStraightHoleZuEvidence(
    const StraightWuLiVoidAudit& audit,
    const HoleHouXuanJuLei::Cluster& cluster,
    double resolvedRadius,
    const HoleMiaoshu& d,
    bool mouthSearchPersistentCoreOccupation,
    int mouthSearchCoreDepthBins,
    double mouthSearchCoreToAnnulusRatio)
{
    StraightHoleZuEvidence out;
    if (d.type != 1 || !(resolvedRadius > 1.0)) return out;

    out.deepEvidence = d.jointValidSlices >= 3
        || (d.measuredInnerRadiusValid && d.measuredInnerUsedSlices >= 3)
        || (d.observedBoreRadiusValid && d.observedBoreUsedSlices >= 3);

    out.visibleFraction = cluster.openArcStable
        // 周向资格已在候选层按“单段>=90° / 多段连续合计>=180°”审核；
        // 物理证据缩放使用实际总可见覆盖率，兼容多段残缺圆弧。
        ? std::clamp(cluster.meanCoverage, 0.20, 1.0)
        : std::clamp(cluster.meanCoverage, 0.20, 1.0);
    const double visibleScale = std::clamp(
        (out.visibleFraction - 0.20) / 0.80, 0.0, 1.0);
    out.annulusPointsMin = static_cast<int>(std::lround(18.0 + 14.0 * visibleScale));
    out.annulusSectorsMin = static_cast<int>(std::lround(7.0 + 3.0 * visibleScale));
    out.qianCengPointsMin = static_cast<int>(std::lround(8.0 + 4.0 * visibleScale));
    out.qianCengSectorsMin = static_cast<int>(std::lround(4.0 + 4.0 * visibleScale));
    out.qianCengDepthMin = static_cast<float>(0.24 + 0.06 * visibleScale);

    const bool strongSingleLayerOpenMouth = cluster.stable
        && cluster.openArcStable
        && cluster.supportLayers == 1
        && cluster.meanCoverage >= 0.25
        && cluster.meanContinuousArcCoverage >= 0.25
        && cluster.meanInteriorCleanRatio >= 0.90
        && cluster.meanFirstEdgeRatio >= 0.88
        && cluster.meanResidual <= std::max(0.30, 0.07 * resolvedRadius);
    out.candidateStable = cluster.stable
        && cluster.mouthAttachmentValid
        && !cluster.deepContinuation
        && ((cluster.supportLayers >= 2 && cluster.trajectoryContinuity >= 0.45)
            || strongSingleLayerOpenMouth);
    out.arcQualityValid = !cluster.openArcStable
        || (cluster.meanInteriorCleanRatio >= 0.80
            // 角度资格不在这里二次改写；这里只保留与 90° 最小覆盖一致的总覆盖下限。
            && cluster.meanCoverage >= 0.25);
    out.coreEmpty = audit.executed
        && !audit.rejectSolidCore
        && audit.coreOccupiedDepthBins <= 1
        && audit.coreToAnnulusRatio <= 0.06f
        && !mouthSearchPersistentCoreOccupation
        && mouthSearchCoreDepthBins <= 1
        && mouthSearchCoreToAnnulusRatio <= 0.08;
    out.visibleRingValid = audit.annulusPoints >= out.annulusPointsMin
        && audit.annulusSectors >= out.annulusSectorsMin;
    out.qianCengWallValid = audit.qianCengAnnulusPoints >= out.qianCengPointsMin
        && audit.qianCengAnnulusSectors >= out.qianCengSectorsMin
        && audit.qianCengAnnulusMaxDepth >= out.qianCengDepthMin;
    out.radiusAgreement = std::abs(static_cast<double>(d.rTop) - resolvedRadius);
    out.radiusAgreementLimit = std::max(
        1.00, 0.18 * static_cast<double>(std::max(d.rTop, 1.0f)));

    out.shallowEvidence = out.candidateStable
        && out.arcQualityValid
        && out.coreEmpty
        && out.visibleRingValid
        && out.qianCengWallValid
        && out.radiusAgreement <= out.radiusAgreementLimit;

    // 严重残缺直孔可能只剩上表面上的真实孔口弧，扫描物理上没有任何可见孔壁。
    // 这种情况下不能虚构深度/下口，但只要开放圆弧本身稳定、核心为空、口沿环带真实、
    // 半径与正式几何一致，就允许保留“可测上口”的直孔结果。普通候选仍不会走该分支。
    const bool mouthOnlySeverePartialEvidence = strongSingleLayerOpenMouth
        && out.arcQualityValid
        && out.coreEmpty
        && out.visibleRingValid
        && out.radiusAgreement <= out.radiusAgreementLimit;
    out.valid = out.deepEvidence || out.shallowEvidence || mouthOnlySeverePartialEvidence;
    return out;
}


struct HoleKouSearchSeedSurfaceAnchor
{
    bool valid = false;
    Eigen::Vector3f point{0.0f, 0.0f, 0.0f};
    float planeW = 0.0f;
    float shiftMm = 0.0f;
    int support = 0;
    int sectors = 0;
};

/** 【函数导航】
 * 作用：估计“guJiMouthSearchSeedOuterSurfaceAnchor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static HoleKouSearchSeedSurfaceAnchor guJiMouthSearchSeedOuterSurfaceAnchor(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const ShouDongJuBuZuoBiao& globalFrame,
    const Eigen::Vector3f& seedWorld,
    float configuredMaxRadius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleKouSearchSeedSurfaceAnchor out;
    out.point = seedWorld;
    if (!cloud || cloud->empty() || !globalFrame.valid || !seedWorld.allFinite()) return out;

    const Eigen::Vector3f seedDelta = seedWorld - globalFrame.origin;
    const float seedU = seedDelta.dot(globalFrame.u);
    const float seedV = seedDelta.dot(globalFrame.v);
    const float seedW = seedDelta.dot(globalFrame.n);

    const float maxR = std::clamp(configuredMaxRadius, 2.0f, 30.0f);
    const float innerRadial = std::clamp(0.25f * maxR, 2.5f, 6.0f);
    const float outerRadial = std::clamp(std::max(14.0f, maxR + 6.0f), 14.0f, 36.0f);
    const float axialReach = std::clamp(std::max(14.0f, maxR + 6.0f), 14.0f, 36.0f);
    const float sphereRadius =
        std::sqrt(outerRadial * outerRadial + axialReach * axialReach) + 0.5f;

    /** 【类型导航注释】
     * MaoDianPoint：Hole 识别总编排中的自定义 结构体。
     * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct MaoDianPoint {
        float w = 0.0f;
        float u = 0.0f;
        float v = 0.0f;
        // 扇区只由点相对 seed 的 (u,v) 决定，与滑动 W 窗口无关。
        // 每个点的角向 sector 只计算一次，滑动窗口只维护增量计数；高密度局部 ROI 下
        // 这会把本应 O(N) 的窗口扫描放大成 O(N*K)。预先计算后可用计数器增删点，
        // 保持完全相同的 24 扇区判定语义，同时把该段恢复为线性滑窗。
        unsigned char sector = 0U;
    };
    static thread_local std::vector<MaoDianPoint> points;
    static thread_local std::vector<int> indices;
    static thread_local std::vector<float> squaredDistances;
    points.clear();
    indices.clear();
    squaredDistances.clear();
    if (points.capacity() < 4096U) points.reserve(4096U);

    auto consider = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const Eigen::Vector3f world(p.x, p.y, p.z);
        const Eigen::Vector3f delta = world - globalFrame.origin;
        const float u = delta.dot(globalFrame.u);
        const float v = delta.dot(globalFrame.v);
        const float w = delta.dot(globalFrame.n);
        const float du = u - seedU;
        const float dv = v - seedV;
        const float radial = std::hypot(du, dv);
        if (radial < innerRadial || radial > outerRadial) return;
        if (std::abs(w - seedW) > axialReach) return;
        points.push_back({w, u, v, 0U});
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = seedWorld.x();
        query.y = seedWorld.y();
        query.z = seedWorld.z();
        RoiRuntime::WuPaiXuKdShouHu unsortedGuard(kdtree);
        if (kdtree->radiusSearch(query, sphereRadius, indices, squaredDistances) > 0) {
            for (int index : indices) consider(index);
        }
    } else {
        for (std::size_t index = 0; index < cloud->size(); ++index)
            consider(static_cast<int>(index));
    }
    if (points.size() < 80U) return out;

    std::sort(points.begin(), points.end(), [](const MaoDianPoint& a, const MaoDianPoint& b) {
        if (a.w != b.w) return a.w < b.w;
        if (a.u != b.u) return a.u < b.u;
        return a.v < b.v;
    });

    constexpr float windowWidth = 0.36f;
    std::size_t right = 0U;
    std::size_t maxSupport = 0U;
    for (std::size_t left = 0U; left < points.size(); ++left) {
        if (right < left) right = left;
        while (right < points.size() && points[right].w < points[left].w + windowWidth) ++right;
        maxSupport = std::max(maxSupport, right - left);
    }
    if (maxSupport < 30U) return out;

    // 扇区与 W 滑窗无关，只需对每个候选点计算一次。放在 maxSupport 早退之后，
    // 使稀疏/明显无支撑的失败 ROI 连这一次 atan2 都不承担。
    constexpr int sectorTotal = 24;
    for (MaoDianPoint& point : points) {
        const float du = point.u - seedU;
        const float dv = point.v - seedV;
        float angle = std::atan2(dv, du);
        if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
        int sector = static_cast<int>(std::floor(
            angle / static_cast<float>(2.0 * M_PI) * sectorTotal));
        sector = std::clamp(sector, 0, sectorTotal - 1);
        point.sector = static_cast<unsigned char>(sector);
    }

    // 外表面点在残缺孔附近可能远少于深层底面点，因此不能再要求达到“最密层的 50%”。
    // 使用绝对且随局部样本量缓慢增长的门槛，再叠加周向扇区覆盖，优先选择最高可靠支撑面。
    // 这使锥壁/孔底拾取点仍能回锚到真正上表面，而不是被最密的底部半圆劫持。
    const std::size_t supportGate = std::max<std::size_t>(
        64U, std::min<std::size_t>(
            200U, static_cast<std::size_t>(
                std::ceil(0.04 * static_cast<double>(points.size())))));
    /** 【类型导航注释】
     * ChuangKouXuanZe：Hole 识别总编排中的自定义 结构体。
     * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct ChuangKouXuanZe {
        bool valid = false;
        std::size_t begin = 0U;
        std::size_t end = 0U;
        float centerW = 0.0f;
        int sectors = 0;
    } best;

    right = 0U;
    std::array<int, sectorTotal> sectorCounts{};
    int coveredSectors = 0;
    for (std::size_t left = 0U; left < points.size(); ++left) {
        if (right < left) right = left;
        while (right < points.size() && points[right].w < points[left].w + windowWidth) {
            const std::size_t sector = static_cast<std::size_t>(points[right].sector);
            if (sectorCounts[sector]++ == 0) ++coveredSectors;
            ++right;
        }
        const std::size_t support = right - left;
        if (support >= supportGate && coveredSectors >= 8) {
            const int sectors = coveredSectors;
            const float centerW = 0.5f * (points[left].w + points[right - 1U].w);
            if (!best.valid || centerW > best.centerW
                || (centerW == best.centerW && sectors > best.sectors)) {
                best.valid = true;
                best.begin = left;
                best.end = right;
                best.centerW = centerW;
                best.sectors = sectors;
            }
        }

        // 下一次 left 前移时，从当前滑窗移除这一点。由于 windowWidth>0，
        // points[left] 一定已进入 [left,right)；因此增量计数与“当前窗口重新统计 sectorHit”的数学语义一致。
        if (left < right) {
            const std::size_t sector = static_cast<std::size_t>(points[left].sector);
            if (--sectorCounts[sector] == 0) --coveredSectors;
        }
    }
    if (!best.valid) return out;

    static thread_local std::vector<float> wScratch;
    wScratch.clear();
    wScratch.reserve(best.end - best.begin);
    for (std::size_t i = best.begin; i < best.end; ++i) wScratch.push_back(points[i].w);
    const float planeW = shouDongZhongWeiShu(wScratch);
    const float shift = planeW - seedW;

    const float maximumForwardShift = std::clamp(std::max(12.0f, maxR + 4.0f), 12.0f, 34.0f);
    if (!std::isfinite(planeW) || shift < -0.65f || shift > maximumForwardShift) return out;

    out.planeW = planeW;
    out.shiftMm = shift;
    out.support = static_cast<int>(best.end - best.begin);
    out.sectors = best.sectors;
    out.point = seedWorld + globalFrame.n * shift;
    return out;
}


// 【困难场景的第二级初始粗法线：主平面法线】
// 这一步仍然属于“找 Hole 之前”的搜索坐标准备，不是最终角度测量。
// 常规的局部粗法线会直接对 seed 邻域做稳健平面拟合；但当孔沿、孔壁或边界点占比过高时，
// 这些点可能让普通局部平面失真。只有在“当前没有可靠 Hole 口”或“局部法线与全局方向差异很大”时，
// 才启用这里较重的 RANSAC：先从局部 ROI 分离占比最大的板面，再用 PCA 精修这个板面的法线。
// 因此它与常规局部粗法线不是每个 seed 并行执行的两个精拟合器，而是困难场景下的低频主平面观察。
// 后续一旦 Hole 口椭圆建立，最终角度仍由椭圆反推链负责；这里的法线不会拥有最终法线解释权。
struct HoleChuShiFaXianZhuPingMian
{
    bool valid = false;
    Eigen::Vector3f point{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    int support = 0;
    int sectors = 0;
    float rmse = 0.0f;
    float deltaDegrees = 0.0f;
};

/**
 * 【疑难 seed 的第二级主平面粗法线】
 * 目的仍然只是建立搜索 frame，不是最终法线。
 * 与常规 guJiHoleChuShiJuBuFaXian() 的区别：常规方法对邻域点整体做稳健 PCA；本方法先在 ROI 中用
 * PCL SAC_RANSAC 分割“占主导的工件表面”，再用 PCA 精修，因此更能抵抗 Hole 壁、孔缘、杂点把局部平面拉偏。
 * 调用条件：仅当常规 frame 未形成可靠 Hole 族，或常规局部法线相对全局粗法线偏差 >= 10° 时才尝试。
 * 因此正常 seed 不会同时完整支付两套法线成本；保留它主要用于高倾角、边界或孔壁污染场景。
 */
static HoleChuShiFaXianZhuPingMian guJiHoleChuShiZhuPingMianFaXian(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const ShouDongJuBuZuoBiao& globalFrame,
    const Eigen::Vector3f& seedWorld,
    float configuredMaxRadius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleChuShiFaXianZhuPingMian out;
    if (!cloud || cloud->empty() || !globalFrame.valid || !seedWorld.allFinite()) return out;

    Eigen::Vector3f reference = globalFrame.n;
    if (!reference.allFinite() || reference.norm() < 1e-6f) reference = Eigen::Vector3f::UnitZ();
    reference.normalize();
    if (reference.z() < 0.0f) reference = -reference;
    out.normal = reference;

    const float maxR = std::clamp(configuredMaxRadius, 2.0f, 30.0f);
    // 默认最大孔半径 10 mm 时使用 18.5 mm 支撑邻域，并随最大孔径自适应。
    // 这是“估板面”的邻域，不改变正式 25/30 mm 孔口 ROI，也不扩大到整云。
    const float supportRadius = std::clamp(std::max(18.5f, maxR + 8.5f), 18.5f, 30.0f);

    static thread_local std::vector<int> indices;
    static thread_local std::vector<float> squaredDistances;
    indices.clear();
    squaredDistances.clear();
    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = seedWorld.x(); query.y = seedWorld.y(); query.z = seedWorld.z();
        RoiRuntime::WuPaiXuKdShouHu unsortedGuard(kdtree);
        kdtree->radiusSearch(query, supportRadius, indices, squaredDistances);
    } else {
        const float r2 = supportRadius * supportRadius;
        indices.reserve(cloud->size());
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = (*cloud)[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const Eigen::Vector3f q(p.x, p.y, p.z);
            if ((q - seedWorld).squaredNorm() <= r2) indices.push_back(static_cast<int>(i));
        }
    }
    if (indices.size() < 120U) return out;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local(new pcl::PointCloud<pcl::PointXYZRGB>());
    local->reserve(indices.size());
    for (int index : indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) continue;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        local->push_back(p);
    }
    if (local->size() < 120U) return out;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(160);
    // 只用于分离主板面，阈值对应线激光板面噪声量级，不参与孔径/孔深判断。
    seg.setDistanceThreshold(0.18);
    seg.setInputCloud(local);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    seg.segment(inliers, coefficients);
    if (inliers.indices.size() < 100U || coefficients.values.size() < 4U) return out;

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int index : inliers.indices) {
        const auto& p = (*local)[static_cast<std::size_t>(index)];
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= static_cast<float>(inliers.indices.size());

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int index : inliers.indices) {
        const auto& p = (*local)[static_cast<std::size_t>(index)];
        const Eigen::Vector3f d = Eigen::Vector3f(p.x, p.y, p.z) - centroid;
        covariance.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) return out;
    Eigen::Vector3f normal = solver.eigenvectors().col(0);
    if (!normal.allFinite() || normal.norm() < 1e-6f) return out;
    normal.normalize();
    if (normal.dot(reference) < 0.0f) normal = -normal;
    if (normal.z() < 0.0f) normal = -normal;

    const float cosine = std::clamp(std::abs(normal.dot(reference)), 0.0f, 1.0f);
    const float deltaDeg = std::acos(cosine) * 180.0f / static_cast<float>(M_PI);
    // 结构面保护允许较大板面倾角，但排除近乎正交的孔壁或侧壁。
    if (deltaDeg > 40.0f) return out;

    double squaredError = 0.0;
    for (int index : inliers.indices) {
        const auto& p = (*local)[static_cast<std::size_t>(index)];
        const Eigen::Vector3f q(p.x, p.y, p.z);
        const double distance = static_cast<double>((q - centroid).dot(normal));
        squaredError += distance * distance;
    }
    const float rmse = static_cast<float>(std::sqrt(
        squaredError / static_cast<double>(std::max<std::size_t>(1U, inliers.indices.size()))));

    const Eigen::Vector3f helper = std::abs(normal.x()) < 0.85f
        ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
    const Eigen::Vector3f axisU = normal.cross(helper).normalized();
    const Eigen::Vector3f axisV = normal.cross(axisU).normalized();
    constexpr int kSectors = 36;
    std::array<unsigned char, kSectors> sectorHit{};
    for (int index : inliers.indices) {
        const auto& p = (*local)[static_cast<std::size_t>(index)];
        const Eigen::Vector3f d = Eigen::Vector3f(p.x, p.y, p.z) - seedWorld;
        const float u = d.dot(axisU);
        const float v = d.dot(axisV);
        const float radial = std::hypot(u, v);
        if (radial < 0.15f * supportRadius || radial > supportRadius) continue;
        float angle = std::atan2(v, u);
        if (angle < 0.0f) angle += 2.0f * static_cast<float>(M_PI);
        int sector = static_cast<int>(std::floor(
            angle / (2.0f * static_cast<float>(M_PI)) * static_cast<float>(kSectors)));
        sector = std::clamp(sector, 0, kSectors - 1);
        sectorHit[static_cast<std::size_t>(sector)] = 1U;
    }
    const int sectors = static_cast<int>(std::count(
        sectorHit.begin(), sectorHit.end(), static_cast<unsigned char>(1U)));
    // 主平面至少需要半圈角向证据；扫描边界允许不是完整 360°。
    if (sectors < 18 || rmse > 0.25f) return out;

    const float signedDistance = (seedWorld - centroid).dot(normal);
    out.point = seedWorld - normal * signedDistance;
    out.normal = normal;
    out.support = static_cast<int>(inliers.indices.size());
    out.sectors = sectors;
    out.rmse = rmse;
    out.deltaDegrees = deltaDeg;
    return out;
}

/** 【函数导航】
 * 作用：执行“mouthSearchCollectWorldSamples”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static void mouthSearchCollectWorldSamples(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& center,
    float radius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree,
    std::vector<HoleFaXian::DianYangBen>& out,
    std::vector<HoleFaXian::DianYangBen>* distanceOrderedOut = nullptr)
{
    out.clear();
    if (distanceOrderedOut) distanceOrderedOut->clear();
    if (!cloud || cloud->empty() || !center.allFinite() || !(radius > 0.0f)) return;
    const float radiusSquared = radius * radius;
    auto append = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const Eigen::Vector3f point(p.x, p.y, p.z);
        if ((point - center).squaredNorm() > radiusSquared) return;
        HoleFaXian::DianYangBen sample;
        sample.point = point.cast<double>();
        sample.originalIndex = index;
        out.push_back(sample);
    };
    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = center.x(); query.y = center.y(); query.z = center.z();
        static thread_local std::vector<int> indices;
        static thread_local std::vector<float> squaredDistances;
        indices.clear();
        squaredDistances.clear();
        auto* mutableTree = const_cast<pcl::KdTreeFLANN<pcl::PointXYZRGB>*>(kdtree);
        const bool oldSortedResults = mutableTree->getSortedResults();
        if (distanceOrderedOut && !oldSortedResults) mutableTree->setSortedResults(true);
        const int found = kdtree->radiusSearch(query, radius, indices, squaredDistances);
        if (distanceOrderedOut && !oldSortedResults) mutableTree->setSortedResults(false);
        if (found > 0) {
            if (out.capacity() < indices.size()) out.reserve(indices.size());
            for (int index : indices) append(index);

            // PCL 在 sortedResults=true 时这里仍保持 LianXuZhuiWeizi 使用过的距离顺序。
            // 正式主流程随后会按 originalIndex 排序；锥孔末端如果需要恢复
            // LianXuZhuiWeizi 连续位姿效果，就在排序前复制这一份局部视图。这样只做
            // 一次 KD 查询，不回退当前 ROI 快路径，也不需要第二次排序。
            if (distanceOrderedOut) {
                distanceOrderedOut->assign(out.begin(), out.end());
            }
        }
    } else {
        if (out.capacity() < cloud->size()) out.reserve(cloud->size());
        for (std::size_t index = 0; index < cloud->size(); ++index)
            append(static_cast<int>(index));
        if (distanceOrderedOut) {
            // 没有 KDTree 时 LianXuZhuiWeizi 本来就是按点云存储顺序扫描，保持相同行为。
            distanceOrderedOut->assign(out.begin(), out.end());
        }
    }
    std::sort(out.begin(), out.end(), [](const auto& left, const auto& right) {
        return left.originalIndex < right.originalIndex;
    });
}


/** 【函数导航】
 * 作用：把 HoleWeizi 位姿/孔壁测量结果提交回最终 Hole 描述。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：每个 seed 完成 canonical FinalGeometry 与孔型初审之后调用一次。
 *
 * 几何所有权约定：
 * - rTop / rBot / depth / type 由前面的正式几何与孔型链锁定，本函数只能提供位姿证据，
 *   不能重新改变这些物理语义。
 * - finalSurface / finalAxis 可以由外表面、孔壁、椭圆或连续锥壁证据继续精修。
 * - centerTop 的平面内位置允许 HoleWeizi 利用孔壁继续修正；但“机械上口位于哪个轴向平面”
 *   永远由进入本函数前的 canonical mouth 决定。内喉、深层孔壁只能作为测量证据，不能
 *   把内部截面中心直接写成机械上口中心。
 *
 * 维护提示：这里是上口 Z 语义的唯一提交边界。若以后增加新的孔壁/位姿算法，应继续遵守
 * “canonical 决定上口平面、pose 决定轴和切向中心”的规则，避免同一文件内出现两套 Z 定义。
 */
static void wallGeometryFinalizeMeasuredWallGeometry(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree,
    HoleMiaoshu& d,
    std::vector<FenxiCacheWeiziPairCacheEntry>* posePairs,
    bool directMouthNormalOwned)
{

    const int lockedType = d.type;
    const float lockedRTop = d.rTop;
    const float lockedRadius = d.radius;
    const float lockedRBot = d.rBot;
    const float lockedDepth = d.depth;
    const float lockedDepthMeasured = d.depthMeasured;
    const float lockedDepthFinal = d.depthFinal;
    const float lockedRBotProfile = d.rBotProfile;
    const float lockedProfileBasedDr = d.profileBasedDr;
    const float lockedSlopeDeg = d.slopeDeg;
    const bool lockedReliableBottomRadius = d.reliableBottomRadius;
    const bool previousFinalRadiusLocked = d.finalRadiusLocked;
    const float previousFinalRadiusLockedValue = d.finalRadiusLockedValue;
    const std::string previousFinalRadiusLockSource = d.finalRadiusLockSource;

    auto restoreSemanticGeometry = [&]() {
        d.type = lockedType;
        d.rTop = lockedRTop;
        d.radius = lockedRadius;
        d.rBot = lockedRBot;
        d.depth = lockedDepth;
        d.depthMeasured = lockedDepthMeasured;
        d.depthFinal = lockedDepthFinal;
        d.rBotProfile = lockedRBotProfile;
        d.profileBasedDr = lockedProfileBasedDr;
        d.slopeDeg = lockedSlopeDeg;
        d.reliableBottomRadius = lockedReliableBottomRadius;
        d.finalRadiusLocked = previousFinalRadiusLocked || lockedRTop > 0.0f;
        d.finalRadiusLockedValue = previousFinalRadiusLocked
            ? previousFinalRadiusLockedValue : lockedRTop;
        d.finalRadiusLockSource = previousFinalRadiusLockSource.empty()
            ? "FinalGeometryTopR" : previousFinalRadiusLockSource;
    };

    d.refinedSurfaceNormalValid = false;
    d.refinedSurfaceNx = 0.0f;
    d.refinedSurfaceNy = 0.0f;
    d.refinedSurfaceNz = 1.0f;
    d.refinedSurfaceTiltDeg = 0.0f;
    d.refinedSurfaceSupportPts = 0;
    d.refinedSurfaceCoveredSectors = 0;
    d.refinedSurfaceCoverage = 0.0f;
    d.refinedSurfaceFitRmse = 0.0f;
    d.refinedSurfaceSource = "WallEvidence_SURFACE_EVIDENCE_UNAVAILABLE";

    d.holeAxisInValid = false;
    d.holeAxisInNx = 0.0f;
    d.holeAxisInNy = 0.0f;
    d.holeAxisInNz = -1.0f;
    d.holeAxisTiltDeg = 0.0f;
    d.holeAxisSource = "WallEvidence_WALL_SURFACE_EVIDENCE_WEAK";

    d.wallAxisValidSlices = 0;
    d.wallAxisIterations = 0;
    d.wallAxisCenterRmse = 0.0f;
    d.wallAxisMeanSliceRmse = 0.0f;
    d.wallAxisMeanCoverage = 0.0f;
    d.wallAxisRadiusSlope = 0.0f;
    d.wallAxisRadiusMonotonicity = 0.0f;
    d.wallAxisCorrectionDeg = 0.0f;
    d.wallAxisConfidence = 0.0f;

    d.jointGeometryExecuted = false;
    d.jointGeometryAxisValid = false;
    d.jointGeometryMouthValid = false;
    d.jointWallCandidatePoints = 0;
    d.jointWallNormalPoints = 0;
    d.jointWallSectors = 0;
    d.jointValidSlices = 0;
    d.jointWallCoverage = 0.0f;
    d.jointWallSymmetry = 0.0f;
    d.jointGlobalPriorWeight = 0.0f;
    d.jointCenterEvidenceWeight = 0.0f;
    d.jointAxisCorrectionDeg = 0.0f;
    d.jointMeanSliceRmse = 0.0f;
    d.jointMeanSliceCoverage = 0.0f;
    d.jointCenterSpread = 0.0f;
    d.jointAllCenterSpread = 0.0f;
    d.jointAxisInlierThreshold = 0.0f;
    d.jointAxisUsedSlices = 0;
    d.jointAxisRejectedSlices = 0;
    d.jointTransitionSlices = 0;
    d.jointStableStartSlice = 0;
    d.jointCenterShift = 0.0f;
    d.jointRadiusShift = 0.0f;
    d.jointWallRadiusAtZero = 0.0f;
    d.jointLockedMouthDepth = 0.0f;
    d.jointSemanticLockPreserved = true;
    d.jointRadiusSlope = 0.0f;
    d.jointConfidence = 0.0f;
    d.jointGeometrySource = "WallEvidence_NOT_EXECUTED";
    d.coarseCenterExecuted = false;
    d.coarseCenterValid = false;
    d.coarseCenterShift = 0.0f;
    d.coarseBoundaryRadius = 0.0f;
    d.coarseBoundaryMad = 0.0f;
    d.coarseBoundaryCoverage = 0.0f;
    d.coarseInnerDensity = 0.0f;
    d.coarseAnnulusDensity = 0.0f;
    d.coarseSurfacePoints = 0;
    d.coarseEvaluatedCandidates = 0;
    d.coarseVisitedSurfacePoints = 0;
    d.coarseParallelWorkers = 1;
    d.measuredInnerRadiusValid = false;
    d.measuredInnerRadius = 0.0f;
    d.measuredInnerRadiusMad = 0.0f;
    d.measuredInnerDepthStart = 0.0f;
    d.measuredInnerDepthEnd = 0.0f;
    d.measuredInnerDepthSpan = 0.0f;
    d.measuredInnerCoverage = 0.0f;
    d.measuredInnerRmse = 0.0f;
    d.measuredInnerCenterLineRmse = 0.0f;
    d.measuredInnerCandidates = 0;
    d.measuredInnerFamilies = 0;
    d.measuredInnerUsedSlices = 0;
    d.measuredInnerHistogramWindows = 0;
    d.measuredInnerHistogramPointVisits = 0;
    d.measuredInnerFitPointVisits = 0;
    d.measuredInnerDecision = "NOT_EXECUTED";
    d.rawConeWallExecuted = false;
    d.rawConeWallValid = false;
    d.rawConeWallCells = 0;
    d.rawConeWallSectors = 0;
    d.rawConeWallDepthBins = 0;
    d.rawConeWallRoiPoints = 0;
    d.rawConeWallAxisEvaluations = 0;
    d.rawConeWallParallelWorkers = 1;
    d.rawConeWallRmse = 0.0f;
    d.rawConeWallScale = 0.0f;
    d.rawConeWallNoDriftRmse = 0.0f;
    d.rawConeWallBaselineEvenRmse = 0.0f;
    d.rawConeWallBaselineOddRmse = 0.0f;
    d.rawConeWallEvenRmse = 0.0f;
    d.rawConeWallOddRmse = 0.0f;
    d.rawConeWallEvenImprovement = 0.0f;
    d.rawConeWallOddImprovement = 0.0f;
    d.rawConeWallImprovement = 0.0f;
    d.rawConeWallAxisCorrectionDeg = 0.0f;
    d.rawConeWallCenterShift = 0.0f;
    d.rawConeWallDecision = "NOT_EXECUTED";
    d.apertureContourExecuted = false;
    d.apertureContourValid = false;
    d.apertureContourPoints = 0;
    d.apertureContourCoverage = 0.0f;
    d.apertureContourDepthMin = 0.0f;
    d.apertureContourDepthMax = 0.0f;
    d.apertureContourDepthMedian = 0.0f;
    d.apertureContourDepthSpan = 0.0f;
    d.apertureContourRadiusMin = 0.0f;
    d.apertureContourRadiusMax = 0.0f;
    d.aperturePlanarRingPlaneResidualMax = 0.0f;
    d.apertureContourPlaneResidualMax = 0.0f;
    d.apertureContourConfidence = 0.0f;
    d.apertureContourCanonicalRadius = 0.0f;
    d.apertureContourEvidenceRadius = 0.0f;
    d.apertureContourEvidenceScale = 0.0f;
    d.apertureContourEvidenceCenterShift = 0.0f;
    d.apertureContourEvidenceRawSectors = 0;
    d.apertureContourEvidenceUsedSectors = 0;
    d.apertureContourEvidenceSupportCount = 0;
    d.apertureContourWorld.fill(Eigen::Vector3f::Zero());
    d.apertureContourMask.fill(0U);

    Eigen::Vector3f recognitionSurface(
        d.localPlaneNx, d.localPlaneNy, d.localPlaneNz);
    if (!recognitionSurface.allFinite() || recognitionSurface.norm() < 1e-6f)
        recognitionSurface = Eigen::Vector3f::UnitZ();
    else
        recognitionSurface.normalize();

    auto keepRecognitionAxis = [&]() {
        if (!HoleJiXing::known(d.inwardPolarity)) return;
        Eigen::Vector3f axis = recognitionSurface
            * static_cast<float>(HoleJiXing::signOrZero(d.inwardPolarity));
        if (!axis.allFinite() || axis.norm() < 1e-6f) return;
        axis.normalize();
        d.holeAxisInValid = true;
        d.holeAxisInNx = axis.x();
        d.holeAxisInNy = axis.y();
        d.holeAxisInNz = axis.z();
        d.holeAxisTiltDeg = std::acos(std::clamp(
            std::abs(axis.z()), 0.0f, 1.0f))
            * 180.0f / static_cast<float>(M_PI);
        d.holeAxisSource = "WallEvidence_KEEP_RECOGNITION_AXIS";
    };

    if (!HoleJiXing::known(d.inwardPolarity) || !cloud || cloud->empty()
        || !d.centerTop.allFinite() || !(lockedRTop > 0.5f)
        || (lockedType != 1 && lockedType != 2)) {
        keepRecognitionAxis();
        restoreSemanticGeometry();
        return;
    }

    static thread_local std::vector<HoleFaXian::DianYangBen> samples;
    static thread_local std::vector<HoleFaXian::DianYangBen> continuousConePoseConeSamples;
    samples.clear();
    continuousConePoseConeSamples.clear();
    const float queryRadius = std::max(lockedRTop + 8.0f,
        2.45f * lockedRTop + 2.0f);
    mouthSearchCollectWorldSamples(
        cloud, d.centerTop, queryRadius, kdtree, samples,
        lockedType == 2 ? &continuousConePoseConeSamples : nullptr);

    const Eigen::Vector3f originalCenterTop = d.centerTop;
    LianXuZhuiWeizi::Result continuousConePose;
    HoleWeiziFinal::Result fit;

    const Eigen::Vector3d mouthKey = d.centerTop.cast<double>();
    const Eigen::Vector3d surfaceKey = recognitionSurface.cast<double>();
    const std::uint64_t mouthX = HoleFastPath::doubleBits(mouthKey.x());
    const std::uint64_t mouthY = HoleFastPath::doubleBits(mouthKey.y());
    const std::uint64_t mouthZ = HoleFastPath::doubleBits(mouthKey.z());
    const std::uint64_t surfaceX =
        HoleFastPath::doubleBits(surfaceKey.x());
    const std::uint64_t surfaceY =
        HoleFastPath::doubleBits(surfaceKey.y());
    const std::uint64_t surfaceZ =
        HoleFastPath::doubleBits(surfaceKey.z());
    const std::uint64_t topR = HoleFastPath::doubleBits(
        static_cast<double>(lockedRTop));
    const std::uint64_t botR = HoleFastPath::doubleBits(
        static_cast<double>(lockedRBot));
    const std::uint64_t depth = HoleFastPath::doubleBits(
        static_cast<double>(lockedDepth));
    std::uint64_t hash = 14695981039346656037ULL;
    for (const HoleFaXian::DianYangBen& sample : samples) {
        const std::uint32_t bits =
            static_cast<std::uint32_t>(sample.originalIndex);
        for (int byteIndex = 0; byteIndex < 4; ++byteIndex) {
            hash ^= static_cast<std::uint8_t>(bits >> (byteIndex * 8));
            hash *= 1099511628211ULL;
        }
    }
    bool posePairHit = false;
    FenxiCacheWeiziPairCacheEntry* hitEntry = nullptr;
    if (posePairs) {
        for (FenxiCacheWeiziPairCacheEntry& entry : *posePairs) {
            if (entry.sampleCount != samples.size()
                || entry.sampleIndexHash != hash
                || entry.mouthXBits != mouthX || entry.mouthYBits != mouthY
                || entry.mouthZBits != mouthZ
                || entry.surfaceXBits != surfaceX
                || entry.surfaceYBits != surfaceY
                || entry.surfaceZBits != surfaceZ
                || entry.topRadiusBits != topR
                || entry.bottomRadiusBits != botR
                || entry.depthBits != depth
                || entry.polarity != d.inwardPolarity
                || entry.holeType != lockedType
                || entry.originalIndices.size() != samples.size()) {
                continue;
            }
            bool sameIndices = true;
            for (std::size_t i = 0; i < samples.size(); ++i) {
                if (entry.originalIndices[i] != samples[i].originalIndex) {
                    sameIndices = false;
                    break;
                }
            }
            if (!sameIndices) continue;
            posePairHit = true;
            hitEntry = &entry;
            break;
        }
    }

    if (posePairHit) {
        fit = hitEntry->basePose;
        continuousConePose = hitEntry->continuousConePose;
    } else {

        std::future<LianXuZhuiWeizi::Result> conePoseFuture;
        if (lockedType == 2) {
            const Eigen::Vector3d continuousConePoseMouth = d.centerTop.cast<double>();
            const Eigen::Vector3d continuousConePoseSurface = recognitionSurface.cast<double>();
            const double continuousConePoseTopR = static_cast<double>(lockedRTop);
            const double continuousConePoseBotR = static_cast<double>(lockedRBot);
            const double continuousConePoseDepth = static_cast<double>(lockedDepth);
            const int continuousConePosePolarity = d.inwardPolarity;
            const int continuousConePoseType = lockedType;
            // LianXuZhuiWeizi 的连续锥位姿使用 KD 半径查询原始的距离顺序；
            // 主 HoleWeiziFinal 仍使用当前按 originalIndex 整理后的 samples。
            // 两个估计器继续并发执行，因此直孔零额外开销，锥孔只增加一次
            // 局部 vector 复制，不增加 KD 查询，也不回退现有性能优化。
            const std::vector<HoleFaXian::DianYangBen>& concurrentConeSamples =
                continuousConePoseConeSamples.empty() ? samples : continuousConePoseConeSamples;
            conePoseFuture = std::async(std::launch::async,
                [&concurrentConeSamples, continuousConePoseMouth, continuousConePoseSurface, continuousConePoseTopR,
                 continuousConePoseBotR, continuousConePoseDepth, continuousConePosePolarity, continuousConePoseType]() {
                    return LianXuZhuiWeizi::estimate(
                        concurrentConeSamples, continuousConePoseMouth, continuousConePoseSurface,
                        continuousConePoseTopR, continuousConePoseBotR, continuousConePoseDepth,
                        continuousConePosePolarity, continuousConePoseType);
                });
        }
        fit = HoleWeiziFinal::estimate(
            samples,
            d.centerTop.cast<double>(),
            recognitionSurface.cast<double>(),
            lockedRTop,
            lockedRBot,
            lockedDepth,
            d.inwardPolarity,
            lockedType);
        if (conePoseFuture.valid()) continuousConePose = conePoseFuture.get();

        if (posePairs) {
            if (posePairs->size() >= 512U) posePairs->erase(posePairs->begin());
            FenxiCacheWeiziPairCacheEntry entry;
            entry.sampleCount = samples.size();
            entry.sampleIndexHash = hash;
            entry.mouthXBits = mouthX;
            entry.mouthYBits = mouthY;
            entry.mouthZBits = mouthZ;
            entry.surfaceXBits = surfaceX;
            entry.surfaceYBits = surfaceY;
            entry.surfaceZBits = surfaceZ;
            entry.topRadiusBits = topR;
            entry.bottomRadiusBits = botR;
            entry.depthBits = depth;
            entry.polarity = d.inwardPolarity;
            entry.holeType = lockedType;
            entry.originalIndices.reserve(samples.size());
            for (const HoleFaXian::DianYangBen& sample : samples)
                entry.originalIndices.push_back(sample.originalIndex);
            entry.basePose = fit;
            entry.continuousConePose = continuousConePose;
            posePairs->push_back(std::move(entry));

            hitEntry = &posePairs->back();
        }
    }
    d.coarseCenterExecuted = fit.coarseCenter.executed;
    d.coarseCenterValid = fit.coarseCenter.valid;
    d.coarseCenterShift = static_cast<float>(fit.coarseCenter.shift);
    d.coarseBoundaryRadius = static_cast<float>(fit.coarseCenter.boundaryRadius);
    d.coarseBoundaryMad = std::isfinite(fit.coarseCenter.boundaryMad)
        ? static_cast<float>(fit.coarseCenter.boundaryMad) : 0.0f;
    d.coarseBoundaryCoverage = static_cast<float>(fit.coarseCenter.coverage);
    d.coarseInnerDensity = static_cast<float>(fit.coarseCenter.innerDensity);
    d.coarseAnnulusDensity = static_cast<float>(fit.coarseCenter.annulusDensity);
    d.coarseSurfacePoints = fit.coarseCenter.surfacePoints;
    d.coarseEvaluatedCandidates = fit.coarseCenter.evaluatedCandidates;
    d.coarseVisitedSurfacePoints = fit.coarseCenter.visitedSurfacePoints;
    d.coarseParallelWorkers = fit.coarseCenter.parallelWorkers;
    d.measuredInnerRadiusValid = fit.measuredInnerRadiusValid;
    d.measuredInnerRadius = static_cast<float>(fit.measuredInnerRadius);
    d.measuredInnerRadiusMad = static_cast<float>(fit.innerCylinder.radiusMad);
    d.measuredInnerDepthStart = static_cast<float>(fit.innerCylinder.depthStart);
    d.measuredInnerDepthEnd = static_cast<float>(fit.innerCylinder.depthEnd);
    d.measuredInnerDepthSpan = static_cast<float>(fit.innerCylinder.depthSpan);
    d.measuredInnerCoverage = static_cast<float>(fit.innerCylinder.meanCoverage);
    d.measuredInnerRmse = static_cast<float>(fit.innerCylinder.medianRmse);
    d.measuredInnerCenterLineRmse = static_cast<float>(fit.innerCylinder.centerLineRmse);
    d.measuredInnerCandidates = fit.innerCylinder.candidateCount;
    d.measuredInnerFamilies = fit.innerCylinder.familyCount;
    d.measuredInnerUsedSlices = fit.innerCylinder.usedSlices;
    d.measuredInnerHistogramWindows = fit.innerCylinder.histogramWindows;
    d.measuredInnerHistogramPointVisits = fit.innerCylinder.histogramPointVisits;
    d.measuredInnerFitPointVisits = fit.innerCylinder.fitPointVisits;
    d.measuredInnerDecision = fit.innerCylinder.decision ? fit.innerCylinder.decision : "UNKNOWN";
    d.rawConeWallExecuted = fit.rawCone.executed;
    d.rawConeWallValid = fit.rawCone.valid;
    d.rawConeWallCells = fit.rawCone.cellCount;
    d.rawConeWallSectors = fit.rawCone.coveredSectors;
    d.rawConeWallDepthBins = fit.rawCone.coveredDepthBins;
    d.rawConeWallRoiPoints = fit.rawCone.roiPoints;
    d.rawConeWallAxisEvaluations = fit.rawCone.axisEvaluations;
    d.rawConeWallParallelWorkers = fit.rawCone.parallelWorkers;
    d.rawConeWallRmse = static_cast<float>(fit.rawCone.rmse);
    d.rawConeWallScale = static_cast<float>(fit.rawCone.scale);
    d.rawConeWallNoDriftRmse = static_cast<float>(fit.rawCone.noDriftRmse);
    d.rawConeWallBaselineEvenRmse = static_cast<float>(fit.rawCone.baselineEvenRmse);
    d.rawConeWallBaselineOddRmse = static_cast<float>(fit.rawCone.baselineOddRmse);
    d.rawConeWallEvenRmse = static_cast<float>(fit.rawCone.evenRmse);
    d.rawConeWallOddRmse = static_cast<float>(fit.rawCone.oddRmse);
    d.rawConeWallEvenImprovement = static_cast<float>(fit.rawCone.evenImprovement);
    d.rawConeWallOddImprovement = static_cast<float>(fit.rawCone.oddImprovement);
    d.rawConeWallImprovement = static_cast<float>(fit.rawCone.improvement);
    d.coneFamilyConeFamilyAngleDeg = static_cast<float>(fit.coneFamilyConeFamilyAngleDeg);
    d.rawConeWallAxisCorrectionDeg = static_cast<float>(fit.rawCone.axisCorrectionDegrees);
    d.rawConeWallCenterShift = static_cast<float>(fit.rawCone.centerShift);
    d.rawConeWallDecision = fit.rawCone.decision ? fit.rawCone.decision : "UNKNOWN";

    d.jointGeometryExecuted = fit.executed;
    d.jointGeometryAxisValid = fit.valid;
    d.jointGeometryMouthValid = fit.valid;
    d.jointWallCandidatePoints = fit.wall.totalSupport;
    d.jointWallNormalPoints = fit.outer.supportCount;
    d.jointWallSectors = fit.wall.medianCoveredSectors;
    d.jointValidSlices = fit.wall.validSlices;
    d.jointWallCoverage = static_cast<float>(fit.wall.meanCoverage);
    d.jointWallSymmetry = static_cast<float>(fit.wall.radiusMad);
    d.jointGlobalPriorWeight = static_cast<float>(fit.wallRadiusWeight);
    d.jointCenterEvidenceWeight = static_cast<float>(fit.centerEvidenceWeight);
    d.jointAxisCorrectionDeg = static_cast<float>(fit.axisCorrectionDegrees);
    d.jointMeanSliceRmse = static_cast<float>(fit.wall.meanRmse);
    d.jointMeanSliceCoverage = static_cast<float>(fit.outer.coverage);
    d.jointCenterSpread = static_cast<float>(fit.wall.centerLineRmse);
    d.jointAllCenterSpread = static_cast<float>(fit.wall.allSliceCenterLineRmse);
    d.jointAxisInlierThreshold = static_cast<float>(fit.wall.axisInlierThreshold);
    d.jointAxisUsedSlices = fit.wall.axisUsedSlices;
    d.jointAxisRejectedSlices = fit.wall.axisRejectedSlices;
    d.jointTransitionSlices = fit.wall.transitionSlices;
    d.jointStableStartSlice = fit.wall.stableStartSlice;
    d.jointCenterShift = static_cast<float>(fit.centerShift);
    d.jointRadiusShift = 0.0f;
    d.jointWallRadiusAtZero = static_cast<float>(fit.wallRadiusAtZero);
    d.jointLockedMouthDepth = static_cast<float>(fit.lockedMouthDepth);
    d.jointSemanticLockPreserved = fit.semanticRadiusLocked
        && fit.topRadius == static_cast<double>(lockedRTop);
    d.jointRadiusSlope = static_cast<float>(fit.wall.radiusSlope);
    d.jointConfidence = static_cast<float>(fit.confidence);
    d.jointGeometrySource = fit.source
        ? fit.source : "WallEvidence_WALL_SURFACE_EVIDENCE_WEAK";
    if (lockedType == 1 && fit.innerCylinder.executed) {
        int innerSupport = 0;
        int innerSectors = 0;
        for (const auto& slice : fit.innerCylinder.slices) {
            innerSupport += slice.supportCount;
            innerSectors += slice.coveredSectors;
        }
        d.jointWallCandidatePoints = innerSupport;
        d.jointWallSectors = fit.innerCylinder.slices.empty() ? 0
            : innerSectors / static_cast<int>(fit.innerCylinder.slices.size());
        d.jointValidSlices = static_cast<int>(fit.innerCylinder.slices.size());
        d.jointWallCoverage = static_cast<float>(fit.innerCylinder.meanCoverage);
        d.jointWallSymmetry = static_cast<float>(fit.innerCylinder.radiusMad);
        d.jointMeanSliceRmse = static_cast<float>(fit.innerCylinder.medianRmse);
        d.jointCenterSpread = static_cast<float>(fit.innerCylinder.centerLineRmse);
        d.jointAllCenterSpread = static_cast<float>(fit.innerCylinder.centerLineRmse);
        d.jointAxisUsedSlices = fit.innerCylinder.usedSlices;
        d.jointAxisRejectedSlices = fit.innerCylinder.rejectedSlices;
        d.jointTransitionSlices = 0;
        d.jointStableStartSlice = 0;
        d.jointWallRadiusAtZero = static_cast<float>(fit.innerCylinder.radius);
        d.jointLockedMouthDepth = 0.0f;
        d.jointRadiusSlope = static_cast<float>(fit.innerCylinder.radiusSlope);
    }

    d.apertureContourExecuted = fit.contour.executed;
    d.apertureContourValid = fit.contour.valid;
    d.apertureContourPoints = fit.contour.validPoints;
    d.apertureContourCoverage = static_cast<float>(fit.contour.coverage);
    d.apertureContourDepthMin = static_cast<float>(fit.contour.minimumAxialDepth);
    d.apertureContourDepthMax = static_cast<float>(fit.contour.maximumAxialDepth);
    d.apertureContourDepthMedian = static_cast<float>(fit.contour.medianAxialDepth);
    d.apertureContourDepthSpan = static_cast<float>(fit.contour.axialSpan);
    d.apertureContourRadiusMin = static_cast<float>(fit.contour.minimumLocalRadius);
    d.apertureContourRadiusMax = static_cast<float>(fit.contour.maximumLocalRadius);
    d.aperturePlanarRingPlaneResidualMax = static_cast<float>(
        fit.contour.planarRingMaximumPlaneResidual);
    d.apertureContourPlaneResidualMax = static_cast<float>(
        fit.contour.maximumPlaneResidual);
    d.apertureContourConfidence = static_cast<float>(fit.contour.confidence);
    d.apertureContourCanonicalRadius = static_cast<float>(fit.contour.canonicalRadius);
    d.apertureContourEvidenceRadius = static_cast<float>(fit.contour.evidenceRadius);
    d.apertureContourEvidenceScale = std::isfinite(fit.contour.evidenceScale)
        ? static_cast<float>(fit.contour.evidenceScale) : 0.0f;
    d.apertureContourEvidenceCenterShift = static_cast<float>(fit.contour.evidenceCenterShift);
    d.apertureContourEvidenceRawSectors = fit.contour.evidenceRawSectors;
    d.apertureContourEvidenceUsedSectors = fit.contour.evidenceUsedSectors;
    d.apertureContourEvidenceSupportCount = fit.contour.evidenceSupportCount;
    for (std::size_t i = 0; i < fit.contour.points.size(); ++i) {
        const auto& point = fit.contour.points[i];
        d.apertureContourMask[i] = point.valid ? 1U : 0U;
        if (point.valid) {

            d.apertureContourWorld[i] = point.world.cast<float>().eval();
        } else {
            d.apertureContourWorld[i].setZero();
        }
    }

    if (lockedType == 1 && fit.innerCylinder.executed) {
        d.wallAxisValidSlices = static_cast<int>(fit.innerCylinder.slices.size());
        d.wallAxisIterations = fit.innerCylinder.valid ? 1 : 0;
        d.wallAxisCenterRmse = static_cast<float>(fit.innerCylinder.centerLineRmse);
        d.wallAxisMeanSliceRmse = static_cast<float>(fit.innerCylinder.medianRmse);
        d.wallAxisMeanCoverage = static_cast<float>(fit.innerCylinder.meanCoverage);
        d.wallAxisRadiusSlope = static_cast<float>(fit.innerCylinder.radiusSlope);
        d.wallAxisRadiusMonotonicity = 1.0f;
        d.wallAxisCorrectionDeg = static_cast<float>(fit.axisCorrectionDegrees);
        d.wallAxisConfidence = static_cast<float>(fit.confidence);
    } else {
        d.wallAxisValidSlices = fit.wall.validSlices;
        d.wallAxisIterations = fit.wall.valid ? 3 : 0;
        d.wallAxisCenterRmse = static_cast<float>(fit.wall.centerLineRmse);
        d.wallAxisMeanSliceRmse = static_cast<float>(fit.wall.meanRmse);
        d.wallAxisMeanCoverage = static_cast<float>(fit.wall.meanCoverage);
        d.wallAxisRadiusSlope = static_cast<float>(fit.wall.radiusSlope);
        d.wallAxisRadiusMonotonicity = static_cast<float>(fit.wall.monotonicity);
        d.wallAxisCorrectionDeg = static_cast<float>(fit.axisCorrectionDegrees);
        d.wallAxisConfidence = static_cast<float>(fit.wall.confidence);
    }


    if (fit.outer.valid) {
        d.refinedSurfaceNormalValid = true;
        d.refinedSurfaceNx = static_cast<float>(fit.outer.normal.x());
        d.refinedSurfaceNy = static_cast<float>(fit.outer.normal.y());
        d.refinedSurfaceNz = static_cast<float>(fit.outer.normal.z());
        d.refinedSurfaceTiltDeg = std::acos(std::clamp(
            std::abs(d.refinedSurfaceNz), 0.0f, 1.0f))
            * 180.0f / static_cast<float>(M_PI);
        d.refinedSurfaceSupportPts = fit.outer.supportCount;
        d.refinedSurfaceCoveredSectors = fit.outer.coveredSectors;
        d.refinedSurfaceCoverage = static_cast<float>(fit.outer.coverage);
        d.refinedSurfaceFitRmse = static_cast<float>(fit.outer.rmse);
        d.refinedSurfaceSource = fit.outer.source
            ? fit.outer.source : "WallEvidence_OUTER_SUPPORT";
    }

    d.balancedNormalBalancedNormalAttempted = false;
    d.balancedNormalBalancedNormalValid = false;
    d.balancedNormalBalancedNormalCoverage = 0.0f;
    d.balancedNormalBalancedNormalMaxGapDeg = 360.0f;
    d.balancedNormalBalancedNormalResidualMad = 0.0f;
    d.balancedNormalBalancedNormalDeltaDeg = 0.0f;
    d.balancedNormalBalancedNormalNx = 0.0f;
    d.balancedNormalBalancedNormalNy = 0.0f;
    d.balancedNormalBalancedNormalNz = 1.0f;
    d.balancedNormalBalancedNormalDecision = "NOT_ATTEMPTED";
    d.balancedNormalWallCorrectionLimitDeg = 0.0f;
    d.balancedNormalWallCorrectionAppliedDeg = 0.0f;
    Eigen::Vector3f guardSurface = fit.surfaceNormal.cast<float>();
    double balancedNormalWallLimit = 3.0;

    const auto computeBalancedNormal = [&]() {
        FenxiCacheBalancedNormalGuardResult outcome;
        pcl::PointXYZRGB centerPt;
        centerPt.x = d.centerTop.x();
        centerPt.y = d.centerTop.y();
        centerPt.z = d.centerTop.z();
        std::vector<int> indices;
        std::vector<float> sqDist;
        kdtree->radiusSearch(centerPt, 2.60 * d.rTop, indices, sqDist);
        std::vector<holeNormalPingHeng::Vec3> guardPts;
        guardPts.reserve(indices.size());
        for (int idx : indices) {
            const auto& p = (*cloud)[idx];
            guardPts.push_back({ p.x, p.y, p.z });
        }
        const holeNormalPingHeng::Vec3 cTop{ d.centerTop.x(), d.centerTop.y(), d.centerTop.z() };
        const holeNormalPingHeng::Vec3 inputN{ fit.outer.normal.x(), fit.outer.normal.y(), fit.outer.normal.z() };
        holeNormalPingHeng::Config cfg;
        const holeNormalPingHeng::Result ng = holeNormalPingHeng::estimateBalancedOuterNormal(
            guardPts, cTop, d.rTop, inputN, cfg);

        const bool guardApplies = ng.valid
            && ng.coverage >= 0.80
            && ng.maxGapDeg <= 60.0
            && ng.deltaFromInputDeg >= 0.25;
        outcome.ng = ng;
        outcome.guardApplies = guardApplies;
        if (guardApplies) {
            outcome.guardSurface =
                Eigen::Vector3d(ng.normal.x, ng.normal.y, ng.normal.z).cast<float>();
            outcome.guardSurface.normalize();
        }
        const double wallCoverage = (lockedType == 1)
            ? fit.innerCylinder.meanCoverage : fit.wall.meanCoverage;
        outcome.wallLimit =
            holeNormalPingHeng::maxAllowedWallCorrectionDeg(1.5, wallCoverage, ng);
        return outcome;
    };
    if (!directMouthNormalOwned && kdtree && d.rTop > 0.5f && fit.outer.valid) {
        FenxiCacheBalancedNormalGuardResult outcome;
        if (posePairHit && hitEntry && hitEntry->hasBalancedNormalGuard) {
            outcome = hitEntry->balancedNormalGuard;
        } else {
            outcome = computeBalancedNormal();
            if (hitEntry) {
                hitEntry->balancedNormalGuard = outcome;
                hitEntry->hasBalancedNormalGuard = true;
            }
        }
        d.balancedNormalBalancedNormalAttempted = true;
        d.balancedNormalBalancedNormalValid = outcome.ng.valid;
        d.balancedNormalBalancedNormalCoverage = static_cast<float>(outcome.ng.coverage);
        d.balancedNormalBalancedNormalMaxGapDeg = static_cast<float>(outcome.ng.maxGapDeg);
        d.balancedNormalBalancedNormalResidualMad = static_cast<float>(outcome.ng.residualMad);
        d.balancedNormalBalancedNormalDeltaDeg = static_cast<float>(outcome.ng.deltaFromInputDeg);
        d.balancedNormalBalancedNormalDecision = outcome.guardApplies
            ? (outcome.ng.decision ? outcome.ng.decision : "APPLY_BALANCED_OUTER_SURFACE_NORMAL")
            : (outcome.ng.valid ? "SKIP_INSUFFICIENT_DISAGREEMENT_OR_COVERAGE"
                                : (outcome.ng.decision ? outcome.ng.decision : "INVALID"));
        if (outcome.guardApplies) {
            guardSurface = outcome.guardSurface;
            d.balancedNormalBalancedNormalNx = guardSurface.x();
            d.balancedNormalBalancedNormalNy = guardSurface.y();
            d.balancedNormalBalancedNormalNz = guardSurface.z();
            d.balancedNormalBalancedNormalValid = true;
        }
        balancedNormalWallLimit = outcome.wallLimit;
        d.balancedNormalWallCorrectionLimitDeg = static_cast<float>(balancedNormalWallLimit);
    }

    if (!fit.valid || !fit.surfaceNormal.allFinite()
        || !fit.axisIn.allFinite() || !fit.centerTop.allFinite()
        || fit.topRadius != static_cast<double>(lockedRTop)) {
        keepRecognitionAxis();
        restoreSemanticGeometry();
        return;
    }

    Eigen::Vector3f finalSurface = fit.surfaceNormal.cast<float>();
    Eigen::Vector3f finalAxis = fit.axisIn.cast<float>();
    if (finalSurface.norm() < 1e-6f || finalAxis.norm() < 1e-6f) {
        keepRecognitionAxis();
        restoreSemanticGeometry();
        return;
    }
    finalSurface.normalize();
    finalAxis.normalize();

    // 真实机械孔口的圆弧/椭圆已经直接解析出法向并通过原始点几何复核时，
    // 这个法向拥有最终孔轴的主证据权。HoleWeiziFinal 仍用于孔壁、中心、深度等审核，
    // 但它内部旧的 boundedAxisStep(..., 3°) 或平衡表面法向不能再次覆盖直接解析结果。
    // 若直接椭圆法向没有被采用，则完全保留既有孔壁/表面法向链。
    if (directMouthNormalOwned) {
        finalSurface = recognitionSurface;
        if (!finalSurface.allFinite() || finalSurface.norm() < 1e-6f)
            finalSurface = fit.surfaceNormal.cast<float>();
        finalSurface.normalize();
        if (finalSurface.z() < 0.0f) finalSurface = -finalSurface;
        finalAxis = finalSurface
            * static_cast<float>(HoleJiXing::signOrZero(d.inwardPolarity));
        if (!finalAxis.allFinite() || finalAxis.norm() < 1e-6f) {
            keepRecognitionAxis();
            restoreSemanticGeometry();
            return;
        }
        finalAxis.normalize();
        d.balancedNormalWallCorrectionAppliedDeg = 0.0f;
    } else if (d.balancedNormalBalancedNormalValid) {
        Eigen::Vector3f g = guardSurface;
        if (g.dot(finalSurface) < 0.0f) g = -g;
        Eigen::Vector3f baseAxis = -g;
        baseAxis.normalize();
        const Eigen::Vector3f targetAxis = finalAxis.normalized();
        const double dotA = std::clamp(double(baseAxis.dot(targetAxis)), -1.0, 1.0);
        const double angle = std::acos(dotA) * 180.0 / M_PI;
        const double applied = std::min(angle, balancedNormalWallLimit);
        if (angle > 1e-6 && applied > 1e-6) {
            Eigen::Vector3f rotAxisVec = baseAxis.cross(targetAxis);
            if (rotAxisVec.norm() < 1e-9f) rotAxisVec = Eigen::Vector3f::UnitY();
            rotAxisVec.normalize();
            const double rad = applied * M_PI / 180.0;
            finalAxis = baseAxis * static_cast<float>(std::cos(rad))
                      + rotAxisVec.cross(baseAxis) * static_cast<float>(std::sin(rad))
                      + rotAxisVec * rotAxisVec.dot(baseAxis) * static_cast<float>(1.0 - std::cos(rad));
            finalAxis.normalize();
        } else {
            finalAxis = baseAxis;
        }
        finalSurface = -finalAxis;
        finalSurface.normalize();
        d.balancedNormalWallCorrectionAppliedDeg = static_cast<float>(applied);
    } else {
        d.balancedNormalWallCorrectionAppliedDeg = static_cast<float>(fit.axisCorrectionDegrees);
    }

    const Eigen::Vector3f axisBeforeContinuousFit = finalAxis;

    bool continuousAxisApplied = false;
    std::string continuousAxisReason = "NOT_CONE";
    if (lockedType == 2 && directMouthNormalOwned) {
        continuousAxisReason = "SKIP_DIRECT_MOUTH_ELLIPSE_AXIS_OWNED";
    } else if (lockedType == 2) {
        if (!continuousConePose.executed || !continuousConePose.valid) {
            continuousAxisReason = "CONTINUOUS_FIT_INVALID";
        } else if (!continuousConePose.axis.allFinite()
                   || continuousConePose.axis.squaredNorm() < 1e-24) {
            continuousAxisReason = "CONTINUOUS_FIT_INVALID";
        } else {
            const Eigen::Vector3d inwardConvention =
                (recognitionSurface.cast<double>()
                 * static_cast<double>(d.inwardPolarity)).normalized();
            if (continuousConePose.axis.dot(inwardConvention) <= 0.0) {
                continuousAxisReason = "ORIENTATION_CONTRADICTION";
            } else {
                // 连续锥壁只允许在前一阶段已经通过外表面/孔壁约束的轴附近做小幅精修。
                // 旧代码保存了 axisBeforeContinuousFit 却没有使用，导致连续锥壁轴可无上限覆盖
                // balanced outer-surface guard，局部残缺孔会把最终轴拉歪。这里补上原本缺失的边界。
                Eigen::Vector3d continuousCandidate = continuousConePose.axis.normalized();
                double continuousStepLimitDeg = 1.50;
                if (d.balancedNormalBalancedNormalAttempted
                    && d.balancedNormalBalancedNormalValid) {
                    continuousStepLimitDeg = std::clamp(
                        balancedNormalWallLimit, 0.0, 1.50);
                }
                const Eigen::Vector3d boundedContinuousAxis =
                    HoleWeiziBase::boundedAxisStep(
                        axisBeforeContinuousFit.cast<double>(),
                        continuousCandidate,
                        continuousStepLimitDeg);
                finalAxis = boundedContinuousAxis.cast<float>().normalized();
                continuousAxisApplied = true;
                continuousAxisReason = "APPLIED_BOUNDED_TO_PRECONTINUOUS_AXIS";
            }
        }
    }

    // 机械 Hole 上口中心的“轴向高度”统一由 canonical mouth 决定。
    // originalCenterTop 是进入 HoleWeizi 位姿精修之前，FinalGeometry 已经基于外表面/孔口
    // 几何确定的机械上口中心；fit.centerTop 则可能来自内喉、孔壁或锥壁的深层测量点。
    // 深层证据非常适合修正孔轴和横向圆心，但不能把“最小内喉所在深度”重新定义成上口 Z。
    //
    // 因此这里统一采用“孔轴与 canonical 机械上口平面的交点”作为最终 centerTop：
    //   1) HoleWeizi 仍可利用孔壁把圆心在平面内移动，保留其 XY/切向精修能力；
    //   2) 沿孔轴的深度分量被投影回 canonical 上口平面，直孔和锥孔使用同一套语义；
    //   3) 不增加 KD 查询、拟合或候选搜索，只做一次点积和向量运算。
    // 这条规则的目的不是强制世界坐标 Z 相等，而是保证所有 Hole 的 centerTop 都代表
    // 同一个物理截面——机械外表面孔口，而不是某个内部喉部/孔壁截面。
    const Eigen::Vector3f weiziCenterHouXuan = fit.centerTop.cast<float>();
    Eigen::Vector3f holeKouPlaneFaXian = recognitionSurface;
    if (!holeKouPlaneFaXian.allFinite() || holeKouPlaneFaXian.norm() < 1e-6f)
        holeKouPlaneFaXian = finalSurface;
    holeKouPlaneFaXian.normalize();

    const float weiziZhouXiangPianYi =
        (weiziCenterHouXuan - originalCenterTop).dot(holeKouPlaneFaXian);
    const float holeAxisDianCheng = finalAxis.dot(holeKouPlaneFaXian);
    Eigen::Vector3f tongYiHoleKouCenter = weiziCenterHouXuan;
    if (std::abs(holeAxisDianCheng) >= 0.20f) {
        // 优先沿最终孔轴回到上口平面。对于倾斜孔，这比单纯沿表面法向投影更符合
        // “孔轴与机械上口平面交点”的几何定义，同时保留孔壁拟合得到的横向中心修正。
        tongYiHoleKouCenter -= finalAxis * (weiziZhouXiangPianYi / holeAxisDianCheng);
    } else {
        // 极端退化保护：正常 Hole 的孔轴应近似垂直于表面，不会进入这里；若数值异常，
        // 至少沿上口平面法向去除轴向分量，避免内部截面点直接写回 centerTop。
        tongYiHoleKouCenter -= holeKouPlaneFaXian * weiziZhouXiangPianYi;
    }
    d.centerTop = tongYiHoleKouCenter;

    // canonical mechanical mouth plane 的法向与孔轴是两个不同物理量。
    // HoleWeizi 可以修最终孔轴，但不能把机械上口平面改成“垂直于最终孔轴”的平面。
    // 旧代码这里把 localPlane 直接写成 finalSurface；对锥孔 finalSurface 又常由 finalAxis
    // 推回，因此会出现上口中心位置正确、但上口示意圆/上表面整体倾斜错误的现象。
    // 这里保持进入位姿精修前已经锁定的 canonical mouth plane 法向。
    Eigen::Vector3f canonicalMouthSurface = holeKouPlaneFaXian;
    if (!canonicalMouthSurface.allFinite() || canonicalMouthSurface.norm() < 1e-6f)
        canonicalMouthSurface = recognitionSurface;
    if (!canonicalMouthSurface.allFinite() || canonicalMouthSurface.norm() < 1e-6f)
        canonicalMouthSurface = finalSurface;
    canonicalMouthSurface.normalize();
    if (canonicalMouthSurface.dot(recognitionSurface) < 0.0f)
        canonicalMouthSurface = -canonicalMouthSurface;
    d.localPlaneNx = canonicalMouthSurface.x();
    d.localPlaneNy = canonicalMouthSurface.y();
    d.localPlaneNz = canonicalMouthSurface.z();
    d.localPlaneTiltDeg = std::acos(std::clamp(
        std::abs(canonicalMouthSurface.z()), 0.0f, 1.0f))
        * 180.0f / static_cast<float>(M_PI);

    d.holeAxisInValid = true;
    d.holeAxisInNx = finalAxis.x();
    d.holeAxisInNy = finalAxis.y();
    d.holeAxisInNz = finalAxis.z();
    d.holeAxisTiltDeg = std::acos(std::clamp(
        std::abs(finalAxis.z()), 0.0f, 1.0f))
        * 180.0f / static_cast<float>(M_PI);
    d.holeAxisSource = directMouthNormalOwned
        ? "MOUTH_ELLIPSE_DIRECT_AXIS"
        : (continuousAxisApplied ? "CONTINUOUS_CONE_AXIS_BOUNDED" : d.jointGeometrySource);

    d.centerBot = lockedDepth > 0.0f
        ? d.centerTop + finalAxis * lockedDepth : d.centerTop;
    d.center = 0.5f * (d.centerTop + d.centerBot);
    d.jointCenterShift = (d.centerTop - originalCenterTop).norm();


    restoreSemanticGeometry();
    d.jointRadiusShift = 0.0f;
    d.jointSemanticLockPreserved = d.type == lockedType
        && d.rTop == lockedRTop && d.radius == lockedRadius
        && d.rBot == lockedRBot && d.depth == lockedDepth;
}


// ============================================================================
// 最终孔轴下的直孔上口半径 V1 保守复核
// ============================================================================
/*
设计边界：
1. 只处理已经最终判定为直孔的结果；不参与候选搜索、孔型判断、90°/180°资格和上口中心/法向。
2. 只允许把明显被圆角/入口过渡放大的 R 向下修正，绝不向上扩半径；证据不足时原样保留。
3. 复用 finalGeometrySamples 和 HoleWeizi 已经得到的最终孔轴，不新增 KD-tree 查询。
4. 不直接使用 minimumInnerCylinder 的“最小内喉半径”（它可能落到更深处而过小）；这里只看
   上口后 0.15~0.75 mm 的浅层持久孔壁，取最前面三层的稳健中值作为候选。
5. 必须同时有 HoleWeizi 的持久内圆柱证据，并限制修正幅度/比例，防止对正常直孔造成回归。
*/
struct StraightBoreRadiusReviewV1Result
{
    bool used = false;
    double oldRadius = 0.0;
    double newRadius = 0.0;
};

namespace StraightBoreRadiusReviewV1Detail {
constexpr int kSectors = 72;
constexpr double kPi = 3.14159265358979323846;

struct Sample {
    double depth = 0.0;
    double radius = 0.0;
    int sector = 0;
};
struct Layer {
    bool valid = false;
    double depth = 0.0;
    double radius = 0.0;
    double mad = 0.0;
    int sectors = 0;
    int points = 0;
};

static double medianStraightV1(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2U;
    return (values.size() & 1U) ? values[mid]
        : 0.5 * (values[mid - 1U] + values[mid]);
}

static std::vector<Sample> prepareStraightV1(
    const std::vector<HoleJiheFinal::Sample>& source,
    const ShouDongJuBuZuoBiao& frame,
    const HoleMiaoshu& d,
    Eigen::Vector3f axis)
{
    std::vector<Sample> out;
    if (!frame.valid || source.empty() || !axis.allFinite() || axis.norm() < 1e-6f)
        return out;
    axis.normalize();

    Eigen::Vector3f axisU = frame.u - axis * frame.u.dot(axis);
    if (!axisU.allFinite() || axisU.norm() < 1e-4f)
        axisU = frame.v - axis * frame.v.dot(axis);
    if (!axisU.allFinite() || axisU.norm() < 1e-4f) {
        const Eigen::Vector3f helper = std::abs(axis.x()) < 0.8f
            ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
        axisU = axis.cross(helper);
    }
    if (!axisU.allFinite() || axisU.norm() < 1e-4f) return out;
    axisU.normalize();
    Eigen::Vector3f axisV = axis.cross(axisU);
    if (!axisV.allFinite() || axisV.norm() < 1e-4f) return out;
    axisV.normalize();

    const double radialMax = 1.12 * static_cast<double>(d.rTop) + 0.20;
    out.reserve(source.size() / 3U + 32U);
    for (const HoleJiheFinal::Sample& s : source) {
        if (!std::isfinite(s.u) || !std::isfinite(s.v) || !std::isfinite(s.w)) continue;
        const Eigen::Vector3f world = frame.origin
            + frame.u * static_cast<float>(s.u)
            + frame.v * static_cast<float>(s.v)
            + frame.n * static_cast<float>(s.w);
        const Eigen::Vector3f delta = world - d.centerTop;
        const double depth = static_cast<double>(delta.dot(axis));
        if (depth < 0.08 || depth > 0.88) continue;
        const Eigen::Vector3f radialVec = delta - axis * static_cast<float>(depth);
        const double radius = static_cast<double>(radialVec.norm());
        if (!std::isfinite(radius) || radius < 0.55 * d.rTop || radius > radialMax) continue;
        double angle = std::atan2(static_cast<double>(radialVec.dot(axisV)),
                                  static_cast<double>(radialVec.dot(axisU)));
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * kSectors));
        sector = std::clamp(sector, 0, kSectors - 1);
        out.push_back({depth, radius, sector});
    }
    return out;
}

static Layer shallowLayerStraightV1(const std::vector<Sample>& samples,
                                    double depth,
                                    double oldRadius)
{
    Layer out;
    out.depth = depth;
    std::array<std::vector<double>, kSectors> perSector;
    const double radialLo = 0.55 * oldRadius;
    const double radialHi = 1.12 * oldRadius;
    int points = 0;
    for (const Sample& s : samples) {
        if (std::abs(s.depth - depth) > 0.055) continue;
        if (s.radius < radialLo || s.radius > radialHi) continue;
        perSector[static_cast<std::size_t>(s.sector)].push_back(s.radius);
        ++points;
    }
    if (points < 18) return out;

    std::vector<double> sectorMedians;
    sectorMedians.reserve(kSectors);
    for (auto& values : perSector) {
        if (!values.empty()) sectorMedians.push_back(medianStraightV1(values));
    }
    if (static_cast<int>(sectorMedians.size()) < 18) return out;
    const double radius = medianStraightV1(sectorMedians);
    std::vector<double> deviations;
    deviations.reserve(sectorMedians.size());
    for (double r : sectorMedians) deviations.push_back(std::abs(r - radius));
    const double mad = 1.4826 * medianStraightV1(deviations);
    if (!std::isfinite(radius) || !std::isfinite(mad)) return out;
    if (mad > 0.16) return out;

    out.radius = radius;
    out.mad = mad;
    out.sectors = static_cast<int>(sectorMedians.size());
    out.points = points;
    return out;
}
}

static StraightBoreRadiusReviewV1Result straightBoreRadiusReviewV1(
    const std::vector<HoleJiheFinal::Sample>& finalGeometrySamples,
    const ShouDongJuBuZuoBiao& frame,
    const HoleMiaoshu& d)
{
    using namespace StraightBoreRadiusReviewV1Detail;
    StraightBoreRadiusReviewV1Result out;
    out.oldRadius = d.rTop;
    out.newRadius = d.rTop;

    auto finish = [&](const char*) { return out; };

    if (d.type != 1) return finish("SKIP_NON_STRAIGHT");
    // 大直孔入口可能包含非标准过渡/孔口结构；本复核只解决常规直孔的入口圆角放大，
    // 因此 R>=5.20 mm 明确保持原结果，不参与自动修正。
    if (d.rTop >= 5.20f) return finish("SKIP_LARGE_STRAIGHT_APERTURE");
    if (!frame.valid || finalGeometrySamples.empty() || !d.centerTop.allFinite()
        || !(d.rTop > 1.0f) || !d.holeAxisInValid)
        return finish("REJECT_INVALID_GEOMETRY");

    // minimumInnerCylinder 只作为“确实存在持久内壁”的门槛，不直接拿其最小半径替换上口 R。
    if (!d.measuredInnerRadiusValid || d.measuredInnerUsedSlices < 3
        || d.measuredInnerCoverage < 0.20f
        || !(d.measuredInnerRmse >= 0.0f) || d.measuredInnerRmse > 0.18f
        || !(d.measuredInnerRadius > 0.50f))
        return finish("REJECT_INNER_WALL_EVIDENCE_WEAK");

    Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    const std::vector<Sample> prepared = prepareStraightV1(
        finalGeometrySamples, frame, d, axis);
    if (prepared.size() < 80U) return finish("REJECT_TOO_FEW_SHALLOW_POINTS");

    std::vector<Layer> layers;
    for (double depth = 0.15; depth <= 0.75 + 1e-9; depth += 0.10) {
        Layer layer = shallowLayerStraightV1(prepared, depth, d.rTop);
        if (layer.valid) layers.push_back(layer);
    }
    if (layers.size() < 3U) return finish("REJECT_TOO_FEW_SHALLOW_LAYERS");

    // 必须从入口开始就连续看到孔壁，不能从较深处偶然出现的一小段内喉接管 R。
    std::sort(layers.begin(), layers.end(), [](const Layer& a, const Layer& b) {
        return a.depth < b.depth;
    });
    if (layers.front().depth > 0.26) return finish("REJECT_NO_ENTRY_SHALLOW_WALL");
    std::vector<Layer> run;
    run.push_back(layers.front());
    for (std::size_t i = 1; i < layers.size(); ++i) {
        if (layers[i].depth - run.back().depth <= 0.115 + 1e-9)
            run.push_back(layers[i]);
        else
            break;
    }
    if (run.size() < 3U) return finish("REJECT_SHALLOW_WALL_NOT_PERSISTENT");

    std::vector<double> firstThree = {run[0].radius, run[1].radius, run[2].radius};
    const double candidate = medianStraightV1(firstThree);
    const double delta = static_cast<double>(d.rTop) - candidate;
    out.newRadius = candidate;


    if (!std::isfinite(candidate)) return finish("REJECT_NONFINITE_CANDIDATE");
    if (delta < 0.20) return finish("KEEP_NO_MEANINGFUL_INFLATION");
    if (delta > 0.65) return finish("REJECT_CORRECTION_TOO_LARGE");
    if (candidate < 0.82 * static_cast<double>(d.rTop))
        return finish("REJECT_CANDIDATE_TOO_SMALL");
    if (candidate > 0.97 * static_cast<double>(d.rTop))
        return finish("KEEP_CANDIDATE_TOO_CLOSE");

    // 深层持久内壁允许比浅层候选更小（入口有圆角/过渡），但若差异极端则说明候选家族不一致。
    const double innerRadius = static_cast<double>(d.measuredInnerRadius);
    if (innerRadius > candidate + std::max(0.20, 0.055 * d.rTop))
        return finish("REJECT_SHALLOW_BELOW_PERSISTENT_BORE");
    if (innerRadius < candidate - std::max(0.95, 0.26 * d.rTop))
        return finish("REJECT_INNER_THROAT_FAMILY_INCONSISTENT");

    out.used = true;
    return finish("APPLY_SHALLOW_PERSISTENT_BORE_RADIUS");
}

// ============================================================================
// 最终孔轴下的锥孔下口 V2 局部计量复核
// ============================================================================
/*
设计边界（生产稳定性优先）：
1. 本阶段位于 HoleWeizi 最终孔轴之后，因此不参与 mouth-search、90°/180°圆弧资格、
   canonical mouth、候选竞争和孔型判断；识别成败与 R/TOP 的既有语义不在这里修改。
2. 只处理已经被 DepthBottomReview 判定为可靠锥孔、且旧来源为
   TERMINAL_WALL_HALF_LAYER_EXTENSION 的结果；明确检测到内孔平台时继续使用原平台结果。
3. 不新增 KD-tree 查询。复用 finalGeometrySamples：全段仍按 0.20 mm 稀疏步长建立薄层锥壁模型，
   只有旧下口附近才按 0.05 mm 细层复核，因此不会把全孔深改成密集扫描。
4. r 与 depth 必须服从同一最终孔轴。小 r 的修正采用高覆盖终端薄层；depth 只有在最终轴下
   的连续锥壁支撑与旧值发生明确矛盾时才接管。证据不足时完整保留旧值。
5. 所有修改继续经过既有 reviewMeasuredCone() 物理审核，并限制单次改变量，防止计量后处理
   反向破坏已稳定的识别结果。
*/
struct FinalAxisConeBottomV2Result
{
    bool used = false;
    bool radiusChanged = false;
    bool depthChanged = false;
    double radius = 0.0;
    double depth = 0.0;
    std::string mode;
};

namespace FinalAxisConeBottomV2Detail {

constexpr int kSectors = 48;
constexpr double kPi = 3.14159265358979323846;

struct Sample
{
    double depth = 0.0;
    double radius = 0.0;
    int sector = 0;
};

struct Layer
{
    bool valid = false;
    double depth = 0.0;
    double radius = 0.0;
    double mad = 0.0;
    int sectors = 0;
    int points = 0;
};

struct Fit
{
    bool valid = false;
    double intercept = 0.0;
    double slope = 0.0;
    double rmse = 0.0;
};

static bool finiteV2(double v) noexcept { return std::isfinite(v); }

static double quantileV2(std::vector<double> values, double q)
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

static double medianV2(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    return (values.size() & 1U) ? values[mid] : 0.5 * (values[mid - 1] + values[mid]);
}

static Layer sectionV2(const std::vector<Sample>& samples,
                       double centerDepth,
                       double halfWidth,
                       double radialLo,
                       double radialHi,
                       double sectorQuantile,
                       int minSectors)
{
    Layer out;
    out.depth = centerDepth;
    std::array<std::vector<double>, kSectors> perSector;
    int points = 0;
    for (const Sample& s : samples) {
        if (std::abs(s.depth - centerDepth) > halfWidth) continue;
        if (s.radius < radialLo || s.radius > radialHi) continue;
        perSector[static_cast<std::size_t>(s.sector)].push_back(s.radius);
        ++points;
    }
    if (points < std::max(10, minSectors)) return out;

    std::vector<double> sectorValues;
    sectorValues.reserve(kSectors);
    for (auto& values : perSector) {
        if (!values.empty()) sectorValues.push_back(quantileV2(values, sectorQuantile));
    }
    if (static_cast<int>(sectorValues.size()) < minSectors) return out;

    const double radius = medianV2(sectorValues);
    std::vector<double> deviations;
    deviations.reserve(sectorValues.size());
    for (double value : sectorValues) deviations.push_back(std::abs(value - radius));
    const double mad = medianV2(deviations);
    if (!finiteV2(radius) || !finiteV2(mad)) return out;

    out.radius = radius;
    out.mad = mad;
    out.sectors = static_cast<int>(sectorValues.size());
    out.points = points;
    return out;
}

static Fit fitWeightedV2(const std::vector<Layer>& layers)
{
    Fit out;
    if (layers.size() < 5) return out;
    std::vector<double> baseW(layers.size(), 1.0);
    std::vector<double> workW(layers.size(), 1.0);
    for (std::size_t i = 0; i < layers.size(); ++i) {
        baseW[i] = std::max(1.0,
            static_cast<double>(layers[i].sectors) + 0.02 * static_cast<double>(layers[i].points));
        workW[i] = baseW[i];
    }

    for (int iter = 0; iter < 4; ++iter) {
        double sw = 0.0, sx = 0.0, sy = 0.0;
        for (std::size_t i = 0; i < layers.size(); ++i) {
            sw += workW[i];
            sx += workW[i] * layers[i].depth;
            sy += workW[i] * layers[i].radius;
        }
        if (sw <= 1e-12) return {};
        const double mx = sx / sw;
        const double my = sy / sw;
        double num = 0.0, den = 0.0;
        for (std::size_t i = 0; i < layers.size(); ++i) {
            const double dx = layers[i].depth - mx;
            num += workW[i] * dx * (layers[i].radius - my);
            den += workW[i] * dx * dx;
        }
        if (den <= 1e-12) return {};
        out.slope = num / den;
        out.intercept = my - out.slope * mx;

        std::vector<double> residuals;
        residuals.reserve(layers.size());
        for (const Layer& layer : layers)
            residuals.push_back(layer.radius - (out.intercept + out.slope * layer.depth));
        const double residualMedian = medianV2(residuals);
        std::vector<double> absoluteResiduals;
        absoluteResiduals.reserve(residuals.size());
        for (double residual : residuals)
            absoluteResiduals.push_back(std::abs(residual - residualMedian));
        const double scale = 1.4826 * medianV2(absoluteResiduals) + 1e-6;
        for (std::size_t i = 0; i < residuals.size(); ++i) {
            const double a = std::abs(residuals[i]);
            const double huber = a > 1.5 * scale ? (1.5 * scale / a) : 1.0;
            workW[i] = baseW[i] * huber;
        }
    }

    double sw = 0.0, sse = 0.0;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        const double e = layers[i].radius - (out.intercept + out.slope * layers[i].depth);
        sw += workW[i];
        sse += workW[i] * e * e;
    }
    if (sw <= 1e-12) return {};
    out.rmse = std::sqrt(sse / sw);
    out.valid = finiteV2(out.intercept) && finiteV2(out.slope) && finiteV2(out.rmse);
    return out;
}

static std::vector<Sample> prepareV2(
    const std::vector<HoleJiheFinal::Sample>& source,
    const ShouDongJuBuZuoBiao& frame,
    const HoleMiaoshu& d,
    const Eigen::Vector3f& finalAxis)
{
    std::vector<Sample> out;
    if (!frame.valid || source.empty()) return out;
    Eigen::Vector3f axis = finalAxis;
    if (!axis.allFinite() || axis.norm() < 1e-6f) return out;
    axis.normalize();

    Eigen::Vector3f axisU = frame.u - axis * frame.u.dot(axis);
    if (!axisU.allFinite() || axisU.norm() < 1e-4f)
        axisU = frame.v - axis * frame.v.dot(axis);
    if (!axisU.allFinite() || axisU.norm() < 1e-4f) {
        const Eigen::Vector3f helper = std::abs(axis.x()) < 0.8f
            ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
        axisU = axis.cross(helper);
    }
    if (!axisU.allFinite() || axisU.norm() < 1e-4f) return out;
    axisU.normalize();
    Eigen::Vector3f axisV = axis.cross(axisU);
    if (!axisV.allFinite() || axisV.norm() < 1e-4f) return out;
    axisV.normalize();

    const double depthMax = std::min(10.0, static_cast<double>(d.depth) + 0.70);
    const double radiusMax = static_cast<double>(d.rTop) + 1.20;
    out.reserve(source.size() / 2 + 32);
    for (const HoleJiheFinal::Sample& s : source) {
        if (!finiteV2(s.u) || !finiteV2(s.v) || !finiteV2(s.w)) continue;
        const Eigen::Vector3f world = frame.origin
            + frame.u * static_cast<float>(s.u)
            + frame.v * static_cast<float>(s.v)
            + frame.n * static_cast<float>(s.w);
        const Eigen::Vector3f delta = world - d.centerTop;
        const double depth = static_cast<double>(delta.dot(axis));
        if (depth < 0.10 || depth > depthMax) continue;
        const Eigen::Vector3f radialVec = delta - axis * static_cast<float>(depth);
        const double radius = static_cast<double>(radialVec.norm());
        if (!finiteV2(radius) || radius < 0.20 || radius > radiusMax) continue;
        double angle = std::atan2(static_cast<double>(radialVec.dot(axisV)),
                                  static_cast<double>(radialVec.dot(axisU)));
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(angle / (2.0 * kPi) * kSectors));
        sector = std::clamp(sector, 0, kSectors - 1);
        out.push_back({depth, radius, sector});
    }
    return out;
}

static std::vector<Layer> coarseConeV2(const std::vector<Sample>& samples,
                                       double topRadius,
                                       double oldRadius,
                                       double oldDepth)
{
    std::vector<Layer> layers;
    const double legacySlope = (oldRadius - topRadius) / std::max(oldDepth, 0.50);
    for (double d = 0.30; d <= std::max(0.61, oldDepth - 0.05) + 1e-9; d += 0.20) {
        const double pred = topRadius + legacySlope * d;
        const double band = std::max(0.55, 0.10 * topRadius);
        Layer layer = sectionV2(samples, d, 0.09,
            std::max(0.20, pred - band), std::min(topRadius + 1.20, pred + band), 0.50, 6);
        if (!layer.valid || layer.mad > std::max(0.22, 0.045 * topRadius)) continue;
        layers.push_back(layer);
    }
    return layers;
}

static std::vector<Layer> supportFineV2(const std::vector<Sample>& samples,
                                        double topRadius,
                                        double oldDepth,
                                        const Fit& fit)
{
    std::vector<Layer> out;
    const double first = std::max(0.30, oldDepth - 0.95);
    const double last = oldDepth + 0.25;
    for (double d = first; d <= last + 1e-9; d += 0.05) {
        const double pred = fit.intercept + fit.slope * d;
        const double band = std::max(0.32, 0.055 * topRadius);
        Layer layer = sectionV2(samples, d, 0.06,
            std::max(0.20, pred - band), std::min(topRadius + 1.0, pred + band), 0.50, 6);
        if (!layer.valid) continue;
        if (layer.mad > std::max(0.18, 0.035 * topRadius)) continue;
        if (std::abs(layer.radius - pred) > std::max(0.24, 0.04 * topRadius)) continue;
        out.push_back(layer);
    }
    return out;
}

static bool deepestSupportRunV2(const std::vector<Layer>& layers,
                                double oldDepth,
                                double& lastDepth,
                                int& runLayers)
{
    if (layers.empty()) return false;
    std::vector<std::vector<Layer>> runs;
    std::vector<Layer> current;
    for (const Layer& layer : layers) {
        if (current.empty() || layer.depth - current.back().depth <= 0.11 + 1e-9) {
            current.push_back(layer);
        } else {
            if (current.size() >= 4) runs.push_back(current);
            current.clear();
            current.push_back(layer);
        }
    }
    if (current.size() >= 4) runs.push_back(current);
    const std::vector<Layer>* best = nullptr;
    for (const auto& run : runs) {
        if (run.back().depth < oldDepth - 0.85) continue;
        if (!best || run.back().depth > best->back().depth + 1e-9
            || (std::abs(run.back().depth - best->back().depth) <= 1e-9
                && run.size() > best->size())) best = &run;
    }
    if (!best) return false;
    lastDepth = best->back().depth;
    runLayers = static_cast<int>(best->size());
    return true;
}

static bool terminalRadiusV2(const std::vector<Sample>& samples,
                             double topRadius,
                             double oldRadius,
                             double oldDepth,
                             double& terminalDepth,
                             double& terminalRadius,
                             int& terminalSectors,
                             int& validLayers)
{
    struct MaybeLayer { double depth = 0.0; Layer layer; };
    std::vector<MaybeLayer> fine;
    const double lo = std::max(0.20, oldRadius - 0.85);
    const double hi = std::min(0.82 * topRadius, oldRadius + 0.85);
    const double first = std::max(0.30, oldDepth - 0.60);
    const double last = oldDepth + 0.55;
    for (double d = first; d <= last + 1e-9; d += 0.05) {
        Layer layer = sectionV2(samples, d, 0.06, lo, hi, 0.75, 10);
        if (layer.valid && layer.mad <= std::max(0.18, 0.035 * topRadius))
            fine.push_back({d, layer});
        else
            fine.push_back({d, {}});
    }
    validLayers = 0;
    int start = -1;
    for (std::size_t i = 0; i < fine.size(); ++i) {
        if (fine[i].depth + 1e-9 < oldDepth - 0.45) continue;
        if (fine[i].layer.valid) { start = static_cast<int>(i); break; }
    }
    if (start < 0) return false;
    int end = start;
    int misses = 0;
    for (int i = start; i < static_cast<int>(fine.size()); ++i) {
        if (fine[static_cast<std::size_t>(i)].layer.valid) {
            end = i; misses = 0; ++validLayers;
        } else {
            ++misses;
            if (misses >= 2) break;
        }
    }
    for (int i = end; i >= start; --i) {
        const Layer& layer = fine[static_cast<std::size_t>(i)].layer;
        if (!layer.valid) continue;
        terminalDepth = fine[static_cast<std::size_t>(i)].depth;
        terminalRadius = layer.radius;
        terminalSectors = layer.sectors;
        return true;
    }
    return false;
}

} // namespace FinalAxisConeBottomV2Detail

static FinalAxisConeBottomV2Result finalAxisRefineConeBottomV2(
    const std::vector<HoleJiheFinal::Sample>& finalGeometrySamples,
    const ShouDongJuBuZuoBiao& frame,
    const HoleShenduGuJi::Result& depthBottomReview,
    const HoleMiaoshu& d)
{
    using namespace FinalAxisConeBottomV2Detail;
    FinalAxisConeBottomV2Result out;
    out.radius = d.rBot;
    out.depth = d.depth;

    auto finish = [&](const char*) { return out; };

    if (d.type != 2 || !d.reliableBottomRadius || !(d.rTop > d.rBot)
        || !(d.rBot > 0.0f) || !(d.depth > 0.35f))
        return finish("SKIP_NONCONE_OR_NO_RELIABLE_BOTTOM");
    if (!depthBottomReview.used || !depthBottomReview.bottomValid)
        return finish("SKIP_DEPTH_BOTTOM_REVIEW_NOT_OWNED");
    if (depthBottomReview.platformDetected)
        return finish("SKIP_EXPLICIT_PLATFORM");
    if (depthBottomReview.mode != "DepthBottomReview_TERMINAL_WALL_HALF_LAYER_EXTENSION")
        return finish("SKIP_NON_TERMINAL_WALL_MODE");
    if (!frame.valid || finalGeometrySamples.empty())
        return finish("SKIP_NO_CANONICAL_SAMPLES");
    Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    if (!d.holeAxisInValid || !axis.allFinite() || axis.norm() < 1e-6f)
        return finish("SKIP_FINAL_AXIS_UNAVAILABLE");
    axis.normalize();

    const std::vector<Sample> samples = prepareV2(finalGeometrySamples, frame, d, axis);
    if (samples.size() < 40) return finish("INSUFFICIENT_FINAL_AXIS_SAMPLES");

    const double R = static_cast<double>(d.rTop);
    const double oldR = static_cast<double>(d.rBot);
    const double oldD = static_cast<double>(d.depth);
    const std::vector<Layer> coarse = coarseConeV2(samples, R, oldR, oldD);
    if (coarse.size() < 5) return finish("INSUFFICIENT_COARSE_CONE_LAYERS");
    const Fit fit = fitWeightedV2(coarse);
    if (!fit.valid || fit.slope >= -0.12 || fit.rmse > std::max(0.25, 0.045 * R))
        return finish("COARSE_CONE_MODEL_INVALID");

    // A. 小 r：只在旧下口附近做 0.05 mm 薄层；为了避免过修，目前只允许把明显偏大的旧 r 向下修。
    double nr = oldR;
    double nd = oldD;
    double tDepth = 0.0, tRadius = 0.0;
    int tSectors = 0, radiusLayers = 0;
    const bool terminalValid = terminalRadiusV2(samples, R, oldR, oldD,
        tDepth, tRadius, tSectors, radiusLayers);
    if (terminalValid && tSectors >= 10
        && tRadius < oldR - 0.02
        && oldR - tRadius <= std::max(0.45, 0.09 * R)
        && tRadius > 0.20 && tRadius < 0.96 * R) {
        nr = tRadius;
        out.radiusChanged = true;
    }

    // B. depth：先求“最终轴连续锥壁模型在当前 r 上的同截面深度”。它只是候选，不无条件接管。
    const double coherentDepth = std::abs(fit.slope) > 1e-9
        ? (nr - fit.intercept) / fit.slope : oldD;

    // B1. 若 r 本轮被高覆盖终端薄层修正，且小 r/R 不太小（极小孔更容易提前遮挡），
    //     同时连续模型深度与终端可见边界相互吻合，才把二者平均为同截面 depth。
    std::string mode;
    if (out.radiusChanged && nr / std::max(R, 1e-6) >= 0.25 && terminalValid) {
        const double visibleBoundary = tDepth + 0.025;
        if (coherentDepth > 0.35 && coherentDepth < 10.0
            && std::abs(coherentDepth - oldD) <= 0.60
            && std::abs(coherentDepth - visibleBoundary) <= 0.22) {
            nd = 0.5 * (coherentDepth + visibleBoundary);
            out.depthChanged = std::abs(nd - oldD) > 0.01;
            mode = "FinalAxisConeBottomV2_RADIUS_SAME_SECTION";
        }
    }

    // B2. 末端支撑矛盾：只在粗 depth 与最终轴连续锥壁的可见范围明显冲突时修 depth。
    //     小 r/R < 0.24 的孔更容易因遮挡/圆角提前失去孔壁，因此要求更大的“过深矛盾”才接管。
    const std::vector<Layer> supportLayers = supportFineV2(samples, R, oldD, fit);
    double supportLast = 0.0;
    int supportRunLayers = 0;
    const bool supportValid = deepestSupportRunV2(supportLayers, oldD, supportLast, supportRunLayers);

    if (!out.depthChanged && supportValid
        && coherentDepth > 0.35 && coherentDepth < 10.0
        && std::abs(coherentDepth - oldD) <= 0.75) {
        const double ratio = nr / std::max(R, 1e-6);
        const double gap = oldD - supportLast;
        const double overDeepGate = ratio >= 0.24 ? 0.35 : 0.42;
        if (gap >= overDeepGate
            && coherentDepth >= supportLast - 0.15
            && coherentDepth <= oldD - 0.05) {
            nd = coherentDepth;
            out.depthChanged = true;
            mode = "FinalAxisConeBottomV2_OVERDEEP_CONTRADICTION";
        } else if (gap <= -0.10
            && coherentDepth >= oldD + 0.05
            && coherentDepth <= supportLast + 0.15) {
            nd = coherentDepth;
            out.depthChanged = true;
            mode = "FinalAxisConeBottomV2_UNDERDEEP_EXTENSION";
        }
    }

    if (!out.radiusChanged && !out.depthChanged)
        return finish("NO_SAFE_REFINEMENT_EVIDENCE");

    if (!finiteV2(nr) || !finiteV2(nd)
        || nr <= 0.20 || nr >= 0.96 * R || nd <= 0.35 || nd > 10.0
        || std::abs(nr - oldR) > std::max(0.45, 0.09 * R)
        || std::abs(nd - oldD) > 0.75)
        return finish("REFINEMENT_CHANGE_LIMIT_REJECT");

    const HoleShenduGuJi::WuLiReview physical = HoleShenduGuJi::reviewMeasuredCone(R, nr, nd);
    if (physical.rejectHole || physical.classifyStraight)
        return finish("REFINEMENT_FAILS_EXISTING_PHYSICAL_REVIEW");

    out.radius = nr;
    out.depth = nd;
    out.used = true;
    if (mode.empty()) mode = out.radiusChanged
        ? "FinalAxisConeBottomV2_RADIUS_ONLY" : "FinalAxisConeBottomV2_DEPTH_ONLY";
    else if (out.radiusChanged && mode != "FinalAxisConeBottomV2_RADIUS_SAME_SECTION")
        mode += "+RADIUS";
    out.mode = mode;
    return finish("ACCEPTED");
}


/** 【函数导航】
 * 作用：执行“finalizeObservedBoreRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static void finalizeObservedBoreRadius(
    const std::vector<HoleJiheFinal::Sample>& finalGeometrySamples,
    const ShouDongJuBuZuoBiao& frame,
    HoleMiaoshu& d)
{
    d.observedBoreRadiusExecuted = false;
    d.observedBoreRadiusValid = false;
    d.observedBoreRadiusState = 0;
    d.observedBoreRadius = 0.0f;
    d.observedBoreRadiusMad = 0.0f;
    d.observedBoreDepthStart = 0.0f;
    d.observedBoreDepthEnd = 0.0f;
    d.observedBoreDepthSpan = 0.0f;
    d.observedBoreCoverage = 0.0f;
    d.observedBoreRmse = 0.0f;
    d.observedBoreCenterLineRmse = 0.0f;
    d.observedBoreRadiusSlope = 0.0f;
    d.observedBoreAxisCorrectionDeg = 0.0f;
    d.observedBoreCenterShift = 0.0f;
    d.observedBoreUsedSlices = 0;
    d.observedBoreCandidateCount = 0;
    d.observedBoreFamilyCount = 0;
    d.observedBoreDepthRegions = 0;
    d.observedBoreHistogramWindows = 0;
    d.observedBoreHistogramPointVisits = 0;
    d.observedBoreFitPointVisits = 0;
    d.observedBoreSource = "ObservedBore_NOT_EXECUTED";
    d.observedBoreDecision = "NOT_EXECUTED";

    if (d.type != 1) {
        d.observedBoreDecision = "SKIP_NON_STRAIGHT_FINAL_TYPE";
        return;
    }
    if (!frame.valid || finalGeometrySamples.empty() || !d.centerTop.allFinite()
        || !(d.rTop > 0.5f)) {
        d.observedBoreDecision = "SKIP_INVALID_LOCAL_GEOMETRY";
        return;
    }

    Eigen::Vector3f axisWorld(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    if (!d.holeAxisInValid || !axisWorld.allFinite() || axisWorld.norm() < 1e-6f) {
        if (!HoleJiXing::known(d.inwardPolarity)) {
            d.observedBoreDecision = "SKIP_UNKNOWN_INWARD_AXIS";
            return;
        }
        axisWorld = frame.n
            * static_cast<float>(HoleJiXing::signOrZero(d.inwardPolarity));
    }
    if (!axisWorld.allFinite() || axisWorld.norm() < 1e-6f) {
        d.observedBoreDecision = "SKIP_INVALID_INWARD_AXIS";
        return;
    }
    axisWorld.normalize();

    const Eigen::Vector3f delta = d.centerTop - frame.origin;
    const Eigen::Vector3d centerLocal(
        static_cast<double>(delta.dot(frame.u)),
        static_cast<double>(delta.dot(frame.v)),
        static_cast<double>(delta.dot(frame.n)));
    Eigen::Vector3d axisLocal(
        static_cast<double>(axisWorld.dot(frame.u)),
        static_cast<double>(axisWorld.dot(frame.v)),
        static_cast<double>(axisWorld.dot(frame.n)));
    if (!centerLocal.allFinite() || !axisLocal.allFinite()
        || axisLocal.norm() < 1e-9) {
        d.observedBoreDecision = "SKIP_LOCAL_TRANSFORM_FAILED";
        return;
    }
    axisLocal.normalize();

    GuanCeBoreRadius::YiYouZhengJu existing;
    existing.valid = d.measuredInnerRadiusValid;
    existing.radius = static_cast<double>(d.measuredInnerRadius);
    existing.radiusMad = static_cast<double>(d.measuredInnerRadiusMad);
    existing.depthStart = static_cast<double>(d.measuredInnerDepthStart);
    existing.depthEnd = static_cast<double>(d.measuredInnerDepthEnd);
    existing.depthSpan = static_cast<double>(d.measuredInnerDepthSpan);
    existing.coverage = static_cast<double>(d.measuredInnerCoverage);
    existing.rmse = static_cast<double>(d.measuredInnerRmse);
    existing.centerLineRmse = static_cast<double>(d.measuredInnerCenterLineRmse);
    existing.usedSlices = d.measuredInnerUsedSlices;

    const double existingOutsideAllowance = std::max(
        0.075, 2.5 * std::max(0.0, existing.radiusMad) + 0.015);
    const bool existingOutsideMouth = existing.valid
        && (existing.radius < 0.52 * static_cast<double>(d.rTop)
            || existing.radius > static_cast<double>(d.rTop)
                + existingOutsideAllowance);

    const bool allowRawEstimator = existingOutsideMouth
        || !d.apertureContourExecuted || !d.apertureContourValid;
    const auto result = GuanCeBoreRadius::evaluate(
        finalGeometrySamples, centerLocal, axisLocal, static_cast<double>(d.rTop),
        existing, allowRawEstimator);
    d.observedBoreRadiusExecuted = result.executed;
    d.observedBoreRadiusValid = result.valid;
    d.observedBoreRadiusState = static_cast<int>(result.state);
    d.observedBoreRadius = result.valid ? static_cast<float>(result.radius) : 0.0f;
    d.observedBoreRadiusMad = result.valid ? static_cast<float>(result.radiusMad) : 0.0f;
    d.observedBoreDepthStart = result.valid ? static_cast<float>(result.depthStart) : 0.0f;
    d.observedBoreDepthEnd = result.valid ? static_cast<float>(result.depthEnd) : 0.0f;
    d.observedBoreDepthSpan = result.valid ? static_cast<float>(result.depthSpan) : 0.0f;
    d.observedBoreCoverage = result.valid ? static_cast<float>(result.coverage) : 0.0f;
    d.observedBoreRmse = result.valid ? static_cast<float>(result.rmse) : 0.0f;
    d.observedBoreCenterLineRmse = result.valid
        ? static_cast<float>(result.centerLineRmse) : 0.0f;
    d.observedBoreRadiusSlope = result.valid
        ? static_cast<float>(result.radiusSlope) : 0.0f;
    d.observedBoreAxisCorrectionDeg = result.valid
        ? static_cast<float>(result.axisCorrectionDegrees) : 0.0f;
    d.observedBoreCenterShift = result.valid
        ? static_cast<float>(result.centerShift) : 0.0f;
    d.observedBoreUsedSlices = result.usedSlices;
    d.observedBoreCandidateCount = result.candidateCount;
    d.observedBoreFamilyCount = result.familyCount;
    d.observedBoreDepthRegions = result.depthRegions;
    d.observedBoreHistogramWindows = result.histogramWindows;
    d.observedBoreHistogramPointVisits = result.histogramPointVisits;
    d.observedBoreFitPointVisits = result.fitPointVisits;
    d.observedBoreSource = result.source ? result.source : "ObservedBore_UNKNOWN_SOURCE";
    d.observedBoreDecision = result.decision ? result.decision : "UNKNOWN";


}


struct WuLiHoleKouPouMianDiag
{
    bool attempted = false;
    bool valid = false;
    bool used = false;
    int axialPeakCount = 0;
    int profileTrialCount = 0;
    int validProfileCount = 0;
    double score = -std::numeric_limits<double>::infinity();
    double selectedTopW = 0.0;
    HoleJiheSurfacePouMian::Result profile;
    HoleHouXuanJuLei::HoleKouEllipseEvidence ellipse;
    std::string mode = "PROFILE_MOUTH_NOT_ATTEMPTED";
};

// 物理上口剖面用于处理“粗候选并不等于机械上口”的情况。
// 完整圆和部分圆弧最终都进入同一孔壁/孔口几何；这里只在候选存在外层/部分可见语义时执行，避免无意义增加常规孔耗时。

/** 【函数导航】
 * 作用：执行“jieXiZhenShiHoleKouLunKuo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static WuLiHoleKouPouMianDiag jieXiZhenShiHoleKouLunKuo(
    const std::vector<HoleHouXuanJuLei::Sample>& coarseSamples,
    const HoleHouXuanJuLei::Cluster& locatorCluster)
{
    WuLiHoleKouPouMianDiag out;
    if (coarseSamples.size() < 80
        || !std::isfinite(locatorCluster.consensusCenterU)
        || !std::isfinite(locatorCluster.consensusCenterV)
        || !std::isfinite(locatorCluster.consensusTopRadius)
        || !(locatorCluster.consensusTopRadius > 1.0)) {
        out.mode = "PROFILE_MOUTH_INVALID_LOCATOR";
        return out;
    }

    const double centerU = locatorCluster.consensusCenterU;
    const double centerV = locatorCluster.consensusCenterV;
    const double radialLimit = std::clamp(
        1.45 * locatorCluster.consensusTopRadius + 2.0, 10.0, 14.0);
    std::vector<double> localW;
    localW.reserve(coarseSamples.size());
    double minimumW = std::numeric_limits<double>::infinity();
    double maximumW = -std::numeric_limits<double>::infinity();
    for (const auto& sample : coarseSamples) {
        if (!std::isfinite(sample.u) || !std::isfinite(sample.v)
            || !std::isfinite(sample.w)) continue;
        if (std::hypot(sample.u - centerU, sample.v - centerV) > radialLimit) continue;
        localW.push_back(sample.w);
        minimumW = std::min(minimumW, sample.w);
        maximumW = std::max(maximumW, sample.w);
    }
    if (localW.size() < 80 || !std::isfinite(minimumW) || !std::isfinite(maximumW)
        || maximumW - minimumW < 0.40) {
        out.mode = "PROFILE_MOUTH_NO_LOCAL_AXIAL_SUPPORT";
        return out;
    }

    // 上口高度不做固定偏移/逐层暴力扫描，而从当前目标附近真实点的轴向密度峰生成少量候选。
    // 0.20 mm 只决定直方图分辨率；每个峰再验证中心和孔壁连续性，不能单独决定孔口。
    constexpr double histogramBin = 0.20;
    const int binCount = std::max(3, static_cast<int>(
        std::ceil((maximumW - minimumW) / histogramBin)) + 1);
    std::vector<int> histogram(static_cast<std::size_t>(binCount), 0);
    for (double w : localW) {
        int bin = static_cast<int>(std::floor((w - minimumW) / histogramBin));
        bin = std::clamp(bin, 0, binCount - 1);
        ++histogram[static_cast<std::size_t>(bin)];
    }
    const int globalMaximum = *std::max_element(histogram.begin(), histogram.end());
    const int minimumPeakCount = std::max(18,
        static_cast<int>(std::ceil(0.035 * static_cast<double>(globalMaximum))));
    /** 【类型导航注释】
     * FengZhi：Hole 识别总编排中的自定义 结构体。
     * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct FengZhi { int bin = 0; int count = 0; };
    std::vector<FengZhi> peaks;
    for (int bin = 0; bin < binCount; ++bin) {
        const int count = histogram[static_cast<std::size_t>(bin)];
        const int left = bin > 0 ? histogram[static_cast<std::size_t>(bin - 1)] : -1;
        const int right = bin + 1 < binCount
            ? histogram[static_cast<std::size_t>(bin + 1)] : -1;
        if (count >= minimumPeakCount && count >= left && count >= right)
            peaks.push_back({bin, count});
    }
    std::sort(peaks.begin(), peaks.end(), [](const FengZhi& left, const FengZhi& right) {
        if (left.count != right.count) return left.count > right.count;
        return left.bin > right.bin;
    });
    std::vector<FengZhi> selectedPeaks;
    for (const FengZhi& peak : peaks) {
        const double w = minimumW + (static_cast<double>(peak.bin) + 0.5) * histogramBin;
        bool duplicateBand = false;
        for (const FengZhi& existing : selectedPeaks) {
            const double existingW = minimumW
                + (static_cast<double>(existing.bin) + 0.5) * histogramBin;
            if (std::abs(w - existingW) < 0.45) {
                duplicateBand = true;
                break;
            }
        }
        if (!duplicateBand) selectedPeaks.push_back(peak);
        if (selectedPeaks.size() >= 5U) break;
    }
    out.axialPeakCount = static_cast<int>(selectedPeaks.size());
    if (selectedPeaks.empty()) {
        out.mode = "PROFILE_MOUTH_NO_AXIAL_PEAK";
        return out;
    }

    std::vector<HoleJiheSurfacePouMian::Sample> profileSamples;
    profileSamples.reserve(coarseSamples.size());
    for (const auto& sample : coarseSamples)
        profileSamples.push_back({sample.u, sample.v, sample.w});

    const std::array<double, 3> peakOffsets{{-0.12, 0.0, 0.12}};
    for (const FengZhi& peak : selectedPeaks) {
        const double peakW = minimumW
            + (static_cast<double>(peak.bin) + 0.5) * histogramBin;
        for (double offset : peakOffsets) {
            ++out.profileTrialCount;
            HoleJiheSurfacePouMian::Input input;
            input.topU = centerU;
            input.topV = centerV;
            input.topW = peakW + offset;
            input.initialTopRadius = locatorCluster.consensusTopRadius;
            // 候选簇只负责粗定位。3.5 mm 足以吸收残缺弧中心偏差，
            // 同时不会让剖面搜索跨一个孔距去抢邻孔。
            input.maxCenterShift = 3.5;
            input.centerShiftPenalty = 0.08;
            const HoleJiheSurfacePouMian::Result profile =
                HoleJiheSurfacePouMian::evaluate(profileSamples, input);
            if (!profile.valid || !profile.surfaceValid || !profile.profileValid
                || profile.validLayers < 2 || !(profile.topRadius > 1.0)
                || profile.topRadius > 11.0 || profile.centerShift > 3.5)
                continue;

            double meanCoverage = 0.0;
            for (const auto& layer : profile.layers) meanCoverage += layer.coverage;
            meanCoverage /= static_cast<double>(std::max<std::size_t>(1U, profile.layers.size()));
            const double madLimit = std::max(0.35, 0.09 * profile.topRadius);
            if (profile.surfaceSectors < 12 || profile.surfaceMad > madLimit
                || meanCoverage < 0.16 || profile.confidence < 0.52)
                continue;
            if (profile.holeType == 2) {
                const bool measuredCone = profile.bottomValid
                    && profile.bottomRadius > 0.20
                    && profile.bottomRadius < 0.96 * profile.topRadius
                    && profile.depth >= 0.80;
                const bool partialCone = !profile.bottomValid
                    && profile.validLayers >= 3
                    && profile.radiusSlope < -0.25
                    && profile.radiusShrink >= std::max(0.50, 0.07 * profile.topRadius)
                    && profile.monotonicRatio >= 0.60;
                if (!measuredCone && !partialCone) continue;
            } else {
                // 直孔允许半径近似不变，但必须有真正的多层内壁，不能只凭一个表面圆输出孔。
                if (profile.validLayers < 3) continue;
            }

            ++out.validProfileCount;
            double score = 2.0 * static_cast<double>(profile.validLayers)
                + 3.0 * profile.monotonicRatio
                + 3.5 * meanCoverage
                + 2.0 * profile.confidence
                + 0.08 * static_cast<double>(profile.surfaceSectors)
                - 2.0 * profile.surfaceMad
                - 0.12 * profile.centerShift;
            if (profile.holeType == 2) {
                const double bottom = profile.bottomValid ? profile.bottomRadius
                    : (profile.layers.empty() ? profile.topRadius : profile.layers.back().radius);
                score += std::min(3.0, 1.2 * std::max(0.0, profile.topRadius - bottom));
            }
            bool replace = !out.valid || score > out.score;
            if (out.valid && !replace) {
                // 同一真实孔口族中，略微向孔内移动切面有时会因为“多出一层”而虚高评分。
                // 若几何质量没有下降且两个解明显属于同一孔口，就优先更靠外表面的上口锚点：
                // inwardPolarity=-1 时更大的 W 更外，+1 时更小的 W 更外。
                // 这里只做同族近似并列解的物理语义消歧，不扩大候选范围，也不改变孔型门槛。
                const double centerDistance = std::hypot(
                    profile.centerU - out.profile.centerU, profile.centerV - out.profile.centerV);
                const double radiusDistance = std::abs(profile.topRadius - out.profile.topRadius);
                const double radiusLimit = std::max(
                    0.65, 0.13 * std::max(profile.topRadius, out.profile.topRadius));
                const bool sameMouthFamily = profile.inwardPolarity == out.profile.inwardPolarity
                    && centerDistance <= 0.75 && radiusDistance <= radiusLimit;
                const double outwardCandidate =
                    -static_cast<double>(profile.inwardPolarity) * input.topW;
                const double outwardCurrent =
                    -static_cast<double>(out.profile.inwardPolarity) * out.selectedTopW;
                // 上口是“孔壁第一次与外表面相交”的最外侧有效截面。层数越多通常意味着切面
                // 已经向孔内走得更深，因此不能让 validLayers 的评分优势反过来把上口往里拖。
                // 这里只在同一孔口族内比较：候选仍必须拥有足够的多层孔壁、单调性、覆盖与置信度；
                // 满足这些物理证据后，优先最外侧截面，而不要求它和更深截面拥有几乎相同的层数评分。
                const bool outwardGeometryReliable = profile.validLayers >= 5
                    && profile.monotonicRatio >= 0.85
                    && profile.confidence >= 0.90
                    && profile.surfaceSectors >= 14
                    && profile.surfaceMad <= std::max(0.12, out.profile.surfaceMad + 0.05)
                    && profile.centerShift <= out.profile.centerShift + 0.30;
                if (sameMouthFamily && outwardGeometryReliable
                    && outwardCandidate >= outwardCurrent + 0.10) {
                    replace = true;
                }
            }
            if (replace) {
                            out.score = score;
                out.selectedTopW = input.topW;
                out.profile = profile;
            }
        }
    }
    if (!out.valid) {
        out.mode = "PROFILE_MOUTH_NO_PHYSICAL_INWARD_PROFILE";
        return out;
    }
    out.ellipse = HoleHouXuanJuLei::fitResolvedMouthEllipse(
        coarseSamples, out.profile.centerU, out.profile.centerV,
        out.selectedTopW, out.profile.topRadius);
    out.mode = "PROFILE_MOUTH_AXIAL_PEAK_INWARD_WALL";
    return out;
}

struct HoleChuShiFaXianState
{
    bool attempted = false;
    bool valid = false;
    bool accepted = false;
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    int support = 0;
    int sectors = 0;
    float coverage = 0.0f;
    float rmse = 0.0f;
    float mad = 0.0f;
    float radius = 0.0f;
    float deltaDegrees = 0.0f;
};

/**
 * 【每 seed 的常规初始粗法线】
 * 调用时机：Hole 口还没确定，只用于 mouth-search 建局部坐标系。
 * 数据/方法：收集 seed 周围 18.5 mm 点；HoleFaXian::guJiChuShiJuBuFaXian() 在 9/12/15/18 mm
 * 多尺度上做稳健 PCA 平面并选出质量最好的一个。精度目标只是“角度大致正确”。
 * 失败处理：如果周围没有足够实体支撑面，返回 invalid，上层继续使用整云缓存的全局粗法线。
 * 与精确椭圆法线不同：这里发生在 Hole 口建立之前，绝不决定最终输出法线。
 */
static HoleChuShiFaXianState guJiHoleChuShiJuBuFaXian(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& seedWorld,
    const Eigen::Vector3f& initialNormal,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleChuShiFaXianState out;
    static thread_local std::vector<HoleFaXian::DianYangBen> samples;
    mouthSearchCollectWorldSamples(cloud, seedWorld, 18.5f, kdtree, samples);
    const auto fit = HoleFaXian::guJiChuShiJuBuFaXian(
        samples, seedWorld.cast<double>(), initialNormal.cast<double>());
    out.valid = fit.valid;
    if (!fit.valid) return out;
    out.normal = fit.normal.cast<float>();
    if (!out.normal.allFinite() || out.normal.norm() < 1e-6f) {
        out.valid = false;
        return out;
    }
    out.normal.normalize();
    if (out.normal.z() < 0.0f) out.normal = -out.normal;
    out.support = fit.supportCount;
    out.sectors = fit.coveredSectors;
    out.coverage = static_cast<float>(fit.coverage);
    out.rmse = static_cast<float>(fit.rmse);
    out.mad = static_cast<float>(fit.mad);
    out.radius = static_cast<float>(fit.radiusUsed);
    out.deltaDegrees = static_cast<float>(fit.normalDeltaDegrees);
    return out;
}


struct HoleTuoYuanFaXianState
{
    bool attempted = false;
    bool valid = false;
    bool used = false;
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    float deltaDegrees = 0.0f;
    float initialScore = 0.0f;
    float finalScore = 0.0f;
    float initialCoverage = 0.0f;
    float finalCoverage = 0.0f;
    float initialRmse = 0.0f;
    float finalRmse = 0.0f;
    float ellipseAxisRatio = 1.0f;
    float ellipseTiltDegrees = 0.0f;
    float ellipseModelRmse = 0.0f;
    float mirrorScoreGap = 0.0f;
    std::string mode = "ARC_NORMAL_NOT_ATTEMPTED";
};

/** 【函数导航】
 * 作用：估计“jingQueHouXuanTuoYuanFaXian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static HoleTuoYuanFaXianState jingQueHouXuanTuoYuanFaXian(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& mouthCenter,
    const ShouDongJuBuZuoBiao& candidateFrame,
    float topRadius,
    const HoleHouXuanJuLei::Cluster& cluster,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleTuoYuanFaXianState out;
    if (!cloud || cloud->empty() || !mouthCenter.allFinite()
        || !candidateFrame.valid || !candidateFrame.n.allFinite()
        || candidateFrame.n.norm() < 1e-6f || !(topRadius > 1.0f)
        || !cluster.openArcEllipseValid) return out;
    Eigen::Vector3f initialNormal = candidateFrame.n.normalized();
    if (initialNormal.z() < 0.0f) initialNormal = -initialNormal;
    out.normal = initialNormal;

    static thread_local std::vector<HoleFaXian::DianYangBen> samples;
    const float queryRadius = std::max(topRadius + 6.0f, 2.20f * topRadius + 2.0f);
    mouthSearchCollectWorldSamples(cloud, mouthCenter, queryRadius, kdtree, samples);
    const HoleFaXian::YuanHuFaXianJieGuo direct =
        HoleFaXian::jingQueTuoYuanFanTuiFaXian(
            samples, mouthCenter.cast<double>(), initialNormal.cast<double>(),
            candidateFrame.u.cast<double>(), candidateFrame.v.cast<double>(),
            static_cast<double>(topRadius), cluster.openArcEllipseAxisRatio,
            cluster.openArcEllipseTiltU, cluster.openArcEllipseTiltV,
            cluster.openArcEllipseModelRmse, cluster.openArcCenterDriftU,
            cluster.openArcCenterDriftV, cluster.meanCoverage);
    out.valid = direct.valid;
    out.used = direct.used;
    out.normal = direct.normal.cast<float>();
    if (!out.normal.allFinite() || out.normal.norm() < 1e-6f) {
        out.valid = false;
        out.used = false;
        out.normal = initialNormal;
        return out;
    }
    out.normal.normalize();
    if (out.normal.z() < 0.0f) out.normal = -out.normal;
    out.deltaDegrees = static_cast<float>(direct.normalDeltaDegrees);
    out.initialScore = static_cast<float>(direct.initialScore);
    out.finalScore = static_cast<float>(direct.finalScore);
    out.initialCoverage = static_cast<float>(direct.initialCoverage);
    out.finalCoverage = static_cast<float>(direct.finalCoverage);
    out.initialRmse = static_cast<float>(direct.initialCircleRmse);
    out.finalRmse = static_cast<float>(direct.finalCircleRmse);
    out.ellipseAxisRatio = static_cast<float>(direct.ellipseAxisRatio);
    out.ellipseTiltDegrees = static_cast<float>(direct.ellipseTiltDegrees);
    out.ellipseModelRmse = static_cast<float>(direct.ellipseModelRmse);
    out.mirrorScoreGap = static_cast<float>(direct.mirrorScoreGap);
    out.mode = direct.mode ? direct.mode : "ARC_NORMAL_UNKNOWN";
    return out;
}


/**
 * 【Hole 口建立后的精确椭圆反推法线】
 * 这一步属于“测量阶段”，不是第三种初始法线。输入已经是已解析的真实 Hole 口中心、半径和椭圆证据；
 * 根据圆在倾斜观察平面中形成椭圆的轴比/主轴方向，一次解析反推出 Hole 法线（不做角度网格扫描）。
 * 当椭圆接近圆形、信息不足时保持初始法线；一旦 used=true，tuoYuanFaXianOwned 会阻止后续支撑面
 * 或外环法线覆盖这个精确结果，避免两套“最终法线”互相打架。
 */
static HoleTuoYuanFaXianState jingQueHoleKouTuoYuanFaXian(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& mouthCenter,
    const ShouDongJuBuZuoBiao& candidateFrame,
    float topRadius,
    const HoleHouXuanJuLei::HoleKouEllipseEvidence& ellipse,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleTuoYuanFaXianState out;
    if (!ellipse.valid || !cloud || cloud->empty() || !mouthCenter.allFinite()
        || !candidateFrame.valid || !(topRadius > 1.0f)) return out;

    Eigen::Vector3f initialNormal = candidateFrame.n;
    if (!initialNormal.allFinite() || initialNormal.norm() < 1e-6f) return out;
    initialNormal.normalize();
    if (initialNormal.z() < 0.0f) initialNormal = -initialNormal;
    out.normal = initialNormal;

    static thread_local std::vector<HoleFaXian::DianYangBen> samples;
    const float queryRadius = std::max(topRadius + 6.0f, 2.20f * topRadius + 2.0f);
    mouthSearchCollectWorldSamples(cloud, mouthCenter, queryRadius, kdtree, samples);
    // 真实上口已经由孔壁剖面确定，因此这里只把这一真实上口的椭圆二阶形变解析成孔轴。
    // 不使用原先粗候选（可能只是凸台外缘）的椭圆，也没有任何角度网格搜索。
    const HoleFaXian::YuanHuFaXianJieGuo direct =
        HoleFaXian::jingQueTuoYuanFanTuiFaXian(
            samples, mouthCenter.cast<double>(), initialNormal.cast<double>(),
            candidateFrame.u.cast<double>(), candidateFrame.v.cast<double>(),
            static_cast<double>(topRadius), ellipse.axisRatio,
            ellipse.tiltU, ellipse.tiltV, ellipse.modelRmse,
            0.0, 0.0, ellipse.coverage);
    out.valid = direct.valid;
    out.used = direct.used;
    out.normal = direct.normal.cast<float>();
    if (!out.normal.allFinite() || out.normal.norm() < 1e-6f) {
        out.valid = false;
        out.used = false;
        out.normal = initialNormal;
        return out;
    }
    out.normal.normalize();
    if (out.normal.z() < 0.0f) out.normal = -out.normal;
    out.deltaDegrees = static_cast<float>(direct.normalDeltaDegrees);
    out.initialScore = static_cast<float>(direct.initialScore);
    out.finalScore = static_cast<float>(direct.finalScore);
    out.initialCoverage = static_cast<float>(direct.initialCoverage);
    out.finalCoverage = static_cast<float>(direct.finalCoverage);
    out.initialRmse = static_cast<float>(direct.initialCircleRmse);
    out.finalRmse = static_cast<float>(direct.finalCircleRmse);
    out.ellipseAxisRatio = static_cast<float>(direct.ellipseAxisRatio);
    out.ellipseTiltDegrees = static_cast<float>(direct.ellipseTiltDegrees);
    out.ellipseModelRmse = static_cast<float>(direct.ellipseModelRmse);
    out.mirrorScoreGap = static_cast<float>(direct.mirrorScoreGap);
    out.mode = direct.mode ? direct.mode : "RESOLVED_MOUTH_ARC_NORMAL_UNKNOWN";
    return out;
}

struct WaiHuanFrameHouXuanDiag
{
    bool valid = false;
    int quality = 0;
    float innerRadius = 0.0f;
    float outerRadius = 0.0f;
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    int support = 0;
    int sectors = 0;
    float coverage = 0.0f;
    float rmse = 0.0f;
    float mad = 0.0f;
    float deltaDegrees = 0.0f;
    float score = 0.0f;
    std::string mode = "INVALID";
};

struct HoleWaiHuanFaXianState
{
    bool attempted = false;
    bool valid = false;
    bool used = false;
    int quality = 0;
    int selectedCandidate = -1;
    int usableCandidateCount = 0;
    float normalSpreadDegrees = 0.0f;
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    int support = 0;
    int sectors = 0;
    float coverage = 0.0f;
    float rmse = 0.0f;
    float mad = 0.0f;
    float deltaDegrees = 0.0f;
    std::array<WaiHuanFrameHouXuanDiag, 3> candidates{};
};

/** 【函数导航】
 * 作用：估计“guJiHoleWaiHuanFaXian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static HoleWaiHuanFaXianState guJiHoleWaiHuanFaXian(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& mouthCenter,
    const Eigen::Vector3f& initialNormal,
    float topRadius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    HoleWaiHuanFaXianState out;
    const float queryRadius = std::max(topRadius + 9.0f, 2.90f * topRadius + 1.0f);
    static thread_local std::vector<HoleFaXian::DianYangBen> samples;
    mouthSearchCollectWorldSamples(cloud, mouthCenter, queryRadius, kdtree, samples);
    const auto multi = HoleFaXian::guJiDuoChiDuWaiHuanFaXian(
        samples, mouthCenter.cast<double>(), initialNormal.cast<double>(), topRadius);
    out.selectedCandidate = multi.selectedIndex;
    out.usableCandidateCount = multi.usableCandidateCount;
    out.normalSpreadDegrees = static_cast<float>(multi.normalSpreadDegrees);
    for (std::size_t i = 0; i < out.candidates.size(); ++i) {
        const auto& fit = multi.candidates[i];
        auto& diag = out.candidates[i];
        diag.valid = fit.valid;
        diag.quality = fit.quality;
        diag.innerRadius = static_cast<float>(fit.annulusInnerRadius);
        diag.outerRadius = static_cast<float>(fit.annulusOuterRadius);
        diag.normal = fit.normal.cast<float>();
        if (diag.normal.allFinite() && diag.normal.norm() > 1e-6f) {
            diag.normal.normalize();
            if (diag.normal.dot(initialNormal) < 0.0f) diag.normal = -diag.normal;
        }
        diag.support = fit.supportCount;
        diag.sectors = fit.coveredSectors;
        diag.coverage = static_cast<float>(fit.coverage);
        diag.rmse = static_cast<float>(fit.rmse);
        diag.mad = static_cast<float>(fit.mad);
        diag.deltaDegrees = static_cast<float>(fit.normalDeltaDegrees);
        diag.score = static_cast<float>(fit.score);
        diag.mode = fit.mode ? fit.mode : "INVALID";
    }

    const auto& fit = multi.best;
    out.valid = fit.valid;
    out.quality = fit.quality;
    if (!fit.valid) return out;
    out.normal = fit.normal.cast<float>();
    if (!out.normal.allFinite() || out.normal.norm() < 1e-6f) {
        out.valid = false;
        out.quality = 0;
        return out;
    }
    out.normal.normalize();
    if (out.normal.dot(initialNormal) < 0.0f) out.normal = -out.normal;
    out.support = fit.supportCount;
    out.sectors = fit.coveredSectors;
    out.coverage = static_cast<float>(fit.coverage);
    out.rmse = static_cast<float>(fit.rmse);
    out.mad = static_cast<float>(fit.mad);
    out.deltaDegrees = static_cast<float>(fit.normalDeltaDegrees);
    return out;
}

/** 【函数导航】
 * 作用：执行“outerAnnulusFrameClusterScoreNormalized”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static double outerAnnulusFrameClusterScoreNormalized(double score) {
    if (!std::isfinite(score)) return 0.0;
    return 1.0 / (1.0 + std::exp(-0.25 * score));
}

/** 【函数导航】
 * 作用：执行“outerAnnulusFrameClusterConfidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static double outerAnnulusFrameClusterConfidence(
    const HoleHouXuanJuLei::Cluster& cluster)
{
    const double score = outerAnnulusFrameClusterScoreNormalized(cluster.score);
    const double mouth = cluster.mouthAttachmentValid ? 1.0 : 0.0;
    const double coverage = std::clamp(cluster.meanCoverage, 0.0, 1.0);
    const double layerSupport = std::clamp(
        static_cast<double>(cluster.supportLayers) / 5.0, 0.0, 1.0);
    const double continuity = std::clamp(cluster.trajectoryContinuity, 0.0, 1.0);
    const double layerConsistency = 0.5 * layerSupport + 0.5 * continuity;
    return std::clamp(0.35 * score + 0.25 * mouth
        + 0.20 * coverage + 0.20 * layerConsistency, 0.0, 1.0);
}

struct GuiFanSearchZhichengPlane
{
    bool valid = false;
    Eigen::Vector3f center{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    int support = 0;
    float mad = 0.0f;
};

/** 【函数导航】
 * 作用：估计“guJiCanonicalSearchClusterSupportPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static GuiFanSearchZhichengPlane guJiCanonicalSearchClusterSupportPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const Eigen::Vector3f& coarseCenter,
    Eigen::Vector3f initialNormal,
    float radius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    GuiFanSearchZhichengPlane out;
    out.center = coarseCenter;
    if (!cloud || cloud->empty() || !coarseCenter.allFinite()) return out;
    if (!initialNormal.allFinite() || initialNormal.norm() < 1e-6f)
        initialNormal = Eigen::Vector3f::UnitZ();
    initialNormal.normalize();
    if (initialNormal.z() < 0.0f) initialNormal = -initialNormal;
    const ShouDongJuBuZuoBiao initialFrame =
        gouJianShouDongJuBuZuoBiao(coarseCenter, initialNormal);
    if (!initialFrame.valid) return out;

    const float innerRadius = std::max(radius + 0.8f, 2.5f);
    const float outerRadius = std::max(radius + 8.0f, 10.0f);
    const float searchRadius = std::sqrt(outerRadius * outerRadius + 4.0f * 4.0f) + 0.5f;
    /** 【类型导航注释】
     * ZhichengWorkspace：Hole 识别总编排中的自定义 结构体。
     * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct ZhichengWorkspace {
        std::vector<Eigen::Vector3f> points;
        std::vector<unsigned char> active;
        std::vector<float> distances;
        std::vector<float> medianScratch;
        std::vector<float> deviations;
        std::vector<float> finalOffsets;
        std::vector<int> indices;
        std::vector<float> squaredDistances;
    };
    static thread_local ZhichengWorkspace workspace;
    auto& points = workspace.points;
    points.clear();
    if (points.capacity() < 4096) points.reserve(4096);

    auto consider = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        const Eigen::Vector3f world(p.x, p.y, p.z);
        if (!world.allFinite()) return;
        const Eigen::Vector3f delta = world - coarseCenter;
        const float u = delta.dot(initialFrame.u);
        const float v = delta.dot(initialFrame.v);
        const float w = delta.dot(initialFrame.n);
        const float radial = std::hypot(u, v);
        if (radial < innerRadius || radial > outerRadius || std::abs(w) > 4.0f) return;
        points.push_back(world);
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = coarseCenter.x();
        query.y = coarseCenter.y();
        query.z = coarseCenter.z();
        auto& indices = workspace.indices;
        auto& squaredDistances = workspace.squaredDistances;
        indices.clear();
        squaredDistances.clear();
        if (kdtree->radiusSearch(query, searchRadius, indices, squaredDistances) > 0) {
            for (int index : indices) consider(index);
        }
    } else {
        for (std::size_t index = 0; index < cloud->size(); ++index)
            consider(static_cast<int>(index));
    }
    if (points.size() < 40) {
            out.normal = initialNormal;
        out.support = static_cast<int>(points.size());
        return out;
    }

    auto& active = workspace.active;
    active.assign(points.size(), 1);
    Eigen::Vector3f normal = initialNormal;
    Eigen::Vector3f centroid = coarseCenter;
    float mad = 0.0f;
    // 外支撑面稳健精修固定 4 轮；增加轮数会直接增加耗时，只有出现未收敛证据时才调整。
    for (int iteration = 0; iteration < 4; ++iteration) {
        centroid.setZero();
        int count = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!active[i]) continue;
            centroid += points[i];
            ++count;
        }
        if (count < 30) break;
        centroid /= static_cast<float>(count);
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!active[i]) continue;
            const Eigen::Vector3f delta = points[i] - centroid;
            covariance += delta * delta.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        if (solver.info() != Eigen::Success) break;
        normal = solver.eigenvectors().col(0).normalized();
        if (normal.dot(initialNormal) < 0.0f) normal = -normal;

        auto& distances = workspace.distances;
        distances.clear();
        if (distances.capacity() < points.size()) distances.reserve(points.size());
        for (const auto& point : points)
            distances.push_back((point - centroid).dot(normal));
        auto& distanceCopy = workspace.medianScratch;
        distanceCopy.assign(distances.begin(), distances.end());
        const float planeOffset = shouDongZhongWeiShu(distanceCopy);
        auto& deviations = workspace.deviations;
        deviations.clear();
        if (deviations.capacity() < distances.size()) deviations.reserve(distances.size());
        for (float distance : distances) deviations.push_back(std::abs(distance - planeOffset));
        mad = shouDongZhongWeiShu(deviations);
        const float limit = std::clamp(3.0f * mad + 0.18f, 0.35f, 1.20f);
        for (std::size_t i = 0; i < active.size(); ++i)
            active[i] = std::abs(distances[i] - planeOffset) <= limit ? 1 : 0;
    }

    auto& finalOffsets = workspace.finalOffsets;
    finalOffsets.clear();
    if (finalOffsets.capacity() < points.size()) finalOffsets.reserve(points.size());
    int support = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!active[i]) continue;
        finalOffsets.push_back((points[i] - coarseCenter).dot(normal));
        ++support;
    }
    if (!finalOffsets.empty()) {
        const float shift = shouDongZhongWeiShu(finalOffsets);
        out.center = coarseCenter + normal * shift;
    }
    out.valid = support >= 25;
    out.normal = normal;
    out.support = support;
    out.mad = mad;

    // 小外表面孔口保护：铸造件的真实上口平台可能只在孔沿外侧很窄的一圈内存在。
    // 原宽环带 [R+0.8, R+8] 在大平面试件上很稳，但当孔口平台很小、再往外已经
    // 落到更低的铸造曲面时，宽环带会把两个不同高度的表面混在一起，从而把机械
    // 上口沿孔内方向压低。这里从已经收集到的外环点中再拟合一份“近口窄环带”
    // [R+0.8, R+2.2]。只有窄环带自身平整、角向覆盖充分，并且相对宽环带明显更
    // 靠外时才接管上口支撑面；正常大平面两者几乎重合，因此不会改变既有结果。
    GuiFanSearchZhichengPlane nearMouthSupport;
    nearMouthSupport.center = coarseCenter;
    nearMouthSupport.normal = initialNormal;
    const float nearOuterRadius = std::max(radius + 2.2f, innerRadius + 0.8f);
    std::vector<Eigen::Vector3f> nearPoints;
    nearPoints.reserve(points.size());
    for (const Eigen::Vector3f& point : points) {
        const Eigen::Vector3f delta = point - coarseCenter;
        const float u = delta.dot(initialFrame.u);
        const float v = delta.dot(initialFrame.v);
        const float radial = std::hypot(u, v);
        if (radial <= nearOuterRadius) nearPoints.push_back(point);
    }

    int nearAngularSectors = 0;
    if (nearPoints.size() >= 40U) {
        std::vector<unsigned char> nearActive(nearPoints.size(), 1);
        Eigen::Vector3f nearNormal = initialNormal;
        Eigen::Vector3f nearCentroid = coarseCenter;
        float nearMad = 0.0f;
        for (int iteration = 0; iteration < 4; ++iteration) {
            nearCentroid.setZero();
            int count = 0;
            for (std::size_t i = 0; i < nearPoints.size(); ++i) {
                if (!nearActive[i]) continue;
                nearCentroid += nearPoints[i];
                ++count;
            }
            if (count < 30) break;
            nearCentroid /= static_cast<float>(count);
            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (std::size_t i = 0; i < nearPoints.size(); ++i) {
                if (!nearActive[i]) continue;
                const Eigen::Vector3f delta = nearPoints[i] - nearCentroid;
                covariance += delta * delta.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            if (solver.info() != Eigen::Success) break;
            nearNormal = solver.eigenvectors().col(0).normalized();
            if (nearNormal.dot(initialNormal) < 0.0f) nearNormal = -nearNormal;

            std::vector<float> distances;
            distances.reserve(nearPoints.size());
            for (const auto& point : nearPoints)
                distances.push_back((point - nearCentroid).dot(nearNormal));
            std::vector<float> scratch = distances;
            const float planeOffset = shouDongZhongWeiShu(scratch);
            std::vector<float> deviations;
            deviations.reserve(distances.size());
            for (float distance : distances)
                deviations.push_back(std::abs(distance - planeOffset));
            nearMad = shouDongZhongWeiShu(deviations);
            const float limit = std::clamp(3.0f * nearMad + 0.18f, 0.35f, 0.85f);
            for (std::size_t i = 0; i < nearActive.size(); ++i)
                nearActive[i] = std::abs(distances[i] - planeOffset) <= limit ? 1 : 0;
        }

        std::vector<float> nearOffsets;
        nearOffsets.reserve(nearPoints.size());
        std::array<int, 12> sectorCounts{};
        int nearSupportCount = 0;
        for (std::size_t i = 0; i < nearPoints.size(); ++i) {
            if (!nearActive[i]) continue;
            nearOffsets.push_back((nearPoints[i] - coarseCenter).dot(nearNormal));
            const Eigen::Vector3f delta = nearPoints[i] - coarseCenter;
            const float u = delta.dot(initialFrame.u);
            const float v = delta.dot(initialFrame.v);
            double angle = std::atan2(static_cast<double>(v), static_cast<double>(u));
            if (angle < 0.0) angle += 2.0 * M_PI;
            int sector = static_cast<int>(std::floor(angle / (2.0 * M_PI / 12.0)));
            sector = std::clamp(sector, 0, 11);
            ++sectorCounts[static_cast<std::size_t>(sector)];
            ++nearSupportCount;
        }
        for (int count : sectorCounts) {
            if (count >= 3) ++nearAngularSectors;
        }
        if (!nearOffsets.empty()) {
            const float shift = shouDongZhongWeiShu(nearOffsets);
            nearMouthSupport.center = coarseCenter + nearNormal * shift;
        }
        nearMouthSupport.normal = nearNormal;
        nearMouthSupport.support = nearSupportCount;
        nearMouthSupport.mad = nearMad;
        const float normalAlignment = std::abs(nearNormal.dot(initialNormal));
        nearMouthSupport.valid = nearSupportCount >= 40
            && nearAngularSectors >= 6
            && nearMad <= 0.22f
            && normalAlignment >= 0.975f;
    }

    if (nearMouthSupport.valid) {
        const float outwardShift = (nearMouthSupport.center - out.center).dot(initialNormal);
        const bool broadSupportWeak = !out.valid || out.support < 45 || out.mad > 0.35f;
        const bool nearPlaneClearlyOutside = !out.valid || outwardShift >= 0.18f;
        if (nearPlaneClearlyOutside || (broadSupportWeak && outwardShift >= -0.05f)) {
            out = nearMouthSupport;
        }
    }

    if (!out.valid) {
            out.center = coarseCenter;
        out.normal = initialNormal;
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“canonicalSearchClusterSummary”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static std::string canonicalSearchClusterSummary(const HoleHouXuanJuLei::Result& result)
{
    std::ostringstream stream;
    for (std::size_t i = 0; i < result.clusters.size(); ++i) {
        if (i != 0) stream << '|';
        const auto& cluster = result.clusters[i];
        stream << cluster.id << ',' << (cluster.stable ? 1 : 0)
               << ',' << cluster.consensusCenterU << ',' << cluster.consensusCenterV
               << ',' << cluster.consensusTopW << ',' << cluster.consensusTopRadius
               << ',' << cluster.supportLayers << ',' << cluster.candidateCount
               << ',' << cluster.centerStd << ',' << cluster.radiusStd
               << ',' << cluster.meanCoverage << ',' << cluster.meanResidual
               << ',' << cluster.radiusTrendSlope << ',' << cluster.radiusTrendRmse
               << ',' << cluster.radiusTrendMonotonicity << ',' << cluster.radiusTrendSpan
               << ',' << cluster.radiusTrendLayers << ',' << cluster.stabilityMode
               << ',' << cluster.score << ',' << cluster.selectionCost
               << ',' << cluster.role << ',' << cluster.mouthPlaneDistance
               << ',' << (cluster.mouthAttachmentValid ? 1 : 0)
               << ',' << (cluster.deepContinuation ? 1 : 0)
               << ',' << cluster.parentMouthClusterId << ',' << cluster.roleReason;
    }
    return stream.str();
}

/** 【函数导航】
 * 作用：执行“profileAttachmentJoinDoubles”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static std::string profileAttachmentJoinDoubles(const std::vector<double>& values)
{
    std::ostringstream stream;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) stream << '|';
        stream << values[i];
    }
    return stream.str();
}

/** 【函数导航】
 * 作用：执行“profileAttachmentJoinInts”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static std::string profileAttachmentJoinInts(const std::vector<int>& values)
{
    std::ostringstream stream;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) stream << '|';
        stream << values[i];
    }
    return stream.str();
}

struct GuiFanRoi
{
    std::vector<HoleJiheFinal::Sample> samples;
    std::size_t pointCount = 0;
    std::uint64_t hash = 1469598103934665603ULL;
    float radialLimit = 0.0f;
    float outwardLimit = 0.0f;
    float inwardLimit = 0.0f;

};

/** 【函数导航】
 * 作用：执行“canonicalRoiHashBytes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static void canonicalRoiHashBytes(std::uint64_t& hash, const void* data, std::size_t size)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= 1099511628211ULL;
    }
}

/** 【函数导航】
 * 作用：构建“gouJianCanonicalRoiCanonicalRoi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static GuiFanRoi gouJianCanonicalRoiCanonicalRoi(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const ShouDongJuBuZuoBiao& frame,
    int inwardPolarity,
    float topRadius,
    const pcl::KdTreeFLANN<pcl::PointXYZRGB>* kdtree)
{
    GuiFanRoi out;
    if (!cloud || cloud->empty() || !frame.valid || !(topRadius > 0.0f)) return out;

    out.radialLimit = std::max(12.0f, topRadius + 4.0f);

    (void)inwardPolarity;
    out.outwardLimit = 13.0f;
    out.inwardLimit = 13.0f;
    const float axialMax = 13.0f;
    const float sphereRadius = std::sqrt(
        out.radialLimit * out.radialLimit + axialMax * axialMax) + 0.5f;

    /** 【类型导航注释】
     * PaiXuPoint：Hole 识别总编排中的自定义 结构体。
     * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct PaiXuPoint {
        float u = 0.0f;
        float v = 0.0f;
        float w = 0.0f;
        long long quantizedDepth = 0;
        long long quantizedRadius = 0;
        long long quantizedAngle = 0;
        int originalIndex = -1;
    };
    static thread_local std::vector<PaiXuPoint> accepted;
    accepted.clear();
    if (accepted.capacity() < 8192) accepted.reserve(8192);

    auto consider = [&](int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) return;
        const auto& p = (*cloud)[static_cast<std::size_t>(index)];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
        const Eigen::Vector3f world(p.x, p.y, p.z);
        const Eigen::Vector3f delta = world - frame.origin;
        const float u = delta.dot(frame.u);
        const float v = delta.dot(frame.v);
        const float w = delta.dot(frame.n);
        const float depth = w;

        const float radius = std::hypot(u, v);
        if (radius > out.radialLimit) return;
        if (depth < -out.outwardLimit || depth > out.inwardLimit) return;
        float angle = std::atan2(v, u);
        if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
        accepted.push_back({u, v, w,
            HoleFastPath::quantize1e4(depth),
            HoleFastPath::quantize1e4(radius),
            HoleFastPath::quantize1e4(angle), index});
    };

    if (kdtree) {
        pcl::PointXYZRGB query;
        query.x = frame.origin.x();
        query.y = frame.origin.y();
        query.z = frame.origin.z();
        static thread_local std::vector<int> indices;
        static thread_local std::vector<float> squaredDistances;
        indices.clear();
        squaredDistances.clear();
        if (kdtree->radiusSearch(query, sphereRadius, indices, squaredDistances) > 0) {
            for (int index : indices) consider(index);
        }
    } else {
        for (std::size_t index = 0; index < cloud->size(); ++index)
            consider(static_cast<int>(index));
    }

    std::sort(accepted.begin(), accepted.end(), [](const PaiXuPoint& a, const PaiXuPoint& b) {
        if (a.quantizedDepth != b.quantizedDepth)
            return a.quantizedDepth < b.quantizedDepth;
        if (a.quantizedRadius != b.quantizedRadius)
            return a.quantizedRadius < b.quantizedRadius;
        if (a.quantizedAngle != b.quantizedAngle)
            return a.quantizedAngle < b.quantizedAngle;
        return a.originalIndex < b.originalIndex;
    });
    out.pointCount = accepted.size();
    out.samples.reserve(accepted.size());
    for (const PaiXuPoint& item : accepted) {
        out.samples.push_back({item.u, item.v, item.w});
        canonicalRoiHashBytes(out.hash, &item.originalIndex, sizeof(item.originalIndex));
        canonicalRoiHashBytes(out.hash, &item.u, sizeof(item.u));
        canonicalRoiHashBytes(out.hash, &item.v, sizeof(item.v));
        canonicalRoiHashBytes(out.hash, &item.w, sizeof(item.w));
    }
    return out;
}
// 备忘：这个函数把孔描述从局部坐标统一转回世界坐标，保证 UI 和保存结果能直接使用。

/** 【函数导航】
 * 作用：执行“shouDongHoleMiaoShuDaoShiJie”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static void shouDongHoleMiaoShuDaoShiJie(HoleMiaoshu& d, const ShouDongJuBuZuoBiao& frame)
{
    d.center = juBuZuoBiaoDaoShiJie(frame, d.center);
    d.centerTop = juBuZuoBiaoDaoShiJie(frame, d.centerTop);
    d.centerBot = juBuZuoBiaoDaoShiJie(frame, d.centerBot);
    if (d.mouthGridExecuted) {
        d.mouthGridCenter = juBuZuoBiaoDaoShiJie(frame, d.mouthGridCenter);
    }

    d.localPlaneNx = frame.n.x();
    d.localPlaneNy = frame.n.y();
    d.localPlaneNz = frame.n.z();
    d.sourcePath = d.sourcePath.empty() ? "ShouDongJuBuROI" : d.sourcePath;
    d.sourceReason = d.sourceReason.empty() ? "UserClickedSeed_LocalNormal_DirectMeasurement" : d.sourceReason;
    if (d.seedSourcePath.empty()) d.seedSourcePath = "ShouDongJuBuROI";
}
// 备忘：这个函数比较两个孔识别结果谁更可信，用在多姿态候选结果选择。

/** 【函数导航】
 * 作用：执行“shouDongHoleMiaoShuGengYou”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static bool shouDongHoleMiaoShuGengYou(const HoleMiaoshu& a, const HoleMiaoshu& b)
{
    if (a.highConfidenceScore != b.highConfidenceScore)
        return a.highConfidenceScore > b.highConfidenceScore;
    if (a.finalHoleJudgment != b.finalHoleJudgment)
        return a.finalHoleJudgment == HoleMiaoshu::FinalHolePanDing::TrueHole;
    if (a.validLayerCount != b.validLayerCount)
        return a.validLayerCount > b.validLayerCount;
    return a.depthMeasured > b.depthMeasured;
}

}

namespace {

using holeJihe::kasaZhongWeiShu;
using holeJihe::kasaNiHeYuan;
using holeJihe::kasaJiaoJunBianJie;
using holeJihe::kasaJieDuanNiHe;
using holeJihe::ransacNiHeYuanSanDian;
using holeJihe::ransacNiHeSanWeiYuan;
using holeJihe::shouDongZhongWeiShu;
using holeJihe::shouDongFenWeiShu;
using holeZhicheng::shouDongZhuZaiFaXiangGao;
using holeZhicheng::shouDongJinLinZhuZaiFaXiangGao;
using holeZhicheng::shouDongJinLinZhiChengMianFaXiangGao;
using holeShendu::ShouDongFenCengTongJi;
using holeShendu::shouDongFenCengBanJing;
using holeShendu::shouDongZhongWeiBanJing;
using holeShendu::shouDongJinLinShenDuBanJing;
using holeShendu::ShouDongJingXiangShenDu;
using holeShendu::shouDongJingXiangShenDu;
using holeKou::ShouDongZhuiHoleKouBaoLuoJingXiu;
using holeKou::ShouDongZhuiBiKouBuWaiTui;
using holeKou::shouDongGuJiZhuiHoleKouBuBaoLuo;
using holeKou::shouDongGuJiZhuiBiKouBuWaiTui;
using holeKou::ShouDongWangGeHoleKou;
using holeKou::guJiWangGeHoleKouYinYing;
using holeKou::guJiDaBanJingHoleKouWaiBianJie;

/** 【函数导航】
 * 作用：执行“angleDegrees”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、ShouDongHole_JiheJianCe.cpp、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static float angleDegrees(
    const shouDongHole::Vec3d& a,
    const shouDongHole::Vec3d& b)
{
    const double an = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    const double bn = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    if (!(an > 1e-12) || !(bn > 1e-12)) return 0.0f;
    const double cosine = std::clamp(
        std::abs((a.x * b.x + a.y * b.y + a.z * b.z) / (an * bn)),
        0.0, 1.0);
    return static_cast<float>(std::acos(cosine) * 180.0 / M_PI);
}

/** 【函数导航】
 * 作用：执行“mouthCenterWorld”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static Eigen::Vector3f mouthCenterWorld(
    const ShouDongHoleSeed& seed,
    const shouDongHole::PclJianCeResult& detection)
{
    (void)seed;
    const auto& g = detection.geometry;
    return Eigen::Vector3f(
        static_cast<float>(g.centerX),
        static_cast<float>(g.centerY),
        static_cast<float>(detection.centerZ));
}

/** 【函数导航】
 * 作用：构建“buildSurfaceTuoDiDescriptor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static void buildSurfaceTuoDiDescriptor(
    HoleMiaoshu& d,
    float radius,
    float confidence)
{
    d.centerTop = Eigen::Vector3f::Zero();
    d.centerBot = Eigen::Vector3f::Zero();
    d.center = Eigen::Vector3f::Zero();
    d.radius = radius;
    d.rTop = radius;
    d.rBot = radius;
    d.depth = 0.0f;
    d.depthMeasured = 0.0f;
    d.depthFinal = 0.0f;
    d.depthSource = 2;
    d.depthClamped = false;
    d.validLayerCount = 0;
    d.type = 1;
    d.slopeDeg = 0.0f;
    d.geometryType = HoleMiaoshu::JiheType::Straight;
    d.finalGeometryType = HoleMiaoshu::FinalJiheType::Straight;
    d.physicalHoleTypeDisplay = HoleMiaoshu::WuLiHoleLeixingXianshi::Straight;
    d.finalHoleJudgment = HoleMiaoshu::FinalHolePanDing::TrueHole;
    d.highConfidenceStatus = HoleMiaoshu::HighConfidenceStatus::HighConfidenceHole;
    d.highConfidenceScore = confidence;
    d.parameterReliability = "NeedsReview";
    d.parameterReliabilityReason = "孔口已确认；深度和孔形暂时无法可靠测量";
    d.centerReliability = "High";
    d.radiusReliability = "High";
    d.depthReliability = "Low";
    d.slopeReliability = "Low";
    d.typeReliability = "Low";
    d.guiDisplayStatus = "Visible";
    d.displayStatus = "Visible";
    d.displayDecisionReason = "ManualHole_MouthOnlyFallback";
}

// 每个识别线程复用自己的整云 KDTree、全局法向和自适应帧提示，避免多线程共享 PCL 搜索器内部状态。
// 同一线程切换点云时，用对象地址、点数据地址、尺寸和 XYZ 位哈希共同判断缓存是否失效；
// 这样既避免重复建树，也能在点云内容发生原地修改时及时重建。
struct FenxiCacheZhichengPlaneCacheEntry
{
    std::array<std::uint32_t, 7> key{};
    GuiFanSearchZhichengPlane support;
};

/** 【函数导航】
 * 作用：执行“analysisCacheSupportPlaneKey”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static std::array<std::uint32_t, 7> analysisCacheSupportPlaneKey(
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& normal,
    float radius) noexcept
{
    return {{
        DianYunBiaoshi::floatBits(center.x()),
        DianYunBiaoshi::floatBits(center.y()),
        DianYunBiaoshi::floatBits(center.z()),
        DianYunBiaoshi::floatBits(normal.x()),
        DianYunBiaoshi::floatBits(normal.y()),
        DianYunBiaoshi::floatBits(normal.z()),
        DianYunBiaoshi::floatBits(radius)}};
}

struct FenxiCacheGuiFanJiheCacheEntry
{
    std::uint64_t canonicalHash = 0;
    std::uint64_t initialRadiusBits = 0;
    std::uint64_t canonicalCenterUBits = 0;
    std::uint64_t canonicalCenterVBits = 0;
    std::vector<HoleJiheFinal::Sample> samples;
    HoleJiheFinal::Result geometry;
};

/** 【函数导航】
 * 作用：执行“analysisCacheSameCanonicalSamples”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static bool analysisCacheSameCanonicalSamples(
    const std::vector<HoleJiheFinal::Sample>& left,
    const std::vector<HoleJiheFinal::Sample>& right) noexcept
{
    if (left.size() != right.size()) return false;
    for (std::size_t i = 0; i < left.size(); ++i) {
        if (HoleFastPath::doubleBits(left[i].u) != HoleFastPath::doubleBits(right[i].u)
            || HoleFastPath::doubleBits(left[i].v) != HoleFastPath::doubleBits(right[i].v)
            || HoleFastPath::doubleBits(left[i].w) != HoleFastPath::doubleBits(right[i].w)) {
            return false;
        }
    }
    return true;
}

struct ZuoBiaoXiTishi
{
    Eigen::Vector3f center{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
    float radius = 0.0f;
    int clusterId = -1;
    double consensusCenterU = 0.0;
    double consensusCenterV = 0.0;
    double consensusTopW = 0.0;
    double consensusTopRadius = 0.0;
    bool frozenClusterValid = false;
};


struct KongJianPrewarmQuickDianYunKey
{
    const void* cloudObject = nullptr;
    const void* pointsData = nullptr;
    std::size_t size = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

/** 【函数导航】
 * 作用：执行“spatialPrewarmQuickKey”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static KongJianPrewarmQuickDianYunKey spatialPrewarmQuickKey(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) noexcept
{
    KongJianPrewarmQuickDianYunKey key;
    if (!cloud) return key;
    key.cloudObject = cloud.get();
    key.pointsData = cloud->points.empty() ? nullptr : cloud->points.data();
    key.size = cloud->size();
    key.width = cloud->width;
    key.height = cloud->height;
    return key;
}

/** 【函数导航】
 * 作用：执行“spatialPrewarmSameQuickKey”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static bool spatialPrewarmSameQuickKey(
    const KongJianPrewarmQuickDianYunKey& a,
    const KongJianPrewarmQuickDianYunKey& b) noexcept
{
    return a.cloudObject == b.cloudObject
        && a.pointsData == b.pointsData
        && a.size == b.size
        && a.width == b.width
        && a.height == b.height;
}

struct KongJianYuReJieGuo
{
    KongJianPrewarmQuickDianYunKey quickKey;
    DianYunBiaoshi::ZhiWen fingerprint;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB>> tree;
    shouDongHole::Vec3d globalNormal{0.0, 0.0, 1.0};
};

struct KongJianYuReDengJi
{
    /** 【类型导航注释】
     * State：Hole 识别总编排中的自定义 枚举。
     * 主要使用位置：HoleFenxi_Analysis.cpp、HoleFenxi_Analysis.h。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class State { Idle, Building, Ready };
    std::mutex mutex;
    std::condition_variable cv;
    State state = State::Idle;
    KongJianPrewarmQuickDianYunKey key;
    std::uint64_t generation = 0;
    std::shared_ptr<KongJianYuReJieGuo> prepared;
};

/** 【函数导航】
 * 作用：执行“spatialPrewarmRegistry”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static KongJianYuReDengJi& spatialPrewarmRegistry()
{

    static KongJianYuReDengJi* registry = new KongJianYuReDengJi();
    return *registry;
}


/** 【函数导航】
 * 作用：执行“prewarmHoleShibieKongJianContextImpl”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void prewarmHoleShibieKongJianContextImpl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    if (!cloud || cloud->empty()) return;
    KongJianYuReDengJi& registry = spatialPrewarmRegistry();
    const KongJianPrewarmQuickDianYunKey key = spatialPrewarmQuickKey(cloud);
    std::uint64_t myGeneration = 0;
    {
        std::unique_lock<std::mutex> lock(registry.mutex);
        if (spatialPrewarmSameQuickKey(registry.key, key)) {
            if (registry.state == KongJianYuReDengJi::State::Ready && registry.prepared) return;
            if (registry.state == KongJianYuReDengJi::State::Building) return;
        }
        registry.key = key;
        registry.prepared.reset();
        registry.state = KongJianYuReDengJi::State::Building;
        myGeneration = ++registry.generation;
    }
    auto prepared = std::make_shared<KongJianYuReJieGuo>();
    prepared->quickKey = key;
    bool buildOk = false;
    try {
        prepared->fingerprint = DianYunBiaoshi::fingerprintDianYun(cloud);
        prepared->tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB>>();
        prepared->tree->setInputCloud(cloud);
        prepared->globalNormal = shouDongHole::estimateGlobalNormalDeterministicPclWithTreeUncached(
            cloud, *prepared->tree);
        buildOk = static_cast<bool>(prepared->tree);
    } catch (...) {
        buildOk = false;
    }
    {
        std::unique_lock<std::mutex> lock(registry.mutex);
        if (registry.generation == myGeneration && spatialPrewarmSameQuickKey(registry.key, key)) {
            if (buildOk) {
                registry.prepared = prepared;
                registry.state = KongJianYuReDengJi::State::Ready;
            } else {
                registry.prepared.reset();
                registry.state = KongJianYuReDengJi::State::Idle;
            }
        }
    }
    registry.cv.notify_all();
}


/** 【函数导航】
 * 作用：执行“spatialPrewarmTryAcquirePrepared”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static std::shared_ptr<KongJianYuReJieGuo> spatialPrewarmTryAcquirePrepared(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    if (!cloud || cloud->empty()) return {};
    KongJianYuReDengJi& registry = spatialPrewarmRegistry();
    const KongJianPrewarmQuickDianYunKey key = spatialPrewarmQuickKey(cloud);
    std::unique_lock<std::mutex> lock(registry.mutex);
    if (registry.state == KongJianYuReDengJi::State::Building
        && spatialPrewarmSameQuickKey(registry.key, key)) {
        (void)registry.cv.wait_for(lock, std::chrono::seconds(2), [&]() {
            return registry.state != KongJianYuReDengJi::State::Building
                || !spatialPrewarmSameQuickKey(registry.key, key);
        });
    }
    if (registry.state != KongJianYuReDengJi::State::Ready
        || !registry.prepared
        || !spatialPrewarmSameQuickKey(registry.key, key)) {
        return {};
    }
    // 预热空间核心是只读共享资源，不能被第一个 QThreadPool 工作线程 move 掉。
    // 保留它以后，不同 QThreadPool 工作线程处理同一份点云时都能复用同一棵 PCL KD-tree
    // 和全局法向，不再因为线程迁移反复重建。
    return registry.prepared;
}


struct FenxiCacheThreadDianYunFenxiCache
{
    KongJianPrewarmQuickDianYunKey quickKey;
    DianYunBiaoshi::ZhiWen fingerprint;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB>> tree;
    shouDongHole::Vec3d globalNormal{0.0, 0.0, 1.0};
    bool globalNormalValid = false;
    std::vector<ZuoBiaoXiTishi> frameHints;
    std::vector<FenxiCacheZhichengPlaneCacheEntry> supportPlanes;
    std::vector<FenxiCacheGuiFanJiheCacheEntry> canonicalGeometry;
    std::vector<FenxiCacheWeiziPairCacheEntry> posePairs;
};

/** 【函数导航】
 * 作用：执行“analysisCacheAcquireThreadDianYunFenxi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static FenxiCacheThreadDianYunFenxiCache* analysisCacheAcquireThreadDianYunFenxi(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    thread_local FenxiCacheThreadDianYunFenxiCache cache;
    if (!cloud || cloud->empty()) return nullptr;
    const KongJianPrewarmQuickDianYunKey quickKey = spatialPrewarmQuickKey(cloud);

    // 同一个工作线程再次处理同一 Cloud 对象时，先用 O(1) 身份键命中，避免重复计算稳定云身份。
    // 40 万级点云做完整 XYZ 双哈希，缓存虽然命中，仍然承担 O(N) 扫描成本。
    if (cache.tree && cache.globalNormalValid
        && spatialPrewarmSameQuickKey(cache.quickKey, quickKey)) {
        return &cache;
    }

    // Qt 全局线程池可能把连续两次识别调度到不同 QThreadPool 工作线程。优先取得全局保留的
    // 预热空间核心；这条热路径同样只比较 O(1) 身份键，不重新遍历整云。
    const std::shared_ptr<KongJianYuReJieGuo> prepared =
        spatialPrewarmTryAcquirePrepared(cloud);
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB>> tree;
    shouDongHole::Vec3d globalNormal{0.0, 0.0, 1.0};
    DianYunBiaoshi::ZhiWen fingerprint;
    if (prepared && prepared->tree
        && spatialPrewarmSameQuickKey(prepared->quickKey, quickKey)) {
        tree = prepared->tree;
        globalNormal = prepared->globalNormal;
        fingerprint = prepared->fingerprint;
    } else {
        // 只有首次处理新点云、或界面没有来得及预热时才承担完整指纹 + KD-tree + 全局法向建立。
        fingerprint = DianYunBiaoshi::fingerprintDianYun(cloud);
        tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB>>();
        tree->setInputCloud(cloud);
        globalNormal = shouDongHole::estimateGlobalNormalDeterministicPclWithTreeUncached(
            cloud, *tree);

        // 无预热直接识别也把建立好的空间核心回填到全局注册表，后续不同 Qt 工作线程可复用。
        KongJianYuReDengJi& registry = spatialPrewarmRegistry();
        auto retained = std::make_shared<KongJianYuReJieGuo>();
        retained->quickKey = quickKey;
        retained->fingerprint = fingerprint;
        retained->tree = tree;
        retained->globalNormal = globalNormal;
        {
            std::unique_lock<std::mutex> lock(registry.mutex);
            registry.key = quickKey;
            registry.prepared = retained;
            registry.state = KongJianYuReDengJi::State::Ready;
            ++registry.generation;
        }
        registry.cv.notify_all();
    }
    cache.quickKey = quickKey;
    cache.fingerprint = fingerprint;
    cache.tree = std::move(tree);
    cache.globalNormal = globalNormal;
    cache.globalNormalValid = true;
    cache.frameHints.clear();
    cache.supportPlanes.clear();
    cache.canonicalGeometry.clear();
    cache.posePairs.clear();
    return &cache;
}



/** 【函数导航】
 * 作用：执行“shouDongJuBuHoleShibieProductionImpl”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
static HoleShibieResult shouDongJuBuHoleShibieProductionImpl(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const std::vector<ShouDongHoleSeed>& seeds,
    const char* cloudName)
{
    (void)cloudName;

    HoleShibieResult result;
    if (!cloud || cloud->empty()) {
        result.error = QStringLiteral("点云为空，无法执行手动种子孔识别。");
        return result;
    }
    if (seeds.empty()) {
        result.error = QStringLiteral("未选择孔种子点。");
        return result;
    }

    FenxiCacheThreadDianYunFenxiCache* cloudAnalysis =
        analysisCacheAcquireThreadDianYunFenxi(cloud);
    if (!cloudAnalysis || !cloudAnalysis->tree) {
        result.error = QStringLiteral("AnalysisCache整云分析缓存建立失败。");
        return result;
    }
    pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree = *cloudAnalysis->tree;
    const shouDongHole::Vec3d cachedGlobalNormalD = cloudAnalysis->globalNormal;
    std::vector<ZuoBiaoXiTishi>& frameHintFrameHints = cloudAnalysis->frameHints;
    std::vector<FenxiCacheZhichengPlaneCacheEntry>& analysisCacheSupportPlanes =
        cloudAnalysis->supportPlanes;
    std::vector<FenxiCacheGuiFanJiheCacheEntry>& analysisCacheCanonicalGeometry =
        cloudAnalysis->canonicalGeometry;
    std::vector<FenxiCacheWeiziPairCacheEntry>& analysisCachePosePairs =
        cloudAnalysis->posePairs;
    std::vector<HoleMiaoshu> accepted;
    accepted.reserve(seeds.size());
    std::vector<int> acceptedClusterIds;
    std::vector<std::uint64_t> acceptedCanonicalHashes;
    acceptedClusterIds.reserve(seeds.size());
    acceptedCanonicalHashes.reserve(seeds.size());
    for (size_t si = 0; si < seeds.size(); ++si) {
        const ShouDongHoleSeed& seed = seeds[si];
        /** 【类型导航注释】
         * Sb11HoleKouAttempt：Hole 识别总编排中的自定义 结构体。
         * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
         * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
         */
        struct Sb11HoleKouAttempt {
            bool valid = false;
            ShouDongHoleSeed trialSeed;
            shouDongHole::PclJianCeResult detection;
            Eigen::Vector3f mouthCenterWorld{0.0f, 0.0f, 0.0f};
            Eigen::Vector3f mouthNormal{0.0f, 0.0f, 1.0f};
            ShouDongJuBuZuoBiao frame;
            double normalOffsetMm = 0.0;
            double score = -std::numeric_limits<double>::infinity();
            QString rejectCode;
        };

        Sb11HoleKouAttempt selectedAttempt;
        const QString primaryRejectCode = QStringLiteral("SKIPPED_CLUSTER_FIRST_PRIMARY");
        const QString retryAudit = QStringLiteral("[CandidateRouting_LEGACY_HINT_REMOVED_AnalysisCache]");

        const GuiEntryCelue::LuYouJueDing candidateRoutingRouting =
            GuiEntryCelue::decideRouting(false, false);
        const bool candidateRoutingPreGateSucceeded = candidateRoutingRouting.preGateSucceeded;
        const QString candidateRoutingPreGateRejectCode = candidateRoutingPreGateSucceeded
            ? QStringLiteral("OK") : primaryRejectCode;
        const QString candidateRoutingRetryAudit = retryAudit;
        bool candidateRoutingDirectClusterEntryUsed = candidateRoutingRouting.directClusterFirst;
        if (candidateRoutingDirectClusterEntryUsed) {
            shouDongHole::Vec3d tuoDiNormalD = selectedAttempt.detection.globalNormal;
            double tuoDiNormalNorm = std::sqrt(
                tuoDiNormalD.x * tuoDiNormalD.x
                + tuoDiNormalD.y * tuoDiNormalD.y
                + tuoDiNormalD.z * tuoDiNormalD.z);
            if (!(tuoDiNormalNorm > 1e-12) || !std::isfinite(tuoDiNormalNorm)) {
                tuoDiNormalD = cachedGlobalNormalD;
                tuoDiNormalNorm = std::sqrt(
                    tuoDiNormalD.x * tuoDiNormalD.x
                    + tuoDiNormalD.y * tuoDiNormalD.y
                    + tuoDiNormalD.z * tuoDiNormalD.z);
            }
            Eigen::Vector3f tuoDiNormal(
                static_cast<float>(tuoDiNormalD.x),
                static_cast<float>(tuoDiNormalD.y),
                static_cast<float>(tuoDiNormalD.z));
            if (!tuoDiNormal.allFinite() || tuoDiNormal.norm() < 1e-6f) {
                tuoDiNormal = Eigen::Vector3f::UnitZ();
                tuoDiNormalD = {0.0, 0.0, 1.0};
            } else {
                tuoDiNormal.normalize();
                if (tuoDiNormal.z() < 0.0f) tuoDiNormal = -tuoDiNormal;
                tuoDiNormalD = {tuoDiNormal.x(), tuoDiNormal.y(), tuoDiNormal.z()};
            }

            const double minR = std::isfinite(seed.minRadiusMm)
                ? std::max(0.5, static_cast<double>(seed.minRadiusMm)) : 0.5;
            const double maxR = std::isfinite(seed.maxRadiusMm)
                ? std::max(minR, static_cast<double>(seed.maxRadiusMm)) : std::max(minR, 12.0);
            const double tuoDiRadius = std::clamp(0.5 * (minR + maxR), minR, maxR);

            selectedAttempt = Sb11HoleKouAttempt{};
            selectedAttempt.valid = true;
            selectedAttempt.trialSeed = seed;
            selectedAttempt.mouthCenterWorld = seed.point;
            selectedAttempt.mouthNormal = tuoDiNormal;
            selectedAttempt.frame = gouJianShouDongJuBuZuoBiao(seed.point, tuoDiNormal);
            selectedAttempt.normalOffsetMm = 0.0;
            selectedAttempt.score = -std::numeric_limits<double>::infinity();
            selectedAttempt.rejectCode = candidateRoutingPreGateRejectCode;
            selectedAttempt.detection.geometry.valid = true;
            selectedAttempt.detection.geometry.centerX = seed.point.x();
            selectedAttempt.detection.geometry.centerY = seed.point.y();
            selectedAttempt.detection.geometry.radius = tuoDiRadius;
            selectedAttempt.detection.geometry.normal = tuoDiNormalD;
            selectedAttempt.detection.geometry.tiltDegrees = std::acos(
                std::clamp(static_cast<double>(tuoDiNormal.z()), -1.0, 1.0))
                * 180.0 / M_PI;
            selectedAttempt.detection.geometry.selectedShift = 0.0;
            selectedAttempt.detection.geometry.rawScore = 0.0;
            selectedAttempt.detection.geometry.levelScore = 0.0;
            selectedAttempt.detection.geometry.surfacePointCount = 0;
            selectedAttempt.detection.geometry.source = "canonicalRoib7_direct_cluster_entry";
            selectedAttempt.detection.centerZ = seed.point.z();
            selectedAttempt.detection.initialCenterZ = seed.point.z();
            selectedAttempt.detection.globalNormal = tuoDiNormalD;
            selectedAttempt.detection.error.clear();

            if (!selectedAttempt.frame.valid) {
                                continue;
            }
        }

        const shouDongHole::PclJianCeResult& detection = selectedAttempt.detection;
        auto geometry = detection.geometry;
        if (std::abs(selectedAttempt.normalOffsetMm) > 1e-9) {
            geometry.source += "_outer_retry_height";

            geometry.selectedShift += selectedAttempt.normalOffsetMm;
        }

        const double sb23Pr = geometry.persistentFamilyRadius;
        const bool sb23Finite = std::isfinite(geometry.persistentFamilyCenterX)
            && std::isfinite(geometry.persistentFamilyCenterY)
            && std::isfinite(geometry.persistentFamilyCenterZ);
        const bool sb23Evidence = geometry.persistentFamilyValid && sb23Finite && sb23Pr > 0.0
            && geometry.persistentFamilyLevels >= 4 && geometry.persistentFamilyRawScore >= 3.00
            && geometry.primaryMouthRadius >= 1.25 * sb23Pr && geometry.primaryMouthRadius <= 1.55 * sb23Pr
            && geometry.persistentFamilyCenterGap <= 1.25 && geometry.primaryMouthCoverage >= 0.95
            && geometry.primaryMouthRmse >= 0.85 && geometry.primaryMouthPolarity <= 0.72
            && !geometry.boreEvidenceValid && !geometry.depthCenterUsed
            && sb23Pr >= seed.minRadiusMm * 0.70 && sb23Pr <= seed.maxRadiusMm * 1.08;
        const double sb23Gap = sb23Finite
            ? std::hypot(geometry.centerX-geometry.persistentFamilyCenterX,
                         geometry.centerY-geometry.persistentFamilyCenterY) : 0.0;
        bool sb23PostHuiFuApplied = sb23Evidence
            && (std::abs(geometry.radius-sb23Pr) >= 0.40 || sb23Gap >= 0.40);
        if(sb23PostHuiFuApplied){
            geometry.centerX=geometry.persistentFamilyCenterX;
            geometry.centerY=geometry.persistentFamilyCenterY;
            geometry.radius=sb23Pr;
            geometry.source += "_sb23_post_persistent_rescue";
        }
        Eigen::Vector3f mouthCenterWorld = selectedAttempt.mouthCenterWorld;
        const Eigen::Vector3f mouthNormal = selectedAttempt.mouthNormal;
        ShouDongJuBuZuoBiao frame = selectedAttempt.frame;
        if(sb23PostHuiFuApplied){
            mouthCenterWorld=Eigen::Vector3f(
                static_cast<float>(geometry.persistentFamilyCenterX),
                static_cast<float>(geometry.persistentFamilyCenterY),
                static_cast<float>(geometry.persistentFamilyCenterZ));
            const ShouDongJuBuZuoBiao huiFuChengGongFrame=gouJianShouDongJuBuZuoBiao(mouthCenterWorld,mouthNormal);
            if(huiFuChengGongFrame.valid) frame=huiFuChengGongFrame;
            else {
                geometry=detection.geometry;
                mouthCenterWorld=selectedAttempt.mouthCenterWorld;
                frame=selectedAttempt.frame;
                sb23PostHuiFuApplied=false;
            }
        }

        ShouDongHoleSeed measurementSeed = seed;
        measurementSeed.point = mouthCenterWorld;

        measurementSeed.searchRadiusMm = std::max(seed.searchRadiusMm, 22.0f);
        CloudPtr localCloud;
        if (!candidateRoutingDirectClusterEntryUsed) {
            localCloud = gouJianShouDongJuBuDianYun(
                cloud, measurementSeed, frame, &kdtree);
        }

        const float mouthRadiusSnapshot = static_cast<float>(geometry.radius);
        const float radius = mouthRadiusSnapshot;
        const float confidence = std::clamp(
            0.72f + 0.06f * static_cast<float>(std::max(0.0, geometry.rawScore)),
            0.72f, 0.96f);
        HoleMiaoshu d;

        buildSurfaceTuoDiDescriptor(d, radius, confidence);
        d.centerTop = Eigen::Vector3f::Zero();
        d.centerBot = Eigen::Vector3f::Zero();
        d.center = Eigen::Vector3f::Zero();
        d.rTop = radius;
        d.radius = radius;
        d.rBot = radius;
        d.depth = 0.0f;
        d.depthMeasured = 0.0f;
        d.depthFinal = 0.0f;
        d.type = 1;
        d.geometryType = HoleMiaoshu::JiheType::Straight;
        d.finalGeometryType = HoleMiaoshu::FinalJiheType::Straight;
        d.physicalHoleTypeDisplay = HoleMiaoshu::WuLiHoleLeixingXianshi::Straight;
        d.coneEvidence = false;
        d.reliableBottomRadius = false;
        d.centerReliability = "High";
        d.radiusReliability = "High";

        d.circularity = confidence;
        d.supportPts = std::max(d.supportPts, geometry.surfacePointCount);
        d.detectionPlaneZ = 0.0f;
        d.mouthZ0 = 0.0f;
        d.mouthZ = 0.0f;
        d.bestMouthZ = 0.0f;
        d.manualPoseTag = detection.usedMouthSupportPlane
            ? "MouthSupportPlane_MouthSupportPlane_Projected"
            : (detection.usedLocalPose ? "LocalPose_Projected" : "GlobalPrior_SectorIRLS");
        d.manualNormalDeltaDeg = angleDegrees(geometry.normal, detection.globalNormal);
        d.localPlaneTiltDeg = static_cast<float>(geometry.tiltDegrees);
                d.detectPath = HoleMiaoshu::DetectPath::CuDingWeiSurface;
        d.localVerified = true;
        d.sourcePath = "MOUTH_SUPPORT_PLANE_PROJECTION";
        d.sourceScore = confidence;
        d.sourceReason = std::string("manual_geometry_") + geometry.source;
        d.seedSourcePath = "MOUTH_SUPPORT_PLANE_PROJECTION";

        shouDongHoleMiaoShuDaoShiJie(d, frame);
        d.centerTop = mouthCenterWorld;
        const float signedWorldDepth = (d.centerBot - d.centerTop).dot(frame.n);
        if (std::abs(signedWorldDepth) < 0.1f && d.depth > 0.0f) {
            d.centerBot = d.centerTop - frame.n * d.depth;
        }
        d.center = (d.centerTop + d.centerBot) * 0.5f;
        d.rTop = radius;
        d.radius = radius;
        if (d.type == 1) d.rBot = radius;
        d.localPlaneNx = frame.n.x();
        d.localPlaneNy = frame.n.y();
        d.localPlaneNz = frame.n.z();
        d.localPlaneTiltDeg = static_cast<float>(geometry.tiltDegrees);
        d.detectionPlaneZ = mouthCenterWorld.z();
        d.mouthZ0 = mouthCenterWorld.z();
        d.mouthZ = mouthCenterWorld.z();
        d.bestMouthZ = mouthCenterWorld.z();

        shouDongHole::Vec3d canonicalSearchGlobalNormalD = cachedGlobalNormalD;
        Eigen::Vector3f canonicalSearchGlobalNormal(
            static_cast<float>(canonicalSearchGlobalNormalD.x),
            static_cast<float>(canonicalSearchGlobalNormalD.y),
            static_cast<float>(canonicalSearchGlobalNormalD.z));
        if (!canonicalSearchGlobalNormal.allFinite() || canonicalSearchGlobalNormal.norm() < 1e-6f)
            canonicalSearchGlobalNormal = mouthNormal;
        if (!canonicalSearchGlobalNormal.allFinite() || canonicalSearchGlobalNormal.norm() < 1e-6f)
            canonicalSearchGlobalNormal = Eigen::Vector3f::UnitZ();
        canonicalSearchGlobalNormal.normalize();
        if (canonicalSearchGlobalNormal.z() < 0.0f) canonicalSearchGlobalNormal = -canonicalSearchGlobalNormal;

        // 低置信度情况下使用保守几何路径。
        const ShouDongJuBuZuoBiao canonicalSearchGlobalFrame =
            gouJianShouDongJuBuZuoBiao(Eigen::Vector3f::Zero(), canonicalSearchGlobalNormal);
        if (!canonicalSearchGlobalFrame.valid) {
                        continue;
        }

        static thread_local std::vector<HoleHouXuanJuLei::Sample>
            canonicalSearchCoarseSampleWorkspace;

        // GUI 只暴露一个“最大孔半径”主参数；同一 seed 后续的 ROI、孔口语义和点击归属
        // 必须共享同一个规范化值。这里放在整个单种子识别作用域，避免各阶段重复定义或作用域失配。
        const float configuredMaxRadius = std::clamp(seed.maxRadiusMm, 2.0f, 30.0f);

        // 点击的三维拾取点可能落在锥壁、孔内或残缺边缘。正式搜索仍使用点击的平面位置做目标归属，
        // 但轴向中心先投影到点击附近的独立外支撑面，避免左右两侧点击因深度不同走向不同孔层。
        const HoleKouSearchSeedSurfaceAnchor canonicalSearchSeedAnchor =
            guJiMouthSearchSeedOuterSurfaceAnchor(
                cloud, canonicalSearchGlobalFrame, seed.point, configuredMaxRadius, &kdtree);
        const Eigen::Vector3f canonicalSearchAnchorWorld = canonicalSearchSeedAnchor.valid
            ? canonicalSearchSeedAnchor.point : seed.point;

        // 正式孔口搜索只保留一套算法：闭合圆与部分可见圆弧属于同一候选语义。
        // FrameHint / 自适应法向 / 全局帧 / 紧凑空间只是不同观察坐标或 ROI 尺度；
        // 它们都产生同一种“孔口几何假设”，最终统一按物理质量比较。
        /** 【类型导航注释】
         * HoleKouSearchBundle：Hole 识别总编排中的自定义 结构体。
         * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
         * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
         */
        struct HoleKouSearchBundle {
            GuiFanSearchCuRoi roi;
            HoleHouXuanJuLei::Result cluster;
            ShouDongJuBuZuoBiao frame;
            std::size_t compactPts = 0;
            int compactCandidates = 0;
            int compactStableClusters = 0;
            bool expandedRetryUsed = false;
            bool expandedRetryAccepted = false;
            std::size_t expandedPts = 0;
            int expandedCandidates = 0;
            int expandedStableClusters = 0;
            bool confirmedCompactMatch = false;
            // 当前观察帧选中闭合候选的“核心是否被实体点持续占据”证据。
            // 只用于跨观察帧排序和提前结束安全门，不直接拒绝真正的小孔。
            int selectedCorePoints = 0;
            int selectedCoreDepthBins = 0;
            int selectedAnnulusPoints = 0;
            double selectedCoreToAnnulusRatio = 0.0;
            bool selectedPersistentCoreOccupation = false;
        };

        auto runMouthSearchSearch = [&](const ShouDongJuBuZuoBiao& searchFrame,
                                 const ZuoBiaoXiTishi* confirmedHint,
                                 const pcl::KdTreeFLANN<pcl::PointXYZRGB>* searchTree,
                                 const HoleChuShiFaXianZhuPingMian* anchorOverride) {
            HoleKouSearchBundle bundle;
            bundle.frame = searchFrame;
            if (!searchFrame.valid) return bundle;
            const Eigen::Vector3f searchAnchorWorld =
                (anchorOverride && anchorOverride->valid)
                    ? anchorOverride->point : canonicalSearchAnchorWorld;
            const Eigen::Vector3f clickLocal3 = seed.point - searchFrame.origin;
            const Eigen::Vector3f anchorLocal3 = searchAnchorWorld - searchFrame.origin;
            auto runOne = [&](float radialLimit, float axialLimit) {
                GuiFanSearchCuRoi roi = gouJianCanonicalSearchCoarseRoi(
                    cloud, searchFrame, searchAnchorWorld, radialLimit, axialLimit, searchTree,
                    &canonicalSearchCoarseSampleWorkspace);
                HoleHouXuanJuLei::Input clusterInput;
                clusterInput.seedU = clickLocal3.dot(searchFrame.u);
                clusterInput.seedV = clickLocal3.dot(searchFrame.v);
                clusterInput.seedW = anchorLocal3.dot(searchFrame.n);
                clusterInput.seedWIsOuterSurfaceAnchor =
                    (anchorOverride && anchorOverride->valid) || canonicalSearchSeedAnchor.valid;
                clusterInput.radialHalfExtent = roi.radialLimit;
                clusterInput.axialHalfExtent = roi.axialLimit;
                clusterInput.maxHoleRadius = std::clamp(
                    static_cast<double>(seed.maxRadiusMm), 2.0, 30.0);
                clusterInput.enableOpenArc = true;
                HoleHouXuanJuLei::Result cluster =
                    HoleHouXuanJuLei::evaluate(
                        canonicalSearchCoarseSampleWorkspace, clusterInput);
                return std::make_pair(std::move(roi), std::move(cluster));
            };

            // 粗口沿搜索半径与 GUI 最大孔半径使用同一条唯一规则：默认至少 25 mm；Rmax>10 mm 时按 2*Rmax+5 mm。
            // 例：Rmax=10 -> 25 mm，Rmax=12 -> 29 mm。用户扩大允许孔径时搜索范围同步扩大；
            // 不再使用 1.55*Rmax+9.5；25 mm 只作为默认 10 mm 最大半径对应的稳定下限。
            const float compactRadial = std::max(25.0f, 2.0f * configuredMaxRadius + 5.0f);
            const float compactAxial = std::clamp(
                std::max(15.0f, configuredMaxRadius * 0.80f), 15.0f, 22.0f);
            auto compact = runOne(compactRadial, compactAxial);
            bundle.roi = std::move(compact.first);
            bundle.cluster = std::move(compact.second);
            bundle.compactPts = bundle.roi.pointCount;
            bundle.compactCandidates = bundle.cluster.candidateCount;
            bundle.compactStableClusters = bundle.cluster.stableClusterCount;

            // 线激光薄层里的小封闭采样空斑可能在二维栅格中看起来像完整圆。
            // 在同一观察帧、同一原始 ROI 上直接检查候选中心附近的核心柱体：
            // 真孔核心应基本为空；若多个轴向带持续有点，则该候选不能被当成“充分证据”提前结束其它帧。
            //
            // 同一真实 Hole 附近可能同时形成多个闭合候选；如果只按 seed 边缘距离排序，
            // R≈真实孔口 与 R≈外层采样结构 两个稳定闭合簇。旧排序主要看 seed 到边缘的距离，
            // 因此沿同一孔沿换一个点击方位时，两个簇会交换名次。这里不放宽任何孔接受阈值，
            // 只在“闭合簇竞争很近”或当前首选已经表现为持续实心核心时，使用同一 ROI 的原始点
            // 对竞争簇做一次物理空腔预排序：核心持续实心的簇不能压过核心为空且环带有支撑的簇。
            /** 【类型导航注释】
             * BiHeJuLeiKongDongZhengJu：Hole 识别总编排中的自定义 结构体。
             * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
             * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
             */
            struct BiHeJuLeiKongDongZhengJu {
                int corePoints = 0;
                int coreDepthBins = 0;
                int annulusPoints = 0;
                int annulusSectors = 0;
                int shoulderPoints = 0;
                double coreToAnnulusRatio = 0.0;
                double shoulderToAnnulusRatio = 0.0;
                bool persistentCore = false;
            };
            auto auditClosedClusterVoid = [&](const HoleHouXuanJuLei::Cluster& cluster) {
                BiHeJuLeiKongDongZhengJu evidence;
                if (cluster.openArcStable || !(cluster.consensusTopRadius > 0.5))
                    return evidence;
                const double coreRadius = std::max(0.35, 0.78 * cluster.consensusTopRadius);
                const double annulusInner = 0.84 * cluster.consensusTopRadius;
                const double annulusOuter = 1.20 * cluster.consensusTopRadius;
                const double coreRadius2 = coreRadius * coreRadius;
                const double shoulderRadius = 0.90 * cluster.consensusTopRadius;
                const double shoulderRadius2 = shoulderRadius * shoulderRadius;
                const double annulusInner2 = annulusInner * annulusInner;
                const double annulusOuter2 = annulusOuter * annulusOuter;
                const double axialHalf = std::clamp(
                    0.20 * cluster.consensusTopRadius + 0.45, 0.65, 1.00);
                std::array<unsigned char, 8> coreDepthHit{};
                std::array<unsigned char, 36> annulusSectorHit{};
                for (const auto& sample : canonicalSearchCoarseSampleWorkspace) {
                    const double dw = sample.w - cluster.consensusTopW;
                    if (std::abs(dw) > axialHalf) continue;
                    const double du = sample.u - cluster.consensusCenterU;
                    const double dv = sample.v - cluster.consensusCenterV;
                    const double radial2 = du * du + dv * dv;
                    if (radial2 <= shoulderRadius2)
                        ++evidence.shoulderPoints;
                    if (radial2 <= coreRadius2) {
                        ++evidence.corePoints;
                        const double t = (dw + axialHalf) / std::max(1e-9, 2.0 * axialHalf);
                        const int bin = std::clamp(
                            static_cast<int>(std::floor(t * 8.0)), 0, 7);
                        coreDepthHit[static_cast<std::size_t>(bin)] = 1U;
                    }
                    if (radial2 >= annulusInner2 && radial2 <= annulusOuter2) {
                        ++evidence.annulusPoints;
                        double angle = std::atan2(dv, du);
                        if (angle < 0.0) angle += 2.0 * M_PI;
                        const int sector = std::clamp(
                            static_cast<int>(std::floor(angle / (2.0 * M_PI) * 36.0)), 0, 35);
                        annulusSectorHit[static_cast<std::size_t>(sector)] = 1U;
                    }
                }
                evidence.coreDepthBins = static_cast<int>(std::count(
                    coreDepthHit.begin(), coreDepthHit.end(), static_cast<unsigned char>(1U)));
                evidence.annulusSectors = static_cast<int>(std::count(
                    annulusSectorHit.begin(), annulusSectorHit.end(), static_cast<unsigned char>(1U)));
                evidence.coreToAnnulusRatio = static_cast<double>(evidence.corePoints)
                    / static_cast<double>(std::max(1, evidence.annulusPoints));
                evidence.shoulderToAnnulusRatio = static_cast<double>(evidence.shoulderPoints)
                    / static_cast<double>(std::max(1, evidence.annulusPoints));
                evidence.persistentCore = evidence.corePoints >= 4
                    && evidence.coreDepthBins >= 2
                    && evidence.coreToAnnulusRatio >= 0.10;
                return evidence;
            };

            if (bundle.cluster.valid && bundle.cluster.selectedClusterIndex >= 0
                && static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    < bundle.cluster.clusters.size()) {
                int selectedIndex = bundle.cluster.selectedClusterIndex;
                BiHeJuLeiKongDongZhengJu selectedEvidence = auditClosedClusterVoid(
                    bundle.cluster.clusters[static_cast<std::size_t>(selectedIndex)]);

                const bool closedCompetition = !bundle.cluster.clusters[
                        static_cast<std::size_t>(selectedIndex)].openArcStable
                    && (selectedEvidence.persistentCore
                        || selectedEvidence.shoulderToAnnulusRatio >= 0.12
                        || (std::isfinite(bundle.cluster.clusterMargin)
                            && bundle.cluster.clusterMargin <= 1.25));
                if (closedCompetition) {
                    const double configuredOwnershipLimit = std::clamp(
                        0.70 * static_cast<double>(configuredMaxRadius)
                            + 3.0 * 0.25,
                        4.0, 12.0);
                    auto physicalRank = [&](const HoleHouXuanJuLei::Cluster& cluster,
                                            const BiHeJuLeiKongDongZhengJu& evidence) {
                        // 越小越好。seed 边缘距离仍负责“选哪个孔”；cluster.score 仍负责稳定性；
                        // 只有核心实体占据与真实环带支撑提供额外物理判别。
                        return cluster.seedBoundaryDistance
                            - 0.040 * cluster.score
                            - 0.055 * static_cast<double>(evidence.annulusSectors)
                            + 2.50 * std::min(1.0, evidence.coreToAnnulusRatio)
                            + 4.00 * std::min(1.0, evidence.shoulderToAnnulusRatio)
                            + 0.12 * static_cast<double>(evidence.coreDepthBins)
                            + (evidence.persistentCore ? 8.0 : 0.0);
                    };

                    double selectedRank = physicalRank(
                        bundle.cluster.clusters[static_cast<std::size_t>(selectedIndex)],
                        selectedEvidence);
                    int bestIndex = selectedIndex;
                    BiHeJuLeiKongDongZhengJu bestEvidence = selectedEvidence;
                    double bestRank = selectedRank;

                    // 物理审核每个候选都要扫描一次粗 ROI；为了不让“稳定性修复”反过来拖慢
                    // 普通多孔批处理，先用现有 seed/cluster 轻量分数筛到最多 4 个最相关闭合簇，
                    // 再做原始点物理审核。竞争候选通常集中在排序最前的少量簇，因此不需要全簇扫描。
                    std::vector<std::pair<double, int>> physicalCandidates;
                    physicalCandidates.reserve(bundle.cluster.clusters.size());
                    for (std::size_t clusterIndex = 0;
                         clusterIndex < bundle.cluster.clusters.size(); ++clusterIndex) {
                        if (static_cast<int>(clusterIndex) == selectedIndex) continue;
                        const auto& candidate = bundle.cluster.clusters[clusterIndex];
                        if (!candidate.stable || !candidate.mouthAttachmentValid
                            || candidate.deepContinuation || candidate.openArcStable
                            || !(candidate.consensusTopRadius > 1.0)
                            || candidate.seedBoundaryDistance > configuredOwnershipLimit) {
                            continue;
                        }
                        const double cheapRank = candidate.seedBoundaryDistance
                            - 0.040 * candidate.score;
                        physicalCandidates.emplace_back(cheapRank, static_cast<int>(clusterIndex));
                    }
                    std::sort(physicalCandidates.begin(), physicalCandidates.end(),
                        [](const auto& left, const auto& right) {
                            if (left.first != right.first) return left.first < right.first;
                            return left.second < right.second;
                        });
                    const std::size_t physicalCandidateLimit =
                        std::min<std::size_t>(4U, physicalCandidates.size());
                    for (std::size_t physicalIndex = 0;
                         physicalIndex < physicalCandidateLimit; ++physicalIndex) {
                        const int candidateIndex = physicalCandidates[physicalIndex].second;
                        const auto& candidate = bundle.cluster.clusters[
                            static_cast<std::size_t>(candidateIndex)];
                        const BiHeJuLeiKongDongZhengJu evidence = auditClosedClusterVoid(candidate);
                        // 没有基本环带支撑的空区不能仅因为“核心没点”反抢真实孔。
                        if (evidence.annulusPoints < 10 || evidence.annulusSectors < 6) continue;
                        const double rank = physicalRank(candidate, evidence);
                        const bool selectedPhysicallyBad = selectedEvidence.persistentCore
                            || selectedEvidence.annulusSectors < 6;
                        const bool clearlyBetterPhysical = !evidence.persistentCore
                            && rank + 0.70 < bestRank
                            && candidate.score + 1.25 >= bundle.cluster.clusters[
                                static_cast<std::size_t>(selectedIndex)].score;
                        if ((selectedPhysicallyBad && !evidence.persistentCore && rank < bestRank)
                            || clearlyBetterPhysical) {
                            bestIndex = candidateIndex;
                            bestEvidence = evidence;
                            bestRank = rank;
                        }
                    }
                    if (bestIndex != selectedIndex) {
                        bundle.cluster.selectedClusterIndex = bestIndex;
                        bundle.cluster.selectedClusterId = bundle.cluster.clusters[
                            static_cast<std::size_t>(bestIndex)].id;
                        bundle.cluster.selectMode = "PHYSICAL_VOID_CLUSTER_RERANK";
                        bundle.cluster.reason =
                            "ambiguous closed-mouth clusters reranked by raw core-void and annulus evidence";
                        selectedIndex = bestIndex;
                        selectedEvidence = bestEvidence;
                    }
                }

                bundle.selectedCorePoints = selectedEvidence.corePoints;
                bundle.selectedCoreDepthBins = selectedEvidence.coreDepthBins;
                bundle.selectedAnnulusPoints = selectedEvidence.annulusPoints;
                bundle.selectedCoreToAnnulusRatio = selectedEvidence.coreToAnnulusRatio;
                bundle.selectedPersistentCoreOccupation = selectedEvidence.persistentCore;
            }

            double selectedCompactRadius = 0.0;
            if (bundle.cluster.valid && bundle.cluster.selectedClusterIndex >= 0
                && static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    < bundle.cluster.clusters.size()) {
                selectedCompactRadius = bundle.cluster.clusters[
                    static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)]
                    .consensusTopRadius;
            }
            if (confirmedHint && confirmedHint->frozenClusterValid
                && bundle.cluster.valid && bundle.cluster.selectedClusterIndex >= 0
                && static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    < bundle.cluster.clusters.size()) {
                const auto& compactCluster = bundle.cluster.clusters[
                    static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)];
                bundle.confirmedCompactMatch =
                    HoleFastPath::exactConfirmedClusterMatch(
                        compactCluster.id,
                        compactCluster.consensusCenterU,
                        compactCluster.consensusCenterV,
                        compactCluster.consensusTopW,
                        compactCluster.consensusTopRadius,
                        confirmedHint->clusterId,
                        confirmedHint->consensusCenterU,
                        confirmedHint->consensusCenterV,
                        confirmedHint->consensusTopW,
                        confirmedHint->consensusTopRadius);
            }

            // 单阶段搜索：不再因候选数/点数不足偷偷扩大 ROI 再跑第二遍。
            // 粗 ROI 已按 2*Rmax+5 mm 在这一遍联动尺度；不再因候选不足偷偷扩大后重跑。
            (void)selectedCompactRadius;
            return bundle;
        };

        // 完整圆和部分圆弧属于同一个孔口候选系统。搜索帧可以有全局/自适应/历史提示等多种观察方向，
        // 但这些只是同一算法的不同坐标表达。完整度只作为 coverage 等质量指标；若当前切面已经存在
        // 明确的完整外口沿，跳过重复开放弧计算只是速度优化，不改变后续算法。
        auto selectedSearchIsOpen = [&](const HoleKouSearchBundle& bundle) {
            if (!bundle.cluster.valid || bundle.cluster.selectedClusterIndex < 0
                || static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    >= bundle.cluster.clusters.size()) {
                return false;
            }
            return bundle.cluster.clusters[
                static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)].openArcStable;
        };

        auto selectedSearchQuality = [&](const HoleKouSearchBundle& bundle) {
            if (!bundle.cluster.valid || bundle.cluster.selectedClusterIndex < 0
                || static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    >= bundle.cluster.clusters.size()) {
                return -std::numeric_limits<double>::infinity();
            }
            const auto& cluster = bundle.cluster.clusters[
                static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)];
            // 不再规定“闭合圆一定优先、开放圆弧只能回退”。所有搜索帧使用同一个物理质量分数：
            // 多层支持、覆盖、轨迹连续、拟合残差和中心/半径稳定决定几何可信度；点击代价只防止抢邻孔。
            // 对闭合圆，连续可见度等价于整体覆盖；对部分圆弧则使用实测最大连续弧。
            const double visibleContinuity = cluster.openArcStable
                ? cluster.meanContinuousArcCoverage : cluster.meanCoverage;
            const double normalizedResidual = cluster.meanResidual
                / std::max(1.0, cluster.consensusTopRadius);
            // 完整闭合孔跨观察帧时也直接使用真实孔沿距离，避免 cluster.score 间接压过点击归属；
            // 开放圆弧继续保留已经通过 1.51 验证的 selectionCost 语义，不扰动残缺孔链。
            const bool persistentFamilyOverride =
                bundle.cluster.selectMode == "PERSISTENT_FAMILY_WEAK_SEED_OVERRIDE";
            const double clickPenalty = persistentFamilyOverride
                ? 0.0
                : (cluster.openArcStable
                    ? std::clamp(cluster.selectionCost, 0.0, 12.0)
                    : std::clamp(cluster.seedBoundaryDistance, 0.0, 12.0));
            const double solidCorePenalty = (!cluster.openArcStable
                && bundle.selectedPersistentCoreOccupation) ? 8.0 : 0.0;
            return 1.40 * std::min(6, cluster.supportLayers)
                + 3.00 * cluster.meanCoverage
                + 1.50 * cluster.trajectoryContinuity
                + 0.80 * visibleContinuity
                - 1.40 * cluster.centerStd
                - 0.70 * cluster.radiusStd
                - 3.00 * normalizedResidual
                - 0.80 * clickPenalty
                - solidCorePenalty;
        };

        // 完整闭合孔的“单观察帧已充分”快路径。这里只裁掉重复的坐标帧观察，
        // 不减少任何一个候选层、拟合点或后续 canonical/孔壁审核。
        // 倾斜件优先使用已经通过强局部平面门的 adaptive frame；低倾角件优先 global frame。
        // 开放圆弧/残缺孔永远不走这个快路径，仍保留多帧交叉验证。
        auto selectedSearchIsConclusiveComplete = [&](const HoleKouSearchBundle& bundle) {
            if (!bundle.cluster.valid || bundle.cluster.selectedClusterIndex < 0
                || static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    >= bundle.cluster.clusters.size()) return false;
            const auto& cluster = bundle.cluster.clusters[
                static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)];
            if (!cluster.stable || cluster.openArcStable || !cluster.mouthAttachmentValid
                || cluster.deepContinuation) return false;
            // 真孔的核心柱体应基本为空；若当前闭合圆只是实心表面上的采样空斑，
            // 它即使圆度/覆盖很好也绝不能触发“单观察帧已充分”的性能快路径。
            if (bundle.selectedPersistentCoreOccupation) return false;
            const double radius = std::max(1.0, cluster.consensusTopRadius);
            return cluster.supportLayers >= 3
                && cluster.meanCoverage >= 0.96
                && cluster.trajectoryContinuity >= 0.98
                && cluster.centerStd <= std::max(0.65, 0.12 * radius)
                && cluster.meanResidual <= std::max(0.65, 0.15 * radius);
        };

        // 高倾角完整孔的自适应帧已经由独立局部平面证据证明明显优于全局帧时，
        // 不要求跨层 coverage 必须达到近乎理想的 0.96 才能停止重复 global 搜索。
        // 该快路径只在 adaptive 法向相对 global 至少偏转 8° 时启用，并且仍要求：
        // 闭合稳定簇、真实外口贴附、核心为空、至少三层连续、中心/半径抖动和残差都很小。
        // 因而该快路径只针对中高倾角件的重复观察成本，低倾角 Hole 继续走完整常规路径。
        auto selectedSearchIsStrongAdaptiveComplete = [&](const HoleKouSearchBundle& bundle) {
            if (!bundle.cluster.valid || bundle.cluster.selectedClusterIndex < 0
                || static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)
                    >= bundle.cluster.clusters.size()) return false;
            const auto& cluster = bundle.cluster.clusters[
                static_cast<std::size_t>(bundle.cluster.selectedClusterIndex)];
            if (!cluster.stable || cluster.openArcStable || !cluster.mouthAttachmentValid
                || cluster.deepContinuation || bundle.selectedPersistentCoreOccupation) return false;
            const double radius = std::max(1.0, cluster.consensusTopRadius);
            return cluster.supportLayers >= 3
                && cluster.meanCoverage >= 0.58
                && cluster.trajectoryContinuity >= 0.82
                && cluster.centerStd <= std::max(0.50, 0.10 * radius)
                && cluster.radiusStd <= std::max(0.60, 0.12 * radius)
                && cluster.meanResidual <= std::max(0.34, 0.085 * radius)
                && cluster.seedBoundaryDistance <= std::max(2.75, 0.45 * radius);
        };
        HoleChuShiFaXianState chuShiJuBuFaXian;
        bool chuShiJuBuFaXianUsed = false;
        bool frameHintAdaptivePreflightAttempted = false;
        bool frameHintAdaptivePreflightPreferred = false;
        bool frameHintAdaptivePreflightSearchTried = false;
        bool frameHintGlobalSearchSkipped = false;
        bool mouthSearchConclusiveComplete = false;
        int analysisCacheSupportPlaneCacheHits = 0;
        int analysisCacheSupportPlaneCacheMisses = 0;
        int analysisCacheCanonicalGeometryCacheHits = 0;
        int analysisCacheCanonicalGeometryCacheMisses = 0;
        bool chuShiZhuPingMianAttempted = false;
        bool chuShiZhuPingMianAccepted = false;
        int chuShiZhuPingMianSupport = 0;
        int chuShiZhuPingMianSectors = 0;
        float chuShiZhuPingMianRmse = 0.0f;
        float chuShiZhuPingMianDeltaDeg = 0.0f;
        bool compactHuiFuCompact20HuiFuAttempted = false;
        bool compactHuiFuCompact20HuiFuAccepted = false;
        bool compactHuiFuCompact20SafetyGatePassed = false;
        qint64 compactHuiFuCompact20HuiFuMs = 0;
        std::string compactHuiFuCompact20HuiFuExit = "NOT_RUN";
        std::string compactHuiFuCompact20SafetyGateReason = "NOT_RUN";
        std::string mouthSearchGlobalExitCode = "FrameHint_GLOBAL_NOT_RUN";
        std::string mouthSearchGlobalFailReason = "adaptive preflight selected local support frame";
        HoleKouSearchBundle canonicalSearchChosenSearch;
        double canonicalSearchChosenQuality = -std::numeric_limits<double>::infinity();

        // 每个全局/自适应/历史提示坐标帧只是同一孔口几何的不同观察方向。
        // 所有有效候选都进入同一质量比较；完整度只通过 coverage/连续弧/稳定性自然体现，
        // 不再用“闭合先结束、开放后回退”的流程控制决定结果。
        auto considerSearch = [&](HoleKouSearchBundle&& search) {
            if (!search.cluster.valid) return false;
            const double quality = selectedSearchQuality(search);
            if (!std::isfinite(quality)) return false;
            if (!canonicalSearchChosenSearch.cluster.valid
                || quality > canonicalSearchChosenQuality) {
                canonicalSearchChosenSearch = std::move(search);
                canonicalSearchChosenQuality = quality;
                return true;
            }
            return false;
        };

        bool frameHintFrameHintFound = false;
        bool frameHintFrameHintUsed = false;
        double frameHintFrameHintDistance = std::numeric_limits<double>::infinity();
        Eigen::Vector3f frameHintHintNormal = Eigen::Vector3f::UnitZ();
        ZuoBiaoXiTishi frameHintSelectedHint;
        for (const ZuoBiaoXiTishi& hint : frameHintFrameHints) {
            const double distance = static_cast<double>((canonicalSearchAnchorWorld - hint.center).norm());
            const double limit = std::max(8.0, static_cast<double>(hint.radius) + 4.0);
            if (distance <= limit && distance < frameHintFrameHintDistance) {
                frameHintFrameHintFound = true;
                frameHintFrameHintDistance = distance;
                frameHintHintNormal = hint.normal;
                frameHintSelectedHint = hint;
            }
        }
        if (frameHintFrameHintFound && frameHintHintNormal.allFinite()
            && frameHintHintNormal.norm() > 1e-6f) {
            frameHintHintNormal.normalize();
            const ShouDongJuBuZuoBiao hintFrame =
                gouJianShouDongJuBuZuoBiao(Eigen::Vector3f::Zero(), frameHintHintNormal);
            HoleKouSearchBundle hintSearch = runMouthSearchSearch(
                hintFrame, &frameHintSelectedHint, &kdtree, nullptr);
            const bool hintConclusiveComplete =
                selectedSearchIsConclusiveComplete(hintSearch);
            if (considerSearch(std::move(hintSearch))) {
                frameHintFrameHintUsed = true;
                chuShiJuBuFaXianUsed = true;
                if (hintConclusiveComplete) mouthSearchConclusiveComplete = true;
            }
        }

        const double frameHintGlobalTiltDeg = std::acos(std::clamp(
            std::abs(static_cast<double>(canonicalSearchGlobalNormal.z())), 0.0, 1.0))
            * 180.0 / M_PI;
        const bool frameHintAdaptivePreflightEligible = frameHintGlobalTiltDeg >= 4.0;
        if (frameHintAdaptivePreflightEligible && !mouthSearchConclusiveComplete) {
            frameHintAdaptivePreflightAttempted = true;
            chuShiJuBuFaXian = guJiHoleChuShiJuBuFaXian(
                cloud, canonicalSearchAnchorWorld, canonicalSearchGlobalNormal, &kdtree);
            const bool strongLocalFrame = chuShiJuBuFaXian.valid
                && chuShiJuBuFaXian.support >= 120
                && chuShiJuBuFaXian.coverage >= 0.50f
                && chuShiJuBuFaXian.rmse <= 0.35f
                && chuShiJuBuFaXian.deltaDegrees >= 3.5f;
            if (strongLocalFrame) {
                const ShouDongJuBuZuoBiao localFrame =
                    gouJianShouDongJuBuZuoBiao(Eigen::Vector3f::Zero(),
                                               chuShiJuBuFaXian.normal);
                frameHintAdaptivePreflightSearchTried = true;
                HoleKouSearchBundle localSearch = runMouthSearchSearch(
                    localFrame, nullptr, &kdtree, nullptr);
                const bool localConclusiveComplete =
                    selectedSearchIsConclusiveComplete(localSearch)
                    || (chuShiJuBuFaXian.deltaDegrees >= 8.0f
                        && selectedSearchIsStrongAdaptiveComplete(localSearch));
                if (considerSearch(std::move(localSearch))) {
                    chuShiJuBuFaXianUsed = true;
                    chuShiJuBuFaXian.accepted = true;
                    frameHintAdaptivePreflightPreferred = true;
                    if (localConclusiveComplete) mouthSearchConclusiveComplete = true;
                }
            }
        }

        // 若前面的强 adaptive/hint 已经给出充分闭合孔证据，不再把同一候选系统换一个坐标帧完整重跑。
        // 这正是 1.1 完整孔最主要的重复计算；残缺/open 候选不会触发这里。
        if (!mouthSearchConclusiveComplete) {
            HoleKouSearchBundle globalSearch =
                runMouthSearchSearch(canonicalSearchGlobalFrame, nullptr, &kdtree, nullptr);
            mouthSearchGlobalExitCode = globalSearch.cluster.exitCode;
            mouthSearchGlobalFailReason = globalSearch.cluster.reason;
            const bool globalConclusiveComplete =
                selectedSearchIsConclusiveComplete(globalSearch);
            if (considerSearch(std::move(globalSearch)) && globalConclusiveComplete)
                mouthSearchConclusiveComplete = true;
        } else {
            frameHintGlobalSearchSkipped = true;
            mouthSearchGlobalExitCode = "GLOBAL_SKIPPED_CONCLUSIVE_COMPLETE";
            mouthSearchGlobalFailReason = "strong closed mouth already resolved in preferred frame";
        }

        // 如果预检没有真正搜索过自适应帧，但已经能估出有效局部法向，仍补一次同算法观察。
        if (!mouthSearchConclusiveComplete && !chuShiJuBuFaXian.attempted) {
            chuShiJuBuFaXian = guJiHoleChuShiJuBuFaXian(
                cloud, canonicalSearchAnchorWorld, canonicalSearchGlobalNormal, &kdtree);
        }
        if (!mouthSearchConclusiveComplete
            && chuShiJuBuFaXian.valid && !frameHintAdaptivePreflightSearchTried) {
            const ShouDongJuBuZuoBiao localFrame =
                gouJianShouDongJuBuZuoBiao(Eigen::Vector3f::Zero(),
                                           chuShiJuBuFaXian.normal);
            HoleKouSearchBundle localSearch = runMouthSearchSearch(
                localFrame, nullptr, &kdtree, nullptr);
            const bool localConclusiveComplete =
                selectedSearchIsConclusiveComplete(localSearch);
            if (considerSearch(std::move(localSearch))) {
                chuShiJuBuFaXianUsed = true;
                chuShiJuBuFaXian.accepted = true;
                if (localConclusiveComplete) mouthSearchConclusiveComplete = true;
            }
        }


        // 【为什么初始阶段保留两种局部法线算法】
        // 常规方法 guJiHoleChuShiJuBuFaXian：直接对 seed 周围支撑点做多尺度稳健平面，成本较低，
        // 目的是给 mouth-search 一个“差不多正确”的观察方向。
        // 本段主平面方法 guJiHoleChuShiZhuPingMianFaXian：只在没有可靠 Hole 口，或常规局部法线
        // 与全局方向差异 >=10° 时才执行。它先 RANSAC 分离主板面，专门处理孔壁/边界把普通平面带偏的情况。
        // 两者不是同时追求最终法线精度：前者是常用粗估计，后者是困难场景兜底；正常 seed 不会为第二种方法付费。
        // 10° 只决定是否增加一个观察帧，不参与 Hole 接受/拒绝。真正精确角度由 Hole 口椭圆反推完成。
        const bool chuShiZhuPingMianNeeded =
            !canonicalSearchChosenSearch.cluster.valid
            || (chuShiJuBuFaXian.valid
                && chuShiJuBuFaXian.deltaDegrees >= 10.0f);
        if (chuShiZhuPingMianNeeded) {
            chuShiZhuPingMianAttempted = true;
            const HoleChuShiFaXianZhuPingMian dominantPlaneAnchor =
                guJiHoleChuShiZhuPingMianFaXian(
                    cloud, canonicalSearchGlobalFrame, seed.point, configuredMaxRadius, &kdtree);
            chuShiZhuPingMianSupport = dominantPlaneAnchor.support;
            chuShiZhuPingMianSectors = dominantPlaneAnchor.sectors;
            chuShiZhuPingMianRmse = dominantPlaneAnchor.rmse;
            chuShiZhuPingMianDeltaDeg = dominantPlaneAnchor.deltaDegrees;
            const bool dominantPlaneMayCompete = dominantPlaneAnchor.valid
                && (!canonicalSearchChosenSearch.cluster.valid
                    || dominantPlaneAnchor.deltaDegrees >= 10.0f);
            if (dominantPlaneMayCompete) {
                const ShouDongJuBuZuoBiao dominantFrame =
                    gouJianShouDongJuBuZuoBiao(Eigen::Vector3f::Zero(), dominantPlaneAnchor.normal);
                HoleKouSearchBundle dominantSearch = runMouthSearchSearch(
                    dominantFrame, nullptr, &kdtree, &dominantPlaneAnchor);
                if (considerSearch(std::move(dominantSearch))) {
                    chuShiZhuPingMianAccepted =
                        canonicalSearchChosenSearch.cluster.valid;
                    if (chuShiZhuPingMianAccepted) {
                        chuShiJuBuFaXianUsed = true;
                    }
                }
            }
        }

        // 单阶段搜索使用统一候选链；全局、自适应和历史提示只负责提供初始姿态。
        // 同一搜索尺度下的不同观察坐标，不能在失败后改变空间尺度重新跑一遍。


        const bool unifiedOpenArcSelected =
            canonicalSearchChosenSearch.cluster.valid
            && selectedSearchIsOpen(canonicalSearchChosenSearch);

        ShouDongJuBuZuoBiao canonicalSearchStableFrame = canonicalSearchChosenSearch.frame;
        const int mouthSearchSelectedCorePoints = canonicalSearchChosenSearch.selectedCorePoints;
        const int mouthSearchSelectedCoreDepthBins = canonicalSearchChosenSearch.selectedCoreDepthBins;
        const int mouthSearchSelectedAnnulusPoints = canonicalSearchChosenSearch.selectedAnnulusPoints;
        const double mouthSearchSelectedCoreToAnnulusRatio = canonicalSearchChosenSearch.selectedCoreToAnnulusRatio;
        const bool mouthSearchSelectedPersistentCoreOccupation =
            canonicalSearchChosenSearch.selectedPersistentCoreOccupation;
        GuiFanSearchCuRoi canonicalSearchCoarse = std::move(canonicalSearchChosenSearch.roi);
        HoleHouXuanJuLei::Result canonicalSearchCluster = std::move(canonicalSearchChosenSearch.cluster);
        const std::size_t canonicalSearchCompactPts = canonicalSearchChosenSearch.compactPts;
        const int canonicalSearchCompactCandidates = canonicalSearchChosenSearch.compactCandidates;
        const int canonicalSearchCompactStableClusters = canonicalSearchChosenSearch.compactStableClusters;
        const bool canonicalSearchExpandedRetryUsed = canonicalSearchChosenSearch.expandedRetryUsed;
        const bool canonicalSearchExpandedRetryAccepted = canonicalSearchChosenSearch.expandedRetryAccepted;
        const std::size_t canonicalSearchExpandedPts = canonicalSearchChosenSearch.expandedPts;
        const int canonicalSearchExpandedCandidates = canonicalSearchChosenSearch.expandedCandidates;
        const int canonicalSearchExpandedStableClusters = canonicalSearchChosenSearch.expandedStableClusters;
        const bool analysisCacheConfirmedCompactMatch =
            canonicalSearchChosenSearch.confirmedCompactMatch;

        if (!canonicalSearchCluster.valid || canonicalSearchCluster.selectedClusterIndex < 0
            || static_cast<std::size_t>(canonicalSearchCluster.selectedClusterIndex) >= canonicalSearchCluster.clusters.size()) {
                        continue;
        }
        const HoleHouXuanJuLei::Cluster& canonicalSearchSelectedCluster =
            canonicalSearchCluster.clusters[static_cast<std::size_t>(canonicalSearchCluster.selectedClusterIndex)];

        // 候选簇到这里仍然只承担“当前点击目标粗定位”的职责。
        // 开放弧、凸台底部轮廓或圆台外缘都可能给出一个稳定粗圆，但它们不能直接拥有“真实上口”语义。
        // 只有出现开放候选或同心外层表面歧义时，才复用原有 SurfaceProfile 沿真实孔壁解析一次上口；
        // 普通完整平板孔仍完全走原来的快速路径，不增加这部分计算。
        WuLiHoleKouPouMianDiag physicalMouthProfile;
        GuiFanSearchCuRoi physicalMouthProfileRoi;
        const HoleHouXuanJuLei::Cluster* physicalMouthLocatorCluster =
            &canonicalSearchSelectedCluster;
        const bool physicalMouthProfileRequested = unifiedOpenArcSelected;
        if (physicalMouthProfileRequested) {
            const float physicalMouthRadial = std::clamp(
                std::max(20.0f, configuredMaxRadius * 1.35f + 6.5f), 20.0f, 48.0f);
            physicalMouthProfileRoi = gouJianCanonicalSearchCoarseRoi(
                cloud, canonicalSearchStableFrame, canonicalSearchAnchorWorld, physicalMouthRadial, 15.0f, &kdtree, nullptr);

            // 部分可见孔口同一位置可能同时产生“真实口沿、局部小弧、凸台外轮廓”等多个
            // 粗定位器。粗圆本身没有机械上口语义；只要都通过了同一稳定/点击归属门，
            // 就让已有 SurfaceProfile 用真实向内孔壁决定谁能收敛成机械孔口。
            // 这不是完整/残缺两条算法，而是同一候选集合在最终物理几何层做语义消歧。
            const Eigen::Vector3f physicalSeedLocal3 = seed.point - canonicalSearchStableFrame.origin;
            const double physicalSeedU = physicalSeedLocal3.dot(canonicalSearchStableFrame.u);
            const double physicalSeedV = physicalSeedLocal3.dot(canonicalSearchStableFrame.v);
            std::vector<int> physicalLocatorIndices;
            physicalLocatorIndices.push_back(canonicalSearchCluster.selectedClusterIndex);
            std::vector<std::pair<double, int>> alternativeLocators;
            int strongestGeometryLocator = -1;
            double strongestGeometryScore = -std::numeric_limits<double>::infinity();
            const double locatorEdgeLimitUpper = std::max(
                3.75, std::min(7.0, 0.35 * static_cast<double>(configuredMaxRadius) + 2.50));
            for (std::size_t clusterIndex = 0; clusterIndex < canonicalSearchCluster.clusters.size(); ++clusterIndex) {
                if (static_cast<int>(clusterIndex) == canonicalSearchCluster.selectedClusterIndex) continue;
                const auto& cluster = canonicalSearchCluster.clusters[clusterIndex];
                if (!cluster.stable || !cluster.openArcStable || !cluster.mouthAttachmentValid
                    || !(cluster.consensusTopRadius > 1.0)) continue;
                const double radialDistance = std::hypot(
                    physicalSeedU - cluster.consensusCenterU,
                    physicalSeedV - cluster.consensusCenterV);
                const double boundaryDistance = std::abs(
                    radialDistance - cluster.consensusTopRadius);
                const double outsideDistance = std::max(
                    0.0, radialDistance - cluster.consensusTopRadius);
                const double locatorEdgeLimit = std::clamp(
                    0.30 * cluster.consensusTopRadius + 2.25,
                    3.75, locatorEdgeLimitUpper);
                if (outsideDistance > locatorEdgeLimit) continue;

                // 真实孔口语义消歧也沿用统一的孔沿距离归属。点击在孔内时只给边缘距离小权重，
                // 孔外则完整计入；几何分数仍保留为小权重，避免只按点击位置吸附到假弧。
                const bool seedInsideLocator = radialDistance <= cluster.consensusTopRadius;
                const double clickCost = seedInsideLocator
                    ? 0.22 * boundaryDistance : boundaryDistance;
                const double locatorCost = clickCost - 0.10 * cluster.score;
                alternativeLocators.emplace_back(locatorCost, static_cast<int>(clusterIndex));
                if (cluster.score > strongestGeometryScore) {
                    strongestGeometryScore = cluster.score;
                    strongestGeometryLocator = static_cast<int>(clusterIndex);
                }
            }
            // 残缺孔同一物理口沿可能同时产生多个局部短弧。只按点击距离排序会让“离点击最近的小弧”
            // 占满有限的剖面名额，因此固定保留一个几何分数最高的兼容定位器，再由真实孔壁剖面裁决。
            // 总尝试数仍最多 4 个，不增加最坏情况下的 SurfaceProfile 次数。
            if (strongestGeometryLocator >= 0
                && strongestGeometryLocator != canonicalSearchCluster.selectedClusterIndex) {
                physicalLocatorIndices.push_back(strongestGeometryLocator);
            }
            std::sort(alternativeLocators.begin(), alternativeLocators.end(),
                [](const auto& left, const auto& right) {
                    if (left.first != right.first) return left.first < right.first;
                    return left.second < right.second;
                });
            for (const auto& alternative : alternativeLocators) {
                if (physicalLocatorIndices.size() >= 4U) break;
                if (std::find(
                        physicalLocatorIndices.begin(), physicalLocatorIndices.end(),
                        alternative.second) != physicalLocatorIndices.end()) {
                    continue;
                }
                physicalLocatorIndices.push_back(alternative.second);
            }

            bool haveAttempt = false;
            for (int locatorIndex : physicalLocatorIndices) {
                if (locatorIndex < 0
                    || static_cast<std::size_t>(locatorIndex) >= canonicalSearchCluster.clusters.size()) continue;
                const auto& locator = canonicalSearchCluster.clusters[
                    static_cast<std::size_t>(locatorIndex)];
                WuLiHoleKouPouMianDiag candidateProfile = jieXiZhenShiHoleKouLunKuo(
                    physicalMouthProfileRoi.samples, locator);
                if (!haveAttempt) {
                    physicalMouthProfile = candidateProfile;
                    physicalMouthLocatorCluster = &locator;
                    haveAttempt = true;
                }
                if (candidateProfile.valid
                    && (!physicalMouthProfile.valid || candidateProfile.score > physicalMouthProfile.score)) {
                    physicalMouthProfile = std::move(candidateProfile);
                    physicalMouthLocatorCluster = &locator;
                }
            }

            // 锥孔/凸台场景可由真实内壁把粗外轮廓收敛为机械上口；直孔若没有足够深层孔壁，
            // SurfaceProfile 天然可能无结果，此时仍把当前统一圆弧交给 canonical 几何审核。
            physicalMouthProfile.used = physicalMouthProfile.valid;
        }

        const double canonicalSearchResolvedU = physicalMouthProfile.used
            ? physicalMouthProfile.profile.centerU
            : canonicalSearchSelectedCluster.consensusCenterU;
        const double canonicalSearchResolvedV = physicalMouthProfile.used
            ? physicalMouthProfile.profile.centerV
            : canonicalSearchSelectedCluster.consensusCenterV;
        const double canonicalSearchResolvedW = physicalMouthProfile.used
            ? physicalMouthProfile.selectedTopW
            : canonicalSearchSelectedCluster.consensusTopW;
        const double canonicalSearchResolvedRadius = physicalMouthProfile.used
            ? physicalMouthProfile.profile.topRadius
            : canonicalSearchSelectedCluster.consensusTopRadius;
        const Eigen::Vector3f canonicalSearchSelectedCenterWorld = canonicalSearchStableFrame.origin
            + canonicalSearchStableFrame.u * static_cast<float>(canonicalSearchResolvedU)
            + canonicalSearchStableFrame.v * static_cast<float>(canonicalSearchResolvedV)
            + canonicalSearchStableFrame.n * static_cast<float>(canonicalSearchResolvedW);
        HoleTuoYuanFaXianState tuoYuanFaXianJingXiu;
        Eigen::Vector3f canonicalSearchPreSupportNormal = canonicalSearchStableFrame.n;

        // 【精确法线阶段：Hole 口椭圆反推】
        // 到这里已经不是“找 Hole”，而是对已锁定的机械 Hole 口做角度精修。
        // 圆形 Hole 口在错误观察平面中会投影成椭圆：椭圆轴比给出倾角大小，长轴方向给出倾斜方向；
        // 因此这里可直接解析更精确的法线，不需要再做角度网格搜索。完整圆接近 axisRatio=1 时，
        // 椭圆本身没有稳定主轴，解析器会自然保留前面的粗法线；部分圆弧只要椭圆证据可靠也走同一公式。
        // 一旦本阶段结果被采用，tuoYuanFaXianOwned 会阻止后面的支撑面/孔壁旧法线链再次覆盖它。
        HoleHouXuanJuLei::HoleKouEllipseEvidence resolvedMouthEllipse;
        if (physicalMouthProfile.used && physicalMouthProfile.ellipse.valid) {
            resolvedMouthEllipse = physicalMouthProfile.ellipse;
        } else {
            resolvedMouthEllipse = HoleHouXuanJuLei::fitResolvedMouthEllipse(
                canonicalSearchCoarse.samples,
                canonicalSearchResolvedU, canonicalSearchResolvedV,
                canonicalSearchResolvedW, canonicalSearchResolvedRadius);
        }

        if (resolvedMouthEllipse.valid) {
            tuoYuanFaXianJingXiu = jingQueHoleKouTuoYuanFaXian(
                cloud, canonicalSearchSelectedCenterWorld, canonicalSearchStableFrame,
                static_cast<float>(canonicalSearchResolvedRadius),
                resolvedMouthEllipse, &kdtree);
        } else if (canonicalSearchSelectedCluster.openArcStable) {
            // 极端短弧下新鲜上口椭圆拟合可能因点数不足失效，此时退回候选簇已经通过
            // 跨层稳定审核的椭圆证据；仍然是解析求法向，不存在角度网格搜索。
            tuoYuanFaXianJingXiu = jingQueHouXuanTuoYuanFaXian(
                cloud, canonicalSearchSelectedCenterWorld, canonicalSearchStableFrame,
                static_cast<float>(canonicalSearchResolvedRadius),
                canonicalSearchSelectedCluster, &kdtree);
        }
        if (tuoYuanFaXianJingXiu.used)
            canonicalSearchPreSupportNormal = tuoYuanFaXianJingXiu.normal;
        if (chuShiJuBuFaXianUsed || frameHintGlobalTiltDeg < 4.0
            || canonicalSearchSelectedCluster.openArcStable || physicalMouthProfile.used) {
            const Eigen::Vector3f hintCenter = canonicalSearchSelectedCenterWorld;
            bool updated = false;
            for (ZuoBiaoXiTishi& hint : frameHintFrameHints) {
                if ((hint.center - hintCenter).norm()
                    <= std::max(4.0f, static_cast<float>(canonicalSearchResolvedRadius))) {
                    hint.center = hintCenter;
                    hint.normal = canonicalSearchPreSupportNormal;
                    hint.radius = static_cast<float>(canonicalSearchResolvedRadius);
                    hint.clusterId = canonicalSearchSelectedCluster.id;
                    hint.consensusCenterU = canonicalSearchSelectedCluster.consensusCenterU;
                    hint.consensusCenterV = canonicalSearchSelectedCluster.consensusCenterV;
                    hint.consensusTopW = canonicalSearchSelectedCluster.consensusTopW;
                    hint.consensusTopRadius = canonicalSearchSelectedCluster.consensusTopRadius;
                    hint.frozenClusterValid = true;
                    updated = true;
                    break;
                }
            }
            if (!updated) {
                ZuoBiaoXiTishi hint;
                hint.center = hintCenter;
                hint.normal = canonicalSearchPreSupportNormal;
                hint.radius = static_cast<float>(canonicalSearchResolvedRadius);
                hint.clusterId = canonicalSearchSelectedCluster.id;
                hint.consensusCenterU = canonicalSearchSelectedCluster.consensusCenterU;
                hint.consensusCenterV = canonicalSearchSelectedCluster.consensusCenterV;
                hint.consensusTopW = canonicalSearchSelectedCluster.consensusTopW;
                hint.consensusTopRadius = canonicalSearchSelectedCluster.consensusTopRadius;
                hint.frozenClusterValid = true;
                frameHintFrameHints.push_back(hint);
            }
        }
        const double outerAnnulusFrameSelectedClusterConfidence =
            outerAnnulusFrameClusterConfidence(canonicalSearchSelectedCluster);
        const double outerAnnulusFrameSelectedClusterScoreNorm =
            outerAnnulusFrameClusterScoreNormalized(canonicalSearchSelectedCluster.score);
        const double outerAnnulusFrameSelectedClusterLayerConsistency = 0.5
            * std::clamp(static_cast<double>(canonicalSearchSelectedCluster.supportLayers) / 5.0, 0.0, 1.0)
            + 0.5 * std::clamp(canonicalSearchSelectedCluster.trajectoryContinuity, 0.0, 1.0);

        // mouth-search 给出的中心首先只是“Hole 族横向定位器”：它能稳定说明是哪一个 Hole，
        // 但 consensusTopW 可能落在孔口内侧，尤其是降噪后深层孔壁被删掉时。机械上口的
        // 轴向位置统一交给外支撑面投影确定。这样 Hole 身份与机械上口平面从一开始就是
        // 同一条正式几何链，不再让后段审核围绕一个可能位于孔内的粗定位中心工作。
        const Eigen::Vector3f holeZuLocatorCenterWorld = canonicalSearchSelectedCenterWorld;
        Eigen::Vector3f canonicalSearchConsensusCenterWorld = holeZuLocatorCenterWorld;
        const float canonicalSearchSupportRadius =
            static_cast<float>(canonicalSearchResolvedRadius);
        const auto canonicalSearchSupportKey = analysisCacheSupportPlaneKey(
            canonicalSearchConsensusCenterWorld, canonicalSearchPreSupportNormal, canonicalSearchSupportRadius);
        GuiFanSearchZhichengPlane canonicalSearchSupport;
        bool canonicalSearchSupportCacheHit = false;
        for (const FenxiCacheZhichengPlaneCacheEntry& entry : analysisCacheSupportPlanes) {
            if (entry.key != canonicalSearchSupportKey) continue;
            canonicalSearchSupport = entry.support;
            canonicalSearchSupportCacheHit = true;
            ++analysisCacheSupportPlaneCacheHits;
            break;
        }
        if (!canonicalSearchSupportCacheHit) {
            canonicalSearchSupport = guJiCanonicalSearchClusterSupportPlane(
                cloud, canonicalSearchConsensusCenterWorld, canonicalSearchPreSupportNormal,
                canonicalSearchSupportRadius, &kdtree);
            ++analysisCacheSupportPlaneCacheMisses;
            if (analysisCacheSupportPlanes.size() >= 512U)
                analysisCacheSupportPlanes.erase(analysisCacheSupportPlanes.begin());
            analysisCacheSupportPlanes.push_back({canonicalSearchSupportKey, canonicalSearchSupport});
        }
        // 支撑面 center 只沿支撑面法向移动 coarseCenter，不改变 U/V 横向 Hole 中心。
        // 因而完整圆和稳定圆弧都使用同一个机械上口平面投影；开放圆弧不再因为“非闭合”
        // 而保留可能位于孔内的 consensusTopW。圆弧本身的横向中心仍完全保持前端结果。
        if (canonicalSearchSupport.valid)
            canonicalSearchConsensusCenterWorld = canonicalSearchSupport.center;
        const Eigen::Vector3f holeZuAnchorCenterWorld = canonicalSearchConsensusCenterWorld;
        const bool tuoYuanFaXianOwned = tuoYuanFaXianJingXiu.used;
        // 已确认的真实孔口通过椭圆几何直接得到孔轴方向时，支撑面只负责校正孔口中心，
        // 不再把法向重新覆盖回表面拟合值；否则会把“椭圆一次反推”又退回旧的多轮表面法向链。
        const Eigen::Vector3f canonicalSearchConsensusNormal = tuoYuanFaXianOwned
            ? canonicalSearchPreSupportNormal
            : (canonicalSearchSupport.valid ? canonicalSearchSupport.normal : canonicalSearchPreSupportNormal);
        const Eigen::Vector3f holeZuAnchorNormal = canonicalSearchConsensusNormal;
        const float holeZuSupportCenterShift = canonicalSearchSupport.valid
            ? (canonicalSearchSupport.center - holeZuLocatorCenterWorld).norm() : 0.0f;
        const ShouDongJuBuZuoBiao canonicalSearchConsensusFrame =
            gouJianShouDongJuBuZuoBiao(canonicalSearchConsensusCenterWorld, canonicalSearchConsensusNormal);
        if (!canonicalSearchConsensusFrame.valid) {
                        continue;
        }
        /** 【类型导航注释】
         * GuiFanSearchPass：Hole 识别总编排中的自定义 结构体。
         * 主要使用位置：HoleShibie_Recognition.cpp（本模块内部）。
         * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
         */
        struct GuiFanSearchPass {
            bool valid = false;
            ShouDongJuBuZuoBiao frame;
            GuiFanRoi roi;
            std::vector<HoleJiheFinal::Sample> samples;
            HoleJiheFinal::Result geometry;
        };

        auto runCanonicalSearchCanonicalPass = [&](const ShouDongJuBuZuoBiao& sourceFrame,
                                        double initialRadius,
                                        const Eigen::Vector3f& holeZuCenterWorld) {
            GuiFanSearchPass pass;
            pass.frame = sourceFrame;
            if (!pass.frame.valid || !(initialRadius > 0.0) || !holeZuCenterWorld.allFinite()) return pass;
            pass.roi = gouJianCanonicalRoiCanonicalRoi(cloud, pass.frame, 0,
                static_cast<float>(initialRadius), &kdtree);
            if (pass.roi.pointCount < 80) return pass;
            pass.samples = std::move(pass.roi.samples);
            HoleJiheFinal::Input canonicalInput;
            canonicalInput.topU = 0.0;
            canonicalInput.topV = 0.0;
            canonicalInput.topW = 0.0;
            canonicalInput.seedU = 0.0;
            canonicalInput.seedV = 0.0;
            canonicalInput.initialTopRadius = initialRadius;
            canonicalInput.canonicalMode = true;
            // sourceFrame 可以为了表面/孔壁测量而移动，但 canonical 的“目标 Hole 中心”
            // 始终来自前端已经锁定的 Hole 族。这样中心优化仍然存在，只是不再把当前
            // 测量帧原点误当成新的 Hole 身份中心。
            const Eigen::Vector3f holeZuCenterDelta = holeZuCenterWorld - pass.frame.origin;
            canonicalInput.canonicalCenterU = holeZuCenterDelta.dot(pass.frame.u);
            canonicalInput.canonicalCenterV = holeZuCenterDelta.dot(pass.frame.v);
            const std::uint64_t initialRadiusBits = HoleFastPath::doubleBits(initialRadius);
            const std::uint64_t canonicalCenterUBits =
                HoleFastPath::doubleBits(canonicalInput.canonicalCenterU);
            const std::uint64_t canonicalCenterVBits =
                HoleFastPath::doubleBits(canonicalInput.canonicalCenterV);
            bool geometryCacheHit = false;
            for (const FenxiCacheGuiFanJiheCacheEntry& entry : analysisCacheCanonicalGeometry) {
                if (entry.canonicalHash != pass.roi.hash
                    || entry.initialRadiusBits != initialRadiusBits
                    || entry.canonicalCenterUBits != canonicalCenterUBits
                    || entry.canonicalCenterVBits != canonicalCenterVBits
                    || !analysisCacheSameCanonicalSamples(entry.samples, pass.samples)) {
                    continue;
                }
                pass.geometry = entry.geometry;
                geometryCacheHit = true;
                ++analysisCacheCanonicalGeometryCacheHits;
                break;
            }
            if (!geometryCacheHit) {
                pass.geometry = HoleJiheFinal::evaluate(pass.samples, canonicalInput);
                ++analysisCacheCanonicalGeometryCacheMisses;
                FenxiCacheGuiFanJiheCacheEntry entry;
                entry.canonicalHash = pass.roi.hash;
                entry.initialRadiusBits = initialRadiusBits;
                entry.canonicalCenterUBits = canonicalCenterUBits;
                entry.canonicalCenterVBits = canonicalCenterVBits;
                entry.samples = pass.samples;
                entry.geometry = pass.geometry;

                if (analysisCacheCanonicalGeometry.size() >= 512U)
                    analysisCacheCanonicalGeometry.erase(analysisCacheCanonicalGeometry.begin());
                analysisCacheCanonicalGeometry.push_back(std::move(entry));
            }
            pass.valid = pass.geometry.valid
                && pass.geometry.canonicalMode
                && !pass.geometry.rawSeedGateUsed
                && !pass.geometry.rawSeedScoreUsed;
            return pass;
        };

        const GuiFanSearchPass canonicalSearchPass1 = runCanonicalSearchCanonicalPass(
            canonicalSearchConsensusFrame, canonicalSearchResolvedRadius, holeZuAnchorCenterWorld);
        if (!canonicalSearchPass1.valid) {
                        continue;
        }
        const Eigen::Vector3f canonicalSearchPass1CenterWorld = canonicalSearchPass1.frame.origin
            + canonicalSearchPass1.frame.u * static_cast<float>(canonicalSearchPass1.geometry.centerU)
            + canonicalSearchPass1.frame.v * static_cast<float>(canonicalSearchPass1.geometry.centerV)
            + canonicalSearchPass1.frame.n * static_cast<float>(canonicalSearchPass1.geometry.topW);
        const double canonicalSearchPass1ResidualCenter = std::hypot(
            canonicalSearchPass1.geometry.centerU, canonicalSearchPass1.geometry.centerV);
        const double canonicalSearchPass1RadiusDeltaFromCluster = std::abs(
            canonicalSearchPass1.geometry.topRadius - canonicalSearchResolvedRadius);

        const HoleLeixingPouMianGongShi::Result frameHintPass1Type =
            HoleLeixingPouMianGongShi::evaluate(canonicalSearchPass1.samples, canonicalSearchPass1.geometry);
        const bool frameHintPass1TypeValid = frameHintPass1Type.valid;
        const HoleShenduGuJi::Result frameHintPass1Depth = frameHintPass1TypeValid
            ? HoleShenduGuJi::evaluate(
                canonicalSearchPass1.samples, canonicalSearchPass1.geometry, frameHintPass1Type)
            : HoleShenduGuJi::Result{};

        const double canonicalExpansionPass2CenterTrigger = 0.35;
        const double canonicalExpansionPass2TopWTrigger = 0.15;

        const double canonicalExpansionRequiredRadialLimit = std::max(
            12.0, canonicalSearchPass1.geometry.topRadius + 4.0);
        const double canonicalExpansionPass2RadiusTrigger = 0.25;
        const bool canonicalExpansionRadialExpansionRequired =
            canonicalExpansionRequiredRadialLimit
                > static_cast<double>(canonicalSearchPass1.roi.radialLimit)
                    + canonicalExpansionPass2RadiusTrigger;
        const bool frameHintGeometryDisplaced =
            canonicalSearchPass1ResidualCenter > canonicalExpansionPass2CenterTrigger
            || std::abs(canonicalSearchPass1.geometry.topW) > canonicalExpansionPass2TopWTrigger;
        const SecondPassCelue::Result frameHintPass2Policy =
            SecondPassCelue::evaluate({
                frameHintPass1TypeValid,
                frameHintPass1TypeValid ? frameHintPass1Type.holeType : 0,
                frameHintPass1TypeValid ? frameHintPass1Type.inputType : 0,
                frameHintPass1TypeValid ? frameHintPass1Type.baseType : 0,
                frameHintPass1TypeValid ? frameHintPass1Type.axialType : 0,
                frameHintPass1TypeValid && frameHintPass1Type.baseDirectionalTaper,
                frameHintPass1TypeValid && frameHintPass1Type.axialConfirmedCone,
                frameHintPass1Depth.used,
                frameHintPass1Depth.bottomValid,
                canonicalSearchPass1.geometry.bottomValid,
                frameHintGeometryDisplaced,
                canonicalExpansionRadialExpansionRequired});
        const bool frameHintPass1ConeCandidate = frameHintPass2Policy.coneCandidate;
        const bool frameHintTypeAmbiguousPass2 = frameHintPass2Policy.typeAmbiguous;
        const bool frameHintDepthRefinementPass2 = frameHintPass2Policy.depthRefinement;
        const bool canonicalExpansionPass2Required = frameHintPass2Policy.required;
        const ShouDongJuBuZuoBiao canonicalSearchPass2Frame = canonicalExpansionPass2Required
            ? gouJianShouDongJuBuZuoBiao(canonicalSearchPass1CenterWorld, canonicalSearchPass1.frame.n)
            : ShouDongJuBuZuoBiao{};
        const GuiFanSearchPass canonicalSearchPass2 = canonicalExpansionPass2Required
            ? runCanonicalSearchCanonicalPass(
                canonicalSearchPass2Frame, canonicalSearchPass1.geometry.topRadius, holeZuAnchorCenterWorld)
            : GuiFanSearchPass{};
        Eigen::Vector3f canonicalSearchPass2CenterWorld = canonicalSearchPass1CenterWorld;
        double canonicalSearchCenterDelta = std::numeric_limits<double>::infinity();
        double canonicalSearchRadiusDelta = std::numeric_limits<double>::infinity();
        double canonicalSearchTopWDelta = std::numeric_limits<double>::infinity();
        if (canonicalSearchPass2.valid) {
            canonicalSearchPass2CenterWorld = canonicalSearchPass2.frame.origin
                + canonicalSearchPass2.frame.u * static_cast<float>(canonicalSearchPass2.geometry.centerU)
                + canonicalSearchPass2.frame.v * static_cast<float>(canonicalSearchPass2.geometry.centerV)
                + canonicalSearchPass2.frame.n * static_cast<float>(canonicalSearchPass2.geometry.topW);
            canonicalSearchCenterDelta = static_cast<double>(
                (canonicalSearchPass2CenterWorld - canonicalSearchPass1CenterWorld).norm());
            canonicalSearchRadiusDelta = std::abs(
                canonicalSearchPass2.geometry.topRadius - canonicalSearchPass1.geometry.topRadius);
            canonicalSearchTopWDelta = std::abs(static_cast<double>(
                (canonicalSearchPass2CenterWorld - canonicalSearchPass1CenterWorld).dot(canonicalSearchPass1.frame.n)));
        }
        const double canonicalSearchRadiusConvergenceLimit = std::max(
            0.45, 0.12 * canonicalSearchPass1.geometry.topRadius);
        const bool canonicalSearchIteration2Converged = canonicalExpansionPass2Required && canonicalSearchPass2.valid
            && canonicalSearchCenterDelta <= 0.35
            && canonicalSearchRadiusDelta <= canonicalSearchRadiusConvergenceLimit
            && canonicalSearchTopWDelta <= 0.35;
        const bool canonicalSearchIteration2Used = canonicalSearchIteration2Converged;
        const bool canonicalSearchIteration2Failed = canonicalExpansionPass2Required && !canonicalSearchPass2.valid;
        const bool canonicalSearchIteration2Rejected = canonicalExpansionPass2Required
            && canonicalSearchPass2.valid && !canonicalSearchIteration2Converged;
        const bool canonicalExpansionPass2Skipped = !canonicalExpansionPass2Required;
        const GuiFanSearchPass& canonicalSearchFinalPass =
            canonicalSearchIteration2Converged ? canonicalSearchPass2 : canonicalSearchPass1;
        frame = canonicalSearchFinalPass.frame;
        const std::vector<HoleJiheFinal::Sample>& finalGeometrySamples =
            canonicalSearchFinalPass.samples;
        const HoleJiheFinal::Result finalGeometry = canonicalSearchFinalPass.geometry;

        const Eigen::Vector3f canonicalSearchFinalCenterWorld = frame.origin
            + frame.u * static_cast<float>(finalGeometry.centerU)
            + frame.v * static_cast<float>(finalGeometry.centerV)
            + frame.n * static_cast<float>(finalGeometry.topW);

        // 点击归属最终以“已经完成孔口/孔壁几何收敛后的机械孔口”审核，而不是用粗候选半径猜。
        // GUI 点击可能落在孔内壁较深位置，因此轴向距离不作为拒绝条件；只检查到最终孔轴的径向关系。
        // 这样完整圆、35%~45% 的残缺圆弧以及凸台外圈粗定位都使用同一个物理归属规则。
        const Eigen::Vector3f finalSeedDelta = seed.point - canonicalSearchFinalCenterWorld;
        const double finalSeedAxial = std::abs(static_cast<double>(
            finalSeedDelta.dot(frame.n)));
        const double finalSeedDistance2 = std::max(0.0,
            static_cast<double>(finalSeedDelta.squaredNorm()));
        const double finalSeedRadial = std::sqrt(std::max(0.0,
            finalSeedDistance2 - finalSeedAxial * finalSeedAxial));
        const double finalSeedOutside = std::max(0.0,
            finalSeedRadial - finalGeometry.topRadius);
        const double finalSeedOutsideLimitUpper = std::max(
            4.25, std::min(7.0,
                0.35 * static_cast<double>(seed.maxRadiusMm) + 2.50));
        const double finalSeedOutsideLimit = std::clamp(
            std::max(4.25, 0.50 * finalGeometry.topRadius),
            4.25, finalSeedOutsideLimitUpper);
        // 最终机械孔已经通过 canonical/孔壁几何证明后，seed 距孔沿只保留辅助意义，
        // 不再使用 FINAL_MOUTH_CLICK_MISMATCH 把已证明的真实孔重新否决。多真孔之间的归属由前面的
        // 固定 ROI 候选选择负责；这里记录兼容性仅用于审计点击距离，不参与识别成败。
        const bool finalSeedMouthCompatible =
            finalSeedOutside <= finalSeedOutsideLimit;

        const bool frameHintFinalReusesPass1 = !canonicalSearchIteration2Converged;
        const HoleLeixingPouMianGongShi::Result baselineTypeReview = frameHintFinalReusesPass1
            ? frameHintPass1Type
            : HoleLeixingPouMianGongShi::evaluate(finalGeometrySamples, finalGeometry);

        const HoleLeixingFinal::Result coneHuiFuReview =
            HoleLeixingFinal::evaluate(finalGeometrySamples, finalGeometry, baselineTypeReview);
        const HoleLeixingPouMianGongShi::Result& holeTypeReview = coneHuiFuReview.finalType;
        if (!holeTypeReview.valid) {
                        continue;
        }
        const HoleLeixingPouMianBase::Result& multiSectionType = holeTypeReview.base;

        const HoleShenduGuJi::Result depthBottomReview =
            (frameHintFinalReusesPass1 && !coneHuiFuReview.huiFuChengGong)
            ? frameHintPass1Depth
            : HoleShenduGuJi::evaluate(finalGeometrySamples, finalGeometry, holeTypeReview);
        const HoleLeixingPouMianGongShi::Result& holeTypeReviewFinal = holeTypeReview;

        const HoleLeixingPouMianGongShi::Result& holeTypeReviewPoseOwner =
            coneHuiFuReview.huiFuChengGong ? baselineTypeReview : holeTypeReviewFinal;

        // 跨姿态回归发现两类“局部截面与完整孔壁证据冲突”的边界情况：
        // 1) 真正锥孔可能在某一次 cross-shift 截面上出现短平台，但 canonical 已经测得
        //    长距离、单调、可靠的上下口；这种局部平台不能一票否决整个锥段。
        // 2) 直孔入口受扫描/打印层纹影响时，可能只在约 1.25~1.50 mm 的短段内形成
        //    假收缩；即使恰好碰到 1.50 mm 最低深度门，也不能仅凭这一小段升级为锥孔。
        // 这里不改变 25°、1.50 mm、depth/Rtop>=0.20 这些既定物理门，只补充
        // “锥段必须具有持续性”的证据所有权，避免单个局部截面覆盖更完整的孔壁事实。
        double finalMeasuredSlopeDegForSemantic = 0.0;
        double finalMeasuredShrinkRatioForSemantic = 0.0;
        if (finalGeometry.bottomValid && finalGeometry.depth > 1e-9
            && finalGeometry.topRadius > 1e-9
            && finalGeometry.topRadius > finalGeometry.bottomRadius) {
            finalMeasuredSlopeDegForSemantic = std::atan2(
                finalGeometry.topRadius - finalGeometry.bottomRadius,
                finalGeometry.depth) * 180.0 / M_PI;
            finalMeasuredShrinkRatioForSemantic =
                (finalGeometry.topRadius - finalGeometry.bottomRadius) / finalGeometry.topRadius;
        }
        const bool sustainedMeasuredConeHuiFu =
            holeTypeReviewFinal.holeType == 1
            && holeTypeReviewFinal.crossShiftPlatformStraight
            && holeTypeReviewFinal.axialConfirmedCone
            && finalGeometry.holeType == 2
            && finalGeometry.bottomValid
            && finalGeometry.depth >= 2.00
            && finalMeasuredSlopeDegForSemantic >= 25.0
            && finalMeasuredShrinkRatioForSemantic >= 0.25
            && holeTypeReviewFinal.base.depthSpan >= 2.50
            && holeTypeReviewFinal.axial.depthSpan >= 2.50
            && holeTypeReviewFinal.base.monotonicRatio >= 0.80
            && holeTypeReviewFinal.axial.monotonicRatio >= 0.80
            && finalGeometry.confidence >= 0.85;

        // 把锥孔物理限制作为正常“孔型 + 深度”审核的一部分读取。
        // DepthBottomReview 已经完成可靠下口/深度审核时直接使用它的审核结果；
        // 若最终采用 FinalGeometry 下口，则复用同一 DepthBottomReview 物理审核函数，避免不同分支阈值不一致。
        HoleShenduGuJi::WuLiReview conePhysicalReview;
        // 审核对象取“最终孔型链”的锥孔结论，而不是 pose owner。ConeRescueReview 救援时
        // 位姿计算可能仍引用前序孔型审核结果，因此任何锥孔救援都必须再次服从统一物理审核。
        if (holeTypeReviewFinal.holeType == 2 || coneHuiFuReview.huiFuChengGong || sustainedMeasuredConeHuiFu) {
            if (!sustainedMeasuredConeHuiFu
                && depthBottomReview.used && depthBottomReview.bottomValid
                && depthBottomReview.physicalReview.evaluated) {
                conePhysicalReview = depthBottomReview.physicalReview;
            } else if (finalGeometry.bottomValid) {
                conePhysicalReview = HoleShenduGuJi::reviewMeasuredCone(
                    finalGeometry.topRadius, finalGeometry.bottomRadius, finalGeometry.depth);
            } else {
                conePhysicalReview = HoleShenduGuJi::reviewProfileConeWithoutReliableBottom(
                    holeTypeReviewFinal.base, finalGeometry.topRadius);
            }
        }

        const bool minimumDepthConeWithoutSustainedSpan =
            conePhysicalReview.evaluated
            && !conePhysicalReview.rejectHole
            && !conePhysicalReview.classifyStraight
            && finalGeometry.bottomValid
            && finalGeometry.depth >= 1.50 - 1e-9
            && finalGeometry.depth <= 1.50 + 1e-6
            && holeTypeReviewFinal.base.depthSpan <= 1.50 + 1e-6
            && holeTypeReviewFinal.axial.depthSpan <= 1.50 + 1e-6;
        if (minimumDepthConeWithoutSustainedSpan) {
            conePhysicalReview.classifyStraight = true;
            conePhysicalReview.reason =
                "ConePhysicalReview_MIN_DEPTH_WITHOUT_SUSTAINED_PROFILE_TO_STRAIGHT";
        }

        if (conePhysicalReview.rejectHole) {
                        continue;
        }
        const bool coneReviewedAsStraight = conePhysicalReview.classifyStraight;
        std::vector<double> depthBottomReviewProfileDepths;
        std::vector<double> depthBottomReviewProfileRadii;
        std::vector<int> depthBottomReviewProfilePoints;
        std::vector<int> depthBottomReviewProfileSectors;
        
        d.centerTop = frame.origin
            + frame.u * static_cast<float>(finalGeometry.centerU)
            + frame.v * static_cast<float>(finalGeometry.centerV)
            + frame.n * static_cast<float>(finalGeometry.topW);

        // 上口轴向位置弱支撑保护：铸造件/严重残缺孔的孔外环带可能不足，支撑面拟合会失效
        // 或只剩很少点。此时 FinalGeometry 的 topW 更像“孔内第一层可拟合截面”，不应该
        // 拥有机械上口高度解释权。保留 FinalGeometry 的平面内中心修正，但把轴向位置投影回
        // 前端真实口沿锁定的 mouth locator 平面。支撑面充分时完全保持原来的正式结果。
        const bool weakMouthSupport = !canonicalSearchSupport.valid
            || canonicalSearchSupport.support < 45
            || canonicalSearchSupport.mad > 0.55f;
        if (weakMouthSupport && holeZuLocatorCenterWorld.allFinite()) {
            Eigen::Vector3f mouthAxialNormal = holeZuAnchorNormal;
            if (!mouthAxialNormal.allFinite() || mouthAxialNormal.norm() < 1e-6f)
                mouthAxialNormal = frame.n;
            if (mouthAxialNormal.allFinite() && mouthAxialNormal.norm() >= 1e-6f) {
                mouthAxialNormal.normalize();
                const float inwardOffset = (d.centerTop - holeZuLocatorCenterWorld)
                    .dot(mouthAxialNormal);
                d.centerTop -= mouthAxialNormal * inwardOffset;
            }
        }
        d.localPlaneNx = frame.n.x();
        d.localPlaneNy = frame.n.y();
        d.localPlaneNz = frame.n.z();
        d.localPlaneTiltDeg = std::acos(std::clamp(frame.n.z(), -1.0f, 1.0f))
            * 180.0f / static_cast<float>(M_PI);
        d.manualPoseTag = "MouthSearch_AdaptiveTiltClusterFirstCanonical";
        d.manualNormalDeltaDeg = std::acos(std::clamp(
            std::abs(frame.n.dot(canonicalSearchGlobalNormal)), 0.0f, 1.0f))
            * 180.0f / static_cast<float>(M_PI);
        d.supportPts = static_cast<int>(std::min<std::size_t>(
            canonicalSearchFinalPass.roi.pointCount,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
        d.circularity = static_cast<float>(std::clamp(finalGeometry.confidence, 0.0, 1.0));
        d.sourceScore = d.circularity;
        d.localVerified = true;
        d.rTop = static_cast<float>(finalGeometry.topRadius);
        d.radius = d.rTop;
        d.finalRadiusLocked = true;
        d.finalRadiusLockedValue = d.rTop;
        d.finalRadiusLockSource = "FinalGeometryTopR";

        // canonical 几何在“Hole 口身份已锁定、但深层轮廓不足”的情况下允许只保留
        // 机械孔口几何，此时 FinalGeometry 自身可能没有足够证据决定向内极性。多截面
        // 孔型审核已经对正/反两个方向都做过测量，因此仅在 FinalGeometry 未给出极性时
        // 使用正式孔型审核的极性，不额外搜索、不改变已有有效极性。
        const int geometryInwardPolarity = HoleJiXing::normalize(finalGeometry.inwardPolarity);
        const int reviewedInwardPolarity = HoleJiXing::normalize(holeTypeReviewFinal.base.polarity);
        const int inwardPolarity = HoleJiXing::known(geometryInwardPolarity)
            ? geometryInwardPolarity : reviewedInwardPolarity;
        const Eigen::Vector3f inwardAxis = frame.n * static_cast<float>(inwardPolarity);
        d.inwardPolarity = inwardPolarity;
        d.wallDownTrackValid = finalGeometry.profileValid;
        d.validLayerCount = finalGeometry.base.validLayers;
        d.sourcePath = std::string(HoleShibieSource::kId);
        d.sourceReason = holeTypeReviewFinal.reason;
        d.seedSourcePath = d.sourcePath;
        d.finalTypeReason = holeTypeReviewFinal.reason;
        d.physicalHoleTypeReason = holeTypeReviewFinal.reason;
        d.coneHuiFuReviewAttempted = coneHuiFuReview.attempted;
        d.coneHuiFuReviewHuiFuChengGong = coneHuiFuReview.huiFuChengGong;
        d.coneHuiFuReviewBottomEvidenceStrong = coneHuiFuReview.bottomEvidenceStrong;
        d.coneHuiFuReviewNormalizedSupportStrong = coneHuiFuReview.normalizedSupportStrong;
        d.coneHuiFuReviewMultiShiftConeConsistent = coneHuiFuReview.multiShiftConeConsistent;
        d.coneHuiFuReviewValidProfileCount = coneHuiFuReview.validProfileCount;
        d.coneHuiFuReviewConeProfileCount = coneHuiFuReview.coneProfileCount;
        d.coneHuiFuReviewExtraConeProfileCount = coneHuiFuReview.extraConeProfileCount;
        d.coneHuiFuReviewPlatformProfileCount = coneHuiFuReview.platformProfileCount;
        d.coneHuiFuReviewBasePointDensityRatio = static_cast<float>(coneHuiFuReview.basePointDensityRatio);
        d.coneHuiFuReviewAxialPointDensityRatio = static_cast<float>(coneHuiFuReview.axialPointDensityRatio);
        d.coneHuiFuReviewSharedSlope = static_cast<float>(coneHuiFuReview.sharedSlope);
        d.coneHuiFuReviewSharedRmse = static_cast<float>(coneHuiFuReview.sharedRmse);
        d.coneHuiFuReviewSlopeSpread = static_cast<float>(coneHuiFuReview.slopeSpread);
        d.coneHuiFuReviewBottomShrink = static_cast<float>(coneHuiFuReview.bottomShrink);
        d.coneHuiFuReviewBottomRadiusRatio = static_cast<float>(coneHuiFuReview.bottomRadiusRatio);
        d.coneHuiFuReviewMode = coneHuiFuReview.mode;
        d.coneHuiFuReviewReason = coneHuiFuReview.reason;
        if (sustainedMeasuredConeHuiFu) {
            if (!d.sourceReason.empty()) d.sourceReason += ";";
            d.sourceReason += "SustainedMeasuredConeRescue";
            d.finalTypeReason = "SustainedMeasuredConeRescue";
            d.physicalHoleTypeReason = "SustainedMeasuredConeRescue";
        }

        if ((holeTypeReviewPoseOwner.holeType == 2 || sustainedMeasuredConeHuiFu)
            && !coneReviewedAsStraight) {
            d.type = 2;
            d.geometryType = HoleMiaoshu::JiheType::Cone;
            d.finalGeometryType = HoleMiaoshu::FinalJiheType::Cone;
            d.physicalHoleTypeDisplay = HoleMiaoshu::WuLiHoleLeixingXianshi::Cone;
            d.coneEvidence = true;
            if (HoleJiXing::known(inwardPolarity) && depthBottomReview.used && depthBottomReview.bottomValid) {

                d.rBot = std::max(0.05f, static_cast<float>(depthBottomReview.bottomRadius));
                d.depth = static_cast<float>(depthBottomReview.depth);
                d.depthMeasured = d.depth;
                d.depthFinal = d.depth;
                d.centerBot = d.centerTop + inwardAxis * d.depth;
                d.reliableBottomRadius = true;
                d.rBotProfile = d.rBot;
                d.profileBasedDr = d.rTop - d.rBot;
                d.slopeDeg = std::atan2(std::abs(d.profileBasedDr), std::max(d.depth, 0.10f))
                    * 180.0f / static_cast<float>(M_PI);
                d.typeReliability = depthBottomReview.confidence >= 0.72 ? "High" : "Medium";
            } else if (HoleJiXing::known(inwardPolarity) && finalGeometry.bottomValid) {
                d.rBot = std::max(0.05f, static_cast<float>(finalGeometry.bottomRadius));
                d.depth = static_cast<float>(finalGeometry.depth);
                d.depthMeasured = d.depth;
                d.depthFinal = d.depth;
                d.centerBot = d.centerTop + inwardAxis * d.depth;
                d.reliableBottomRadius = true;
                d.rBotProfile = d.rBot;
                d.profileBasedDr = d.rTop - d.rBot;
                d.slopeDeg = std::atan2(std::abs(d.profileBasedDr), std::max(d.depth, 0.10f))
                    * 180.0f / static_cast<float>(M_PI);
                d.typeReliability = finalGeometry.confidence >= 0.75 ? "High" : "Medium";
            } else {

                d.rBot = 0.0f;
                d.depth = 0.0f;
                d.depthMeasured = 0.0f;
                d.depthFinal = 0.0f;
                d.centerBot = d.centerTop;
                d.reliableBottomRadius = false;
                d.rBotProfile = 0.0f;
                d.profileBasedDr = 0.0f;
                d.slopeDeg = 0.0f;
                d.typeReliability = "Medium";
            }
        } else {
            d.type = 1;
            d.geometryType = HoleMiaoshu::JiheType::Straight;
            d.finalGeometryType = HoleMiaoshu::FinalJiheType::Straight;
            d.physicalHoleTypeDisplay = HoleMiaoshu::WuLiHoleLeixingXianshi::Straight;
            d.coneEvidence = false;
            d.rBot = d.rTop;
            d.depth = 0.0f;
            d.depthMeasured = 0.0f;
            d.depthFinal = 0.0f;
            d.centerBot = d.centerTop;
            d.reliableBottomRadius = false;
            d.slopeDeg = 0.0f;
            d.typeReliability = finalGeometry.confidence >= 0.70 ? "High" : "Medium";
        }

        if (coneReviewedAsStraight) {
            d.finalTypeReason = conePhysicalReview.reason;
            d.physicalHoleTypeReason = conePhysicalReview.reason;
            if (!d.sourceReason.empty()) d.sourceReason += ";";
            d.sourceReason += conePhysicalReview.reason;
        }

        HoleWaiHuanFaXianState waiHuanFaXian;
        if (d.type == 2) {
            waiHuanFaXian = guJiHoleWaiHuanFaXian(
                cloud, d.centerTop, frame.n, d.rTop, &kdtree);

            const bool usableOuterAnnulus = waiHuanFaXian.valid
                && waiHuanFaXian.quality >= 2;
            // 真实机械孔口已经由圆弧/椭圆直接解析并复核法向时，OuterAnnulus 只保留辅助价值，
            // 不能再覆盖该解析法向；解析证据不足时仍完整保留旧成熟表面法向兜底。
            if (usableOuterAnnulus && !tuoYuanFaXianOwned) {
                waiHuanFaXian.used = true;
                const Eigen::Vector3f displayNormal = waiHuanFaXian.normal;
                d.localPlaneNx = displayNormal.x();
                d.localPlaneNy = displayNormal.y();
                d.localPlaneNz = displayNormal.z();
                d.localPlaneTiltDeg = std::acos(std::clamp(displayNormal.z(), -1.0f, 1.0f))
                    * 180.0f / static_cast<float>(M_PI);
                if (d.depth > 0.0f && HoleJiXing::known(d.inwardPolarity)) {
                    const Eigen::Vector3f displayInward = displayNormal
                        * HoleJiXing::signOrZero(d.inwardPolarity);
                    d.centerBot = d.centerTop + displayInward * d.depth;
                } else {
                    d.centerBot = d.centerTop;
                }
                d.manualPoseTag = "OuterAnnulusFrame_MultiscaleOuterAnnulusNormalLocked";
            }
        }
        d.center = (d.centerTop + d.centerBot) * 0.5f;

        wallGeometryFinalizeMeasuredWallGeometry(
            cloud, &kdtree, d, &analysisCachePosePairs, tuoYuanFaXianOwned);

        if (coneHuiFuReview.huiFuChengGong && !coneReviewedAsStraight) {

            d.type = 2;
            d.geometryType = HoleMiaoshu::JiheType::Cone;
            d.finalGeometryType = HoleMiaoshu::FinalJiheType::Cone;
            d.physicalHoleTypeDisplay = HoleMiaoshu::WuLiHoleLeixingXianshi::Cone;
            d.coneEvidence = true;

            Eigen::Vector3f huiFuChengGongInwardAxis = inwardAxis;
            if (d.holeAxisInValid) {
                const Eigen::Vector3f measuredAxis(
                    d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
                if (measuredAxis.allFinite() && measuredAxis.norm() > 1e-6f)
                    huiFuChengGongInwardAxis = measuredAxis.normalized();
            }
            if (HoleJiXing::known(inwardPolarity) && depthBottomReview.used && depthBottomReview.bottomValid) {
                d.rBot = std::max(0.05f, static_cast<float>(depthBottomReview.bottomRadius));
                d.depth = static_cast<float>(depthBottomReview.depth);
                d.depthMeasured = d.depth;
                d.depthFinal = d.depth;
                d.centerBot = d.centerTop + huiFuChengGongInwardAxis * d.depth;
                d.reliableBottomRadius = true;
                d.rBotProfile = d.rBot;
                d.profileBasedDr = d.rTop - d.rBot;
                d.slopeDeg = std::atan2(std::abs(d.profileBasedDr), std::max(d.depth, 0.10f))
                    * 180.0f / static_cast<float>(M_PI);
                d.typeReliability = depthBottomReview.confidence >= 0.72 ? "High" : "Medium";
            } else if (HoleJiXing::known(inwardPolarity) && finalGeometry.bottomValid) {
                d.rBot = std::max(0.05f, static_cast<float>(finalGeometry.bottomRadius));
                d.depth = static_cast<float>(finalGeometry.depth);
                d.depthMeasured = d.depth;
                d.depthFinal = d.depth;
                d.centerBot = d.centerTop + huiFuChengGongInwardAxis * d.depth;
                d.reliableBottomRadius = true;
                d.rBotProfile = d.rBot;
                d.profileBasedDr = d.rTop - d.rBot;
                d.slopeDeg = std::atan2(std::abs(d.profileBasedDr), std::max(d.depth, 0.10f))
                    * 180.0f / static_cast<float>(M_PI);
                d.typeReliability = finalGeometry.confidence >= 0.75 ? "High" : "Medium";
            } else {
                d.rBot = 0.0f;
                d.depth = 0.0f;
                d.depthMeasured = 0.0f;
                d.depthFinal = 0.0f;
                d.centerBot = d.centerTop;
                d.reliableBottomRadius = false;
                d.rBotProfile = 0.0f;
                d.profileBasedDr = 0.0f;
                d.slopeDeg = 0.0f;
                d.typeReliability = "Medium";
            }
            d.center = 0.5f * (d.centerTop + d.centerBot);
        }


        // 直孔入口半径保守复核：只修“入口圆角/过渡把 R 放大”的情况，绝不向上扩 R。
        // 它位于最终孔轴之后，但仍不参与识别前端；失败时完全保留旧值。
        const StraightBoreRadiusReviewV1Result straightRadiusReviewV1 =
            straightBoreRadiusReviewV1(finalGeometrySamples, frame, d);
        if (straightRadiusReviewV1.used) {
            const float reviewedRadius = std::max(0.50f,
                static_cast<float>(straightRadiusReviewV1.newRadius));
            d.rTop = reviewedRadius;
            d.radius = reviewedRadius;
            // 直孔没有“有效下口小 r”语义；只修上口/孔径 R，不写 rBot/rBotProfile。
            d.finalRadiusLocked = true;
            d.finalRadiusLockedValue = reviewedRadius;
            d.finalRadiusLockSource = "StraightBoreRadiusReviewV1_SHALLOW_PERSISTENT_BORE";
            d.jointRadiusShift = static_cast<float>(straightRadiusReviewV1.newRadius
                - straightRadiusReviewV1.oldRadius);
            if (!d.sourceReason.empty()) d.sourceReason += ";";
            d.sourceReason += "StraightBoreRadiusReviewV1_SHALLOW_PERSISTENT_BORE";
        }
        

        // HoleWeizi 已经提交最终孔轴以后，再做一次只影响锥孔下口计量的局部精测。
        // 这一阶段不参与识别、候选竞争和孔型判断；失败/证据不足时完全保留旧 r/depth。
        const FinalAxisConeBottomV2Result finalAxisConeBottomV2 =
            finalAxisRefineConeBottomV2(finalGeometrySamples, frame, depthBottomReview, d);
        if (finalAxisConeBottomV2.used) {
            d.rBot = std::max(0.05f, static_cast<float>(finalAxisConeBottomV2.radius));
            d.depth = static_cast<float>(finalAxisConeBottomV2.depth);
            d.depthMeasured = d.depth;
            d.depthFinal = d.depth;
            d.rBotProfile = d.rBot;
            d.profileBasedDr = d.rTop - d.rBot;
            d.slopeDeg = std::atan2(std::abs(d.profileBasedDr), std::max(d.depth, 0.10f))
                * 180.0f / static_cast<float>(M_PI);
            Eigen::Vector3f finalAxisIn(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
            if (d.holeAxisInValid && finalAxisIn.allFinite() && finalAxisIn.norm() > 1e-6f) {
                finalAxisIn.normalize();
                d.centerBot = d.centerTop + finalAxisIn * d.depth;
                d.center = 0.5f * (d.centerTop + d.centerBot);
            }
            d.reliableBottomRadius = true;
            if (!d.sourceReason.empty()) d.sourceReason += ";";
            d.sourceReason += finalAxisConeBottomV2.mode;
        }
        
        finalizeObservedBoreRadius(finalGeometrySamples, frame, d);

        // 最终直孔物理审核采用统一的 Hole 族证据模型。这里不再把“完整闭合圆”、
        // “稳定开放圆弧”、“深层孔壁”和“降噪后只剩浅层孔壁”拆成互相独立的补救路径。
        // 前端已经锁定的是同一个物理 Hole；最终审核只回答两个问题：
        //   1) Hole 核心是否确实为空，而不是实心表面采样空斑；
        //   2) 是否存在足够的孔壁证据。孔壁证据既可以来自深层 wall/inner/observed-bore，
        //      也可以来自机械上口后面仍然存在的真实浅层环带。
        // 因此降噪只删掉深层点、或孔口只剩稳定圆弧时，不会因为“深层 slice 数不足”被
        // 单独否定；反过来，3.4 那类内部有实体点的假大圆仍会被核心占据直接否决。
        HoleMiaoshu holeZuAuditGeometry = d;
        holeZuAuditGeometry.centerTop = holeZuAnchorCenterWorld;
        holeZuAuditGeometry.centerBot = holeZuAnchorCenterWorld;
        holeZuAuditGeometry.center = holeZuAnchorCenterWorld;
        holeZuAuditGeometry.rTop = static_cast<float>(canonicalSearchResolvedRadius);

        // 物理审核的轴也属于 Hole 族。优先使用已经统一了外表面/椭圆证据的 Hole 族法向；
        // inwardPolarity 已经由正式类型链确定时，直接把表面法向翻到孔内。若极端情况下
        // polarity 不可用，auditStraightPhysicalVoid 仍会使用最终 holeAxisIn 作为后备。
        if (HoleJiXing::known(d.inwardPolarity)
            && holeZuAnchorNormal.allFinite() && holeZuAnchorNormal.norm() >= 0.5f) {
            Eigen::Vector3f holeZuAxisIn = holeZuAnchorNormal.normalized()
                * static_cast<float>(HoleJiXing::signOrZero(d.inwardPolarity));
            holeZuAuditGeometry.holeAxisInValid = true;
            holeZuAuditGeometry.holeAxisInNx = holeZuAxisIn.x();
            holeZuAuditGeometry.holeAxisInNy = holeZuAxisIn.y();
            holeZuAuditGeometry.holeAxisInNz = holeZuAxisIn.z();
        }

        const StraightWuLiVoidAudit straightVoidAudit =
            auditStraightPhysicalVoid(cloud, &kdtree, holeZuAuditGeometry);

        const StraightHoleZuEvidence straightHoleZuEvidence = pingGuStraightHoleZuEvidence(
            straightVoidAudit,
            canonicalSearchSelectedCluster,
            canonicalSearchResolvedRadius,
            d,
            mouthSearchSelectedPersistentCoreOccupation,
            mouthSearchSelectedCoreDepthBins,
            mouthSearchSelectedCoreToAnnulusRatio);
        const bool rejectStraightPhysicalVoid = d.type == 1
            && (straightVoidAudit.rejectSolidCore || !straightHoleZuEvidence.valid);

        // centerTop 的轴向所有权已经在 wallGeometryFinalizeMeasuredWallGeometry 中统一：
        // 无论直孔/锥孔、深层/浅层证据，HoleWeizi 只能做孔轴和上口平面内的中心精修，
        // 不能再把内部喉部深度写回机械上口。这里不再针对“浅层直孔”单独二次修正，
        // 避免同一工程中出现两套 Z 语义。该旧字段不再参与生产输出。
        const float holeZuAxialAnchorCorrection = 0.0f;

        if (rejectStraightPhysicalVoid) {
            const char* rejectReason = straightVoidAudit.rejectSolidCore
                ? "STRAIGHT_PHYSICAL_VOID_AUDIT_SOLID_CORE"
                : "STRAIGHT_PHYSICAL_VOID_AUDIT_NO_CAVITY_EVIDENCE";
                        continue;
        }

        
        d.finalTypeLocked = true;
        d.finalTypeLockedValue = d.type;
        d.finalTypeLockedRBot = d.rBot;
        d.finalTypeLockedDepth = d.depth;
        d.finalTypeLockedSlopeDeg = d.slopeDeg;
        d.finalTypeLockedCenterBot = d.centerBot;
        d.finalTypeLockSource = coneReviewedAsStraight
            ? conePhysicalReview.reason
            : (sustainedMeasuredConeHuiFu
                ? "SustainedMeasuredConeRescue"
                : (coneHuiFuReview.huiFuChengGong ? "ConeRescueReviewType" : "HoleTypeReviewType"));
        if (finalAxisConeBottomV2.used)
            d.finalTypeLockSource += ";" + finalAxisConeBottomV2.mode;

        // 结果来源记录当前功能链，便于定位几何、孔型、位姿和深度的计算来源。
        // 该来源字符串不参与任何几何计算或孔型判断。
        geometry.source =
            "mouth_support_plane>radius_center_refine>final_geometry>"
            "hole_type_review>canonical_roi>profile_attachment>"
            "surface_cross_shift>adaptive_normal>depth_bottom_review>final_axis_cone_bottom_v2";

        
        d.parameterReliability = finalGeometry.confidence >= 0.70 ? "High" : "Medium";
        d.parameterReliabilityReason = holeTypeReviewFinal.reason;
        d.detectionPlaneZ = d.centerTop.z();
        d.mouthZ0 = d.centerTop.z();
        d.mouthZ = d.centerTop.z();
        d.bestMouthZ = d.centerTop.z();
        d.center = (d.centerTop + d.centerBot) * 0.5f;
        d.sourcePath = std::string(HoleShibieSource::kId);
        d.seedSourcePath = d.sourcePath;
        const QString typeText = d.type == 2
            ? QStringLiteral("锥孔") : QStringLiteral("直孔");
                                        accepted.push_back(d);
        acceptedClusterIds.push_back(canonicalSearchCluster.selectedClusterId);
        acceptedCanonicalHashes.push_back(canonicalSearchFinalPass.roi.hash);
    }
    std::vector<bool> keep(accepted.size(), true);
    for (size_t i = 0; i < accepted.size(); ++i) {
        if (!keep[i]) continue;
        for (size_t j = i + 1; j < accepted.size(); ++j) {
            if (!keep[j]) continue;
            const bool sameFrozenCluster = acceptedClusterIds[i] == acceptedClusterIds[j]
                && acceptedCanonicalHashes[i] == acceptedCanonicalHashes[j];
            const float distance = (accepted[i].centerTop - accepted[j].centerTop).norm();
            const float radiusGate = std::max(2.5f,
                std::min(6.0f, 0.8f * std::max(accepted[i].rTop, accepted[j].rTop)));
            if (!sameFrozenCluster && distance > radiusGate) continue;
            if (shouDongHoleMiaoShuGengYou(accepted[j], accepted[i])) {
                keep[i] = false;
                break;
            } else {
                keep[j] = false;
            }
        }
    }

    for (size_t i = 0; i < accepted.size(); ++i) {
        if (keep[i])
            result.descriptors.push_back(accepted[i]);
    }

    result.holes = static_cast<int>(result.descriptors.size());
    if (result.descriptors.empty()) {
        result.error = QStringLiteral("手动种子附近未识别到可靠孔；完整圆、稳定圆弧与倾斜局部坐标证据均未形成可通过物理审核的目标。");
    }
    return result;
}

}

/** 【函数导航】
 * 作用：预先建立与正式 Hole 识别一致的空间索引/上下文，减少用户点击识别后的等待。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void prewarmHoleShibieKongJianContext(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    prewarmHoleShibieKongJianContextImpl(cloud);
}

/** 【函数导航】
 * 作用：执行手动 seed 局部 Hole 识别主入口。
 * 所属模块：Hole 识别总编排。
 * 主要引用/调用位置：HoleShibie_Recognition.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleShibieResult shouDongJuBuHoleShibie(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const std::vector<ShouDongHoleSeed>& seeds,
    const char* cloudName)
{
    return shouDongJuBuHoleShibieProductionImpl(cloud, seeds, cloudName);
}
