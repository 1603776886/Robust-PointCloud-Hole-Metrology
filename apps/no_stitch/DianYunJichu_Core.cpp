/*
================================================================================
文件：DianYunJichu_Core.cpp
模块：点云会话实现

【主要职责】
实现无损快照、撤销、点云替换和 Z 高度颜色写入。

【主要调用关系】
由 ZhuChuangKouWindow 的打开、编辑、撤销和显示流程调用。

【线程与状态】
默认 GUI 主线程。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
点云基础状态实现。

维护说明：
本文件按功能整合生产实现。各分区通过明确职责组织，算法阈值和候选顺序集中在对应分区维护。
*/
#include "DianYunJichu_Core.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

// ============================================================================
// 功能分区：点云会话状态实现
// ============================================================================
/*
模块职责：实现当前点云替换、撤销恢复和显示颜色变换的会话级操作。
主要调用位置：ZhuChuangKouWindow。这里不处理界面，也不直接读写磁盘。
维护说明：当前点云、识别参考点云和撤销状态均由 DianYunHuiHua 统一管理，避免同一数据存在多套所有权。
*/
/** 【函数导航】
 * 作用：执行“pushUndo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool DianYunHuiHua::pushUndo()
{
    if (!m_current || m_current->empty()) return false;
    m_undoHistory.push_back(DianYunKuaiZhao::fromCloud(m_current));
    // 撤销栈只保留最近 50 个状态，防止大点云编辑时内存无限增长。
    while (m_undoHistory.size() > 50) m_undoHistory.pop_front();
    return true;
}

/** 【函数导航】
 * 作用：执行“undo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h、ZhuChuangKou_Window.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool DianYunHuiHua::undo()
{
    if (m_undoHistory.empty()) return false;
    m_current = m_undoHistory.back().toCloud();
    m_undoHistory.pop_back();
    return true;
}

/** 【函数导航】
 * 作用：清理/重置“clearUndo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunHuiHua::clearUndo()
{
    std::deque<DianYunKuaiZhao> empty;
    m_undoHistory.swap(empty);
}

/** 【函数导航】
 * 作用：执行“undoRetainedPointPayloadBytes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::size_t DianYunHuiHua::undoRetainedPointPayloadBytes() const
{
    std::size_t total = 0;
    for (const auto& snapshot : m_undoHistory) total += snapshot.pointPayloadBytes();
    return total;
}

/** 【函数导航】
 * 作用：按 Z 高度写入显示颜色，并在需要时执行写时复制保护识别基线。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ZColorYingYongJieGuo DianYunHuiHua::applyZGradient()
{
    ZColorYingYongJieGuo res;
    if (!m_current || m_current->empty()) return res;

    // 文件刚载入时，当前点云与识别参考点云通常共享同一实例。
    // 显示着色不能污染后续孔识别使用的原始 RGB，因此需要时先复制一份显示云。
    if (m_rawCloudOnLoad && m_rawCloudOnLoad.get() == m_current.get()) {
        CloudPtr recolored(new Cloud(*m_current));
        dianYunColor::applyZGradientInPlace(recolored);
        m_current = recolored;
        res.materialized = true;
        return res;
    }

    dianYunColor::applyZGradientInPlace(m_current);
    return res;
}


// ============================================================================
// 功能分区：点云快照实现
// ============================================================================
/*
模块职责：实现点云撤销快照的无损打包与恢复。
主要调用位置：DianYunHuiHua。这里不做文件序列化，也不做浮点量化。
维护说明：坐标与颜色按位保存，不能为了节省内存改成有损量化；任何布局变化都要验证恢复后逐点一致。
*/
#include <cstring>
#include <utility>

namespace {

/** 【函数导航】
 * 作用：执行“floatBits”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::uint32_t floatBits(float v)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return bits;
}

/** 【函数导航】
 * 作用：执行“bitsFloat”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float bitsFloat(std::uint32_t bits)
{
    float v = 0.f;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

}

/** 【函数导航】
 * 作用：执行“fromCloud”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
DianYunKuaiZhao DianYunKuaiZhao::fromCloud(const CloudConstPtr& cloud)
{
    DianYunKuaiZhao s;
    if (!cloud) return s;
    const std::size_t n = cloud->size();
    s.m_points.reserve(n);
    for (const auto& p : *cloud) {
        JinCouPoint pp;
        pp.xBits = floatBits(p.x);
        pp.yBits = floatBits(p.y);
        pp.zBits = floatBits(p.z);
        pp.rgba = p.rgba;
        s.m_points.push_back(pp);
    }
    s.m_capacityHint = cloud->points.capacity();
    s.m_width = cloud->width;
    s.m_height = cloud->height;
    s.m_isDense = cloud->is_dense;
    s.m_headerSeq = cloud->header.seq;
    s.m_headerStamp = cloud->header.stamp;
    s.m_frameId = cloud->header.frame_id;
    s.m_sensorOrigin[0] = cloud->sensor_origin_[0];
    s.m_sensorOrigin[1] = cloud->sensor_origin_[1];
    s.m_sensorOrigin[2] = cloud->sensor_origin_[2];
    s.m_sensorOrigin[3] = cloud->sensor_origin_[3];
    s.m_sensorOrientation[0] = cloud->sensor_orientation_.w();
    s.m_sensorOrientation[1] = cloud->sensor_orientation_.x();
    s.m_sensorOrientation[2] = cloud->sensor_orientation_.y();
    s.m_sensorOrientation[3] = cloud->sensor_orientation_.z();
    return s;
}

/** 【函数导航】
 * 作用：执行“toCloud”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr DianYunKuaiZhao::toCloud() const
{
    CloudPtr out(new Cloud());
    if (m_capacityHint > m_points.size()) out->reserve(m_capacityHint);
    else out->reserve(m_points.size());
    out->width = m_width;
    out->height = m_height;
    out->is_dense = m_isDense;
    out->header.seq = m_headerSeq;
    out->header.stamp = m_headerStamp;
    out->header.frame_id = m_frameId;
    out->sensor_origin_ = Eigen::Vector4f(m_sensorOrigin[0], m_sensorOrigin[1],
                                          m_sensorOrigin[2], m_sensorOrigin[3]);
    out->sensor_orientation_ = Eigen::Quaternionf(m_sensorOrientation[0],
                                                  m_sensorOrientation[1],
                                                  m_sensorOrientation[2],
                                                  m_sensorOrientation[3]);
    out->points.reserve(m_points.size());
    for (const auto& pp : m_points) {
        pcl::PointXYZRGB p;
        p.x = bitsFloat(pp.xBits);
        p.y = bitsFloat(pp.yBits);
        p.z = bitsFloat(pp.zBits);
        p.rgba = pp.rgba;
        out->push_back(p);
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“metadataBytes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::size_t DianYunKuaiZhao::metadataBytes() const
{

    return sizeof(m_width) + sizeof(m_height) + sizeof(m_isDense)
         + sizeof(m_headerSeq) + sizeof(m_headerStamp)
         + sizeof(JinCouPoint) * 2
         + m_frameId.capacity() + 1;
}


// ============================================================================
// 功能分区：显示颜色实现
// ============================================================================
/*
模块职责：
点云显示颜色变换。

主要调用位置：
由 DianYunHuiHua/ZhuChuangKouWindow 在高度着色时调用。

维护说明：
颜色写入会触及点属性和写时复制边界，但不允许改变 XYZ 几何。
*/
#include <algorithm>
#include <cstdint>
#include <vector>

namespace dianYunColor {

/** 【函数导航】
 * 作用：应用/设置“applyZGradientInPlace”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云会话实现。
 * 主要引用/调用位置：DianYunJichu_Core.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void applyZGradientInPlace(const CloudPtr& cloud)
{
    if (!cloud || cloud->empty()) return;

    std::vector<float> zs;
    zs.reserve(cloud->size());
    for (const auto& p : *cloud) zs.push_back(p.z);
    const auto idx = [&](double q) -> std::size_t {
        if (zs.empty()) return 0;
        const double pos = q * (zs.size() - 1);
        return static_cast<std::size_t>(std::clamp(pos, 0.0, double(zs.size() - 1)));
    };
    const std::size_t loIndex = idx(0.02);
    const std::size_t hiIndex = idx(0.98);

    // 这里只需要 2% 和 98% 两个阶次统计量，不需要把整列 Z 完整排序。
    // nth_element 得到与原逻辑相同的分位位置，平均复杂度由 O(N log N) 降为 O(N)。
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(loIndex), zs.end());
    const float zLo = zs[loIndex];
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(hiIndex), zs.end());
    const float zHi = zs[hiIndex];
    const float range = zHi - zLo;
    if (range <= 1e-9f) return;

    const auto lerp = [](float a, float b, float u) { return a + (b - a) * u; };

    for (auto& p : *cloud) {
        float tRaw = (p.z - zLo) / range;
        tRaw = std::clamp(tRaw, 0.0f, 1.0f);

        float t;
        if (tRaw <= 0.5f) {
            t = tRaw * 0.5f;
        } else {
            t = 0.25f + (tRaw - 0.5f) * 1.5f;
        }
        t = std::clamp(t, 0.0f, 1.0f);

        float r = 0, g = 0, bv = 0;
        if      (t < 0.25f) { float u = t / 0.25f;           r = 0;   g = lerp(0, 255, u); bv = 255; }
        else if (t < 0.5f)  { float u = (t - 0.25f) / 0.25f; r = 0;   g = 255;              bv = lerp(255, 0, u); }
        else if (t < 0.75f) { float u = (t - 0.5f) / 0.25f;  r = lerp(0, 255, u); g = 255; bv = 0; }
        else                { float u = (t - 0.75f) / 0.25f; r = 255; g = lerp(255, 0, u); bv = 0; }

        p.r = static_cast<std::uint8_t>(std::clamp(r, 0.0f, 255.0f));
        p.g = static_cast<std::uint8_t>(std::clamp(g, 0.0f, 255.0f));
        p.b = static_cast<std::uint8_t>(std::clamp(bv, 0.0f, 255.0f));
    }
}

}
