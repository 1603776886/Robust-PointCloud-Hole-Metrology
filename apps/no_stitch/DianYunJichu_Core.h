/*
================================================================================
文件：DianYunJichu_Core.h
模块：点云会话接口

【主要职责】
声明当前点云、加载基线、撤销快照和颜色写入的核心状态。

【主要调用关系】
主窗口持有 DianYunHuiHua；显示/识别从会话取得点云。

【线程与状态】
会话由 GUI 主线程拥有；后台只读取稳定 ConstPtr。

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
集中管理单点云识别会话、无损撤销快照和显示颜色写入。
本文件只保存当前点云、原始识别点云和撤销状态；文件对话框、孔识别数学与显示逻辑分别由对应模块负责。

主要调用位置：
ZhuChuangKouWindow 持有 DianYunHuiHua；显示、降噪和孔识别通过会话对象取得当前点云。

维护说明：
1. DianYunKuaiZhao 必须无损保存 XYZ、RGBA 和必要的 PCL 元数据。
2. 打开新文件时必须清空上一文件的撤销栈，避免跨文件撤销。
3. rawDianYunOnLoad 保存识别参考点云；显示着色必须通过写时复制保护它，不能修改原始 XYZ/RGB。
*/

#include "DianYunLeixing_Types.h"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

// ============================================================================
// 功能分区：无损撤销快照
// ============================================================================
/*
模块职责：
把点云坐标、颜色和 PCL 元数据无损打包成撤销快照。

主要调用位置：
DianYunHuiHua 在编辑前创建快照，在撤销时恢复快照。

维护说明：
JinCouPoint 保存浮点位模式与 RGBA，不允许改成有损量化。若增加需要保留的 PCL 元数据，必须同步修改 fromCloud()/toCloud()。
*/
class DianYunKuaiZhao
{
public:

    /** 【类型导航注释】
     * JinCouPoint：点云会话接口中的自定义 结构体。
     * 主要使用位置：DianYunJichu_Core.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct JinCouPoint {
        std::uint32_t xBits = 0;
        std::uint32_t yBits = 0;
        std::uint32_t zBits = 0;
        std::uint32_t rgba = 0;
    };

    DianYunKuaiZhao() = default;

    static DianYunKuaiZhao fromCloud(const CloudConstPtr& cloud);
    CloudPtr toCloud() const;

    /** 【函数导航】
     * 作用：执行“pointCount”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::size_t pointCount() const { return m_points.size(); }
    /** 【函数导航】
     * 作用：执行“pointPayloadBytes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::size_t pointPayloadBytes() const { return m_points.size() * sizeof(JinCouPoint); }
    /** 【函数导航】
     * 作用：执行“capacityHint”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::size_t capacityHint() const { return m_capacityHint; }
    std::size_t metadataBytes() const;
    /** 【函数导航】
     * 作用：执行“totalLogicalBytes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::size_t totalLogicalBytes() const { return pointPayloadBytes() + metadataBytes(); }

    /** 【函数导航】
     * 作用：执行“pointAt”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    const JinCouPoint& pointAt(std::size_t i) const { return m_points[i]; }
    /** 【函数导航】
     * 作用：执行“width”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunXianshi_View.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::uint32_t width() const { return m_width; }
    /** 【函数导航】
     * 作用：执行“height”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunXianshi_View.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::uint32_t height() const { return m_height; }
    /** 【函数导航】
     * 作用：执行“isDense”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool isDense() const { return m_isDense; }
    /** 【函数导航】
     * 作用：执行“headerSeq”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::uint32_t headerSeq() const { return m_headerSeq; }
    /** 【函数导航】
     * 作用：执行“headerStamp”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::uint64_t headerStamp() const { return m_headerStamp; }
    /** 【函数导航】
     * 作用：执行“frameId”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    const std::string& frameId() const { return m_frameId; }
    /** 【函数导航】
     * 作用：执行“sensorOrigin”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    const float* sensorOrigin() const { return m_sensorOrigin; }
    /** 【函数导航】
     * 作用：执行“sensorOrientation”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    const float* sensorOrientation() const { return m_sensorOrientation; } // 四元数分量顺序：w、x、y、z。

private:
    std::vector<JinCouPoint> m_points;
    std::size_t m_capacityHint = 0;
    std::uint32_t m_width = 0;
    std::uint32_t m_height = 0;
    bool m_isDense = true;
    std::uint32_t m_headerSeq = 0;
    std::uint64_t m_headerStamp = 0;
    std::string m_frameId;
    float m_sensorOrigin[4] = {0.f, 0.f, 0.f, 0.f};
    float m_sensorOrientation[4] = {1.f, 0.f, 0.f, 0.f}; // 四元数分量顺序：w、x、y、z。
};



// ============================================================================
// 功能分区：当前点云会话
// ============================================================================
/*
模块职责：
保存当前主点云和识别参考点云，并统一提供替换、撤销和显示颜色操作。

主要调用位置：
ZhuChuangKouWindow。

维护说明：
replaceCurrentDianYun() 只替换当前数据，不自动建立撤销点；普通编辑应先 pushUndo()。新文件加载应额外调用 clearUndo()。
*/
/** 【类型导航注释】
 * ZColorYingYongJieGuo：点云会话接口中的自定义 结构体。
 * 主要使用位置：DianYunJichu_Core.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZColorYingYongJieGuo {
    bool materialized = false;   // 是否因为写时复制产生了新的整云实例。
};

class DianYunHuiHua
{
public:
    DianYunHuiHua() = default;

    // 只读调用优先使用 const 指针，明确表达“读取而不修改”。
    /** 【函数导航】
     * 作用：执行“currentDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    CloudConstPtr currentDianYun() const { return m_current; }
    /** 【函数导航】
     * 作用：执行“currentDianYunPtr”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    CloudPtr currentDianYunPtr() const { return m_current; }

    // 普通处理结果替换当前点云；是否建立撤销点由上层流程决定。
    /** 【函数导航】
     * 作用：执行“replaceCurrentDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void replaceCurrentDianYun(const CloudPtr& cloud) { m_current = cloud; }

    // 打开新文件：清空撤销状态，并同时记录一份识别参考点云。
    /** 【函数导航】
     * 作用：应用/设置“setLoadedDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void setLoadedDianYun(const CloudPtr& cloud)
    {
        clearUndo();
        m_current = cloud;
        m_rawCloudOnLoad = cloud;
    }

    // 识别参考点云通常指向文件刚载入、尚未经过显示着色或降噪的点云。
    /** 【函数导航】
     * 作用：执行“rawDianYunOnLoad”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    CloudPtr rawDianYunOnLoad() const { return m_rawCloudOnLoad; }
    /** 【函数导航】
     * 作用：应用/设置“setRawDianYunOnLoad”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void setRawDianYunOnLoad(const CloudPtr& cloud) { m_rawCloudOnLoad = cloud; }

    bool pushUndo();
    bool undo();
    /** 【函数导航】
     * 作用：执行“canUndo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool canUndo() const { return !m_undoHistory.empty(); }
    /** 【函数导航】
     * 作用：执行“undoDepth”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云会话接口。
     * 主要引用/调用位置：DianYunJichu_Core.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    int undoDepth() const { return static_cast<int>(m_undoHistory.size()); }
    void clearUndo();
    std::size_t undoRetainedPointPayloadBytes() const;

    // 应用 Z 高度颜色。若当前点云与识别参考点云共享同一实例，先复制后着色。
    ZColorYingYongJieGuo applyZGradient();

private:
    CloudPtr m_current;
    CloudPtr m_rawCloudOnLoad;
    std::deque<DianYunKuaiZhao> m_undoHistory;
};



// ============================================================================
// 功能分区：显示颜色写入
// ============================================================================
/*
模块职责：
对点云 RGB 字段应用 Z 高度渐变。只改变颜色，不允许修改 XYZ。
*/
namespace dianYunColor {

void applyZGradientInPlace(const CloudPtr& cloud);

}

