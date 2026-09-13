/*
================================================================================
文件：HoleCanshuShuchu_Export.h
模块：Hole 参数输出接口

【主要职责】
声明内部 Hole 结果到稳定输出记录的转换和 CSV 输出抽象。

【主要调用关系】
主窗口“导出 Hole 参数”流程调用。

【线程与状态】
同步输出；不修改识别结果。

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
把正式孔识别结果整理成稳定、带类型的“对外孔参数记录”，并把数据整理与具体输出方式分开。
当前界面接入 CSV 表格；以后如果需要 HTTP、数据库、串口、共享内存或其他 API，新增 IHoleCanshuShuchu
实现即可，不需要让主窗口或孔识别算法理解新的协议。

主要调用位置：
ZhuChuangKouWindow::onShuchuHoleCanshu() 从孔参数面板下方的“导出孔参数”按钮进入本模块。
HoleCanshuTableBuilder 只读取 HoleMiaoshu，不修改识别结果；CsvHoleCanshuShuchu 只负责文本格式和文件落盘。

维护说明：
接口层保留真实数值类型，不把“无法测量”提前塞进数字字符串。缺失、不适用和方向未知通过 CeliangState 表达，
这样未来 API 可以输出 null/状态码，CSV 则可以输出中文说明。新增字段时先扩展 HoleCanshuRecord，再让各输出器各自序列化。
*/

#include "HoleLeixing_Types.h"

#include <QString>
#include <Eigen/Core>
#include <vector>

namespace holeShuchu {

// 对外字段的测量状态。状态和数值分开保存，避免用 -999、0 或 NaN 冒充业务含义。
/** 【类型导航注释】
 * CeliangState：Hole 参数输出接口中的自定义 枚举。
 * 主要使用位置：HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
enum class CeliangState {
    Measured,      // 已经从当前点云可靠测得，可以对外输出数值。
    Unavailable,   // 当前数据无法可靠测量，例如直孔没有真实可见下口。
    NotApplicable, // 该字段对当前孔形没有物理意义，例如直孔的锥壁坡角。
    Unknown        // 理论上有该字段，但当前证据不足，例如孔轴方向无效。
};

/** 【类型导航注释】
 * ShuziField：Hole 参数输出接口中的自定义 结构体。
 * 主要使用位置：HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShuziField {
    double value = 0.0;
    CeliangState state = CeliangState::Unavailable;
};

/** 【类型导航注释】
 * XiangLiang3Field：Hole 参数输出接口中的自定义 结构体。
 * 主要使用位置：HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct XiangLiang3Field {
    Eigen::Vector3f value{ 0.0f, 0.0f, 0.0f };
    CeliangState state = CeliangState::Unavailable;
};

// 单个孔的稳定输出记录。所有坐标、半径和深度沿用项目坐标单位（当前工程为毫米）；角度单位为度。
/** 【类型导航注释】
 * HoleCanshuRecord：Hole 参数输出接口中的自定义 结构体。
 * 主要使用位置：HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleCanshuRecord {
    int holeIndex = 0;                    // 从 1 开始的导出序号，只用于表格/接口定位，不参与孔识别。
    QString holeShape;                    // 与生产界面一致的孔形文案。
    XiangLiang3Field topCenter;                // 上口中心；直孔只有一个可靠孔口时也放在这里。
    XiangLiang3Field bottomCenter;             // 真实下口中心；没有可靠下口时状态为 Unavailable。
    ShuziField topRadius;                // 上口/规范孔口半径。
    ShuziField bottomRadius;             // 可靠下口半径；直孔或不可观测下口不伪造数值。
    XiangLiang3Field normal;                   // 整体孔轴单位法向。
    ShuziField wallSlopeDeg;             // 锥壁坡角；直孔状态为 NotApplicable。
    ShuziField axisTiltDeg;              // 孔轴相对参考方向的倾角。
    ShuziField depth;                    // 只有真实下口可靠时才输出测量深度。
};

// 把内部 HoleMiaoshu 转成对外记录；不写文件、不弹窗口，也不修改原始孔结果。
/** 【类型导航注释】
 * HoleCanshuTableBuilder：Hole 参数输出接口中的自定义 类。
 * 主要使用位置：ZhuChuangKou_Window.cpp、HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
class HoleCanshuTableBuilder {
public:
    static std::vector<HoleCanshuRecord> build(const std::vector<HoleMiaoshu>& holes);
};

// 通用孔参数输出接口。target 对 CSV 是文件路径，对未来网络/设备实现可以解释为端点或连接标识。
/** 【类型导航注释】
 * IHoleCanshuShuchu：Hole 参数输出接口中的自定义 类。
 * 主要使用位置：HoleCanshuShuchu_Export.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
class IHoleCanshuShuchu {
public:
    virtual ~IHoleCanshuShuchu() = default;

    virtual QString name() const = 0;
    virtual bool write(const QString& target,
                       const std::vector<HoleCanshuRecord>& records,
                       QString* error = nullptr) const = 0;
};

// 当前生产界面使用的 CSV 输出实现。采用 UTF-8，并写入 BOM，便于 Windows 表格软件直接识别中文。
/** 【类型导航注释】
 * CsvHoleCanshuShuchu：Hole 参数输出接口中的自定义 类。
 * 主要使用位置：ZhuChuangKou_Window.cpp、HoleCanshuShuchu_Export.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
class CsvHoleCanshuShuchu final : public IHoleCanshuShuchu {
public:
    QString name() const override;
    bool write(const QString& target,
               const std::vector<HoleCanshuRecord>& records,
               QString* error = nullptr) const override;
};

} // 结束命名空间 holeShuchu
