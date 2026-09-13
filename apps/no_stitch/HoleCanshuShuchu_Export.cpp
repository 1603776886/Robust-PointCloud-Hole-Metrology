/*
================================================================================
文件：HoleCanshuShuchu_Export.cpp
模块：Hole 参数输出实现

【主要职责】
实现 HoleCanshuRecord 构建、状态转换和 UTF-8 CSV 写入。

【主要调用关系】
由 HoleCanshuShuchu_Export.h 的 builder/output 接口进入。

【线程与状态】
同步文件写入。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
实现孔参数记录构建和 CSV 输出。识别结果先转换成与协议无关、仍保留真实数值类型的 HoleCanshuRecord，
最后只有 CsvHoleCanshuShuchu 会把数值格式化成表格文本。

主要调用位置：
ZhuChuangKouWindow::onShuchuHoleCanshu() 通过 HoleCanshuTableBuilder 和 CsvHoleCanshuShuchu 调用本模块。

维护说明：
没有真实下口时必须明确输出“无法测量”，不能使用旁路估算值、0 或 -999 冒充实测值。
下面的小数位只影响 CSV 显示精度，不参与孔识别，也不改 HoleMiaoshu 中的原始几何值。
*/
#include "HoleCanshuShuchu_Export.h"

#include <QSaveFile>
#include <QStringConverter>
#include <QStringList>
#include <QTextStream>

#include <cmath>
#include <utility>

namespace holeShuchu {
namespace {

constexpr int kCoordinateDecimals = 4;   // XYZ 坐标保留位数；只影响 CSV 文本，单位毫米。
constexpr int kRadiusDepthDecimals = 4;  // 半径和深度保留位数；只影响 CSV 文本，单位毫米。
constexpr int kAngleDecimals = 3;        // 坡角和孔轴倾角保留位数；只影响 CSV 文本，单位度。
constexpr int kVectorDecimals = 6;       // 单位法向分量保留位数；增加位数不会提高识别本身的精度。

/** 【函数导航】
 * 作用：执行“csvEscape”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString csvEscape(const QString& value)
{
    QString out = value;
    out.replace(QLatin1Char('"'), QStringLiteral("\"\""));
    const bool needsQuotes = out.contains(QLatin1Char(','))
        || out.contains(QLatin1Char('"'))
        || out.contains(QLatin1Char('\n'))
        || out.contains(QLatin1Char('\r'));
    return needsQuotes ? QStringLiteral("\"") + out + QStringLiteral("\"") : out;
}

/** 【函数导航】
 * 作用：执行“shapeText”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString shapeText(const HoleMiaoshu& d)
{
    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    switch (d.physicalHoleTypeDisplay) {
    case PHT::Cone:
        return QStringLiteral("锥孔");
    case PHT::Straight:
        return QStringLiteral("直孔");
    case PHT::Straight_DeformedWall:
    case PHT::ConeLike_LowConfidence:
        return QStringLiteral("直孔（壁面变形）");
    case PHT::Artifact:
        return QStringLiteral("假孔特征");
    case PHT::Unknown:
    default:
        break;
    }

    if (d.type == 2)
        return QStringLiteral("锥孔");
    if (d.type == 1)
        return QStringLiteral("直孔");
    return QStringLiteral("未知");
}

/** 【函数导航】
 * 作用：执行“isCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isCone(const HoleMiaoshu& d)
{
    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    switch (d.physicalHoleTypeDisplay) {
    case PHT::Cone:
        return true;
    case PHT::Straight:
    case PHT::Straight_DeformedWall:
    case PHT::ConeLike_LowConfidence:
    case PHT::Artifact:
        return false;
    case PHT::Unknown:
    default:
        return d.type == 2;
    }
}

/** 【函数导航】
 * 作用：执行“topCenterOf”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Eigen::Vector3f topCenterOf(const HoleMiaoshu& d)
{
    if (d.rTop > 0.0f && d.centerTop.allFinite())
        return d.centerTop;
    return d.center;
}

/** 【函数导航】
 * 作用：执行“hasMeasuredBottom”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool hasMeasuredBottom(const HoleMiaoshu& d)
{
    // 不按孔形硬编码“能否有下口”。当前直孔识别若把 reliableBottomRadius 置为 false，
    // 所以仍然输出“无法测量”；未来若算法真正测得直孔下口，输出层无需再改结构。
    if (!d.reliableBottomRadius || !(d.depth > 0.0f) || !std::isfinite(d.depth))
        return false;

    const Eigen::Vector3f top = topCenterOf(d);
    const Eigen::Vector3f delta = d.centerBot - top;
    return top.allFinite() && d.centerBot.allFinite() && delta.squaredNorm() > 1.0e-6f;
}

/** 【函数导航】
 * 作用：执行“measuredNumber”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShuziField measuredNumber(double value)
{
    ShuziField field;
    if (std::isfinite(value)) {
        field.value = value;
        field.state = CeliangState::Measured;
    }
    return field;
}

/** 【函数导航】
 * 作用：执行“measuredVector”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XiangLiang3Field measuredVector(const Eigen::Vector3f& value)
{
    XiangLiang3Field field;
    if (value.allFinite()) {
        field.value = value;
        field.state = CeliangState::Measured;
    }
    return field;
}

/** 【函数导航】
 * 作用：执行“fieldWithState”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShuziField fieldWithState(CeliangState state)
{
    ShuziField field;
    field.state = state;
    return field;
}

/** 【函数导航】
 * 作用：执行“vectorWithState”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XiangLiang3Field vectorWithState(CeliangState state)
{
    XiangLiang3Field field;
    field.state = state;
    return field;
}

/** 【函数导航】
 * 作用：保存/输出“exportedTopRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShuziField exportedTopRadius(const HoleMiaoshu& d)
{
    const float radius = (d.rTop > 0.0f && std::isfinite(d.rTop)) ? d.rTop : d.radius;
    if (!(radius > 0.0f) || !std::isfinite(radius))
        return fieldWithState(CeliangState::Unavailable);
    return measuredNumber(radius);
}

/** 【函数导航】
 * 作用：保存/输出“exportedBottomRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShuziField exportedBottomRadius(const HoleMiaoshu& d)
{
    if (!hasMeasuredBottom(d))
        return fieldWithState(CeliangState::Unavailable);

    if (d.rBotProfile > 0.0f && std::isfinite(d.rBotProfile))
        return measuredNumber(d.rBotProfile);
    if (d.rBot > 0.0f && std::isfinite(d.rBot))
        return measuredNumber(d.rBot);
    return fieldWithState(CeliangState::Unavailable);
}

/** 【函数导航】
 * 作用：保存/输出“exportedSlope”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShuziField exportedSlope(const HoleMiaoshu& d)
{
    if (!isCone(d))
        return fieldWithState(CeliangState::NotApplicable);
    if (d.slopeDeg > 0.0f && std::isfinite(d.slopeDeg))
        return measuredNumber(d.slopeDeg);
    if (d.radiusProfileN >= 2 && d.radiusProfileR2 >= 0.3f
        && d.profileBasedSlope > 0.0f && std::isfinite(d.profileBasedSlope)) {
        return measuredNumber(d.profileBasedSlope);
    }
    return fieldWithState(CeliangState::Unavailable);
}

/** 【函数导航】
 * 作用：保存/输出“exportedAxis”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XiangLiang3Field exportedAxis(const HoleMiaoshu& d)
{
    Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    if (!d.holeAxisInValid || !axis.allFinite() || axis.norm() <= 1.0e-6f)
        return vectorWithState(CeliangState::Unknown);
    return measuredVector(axis.normalized());
}

/** 【函数导航】
 * 作用：执行“stateText”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString stateText(CeliangState state)
{
    switch (state) {
    case CeliangState::Measured:
        return QStringLiteral("已测量");
    case CeliangState::NotApplicable:
        return QStringLiteral("不适用");
    case CeliangState::Unknown:
        return QStringLiteral("未知");
    case CeliangState::Unavailable:
    default:
        return QStringLiteral("无法测量");
    }
}

/** 【函数导航】
 * 作用：执行“numberText”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString numberText(const ShuziField& field, int decimals)
{
    if (field.state != CeliangState::Measured)
        return stateText(field.state);
    return QString::number(field.value, 'f', decimals);
}

/** 【函数导航】
 * 作用：执行“vectorComponentText”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString vectorComponentText(const XiangLiang3Field& field, int component, int decimals)
{
    if (field.state != CeliangState::Measured)
        return field.state == CeliangState::Unknown ? QStringLiteral("方向未知") : stateText(field.state);
    return QString::number(field.value[component], 'f', decimals);
}

/** 【函数导航】
 * 作用：执行“headers”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QStringList headers()
{
    return {
        QStringLiteral("孔编号"),
        QStringLiteral("孔形"),
        QStringLiteral("上口中心X(mm)"),
        QStringLiteral("上口中心Y(mm)"),
        QStringLiteral("TOP(有效上口高度Z,mm)"),
        QStringLiteral("下口中心X(mm)"),
        QStringLiteral("下口中心Y(mm)"),
        QStringLiteral("top(有效下口高度Z,mm)"),
        QStringLiteral("R(有效上口半径,mm)"),
        QStringLiteral("r(有效下口半径,mm)"),
        QStringLiteral("整体法向X"),
        QStringLiteral("整体法向Y"),
        QStringLiteral("整体法向Z"),
        QStringLiteral("坡角(°)"),
        QStringLiteral("孔轴倾角(°)"),
        QStringLiteral("深度(mm)"),
        QStringLiteral("深度状态")
    };
}

/** 【函数导航】
 * 作用：执行“rowValues”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QStringList rowValues(const HoleCanshuRecord& r)
{
    return {
        QString::number(r.holeIndex),
        r.holeShape,
        vectorComponentText(r.topCenter, 0, kCoordinateDecimals),
        vectorComponentText(r.topCenter, 1, kCoordinateDecimals),
        vectorComponentText(r.topCenter, 2, kCoordinateDecimals),
        vectorComponentText(r.bottomCenter, 0, kCoordinateDecimals),
        vectorComponentText(r.bottomCenter, 1, kCoordinateDecimals),
        vectorComponentText(r.bottomCenter, 2, kCoordinateDecimals),
        numberText(r.topRadius, kRadiusDepthDecimals),
        numberText(r.bottomRadius, kRadiusDepthDecimals),
        vectorComponentText(r.normal, 0, kVectorDecimals),
        vectorComponentText(r.normal, 1, kVectorDecimals),
        vectorComponentText(r.normal, 2, kVectorDecimals),
        numberText(r.wallSlopeDeg, kAngleDecimals),
        numberText(r.axisTiltDeg, kAngleDecimals),
        numberText(r.depth, kRadiusDepthDecimals),
        stateText(r.depth.state)
    };
}

} // 结束匿名命名空间

/** 【函数导航】
 * 作用：构建“build”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.h、ZhuChuangKou_Window.cpp、HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HoleCanshuRecord> HoleCanshuTableBuilder::build(const std::vector<HoleMiaoshu>& holes)
{
    std::vector<HoleCanshuRecord> records;
    records.reserve(holes.size());

    for (std::size_t i = 0; i < holes.size(); ++i) {
        const HoleMiaoshu& d = holes[i];
        HoleCanshuRecord record;
        record.holeIndex = static_cast<int>(i + 1);
        record.holeShape = shapeText(d);

        const Eigen::Vector3f top = topCenterOf(d);
        record.topCenter = measuredVector(top);
        record.topRadius = exportedTopRadius(d);

        const bool bottomMeasured = hasMeasuredBottom(d);
        record.bottomCenter = bottomMeasured
            ? measuredVector(d.centerBot)
            : vectorWithState(CeliangState::Unavailable);
        record.bottomRadius = exportedBottomRadius(d);

        record.normal = exportedAxis(d);
        record.axisTiltDeg = record.normal.state == CeliangState::Measured
            ? measuredNumber(d.holeAxisTiltDeg)
            : fieldWithState(CeliangState::Unknown);
        record.wallSlopeDeg = exportedSlope(d);
        record.depth = bottomMeasured
            ? measuredNumber(d.depth)
            : fieldWithState(CeliangState::Unavailable);

        records.push_back(std::move(record));
    }

    return records;
}

/** 【函数导航】
 * 作用：执行“name”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString CsvHoleCanshuShuchu::name() const
{
    return QStringLiteral("CSV 孔参数表");
}

/** 【函数导航】
 * 作用：保存/输出“write”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 参数输出实现。
 * 主要引用/调用位置：HoleCanshuShuchu_Export.h、ZhuChuangKou_Window.cpp、DianYun_IO.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool CsvHoleCanshuShuchu::write(const QString& target,
                                   const std::vector<HoleCanshuRecord>& records,
                                   QString* error) const
{
    if (target.trimmed().isEmpty()) {
        if (error) *error = QStringLiteral("导出路径为空。");
        return false;
    }

    QSaveFile file(target);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) *error = QStringLiteral("无法创建文件：%1").arg(file.errorString());
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << QChar(0xFEFF); // 写 UTF-8 BOM，避免常见 Windows 表格软件把中文识别成乱码。

    const auto writeCsvLine = [&stream](const QStringList& values) {
        for (qsizetype i = 0; i < values.size(); ++i) {
            if (i > 0) stream << QLatin1Char(',');
            stream << csvEscape(values.at(i));
        }
        stream << QLatin1Char('\n');
    };

    writeCsvLine(headers());
    for (const HoleCanshuRecord& record : records)
        writeCsvLine(rowValues(record));

    stream.flush();
    if (stream.status() != QTextStream::Ok) {
        file.cancelWriting();
        if (error) *error = QStringLiteral("写入孔参数表失败。");
        return false;
    }

    if (!file.commit()) {
        if (error) *error = QStringLiteral("保存孔参数表失败：%1").arg(file.errorString());
        return false;
    }

    return true;
}

} // 结束命名空间 holeShuchu
