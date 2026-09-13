/*
================================================================================
文件：ZhuChuangKou_Window.cpp
模块：主窗口流程编排

【主要职责】
实现点云打开/保存、seed 交互、识别触发、结果展示与常用点云操作。

【主要调用关系】
ZhuChuangKouWindow 的槽函数由 Qt 事件系统调用；识别数学由 HoleShibie_Recognition 下沉实现。

【线程与状态】
大多数代码在 GUI 主线程；耗时识别提交后台 worker。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
实现主窗口的用户流程编排，包括单点云文件打开与保存、显示刷新、孔识别触发、孔参数面板和常用点云变换。
本文件只负责“什么时候调用哪个模块”，具体的点云持久化、孔参数序列化、识别数学和渲染细节分别放在独立模块。

主要调用位置：
main.cpp 创建窗口后，Qt 信号进入这里的槽函数；DianYunHuiHua 保存当前点云，DianYunXuanRanQi 负责显示，
Hole 模块负责识别，HoleCanshuShuchu 负责孔参数输出；主窗口只保留生产交互与正式参数展示。

维护说明：
界面文案和纯显示参数可以在本文件调整；任何会影响孔半径、孔型、深度或法向的数值参数都不要在这里临时修改。
如果一个槽函数开始包含独立的数据处理，应把这部分抽成类或纯函数，再由槽函数调用。
*/
#include "ZhuChuangKou_Window.h"

#include "DianYunJichu_Core.h"
#include "DianYun_IO.h"
#include "DianYunXianshi_View.h"
#include "HoleCanshuShuchu_Export.h"
#include "DianYunJiangZao_Filter.h"
#include "DianYunLeixing_Types.h"
#include "HoleLeixing_Types.h"
#include "ui_ZhuChuangKou_Window.h"

#include <QBoxLayout>
#include <QCheckBox>
#include <QColor>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTextStream>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>
#include <QToolButton>
#include <QVBoxLayout>
#include <QThreadPool>
#include <QStringList>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

using dianYunView::DianYunViewOptions;

// ═══════════════════════════════════════════════════════
// 匿名空间：辅助函数
// ═══════════════════════════════════════════════════════
namespace {

// 打开点云时固定从当前 Windows/macOS/Linux 用户的“桌面”目录开始。
// QStandardPaths 由 Qt 按系统账户解析，避免写死 C:/Users/...；若系统未提供桌面路径，Qt 会返回空字符串并退回原生对话框默认位置。
/** 【函数导航】
 * 作用：执行“dianYunOpenChuShiDirectory”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString dianYunOpenChuShiDirectory()
{
    return QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
}


/** 【函数导航】
 * 作用：执行“dianShuWenBen”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString dianShuWenBen(qulonglong n)
{
    return QStringLiteral("%1 点").arg(n);
}

/** 【函数导航】
 * 作用：显示“xianShiXinXi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void xianShiXinXi(QWidget* parent, const QString& title, const QString& text)
{
    QMessageBox::information(parent, title, text);
}

/** 【函数导航】
 * 作用：显示“xianShiJingGao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void xianShiJingGao(QWidget* parent, const QString& title, const QString& text)
{
    QMessageBox::warning(parent, title, text);
}

/** 【函数导航】
 * 作用：执行“xuanZeCsv1WangGeCanShu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool xuanZeCsv1WangGeCanShu(QWidget* parent, dianYunIO::CsvWangGeCanshu& opt,
                           const QString& title = QStringLiteral("CSV1 高度矩阵参数"))
{
    QDialog dlg(parent);
    dlg.setWindowTitle(title);
    QFormLayout form(&dlg);

    auto* spX = new QDoubleSpinBox(&dlg);
    spX->setRange(0.001, 100.0);
    spX->setDecimals(4);
    spX->setSingleStep(0.05);
    spX->setValue(opt.xStepMm);

    auto* spY = new QDoubleSpinBox(&dlg);
    spY->setRange(0.001, 100.0);
    spY->setDecimals(4);
    spY->setSingleStep(0.05);
    spY->setValue(opt.yStepMm);

    auto* hint = new QLabel(QStringLiteral(
        "CSV1 为二维 Z 高度矩阵：行索引对应 X，列索引对应 Y，坐标从 0 开始。\n"
        "CSV0（三列 X,Y,Z）不使用这里的格距。"), &dlg);
    hint->setWordWrap(true);
    form.addRow(hint);
    form.addRow(QStringLiteral("X 格距 (mm)"), spX);
    form.addRow(QStringLiteral("Y 格距 (mm)"), spY);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form.addRow(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return false;
    opt.xStepMm = spX->value();
    opt.yStepMm = spY->value();
    return true;
}

constexpr float kLowZCutoffMm = -100.0f;

// 独立预处理：仅去除Z<-100的低位底板/离群高度点。
/** 【函数导航】
 * 作用：执行“quChuFu100YiXia”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr quChuFu100YiXia(const CloudPtr& in)
{
    if (!in || in->empty()) return CloudPtr();
    CloudPtr out(new Cloud()); out->reserve(in->size());
    for (const auto& p : *in) {
        if (p.z >= kLowZCutoffMm) out->push_back(p);
    }
    return (out && !out->empty()) ? out : CloudPtr();
}

// 组合预处理：先剔除 Z<-100 的无效点，再执行 SOR 降噪。
// 当前用于需要一次性完成基础清理的入口，单步按钮仍可分别调用对应处理函数。
/** 【函数导航】
 * 作用：执行“yunChuLi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr yunChuLi(const CloudPtr& in)
{
    CloudPtr out = quChuFu100YiXia(in);
    if (!out || out->empty()) return in;
    out = tongJiLiQun(out, 50, 1.0, 0.0f);
    return (out && !out->empty()) ? out : in;
}

// 孔可视化补充：仅根据 HoleMiaoshu 生成显示几何，不依赖识别过程中的临时状态。
/** 【函数导航】
 * 作用：执行“buQuanHoleKeShiHua”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void buQuanHoleKeShiHua(HoleShibieResult& res)
{
    CloudPtr allVis(new Cloud());
    std::vector<CloudPtr> holeVis;
    static const std::array<std::array<uint8_t,3>,10> kCol = {{
        {255,0,0},{0,220,0},{0,140,255},{255,180,0},{255,0,220},
        {0,220,220},{255,100,0},{140,0,255},{0,220,100},{255,60,100}}};

    for (size_t fi = 0; fi < res.descriptors.size(); ++fi) {
        const auto& d = res.descriptors[fi];
        const auto& col = kCol[fi % kCol.size()];
        CloudPtr vi(new Cloud());
        Eigen::Vector3f centerAxis = d.centerBot - d.centerTop;
        const float centerAxisLen = centerAxis.norm();
        const float expectedDepth = std::max(0.0f, d.depth);
        const bool centerBotAxisUsable = centerAxisLen > 0.3f
            && (expectedDepth <= 0.0f || std::abs(centerAxisLen - expectedDepth)
                <= std::max(4.0f, expectedDepth * 1.5f));
        const bool polarityKnown = HoleJiXing::known(d.inwardPolarity);
        Eigen::Vector3f explicitAxis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
        const bool explicitAxisUsable = d.holeAxisInValid && explicitAxis.allFinite()
            && explicitAxis.norm() > 0.3f;
        if (explicitAxisUsable) explicitAxis.normalize();

        Eigen::Vector3f surfaceNormal(d.localPlaneNx, d.localPlaneNy, d.localPlaneNz);
        if ((!surfaceNormal.allFinite() || surfaceNormal.norm() < 0.3f)
            && d.refinedSurfaceNormalValid) {
            surfaceNormal = Eigen::Vector3f(
                d.refinedSurfaceNx, d.refinedSurfaceNy, d.refinedSurfaceNz);
        }
        if (!surfaceNormal.allFinite() || surfaceNormal.norm() < 0.3f)
            surfaceNormal = Eigen::Vector3f::UnitZ();
        else
            surfaceNormal.normalize();

        Eigen::Vector3f axis;
        if (explicitAxisUsable)
            axis = explicitAxis;
        else if (centerBotAxisUsable)
            axis = centerAxis.normalized();
        else
            axis = polarityKnown
                ? surfaceNormal * HoleJiXing::signOrZero(d.inwardPolarity)
                : surfaceNormal;
        if (axis.norm() < 0.3f || !axis.allFinite()) axis = Eigen::Vector3f::UnitZ();
        axis.normalize();
        // 显示中心优先采用识别结果中已经锁定的最终下口中心；只有旧数据没有有效中心时，
        // 才由最终孔轴和深度重建。这样主视图与右侧参数/CSV保持同一几何。
        const bool measuredBottomForDisplay = d.reliableBottomRadius
            && d.depth > 0.0f && std::isfinite(d.depth);
        const bool finalCenterBotUsable = measuredBottomForDisplay
            && d.centerBot.allFinite()
            && (d.centerBot - d.centerTop).norm() > 0.001f;
        const Eigen::Vector3f displayCenterBot = finalCenterBotUsable
            ? d.centerBot
            : ((explicitAxisUsable && d.depth > 0.0f)
                ? (d.centerTop + axis * d.depth)
                : d.centerBot);
        float displayBottomRadius = 0.0f;
        if (d.reliableBottomRadius && d.rBotProfile > 0.0f && std::isfinite(d.rBotProfile))
            displayBottomRadius = d.rBotProfile;
        else if (d.reliableBottomRadius && d.rBot > 0.0f && std::isfinite(d.rBot))
            displayBottomRadius = d.rBot;
        Eigen::Vector3f ringU = axis.unitOrthogonal().normalized();
        Eigen::Vector3f ringV = axis.cross(ringU).normalized();
        auto addRingPoint = [&](const Eigen::Vector3f& c, float r, float a, const std::array<uint8_t,3>& color, float scale) {
            Eigen::Vector3f w = c + r * (std::cos(a) * ringU + std::sin(a) * ringV);
            pcl::PointXYZRGB rp;
            rp.x = w.x(); rp.y = w.y(); rp.z = w.z();
            rp.r = (uint8_t)(color[0] * scale);
            rp.g = (uint8_t)(color[1] * scale);
            rp.b = (uint8_t)(color[2] * scale);
            vi->push_back(rp);
        };

        // 中心点（30个重叠点保证在任何缩放级别可见）
        pcl::PointXYZRGB cp;
        cp.x = d.centerTop.x(); cp.y = d.centerTop.y(); cp.z = d.centerTop.z();
        cp.r = col[0]; cp.g = col[1]; cp.b = col[2];
        for (int i = 0; i < 30; ++i) vi->push_back(cp);

        // GUI 示意圆必须直接对应结果栏 rTop。真实三维孔口交线只保留为内部几何证据，
        // 不再覆盖主示意，否则倾斜锥孔会出现“数值半径正确、图上轮廓看起来却不同”的错觉。
        const float rTop = (d.rTop > 0.0f && std::isfinite(d.rTop)) ? d.rTop : d.radius;
        if (rTop > 0.0f && std::isfinite(rTop)) {
            for (int i = 0; i < 96; ++i) {
                const float a = 2.0f * 3.14159265f * i / 96.0f;
                addRingPoint(d.centerTop, rTop, a, col, 1.0f);
            }
        }

        if (d.type == 1) {
            // 直孔：只有真实下口或已知入孔方向时才画虚线轴线。
            const bool drawAxis = explicitAxisUsable || centerBotAxisUsable || polarityKnown;
            float lineLen = std::max(2.0f, d.depth > 0 ? d.depth : 3.0f);
            Eigen::Vector3f lineEnd = explicitAxisUsable
                ? (d.centerTop + axis * lineLen)
                : (centerBotAxisUsable ? d.centerBot : (d.centerTop + axis * lineLen));
            for (int i = 0; drawAxis && i < 20; ++i) {
                if (i % 3 == 0) continue;  // 间隔留空形成虚线
                const float t = (float)i / 19.0f;
                Eigen::Vector3f w = d.centerTop + (lineEnd - d.centerTop) * t;
                pcl::PointXYZRGB lp;
                lp.x = w.x(); lp.y = w.y(); lp.z = w.z();
                lp.r = (uint8_t)(col[0] * 0.5f);
                lp.g = (uint8_t)(col[1] * 0.5f);
                lp.b = (uint8_t)(col[2] * 0.5f);
                vi->push_back(lp);
            }
        } else {
            // 锥孔：下环 + 轴线（上环已在统一绘制中画好）
            if ((explicitAxisUsable || centerBotAxisUsable)
                && displayBottomRadius > 0.0f && std::isfinite(displayBottomRadius)) {
                for (int i = 0; i < 96; ++i) {
                    const float a = 2.0f * 3.14159265f * i / 96.0f;
                    addRingPoint(displayCenterBot, displayBottomRadius, a, col, 0.65f);
                }
            }

            if (d.depth > 0.0f && std::isfinite(d.depth)
                && (explicitAxisUsable || centerBotAxisUsable || polarityKnown)) {
                Eigen::Vector3f lineEnd = explicitAxisUsable
                    ? (d.centerTop + axis * d.depth)
                    : (centerBotAxisUsable ? d.centerBot : (d.centerTop + axis * d.depth));
                for (int i = 0; i < 30; ++i) {
                    const float t = (float)i / 29.0f;
                    Eigen::Vector3f w = d.centerTop + (lineEnd - d.centerTop) * t;
                    pcl::PointXYZRGB lp;
                    lp.x = w.x(); lp.y = w.y(); lp.z = w.z();
                    lp.r = (uint8_t)(col[0] * 0.5f);
                    lp.g = (uint8_t)(col[1] * 0.5f);
                    lp.b = (uint8_t)(col[2] * 0.5f);
                    vi->push_back(lp);
                }
            }
        }

        vi->width = (uint32_t)vi->size(); vi->height = 1; vi->is_dense = false;
        *allVis += *vi;
        holeVis.push_back(vi);
    }
    allVis->width = (uint32_t)allVis->size(); allVis->height = 1; allVis->is_dense = false;
    res.dianYunView = allVis;
    res.holeVis = std::move(holeVis);
    res.holes = (int)res.descriptors.size();
}

// 沿Z轴高度对点云进行伪彩色映射（jet色带）
/** 【函数导航】
 * 作用：执行“holeLeiXingWenBen”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString holeLeiXingWenBen(int t)
{
    switch (t) {
    case 1: return QStringLiteral("直孔");
    case 2: return QStringLiteral("锥孔/坡孔");
    default: return QStringLiteral("孔");
    }
}

}

// ═══════════════════════════════════════════════════════
// ZhuChuangKouWindow 实现
// ═══════════════════════════════════════════════════════

// ── PCD 头注释孔参数读写 ──
// 统一清空“当前文件携带的孔参数”，避免切换到 PLY/CSV 后仍误用上一份 PCD 的孔数据。
/** 【函数导航】
 * 作用：清理/重置“clearEmbeddedHoleData”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::clearEmbeddedHoleData()
{
    m_hasEmbeddedHoles = false;
    m_embeddedHoleResult = HoleShibieResult{};
}

/** 【函数导航】
 * 作用：执行“jiaZaiPcdHoleCanshu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::jiaZaiPcdHoleCanshu(const QString& filePath)
{
    clearEmbeddedHoleData();
    QString err;
    auto holes = holeIO::readEmbeddedHoles(filePath, &err);
    if (holes.empty()) return;
    const int n = (int)holes.size();
    m_embeddedHoleResult.descriptors = std::move(holes);
    m_embeddedHoleResult.holes = n;
    buQuanHoleKeShiHua(m_embeddedHoleResult);
    m_hasEmbeddedHoles = true;
}

/** 【函数导航】
 * 作用：保存/输出“baoCunDaiHole”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::baoCunDaiHole(const QString& fileName)
{
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) return;
    HoleShibieResult hr;
    if (!m_lastHoleResult.descriptors.empty()) {
        hr = m_lastHoleResult;
    } else if (m_hasEmbeddedHoles) {
        hr = m_embeddedHoleResult;
    }
    QString err;
    holeIO::savePcdWithEmbeddedHoles(fileName, cloud, hr.descriptors, &err);
}

// ═══════════════════════════════════════════════════════
// 主窗口构造流程
//
// 功能: 初始化主窗口UI、VTK渲染、菜单栏、工具栏、孔信息停靠面板。
// 关键步骤:
//   1. 初始化 VTK 渲染器和拾取交互。
//   2. 连接打开和保存。
//   3. 连接预处理和孔识别按钮。
//   4. 连接工具菜单(镜像/倾斜回正)
//   5. 设置坐标拾取和选孔交互回调。
//   7. 创建孔参数面板并接入导出按钮。
// ═══════════════════════════════════════════════════════
ZhuChuangKouWindow::ZhuChuangKouWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::ZhuChuangKouWindow)
{
    ui->setupUi(this);
    // 主窗口只持有一个单点云会话对象；会话统一管理当前云、识别参考云和撤销状态。
    if (!m_session) m_session = std::make_unique<DianYunHuiHua>();

    // QVTKOpenGLNativeWidget 基于 QOpenGLWidget，不能强制原生子窗口；否则 Windows 下可能出现黑色窗口覆盖普通控件。
    // 这里明确关闭该属性，让 VTK 视图继续参与 Qt 的正常合成。
    ui->vtkWidget->setAttribute(Qt::WA_NativeWindow, false);
    ui->vtkWidget->setAutoFillBackground(false);
    m_renderer = std::make_unique<dianYunView::DianYunXuanRanQi>(ui->vtkWidget);

    // 左侧高度颜色条只控制显示，不修改点云几何和孔识别结果。
    m_heightBar = new GaoduColorBarWidget(ui->vtkWidget);
    {
        double r, g, b;
        m_renderer->getBackgroundColor(r, g, b);
        m_heightBar->setChangJingBackgroundColor(QColor::fromRgbF(r, g, b));
    }
    connect(m_heightBar, &GaoduColorBarWidget::settingsChanged, this,
            [this](const dianYunView::GaoduColorMapSettings& s) {
                m_renderer->setHeightColorMap(s);
                if (m_holePreviewRenderer) m_holePreviewRenderer->setHeightColorMap(s);
            });

    setWindowTitle(QStringLiteral("点云孔识别"));
    setMinimumSize(900, 640);

    // 主界面下方常驻视图控制：底图透明度 + 全部点云点的点大小
    // （不在孔参数面板中；点大小调节的是底图全部点，不是种子标记）。
    auto applyOpacity = [this]() {
        const double op = std::clamp(ui->sliderBaseOpacity->value() / 100.0, 0.02, 1.0);
        DianYunViewOptions opt = m_renderer->cloudVizOptions();
        opt.baseOpacityFaded = op;
        opt.baseOpacity = op;
        m_renderer->setDianYunViewOptions(opt);
        m_renderer->setBaseOpacity(op);      // 透明度即时生效，不重建点云
    };
    auto applyPointSize = [this]() {
        DianYunViewOptions opt = m_renderer->cloudVizOptions();
        opt.basePointSize = std::clamp((double)ui->sliderPointSize->value(), 1.0, 20.0);
        m_renderer->setDianYunViewOptions(opt);
        m_renderer->refresh();
    };
    connect(ui->sliderBaseOpacity, &QSlider::valueChanged, this, [applyOpacity](int) { applyOpacity(); });
    connect(ui->sliderPointSize, &QSlider::valueChanged, this, [applyPointSize](int) { applyPointSize(); });

    // 启动时先显示欢迎页，避免空 VTK 视口呈现黑屏。
    auto* welcomePage = new QWidget(ui->viewStack);
    welcomePage->setObjectName(QStringLiteral("welcomePage"));
    welcomePage->setStyleSheet(QStringLiteral(
        "#welcomePage { background:#f4f7fb; }"
        "QLabel#previewTitle { color:#172033; font-size:25px; font-weight:700; }"
        "QLabel#previewBody { color:#475569; font-size:14px; }"
        "QPushButton#welcomeOpenButton { background:#2563eb; color:white; border:0;"
        " border-radius:6px; padding:10px 24px; font-size:15px; font-weight:700; }"
        "QPushButton#welcomeOpenButton:hover { background:#1d4ed8; }"));
    auto* welcomeLayout = new QVBoxLayout(welcomePage);
    welcomeLayout->setContentsMargins(56, 40, 56, 40);
    welcomeLayout->addStretch(2);
    auto* title = new QLabel(QStringLiteral("点云孔识别"), welcomePage);
    title->setObjectName(QStringLiteral("previewTitle"));
    title->setAlignment(Qt::AlignCenter);
    welcomeLayout->addWidget(title);
    auto* body = new QLabel(
        QStringLiteral("打开 PCD / PLY / CSV 点云后进入三维视图。\n"
                       "打开单个点云后进入识别视图。\n"
                       "在三维视图中左键依次点击孔位即可自动编号并识别；右键短按可取消选孔。"),
        welcomePage);
    body->setObjectName(QStringLiteral("previewBody"));
    body->setAlignment(Qt::AlignCenter);
    body->setWordWrap(true);
    welcomeLayout->addWidget(body);
    auto* welcomeOpen = new QPushButton(QStringLiteral("打开点云文件…"), welcomePage);
    welcomeOpen->setObjectName(QStringLiteral("welcomeOpenButton"));
    welcomeOpen->setMinimumWidth(220);
    welcomeOpen->setDefault(true);
    auto* buttonRow = new QHBoxLayout();
    buttonRow->addStretch(1);
    buttonRow->addWidget(welcomeOpen);
    buttonRow->addStretch(1);
    welcomeLayout->addLayout(buttonRow);
    welcomeLayout->addStretch(3);
    ui->viewStack->addWidget(welcomePage);
    ui->viewStack->setCurrentWidget(welcomePage);
    connect(welcomeOpen, &QPushButton::clicked, this, &ZhuChuangKouWindow::onOpenDianYunFile);

    connect(ui->actionOpen_PCD, &QAction::triggered, this, &ZhuChuangKouWindow::onOpenDianYunFile);
    connect(ui->actionSave_PCD, &QAction::triggered, this, &ZhuChuangKouWindow::onSavePcd);
    connect(ui->btnDenoise, &QPushButton::clicked, this, &ZhuChuangKouWindow::onDianYunJiangZao);
    if(ui->btnDenoise){ui->btnDenoise->setText(QStringLiteral("SOR降噪"));ui->btnDenoise->setToolTip(QStringLiteral("统计离群点降噪：保留当前点云高度范围，只清理孤立噪点。"));}
    // “点云回正”已完整移入“工具”菜单，主操作区不再保留重复按钮。
    connect(ui->btnHole, &QPushButton::clicked, this, &ZhuChuangKouWindow::onHoleShibie);
    if(ui->btnHole){
        ui->btnHole->setText(QStringLiteral("立即重识别"));
        ui->btnHole->setToolTip(QStringLiteral("左键点击孔位会自动加入选种并触发识别；此按钮用于立即按当前全部编号重新识别。"));
    }

    // 点击即选孔：不再需要“选孔模式”开关。左键加入，右键短按删除，识别自动防抖触发。
    {
        auto* seedHint = new QLabel(QStringLiteral("左键选孔 · 右键取消 · 自动识别"), ui->centralwidget);
        seedHint->setStyleSheet(QStringLiteral(
            "QLabel{padding:5px 10px;border:1px solid #b9c7d8;border-radius:6px;"
            "background:#eef5ff;color:#244a73;font-weight:600;}"));
        seedHint->setToolTip(QStringLiteral(
            "依次左键点击孔位，编号按点击顺序自动生成；右键短按某个已选孔附近可删除该 seed，后续编号自动补齐。"));
        auto* btnClearSeed = new QPushButton(QStringLiteral("清空选孔"), ui->centralwidget);
        btnClearSeed->setToolTip(QStringLiteral("清空全部 seed、连续编号和当前识别结果。"));
        QHBoxLayout* tl = ui->toolbarLayout;
        if (tl) {
            tl->addWidget(seedHint);
            tl->addWidget(btnClearSeed);
        }
        connect(btnClearSeed, &QPushButton::clicked, this, [this]() {
            if (m_autoHoleRecognitionTimer) m_autoHoleRecognitionTimer->stop();
            m_autoRecognitionPending = false;
            m_sdHoleSeed.clear();
            m_preferredHoleSeedNumber = -1;
            ++m_seedRevision;
            shuaXinSdHoleSeedBiaoJi();
            clearHoleView();
            ui->statusbar->showMessage(QStringLiteral("已清空全部选孔和编号。"), 3500);
        });
    }

    m_autoHoleRecognitionTimer = new QTimer(this);
    m_autoHoleRecognitionTimer->setSingleShot(true);
    m_autoHoleRecognitionTimer->setInterval(70); // 近即时触发；连续快速点击仍由 singleShot 自动合并
    connect(m_autoHoleRecognitionTimer, &QTimer::timeout, this, [this]() {
        if (m_sdHoleSeed.empty()) return;
        if (m_holeRecognitionRunning) {
            m_autoRecognitionPending = true;
            return;
        }
        onHoleShibie();
    });

    m_holeRecognitionPool = new QThreadPool(this);
    m_holeRecognitionPool->setMaxThreadCount(1);
    m_holeRecognitionPool->setExpiryTimeout(-1); // 常驻同一 worker，保住识别模块 thread_local 缓存

    connect(ui->actionUndo, &QAction::triggered, this, &ZhuChuangKouWindow::undo);

    // 编辑菜单：颜色设置
    QMenu* editMenu = ui->menu;   // 合并到 UI 中唯一的“编辑”菜单
    QAction* actBgColor = editMenu->addAction(QStringLiteral("背景颜色"));
    actBgColor->setToolTip(QStringLiteral("输入十六进制颜色码，如 #FFFFFF 白色"));
    connect(actBgColor, &QAction::triggered, this, [this]() {
        double r, g, b;
        m_renderer->getBackgroundColor(r, g, b);
        int ir = (int)(r * 255.0 + 0.5), ig = (int)(g * 255.0 + 0.5), ib = (int)(b * 255.0 + 0.5);
        QString cur = QString("#%1%2%3")
            .arg(ir, 2, 16, QChar('0')).arg(ig, 2, 16, QChar('0')).arg(ib, 2, 16, QChar('0'));
        bool ok = false;
        QString hex = QInputDialog::getText(this,
            QStringLiteral("背景颜色"),
            QStringLiteral("十六进制颜色码（如 #FFFFFF 白色，#000000 黑色）："),
            QLineEdit::Normal, cur, &ok);
        if (!ok || hex.isEmpty()) return;
        hex = hex.trimmed();
        if (hex.startsWith('#')) hex = hex.mid(1);
        if (hex.length() != 6) {
            ui->statusbar->showMessage(QStringLiteral("颜色格式错误，需要6位十六进制如 FFFFFF"), 4000);
            return;
        }
        bool convOk = false;
        int nr = hex.mid(0,2).toInt(&convOk, 16);
        if (!convOk) { ui->statusbar->showMessage(QStringLiteral("颜色格式错误"), 4000); return; }
        int ng = hex.mid(2,2).toInt(&convOk, 16);
        if (!convOk) { ui->statusbar->showMessage(QStringLiteral("颜色格式错误"), 4000); return; }
        int nb = hex.mid(4,2).toInt(&convOk, 16);
        if (!convOk) { ui->statusbar->showMessage(QStringLiteral("颜色格式错误"), 4000); return; }
        m_renderer->setBackgroundColor(nr / 255.0, ng / 255.0, nb / 255.0);
        if (m_holePreviewRenderer)
            m_holePreviewRenderer->setBackgroundColor(nr / 255.0, ng / 255.0, nb / 255.0);
        if (m_heightBar) {
            m_heightBar->setChangJingBackgroundColor(QColor(nr, ng, nb));
        }
        ui->statusbar->showMessage(QStringLiteral("背景颜色已更新为 #%1").arg(hex.toUpper()), 4000);
    });
    m_actSaveWithHoles = ui->actionSaveWithHoles;
    QMenu* toolMenu = menuBar()->addMenu(QStringLiteral("工具"));
    QAction* actFlipX = toolMenu->addAction(QStringLiteral("X方向翻转（绕X轴）"));
    actFlipX->setToolTip(QStringLiteral(
        "数据级变换：绕X轴对Y坐标范围反转，原来离X轴最远的点变为原点(0)、"
        "最近的点变为最远点（y' = maxY - y，结果保持非负）；z=-999 无效点一并翻转但保持无效标记。"));
    connect(actFlipX, &QAction::triggered, this, &ZhuChuangKouWindow::onFanZhuanX);
    QAction* actFlipY = toolMenu->addAction(QStringLiteral("Y方向翻转（绕Y轴）"));
    actFlipY->setToolTip(QStringLiteral(
        "数据级变换：绕Y轴对X坐标范围反转，原来离Y轴最远的点变为原点(0)、"
        "最近的点变为最远点（x' = maxX - x，结果保持非负）；z=-999 无效点一并翻转但保持无效标记。"));
    connect(actFlipY, &QAction::triggered, this, &ZhuChuangKouWindow::onFanZhuanY);
    QAction* actXyz = toolMenu->addAction(QStringLiteral("XYZ 平移…"));
    actXyz->setToolTip(QStringLiteral(
        "弹窗输入 X/Y/Z 平移量(mm)，对点云数据本身做平移（参数层，非仅显示）；z=-999 无效点保持不变。"));
    connect(actXyz, &QAction::triggered, this, &ZhuChuangKouWindow::onXyzPingYi);
    QAction* actLevelCloud = toolMenu->addAction(QStringLiteral("点云回正"));
    actLevelCloud->setToolTip(QStringLiteral(
        "RANSAC拟合大平面，仅校正pitch/roll倾斜，永久变换当前点云。建议在识别前执行。"));
    actLevelCloud->setEnabled(m_session->currentDianYunPtr() && !m_session->currentDianYunPtr()->empty());
    connect(actLevelCloud, &QAction::triggered, this, &ZhuChuangKouWindow::onDianYunShuiPing);
    connect(this, &ZhuChuangKouWindow::dianYunChanged, actLevelCloud, [this, actLevelCloud]() {
        actLevelCloud->setEnabled(m_session->currentDianYunPtr() && !m_session->currentDianYunPtr()->empty());
    });


    m_actShowHoleDock = ui->actionShowHoleDock;
    connect(m_actShowHoleDock, &QAction::triggered, this, [this]() {
        ensureHolePanel();
        m_holePanel->setVisible(!m_holePanel->isVisible());
    });
    menuBar()->addAction(m_actShowHoleDock);

    ui->actionUndo->setEnabled(false);
    connect(this, &ZhuChuangKouWindow::dianYunChanged, [this] {
        ui->actionUndo->setEnabled(m_session->canUndo());
        refreshDianYunView();
        updateStatusBar();

        // 点云一就绪就后台建立与正式识别完全相同的空间核心。用户真正点第一孔时通常已是热启动。
        auto warmCloud = m_session->currentDianYunPtr();
        if (!warmCloud || warmCloud->empty()) warmCloud = m_session->rawDianYunOnLoad();
        if (warmCloud && !warmCloud->empty()) {
            QThreadPool::globalInstance()->start([warmCloud]() {
                prewarmHoleShibieKongJianContext(warmCloud);
            });
        }
    });

    auto updateBtn = [=] {
        ui->actionUndo->setEnabled(m_session->canUndo());
    };
    connect(this, &ZhuChuangKouWindow::dianYunChanged, updateBtn);

    QTimer::singleShot(0, this, &ZhuChuangKouWindow::onFirstShow);

    m_labelPicked = new QLabel(QStringLiteral("选中点：X:---  Y:---  Z:---"), this);
    m_labelPicked->setMinimumWidth(300);
    m_labelPicked->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->statusbar->addPermanentWidget(m_labelPicked, 1);
    m_renderer->setPointPickCallback([this](double x, double y, double z) {
        if (m_labelPicked) {
            m_labelPicked->setText(QStringLiteral("选中点：X:%1  Y:%2  Z:%3")
                .arg(x, 0, 'f', 3)
                .arg(y, 0, 'f', 3)
                .arg(z, 0, 'f', 3));
        }

        const Eigen::Vector3f picked((float)x, (float)y, (float)z);
        const int hitHole = findNearestDisplayedHoleIndex(picked);
        if (hitHole >= 0) {
            showHoleAt(hitHole);
            const int seedNumber = (hitHole >= 0 && hitHole < (int)m_normalDisplaySeedNumbers.size())
                ? m_normalDisplaySeedNumbers[(std::size_t)hitHole] : (hitHole + 1);
            ui->statusbar->showMessage(
                QStringLiteral("已切换到孔 #%1 的局部预览与参数。")
                    .arg(seedNumber), 2200);
            return;
        }

        const bool firstSeed = m_sdHoleSeed.empty();
        m_sdHoleSeed.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        m_preferredHoleSeedNumber = static_cast<int>(m_sdHoleSeed.size());
        ++m_seedRevision;
        shuaXinSdHoleSeedBiaoJi();

        // 第一个 seed 到来后立刻后台预热全云上下文；自动识别防抖窗口通常可以覆盖这段准备时间。
        if (firstSeed) {
            auto warmCloud = m_session->currentDianYunPtr();
            if (!warmCloud || warmCloud->empty()) warmCloud = m_session->rawDianYunOnLoad();
            if (warmCloud && !warmCloud->empty()) {
                QThreadPool::globalInstance()->start([warmCloud]() {
                    prewarmHoleShibieKongJianContext(warmCloud);
                });
            }
        }

        scheduleAutoHoleShibie();
        ui->statusbar->showMessage(
            QStringLiteral("已选孔 #%1；左键继续添加，右键可取消，稍后自动识别。")
                .arg((int)m_sdHoleSeed.size()), 3000);
    });

    // 右键短按用于删除最近 seed；右键拖动仍保留原来的缩放交互。
    m_renderer->setRightPointPickCallback([this](double x, double y, double z) {
        if (m_sdHoleSeed.empty()) {
            ui->statusbar->showMessage(QStringLiteral("当前没有可取消的选孔。"), 1800);
            return;
        }
        const Eigen::Vector3f click((float)x, (float)y, (float)z);
        int best = -1;
        float bestD2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < m_sdHoleSeed.size(); ++i) {
            const float d2 = (m_sdHoleSeed[i] - click).squaredNorm();
            if (d2 < bestD2) { bestD2 = d2; best = (int)i; }
        }
        const float maxR = ui->spinMaxHoleRadius
            ? static_cast<float>(ui->spinMaxHoleRadius->value()) : 10.0f;
        const float hitRadius = std::clamp(0.75f * maxR + 1.5f, 4.0f, 10.0f);
        if (best < 0 || bestD2 > hitRadius * hitRadius) {
            ui->statusbar->showMessage(
                QStringLiteral("右键位置离已选 seed 较远；请在编号标记或孔口附近右键。"), 2200);
            return;
        }

        const int removedNumber = best + 1;
        m_sdHoleSeed.erase(m_sdHoleSeed.begin() + best);
        m_preferredHoleSeedNumber = m_sdHoleSeed.empty() ? -1 : std::min(removedNumber, (int)m_sdHoleSeed.size());
        ++m_seedRevision;
        shuaXinSdHoleSeedBiaoJi();
        if (m_sdHoleSeed.empty()) {
            if (m_autoHoleRecognitionTimer) m_autoHoleRecognitionTimer->stop();
            m_autoRecognitionPending = false;
            clearHoleView();
        } else {
            scheduleAutoHoleShibie();
        }
        ui->statusbar->showMessage(
            QStringLiteral("已取消原 #%1；其后编号已自动前移补齐。当前 %2 个选孔。")
                .arg(removedNumber).arg((int)m_sdHoleSeed.size()), 3500);
    });


    ensureHolePanel();
}

// 主窗口到渲染器的唯一桥接：从 DianYunHuiHua 取得当前点云，再交给 DianYunXuanRanQi。
/** 【函数导航】
 * 作用：把 DianYunHuiHua 中的当前点云同步到 VTK 渲染器并刷新可见场景。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::refreshDianYunView()
{
    if (!m_renderer) return;
    m_renderer->setDianYun(m_session->currentDianYunPtr());
    m_renderer->refresh();
    updateGaoduColorBar(false);
}

/** 【函数导航】
 * 作用：更新/刷新“updateGaoduColorBar”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::updateGaoduColorBar(bool resetToDefaults)
{
    if (!m_heightBar || !m_renderer) return;
    if (!m_renderer->hasValidHeightRange()) {
        m_heightBar->setRangeValid(false);
        return;
    }
    const auto r = m_renderer->validHeightRange();
    m_heightBar->setRange(r.first, r.second, resetToDefaults);
}

ZhuChuangKouWindow::~ZhuChuangKouWindow()
{
    // QVTK widget 由 ui/Qt 父子关系销毁，因此先释放持有其裸指针的渲染器，避免析构阶段访问已销毁 widget。
    m_holePreviewRenderer.reset();
    m_renderer.reset();
    delete ui;
}

/** 【函数导航】
 * 作用：更新/刷新“shuaXinSdHoleSeedBiaoJi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::shuaXinSdHoleSeedBiaoJi()
{
    if (m_sdHoleSeed.empty()) {
        m_renderer->clearDieJiaDianYunNoRefresh();
        m_renderer->clearSeedNumberLabels();
        return;
    }

    CloudPtr seedCloud(new Cloud);
    seedCloud->reserve(m_sdHoleSeed.size());
    for (const auto& s : m_sdHoleSeed) {
        pcl::PointXYZRGB p;
        p.x = s.x();
        p.y = s.y();
        p.z = s.z();
        p.r = 255; p.g = 0; p.b = 0;
        seedCloud->push_back(p);
    }

    DianYunViewOptions opt = m_renderer->cloudVizOptions();
    opt.overlayPointSize = std::max(opt.overlayPointSize, 10.0);
    opt.overlayForceColor = false;
    m_renderer->setDianYunViewOptions(opt);
    m_renderer->setDieJiaDianYunNoRefresh(seedCloud);
    // 标签内容由 vector 当前顺序实时生成，所以删除任意 seed 后会自然得到 1..N 连续编号。
    m_renderer->setSeedNumberLabels(m_sdHoleSeed);
}

void ZhuChuangKouWindow::scheduleAutoHoleShibie()
{
    if (m_sdHoleSeed.empty()) return;
    if (m_holeRecognitionRunning) {
        m_autoRecognitionPending = true;
        return;
    }
    if (m_autoHoleRecognitionTimer) m_autoHoleRecognitionTimer->start();
}

/*
Hole 参数输出文件默认路径入口。
程序刚启动且尚未导出过孔参数时沿用 QFileDialog 默认目录；成功导出后记住该目录。这里只管理界面路径，不参与 Hole 识别、几何或参数计算。
*/
int ZhuChuangKouWindow::findNearestDisplayedHoleIndex(const Eigen::Vector3f& worldPt, float maxDistMm) const
{
    if (!worldPt.allFinite() || m_normalDisplayCandidates.empty()) return -1;

    int best = -1;
    float bestD2 = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < m_normalDisplayCandidates.size(); ++i) {
        const HoleMiaoshu& d = m_normalDisplayCandidates[i];
        const Eigen::Vector3f c = d.centerTop.allFinite() ? d.centerTop : d.center;
        if (!c.allFinite()) continue;
        float rTop = (d.rTop > 0.0f && std::isfinite(d.rTop)) ? d.rTop : d.radius;
        if (!std::isfinite(rTop) || rTop <= 0.0f) rTop = 5.0f;
        const float tol = (maxDistMm > 0.0f) ? maxDistMm
            : std::clamp(rTop * 1.15f + 1.5f, 3.0f, 12.0f);
        const float d2 = (c - worldPt).squaredNorm();
        if (d2 <= tol * tol && d2 < bestD2) {
            bestD2 = d2;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void ZhuChuangKouWindow::clearHoleLocalPreview()
{
    if (m_lblHolePreviewInfo) m_lblHolePreviewInfo->setText(QStringLiteral("选择孔后显示局部点云。"));
    if (!m_holePreviewRenderer) return;
    m_holePreviewRenderer->clearHeightScalarProjection();
    m_holePreviewRenderer->setDianYun(CloudPtr());
    m_holePreviewRenderer->clearOverlayLines();
    m_holePreviewRenderer->clearDieJiaDianYunNoRefresh();
    m_holePreviewRenderer->refresh();
}

void ZhuChuangKouWindow::updateHoleLocalPreview(int index)
{
    if (!m_holePreviewRenderer || index < 0 || index >= (int)m_normalDisplayCandidates.size()) return;
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        clearHoleLocalPreview();
        return;
    }

    const HoleMiaoshu& d = m_normalDisplayCandidates[(std::size_t)index];
    const Eigen::Vector3f cTop = d.centerTop.allFinite() ? d.centerTop : d.center;
    if (!cTop.allFinite()) {
        clearHoleLocalPreview();
        return;
    }
    float rTop = (d.rTop > 0.0f) ? d.rTop : d.radius;
    if (!std::isfinite(rTop) || rTop <= 0.0f) rTop = 5.0f;

    // 自动观察范围：孔越大范围越大，但微观窗口始终限制在 8~15 mm，避免带进过多整件结构。
    const float clipRadius = std::clamp(rTop * 1.8f, 8.0f, 15.0f);

    Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
    if (!d.holeAxisInValid || !axis.allFinite() || axis.norm() < 1e-6f) {
        axis = Eigen::Vector3f(d.localPlaneNx, d.localPlaneNy, d.localPlaneNz);
        const float inwardSign = HoleJiXing::signOrZero(d.inwardPolarity);
        if (axis.allFinite() && axis.norm() >= 1e-6f && std::fabs(inwardSign) > 0.5f) axis *= inwardSign;
    }
    if (!axis.allFinite() || axis.norm() < 1e-6f) axis = Eigen::Vector3f::UnitZ();
    else axis.normalize();

    const float inwardSpan = (d.depth > 0.0f && std::isfinite(d.depth))
        ? std::clamp(d.depth + 3.0f, clipRadius, 24.0f)
        : clipRadius;
    const float outwardSpan = clipRadius * 0.45f;
    const float r2Limit = clipRadius * clipRadius;

    CloudPtr local(new Cloud);
    local->reserve(std::min<std::size_t>(cloud->size(), 180000));
    for (const auto& p : *cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= -998.0f) continue;
        const Eigen::Vector3f q(p.x - cTop.x(), p.y - cTop.y(), p.z - cTop.z());
        const float axial = q.dot(axis);
        if (axial < -outwardSpan || axial > inwardSpan) continue;
        const Eigen::Vector3f radialVec = q - axial * axis;
        if (radialVec.squaredNorm() > r2Limit) continue;
        local->push_back(p);
    }

    if (local->empty()) {
        if (m_lblHolePreviewInfo) m_lblHolePreviewInfo->setText(QStringLiteral("局部范围内没有有效点云。"));
        m_holePreviewRenderer->setDianYun(local);
        m_holePreviewRenderer->clearOverlayLines();
        m_holePreviewRenderer->refresh();
        return;
    }

    // 小窗口颜色不再使用世界 Z，而使用“上口中心 + 孔轴”的局部深度：
    // 上口附近为高值（偏红），向孔内为低值（偏蓝）。只改变颜色标量，不改变任何 XYZ 几何。
    m_holePreviewRenderer->setHeightScalarProjection(cTop, -axis);

    double br = 0.0, bg = 0.0, bb = 0.0;
    m_renderer->getBackgroundColor(br, bg, bb);
    m_holePreviewRenderer->setBackgroundColor(br, bg, bb);
    DianYunViewOptions opt = m_renderer->cloudVizOptions();
    opt.basePointSize = std::max(2.5, opt.basePointSize);
    m_holePreviewRenderer->setDianYunViewOptions(opt);
    // 完全复用主视图高度着色设置；点 XYZ 不做几何缩放。
    m_holePreviewRenderer->setHeightColorMap(m_renderer->heightColorMap());

    // 关键：局部点云和“当前这个孔”的高亮几何必须在同一次 refresh 前一起提交。
    // 旧实现先 refresh 再换 overlay，切孔时会让高亮线滞后一帧，看起来像坐标不重合。
    std::vector<HoleMiaoshu> oneHole{d};
    auto holePoly = holeDieJia::buildLineGeometryForPreview(oneHole, 0);
    m_holePreviewRenderer->setDianYun(local);
    m_holePreviewRenderer->setOverlayLineOptions(false, 1.0, 0.0, 0.0, 1.0, 3.0);
    m_holePreviewRenderer->setOverlayLinePolyData(holePoly);
    m_holePreviewRenderer->refresh();

    // 默认视角：孔的局部 XZ 剖面回正显示（直观看深度方向），再绕局部 X 轴倾斜 18°。
    // 关注中心以上口中心为基准并轻微朝孔内偏置，减少上方空旷感；坐标轴继续保留。
    m_holePreviewRenderer->setHoleInspectionView(cTop, axis, 18.0);

    if (m_lblHolePreviewInfo) {
        const int seedNumber = (index >= 0 && index < (int)m_normalDisplaySeedNumbers.size())
            ? m_normalDisplaySeedNumbers[(std::size_t)index]
            : (index + 1);
        m_lblHolePreviewInfo->setText(QStringLiteral("孔 #%1 · XZ 剖面回正 + 18° · 上口居中 · 颜色按孔轴深度 · 局部半径 %2 mm · %3 点 · 左键拖动独立旋转")
            .arg(seedNumber)
            .arg(clipRadius, 0, 'f', 1)
            .arg((qulonglong)local->size()));
    }
}

QString ZhuChuangKouWindow::holeShuchuSuggestedPath(const QString& fileName) const
{
    if (m_holeShuchuDirectory.isEmpty())
        return fileName;
    return m_holeShuchuDirectory + QLatin1Char('/') + fileName;
}

/*
记录最近一次孔参数导出目录。
输入：用户在 QFileDialog 中最终确认的完整文件名。
输出：更新 m_holeShuchuDirectory，供下一次孔参数导出使用。
*/
void ZhuChuangKouWindow::jiLuHoleShuchuDirectory(const QString& fileName)
{
    if (fileName.isEmpty())
        return;
    const QString directory = QFileInfo(fileName).absolutePath();
    if (!directory.isEmpty())
        m_holeShuchuDirectory = directory;
}

/*
孔参数导出入口。
调用位置：孔参数面板下方的“导出孔参数”按钮。
数据来源：优先使用当前界面正在显示的规范孔列表，保证导出内容与用户看到的孔一致；
如果当前只存在从 PCD 读取的嵌入孔，也允许直接导出这些孔。
本函数只处理界面交互，字段整理和 CSV 写入全部交给 HoleCanshuShuchu 模块。
*/
/** 【函数导航】
 * 作用：执行“onShuchuHoleCanshu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onShuchuHoleCanshu()
{
    std::vector<HoleMiaoshu> holes;
    if (!m_normalDisplayCandidates.empty()) {
        holes = m_normalDisplayCandidates;
    } else if (!m_lastHoleResult.descriptors.empty()) {
        const auto indices = holeXianshi::selectGuiFanNormalXianshiHouXuan(
            m_lastHoleResult.descriptors);
        holes.reserve(indices.size());
        for (int index : indices) {
            if (index >= 0 && index < static_cast<int>(m_lastHoleResult.descriptors.size()))
                holes.push_back(m_lastHoleResult.descriptors[static_cast<std::size_t>(index)]);
        }
    } else if (m_hasEmbeddedHoles && !m_embeddedHoleResult.descriptors.empty()) {
        const auto indices = holeXianshi::selectGuiFanNormalXianshiHouXuan(
            m_embeddedHoleResult.descriptors);
        holes.reserve(indices.size());
        for (int index : indices) {
            if (index >= 0 && index < static_cast<int>(m_embeddedHoleResult.descriptors.size()))
                holes.push_back(m_embeddedHoleResult.descriptors[static_cast<std::size_t>(index)]);
        }
    }

    if (holes.empty()) {
        xianShiJingGao(this, QStringLiteral("导出孔参数"),
            QStringLiteral("当前没有可导出的孔。请先完成孔识别，或打开带有孔参数的 PCD 文件。"));
        return;
    }

    QString suggestedName = QStringLiteral("孔参数.csv");
    if (!m_cloudName.empty())
        suggestedName = QString::fromStdString(m_cloudName) + QStringLiteral("_孔参数.csv");

    QString fileName = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("导出孔参数表"),
        holeShuchuSuggestedPath(suggestedName),
        QStringLiteral("CSV 表格 (*.csv)"));
    if (fileName.isEmpty())
        return;
    if (!fileName.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive))
        fileName += QStringLiteral(".csv");
    jiLuHoleShuchuDirectory(fileName);

    const auto records = holeShuchu::HoleCanshuTableBuilder::build(holes);
    const holeShuchu::CsvHoleCanshuShuchu output;
    QString error;
    if (!output.write(fileName, records, &error)) {
        xianShiJingGao(this, QStringLiteral("导出孔参数"),
            error.isEmpty() ? QStringLiteral("导出失败。") : error);
        return;
    }

    ui->statusbar->showMessage(
        QStringLiteral("已导出 %1 个孔：%2").arg(static_cast<qulonglong>(records.size())).arg(fileName),
        8000);
    xianShiXinXi(this, QStringLiteral("导出孔参数"),
        QStringLiteral("已导出 %1 个孔的参数表。\n%2")
            .arg(static_cast<qulonglong>(records.size()))
            .arg(fileName));
}


// ── 撤销 ──
// 撤销状态由 DianYunHuiHua 统一管理，主窗口只负责触发和刷新。
/** 【函数导航】
 * 作用：执行“undo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：DianYunJichu_Core.cpp、DianYunJichu_Core.h、ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::undo()
{
    if (!m_session || !m_session->canUndo()) return;
    clearHoleView();
    if (!m_session->undo()) return;
    refreshDianYunView();
    ui->actionUndo->setEnabled(m_session->canUndo());
}

// ── 数据层变换：XYZ 平移 / X方向翻转 / Y方向翻转 ──
// 两者都直接修改点云数据（非仅显示），并保持 z≈-999 无效点的默认排除约定。
namespace {
/** 【函数导航】
 * 作用：执行“dianShiWuXiaoDian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool dianShiWuXiaoDian(const pcl::PointXYZRGB& p)
{
    return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)
        || p.z <= -998.0f;
}
}

// XYZ 平移：弹窗输入 dx/dy/dz(mm)，对所有有效点做数据层平移。
/** 【函数导航】
 * 作用：执行“onXyzPingYi”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onXyzPingYi()
{
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        xianShiJingGao(this, QStringLiteral("XYZ 平移"), QStringLiteral("点云为空。"));
        return;
    }

    QDialog dlg(this);
    dlg.setWindowTitle(QStringLiteral("XYZ 平移（数据层变换）"));
    auto* form = new QFormLayout(&dlg);
    auto makeSpin = [&dlg]() {
        auto* sb = new QDoubleSpinBox(&dlg);
        sb->setRange(-1000000.0, 1000000.0);
        sb->setDecimals(3);
        sb->setSingleStep(1.0);
        sb->setValue(0.0);
        return sb;
    };
    auto* sbx = makeSpin();
    auto* sby = makeSpin();
    auto* sbz = makeSpin();
    form->addRow(QStringLiteral("X 平移 (mm)"), sbx);
    form->addRow(QStringLiteral("Y 平移 (mm)"), sby);
    form->addRow(QStringLiteral("Z 平移 (mm)"), sbz);
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;

    const float dx = (float)sbx->value();
    const float dy = (float)sby->value();
    const float dz = (float)sbz->value();
    if (dx == 0.0f && dy == 0.0f && dz == 0.0f) {
        xianShiXinXi(this, QStringLiteral("XYZ 平移"),
                     QStringLiteral("平移量为 0，未执行变换。"));
        return;
    }

    m_session->pushUndo();
    CloudPtr out(new Cloud(*cloud));
    std::size_t nValid = 0;
    for (auto& p : out->points) {
        if (dianShiWuXiaoDian(p)) continue;   // -999 无效点保持不变
        p.x += dx;
        p.y += dy;
        p.z += dz;
        ++nValid;
    }
    m_session->replaceCurrentDianYun(out);
    // 数据已变：先清除旧孔覆盖层但不渲染旧云，再由 dianYunChanged 统一刷新一次新云。
    clearHoleView(false);
    emit dianYunChanged();
    xianShiXinXi(this, QStringLiteral("XYZ 平移"),
                 QStringLiteral("已平移 %1 个有效点：(%2, %3, %4) mm；无效点(-999)保持不变，旧识别结果已清除。")
                     .arg((qulonglong)nValid).arg(dx, 0, 'f', 3)
                     .arg(dy, 0, 'f', 3).arg(dz, 0, 'f', 3));
}

// X方向翻转（绕X轴）：对 Y 坐标做范围反转，y' = maxY - y。
// 原来离X轴最远的点变为原点(0)，最近的点变为最远点；结果保持非负。
// 范围只由有效点（z≈-999 以外）计算；-999 无效点也一起翻转，但 z 保持 -999。
/** 【函数导航】
 * 作用：执行“onFanZhuanX”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onFanZhuanX()
{
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        xianShiJingGao(this, QStringLiteral("X方向翻转"), QStringLiteral("点云为空。"));
        return;
    }

    float minY = 0.0f, maxY = 0.0f;
    bool hasValid = false;
    for (const auto& p : cloud->points) {
        if (dianShiWuXiaoDian(p)) continue;   // 排除 z≈-999 无效点
        if (!hasValid) {
            minY = maxY = p.y;
            hasValid = true;
            continue;
        }
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }
    if (!hasValid) {
        xianShiJingGao(this, QStringLiteral("X方向翻转"),
                       QStringLiteral("点云没有有效点（全部为 z=-999 无效点）。"));
        return;
    }

    m_session->pushUndo();
    CloudPtr out(new Cloud(*cloud));
    for (auto& p : out->points) {
        p.y = maxY - p.y;   // 有效点与 -999 无效点一起翻转
    }
    m_session->replaceCurrentDianYun(out);
    // 数据已变：先清除旧孔覆盖层但不渲染旧云，再由 dianYunChanged 统一刷新一次新云。
    clearHoleView(false);
    emit dianYunChanged();
    xianShiXinXi(this, QStringLiteral("X方向翻转"),
                 QStringLiteral("已完成 Y 坐标范围反转（y' = maxY - y）：原最远点变为原点、最近点变为最远点；-999 无效点已一并翻转但仍按 -999 排除，旧识别结果已清除。"));
}

// Y方向翻转（绕Y轴）：对 X 坐标做范围反转，x' = maxX - x。
// 原来离Y轴最远的点变为原点(0)，最近的点变为最远点；结果保持非负。
// 范围只由有效点计算；-999 无效点也一起翻转，但 z 保持 -999。
/** 【函数导航】
 * 作用：执行“onFanZhuanY”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onFanZhuanY()
{
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        xianShiJingGao(this, QStringLiteral("Y方向翻转"), QStringLiteral("点云为空。"));
        return;
    }

    float minX = 0.0f, maxX = 0.0f;
    bool hasValid = false;
    for (const auto& p : cloud->points) {
        if (dianShiWuXiaoDian(p)) continue;   // 排除 z≈-999 无效点
        if (!hasValid) {
            minX = maxX = p.x;
            hasValid = true;
            continue;
        }
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
    }
    if (!hasValid) {
        xianShiJingGao(this, QStringLiteral("Y方向翻转"),
                       QStringLiteral("点云没有有效点（全部为 z=-999 无效点）。"));
        return;
    }

    m_session->pushUndo();
    CloudPtr out(new Cloud(*cloud));
    for (auto& p : out->points) {
        p.x = maxX - p.x;   // 有效点与 -999 无效点一起翻转
    }
    m_session->replaceCurrentDianYun(out);
    // 数据已变：先清除旧孔覆盖层但不渲染旧云，再由 dianYunChanged 统一刷新一次新云。
    clearHoleView(false);
    emit dianYunChanged();
    xianShiXinXi(this, QStringLiteral("Y方向翻转"),
                 QStringLiteral("已完成 X 坐标范围反转（x' = maxX - x）：原最远点变为原点、最近点变为最远点；-999 无效点已一并翻转但仍按 -999 排除，旧识别结果已清除。"));
}

// ═══════════════════════════════════════════════════════
// 点云倾斜回正
//
// 功能: RANSAC拟合大平面 → 计算pitch/roll校正角 → 旋转变换使平面水平。
// 算法: RANSAC平面(阈值1.0mm, 500次) → 提取法向量 → 确保nz>0 → 计算α=atan2(ny,nz), β=atan2(-nx,vzAfterPitch) → 绕X后绕Y旋转(严格不含Z旋转)。
// 安全阈值: 倾斜角<0.05°跳过; 内点率<15%警告; 倾斜角>5°二次确认。
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：执行“onDianYunShuiPing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onDianYunShuiPing()
{
    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        xianShiJingGao(this, QStringLiteral("倾斜回正"), QStringLiteral("点云为空。"));
        return;
    }

    // RANSAC 拟合大平面
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.0f);
    seg.setMaxIterations(500);
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients coeffs;
    pcl::PointIndices inliers;
    seg.segment(inliers, coeffs);

    if (inliers.indices.empty()) {
        xianShiJingGao(this, QStringLiteral("倾斜回正"), QStringLiteral("未找到有效平面。请确保点云包含足够大的参考平面。"));
        return;
    }

    // 平面方程: a*x + b*y + c*z + d = 0，法向量 n_raw = (a, b, c)
    float a = coeffs.values[0], b_plane = coeffs.values[1], c_plane = coeffs.values[2];
    float nLen = std::sqrt(a*a + b_plane*b_plane + c_plane*c_plane);
    if (nLen < 1e-9f) {
        xianShiJingGao(this, QStringLiteral("倾斜回正"), QStringLiteral("平面法向量无效。"));
        return;
    }

    // 确保法向量指向上方 (nz > 0)
    float nx = a / nLen, ny = b_plane / nLen, nz = c_plane / nLen;
    if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; }

    // 平面内点比例
    float inlierRatio = (float)inliers.indices.size() / (float)cloud->size();

    // 安全阈值：内点比例过低则警告
    if (inlierRatio < 0.15f) {
        auto ans = QMessageBox::question(this, QStringLiteral("倾斜回正"),
            QStringLiteral("平面内点比例仅 %.1f%%，点云可能不包含足够大的参考平面。仍要继续吗？")
                .arg(inlierRatio * 100.0f, 0, 'f', 1));
        if (ans != QMessageBox::Yes) return;
    }

    // 总倾斜角 = acos(nz)，即法向量与Z轴夹角
    float tiltAngleDeg = std::acos(std::clamp(nz, -1.0f, 1.0f)) * 180.0f / 3.14159265f;

    // 倾斜角 < 0.05° 跳过
    if (tiltAngleDeg < 0.05f) {
        xianShiXinXi(this, QStringLiteral("倾斜回正"),
            QStringLiteral("点云已基本水平（倾斜角 %.3f° < 0.05°），无需回正。").arg(tiltAngleDeg, 0, 'f', 3));
        return;
    }

    // 倾斜角 > 5° 警告
    if (tiltAngleDeg > 5.0f) {
        auto ans = QMessageBox::question(this, QStringLiteral("倾斜回正"),
            QStringLiteral("检测到较大倾斜 %.2f°，可能影响后续孔识别。确认执行回正？").arg(tiltAngleDeg, 0, 'f', 2));
        if (ans != QMessageBox::Yes) return;
    }

    // 计算 pitch (绕X轴) 和 roll (绕Y轴) 校正角
    // 回正思路：求 α,β 使得 Ry(β)·Rx(α)·n = (0,0,1)
    float pitchRad = std::atan2(ny, nz);       // α
    float vzAfterPitch = ny * std::sin(pitchRad) + nz * std::cos(pitchRad);
    float rollRad = std::atan2(-nx, vzAfterPitch);  // β

    float pitchDeg = pitchRad * 180.0f / 3.14159265f;
    float rollDeg = rollRad * 180.0f / 3.14159265f;

    // 构建旋转矩阵: R = Ry(β) * Rx(α)，严格不含Z轴旋转
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitY()));
    tf.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));

    // 计算审计指标
    float d_plane = coeffs.values[3];
    float sumSqBefore = 0;
    std::vector<float> zValsBefore, zValsAfter;
    zValsBefore.reserve(inliers.indices.size());
    zValsAfter.reserve(inliers.indices.size());

    for (int idx : inliers.indices) {
        const auto& p = (*cloud)[idx];
        float dist = std::abs(a*p.x + b_plane*p.y + c_plane*p.z + d_plane) / nLen;
        sumSqBefore += dist * dist;
        zValsBefore.push_back(p.z);

        Eigen::Vector3f pt(p.x, p.y, p.z);
        Eigen::Vector3f pt2 = tf * pt;
        zValsAfter.push_back(pt2.z());
    }

    std::sort(zValsBefore.begin(), zValsBefore.end());
    float zMedianBefore = zValsBefore[zValsBefore.size() / 2];

    std::sort(zValsAfter.begin(), zValsAfter.end());
    float zMedianAfter = zValsAfter[zValsAfter.size() / 2];

    float rmseBefore = std::sqrt(sumSqBefore / inliers.indices.size());

    float meanZAfter = 0;
    for (float z : zValsAfter) meanZAfter += z;
    meanZAfter /= (float)zValsAfter.size();
    float sumSqZAfter = 0;
    for (float z : zValsAfter) { float dz = z - meanZAfter; sumSqZAfter += dz * dz; }
    float rmseAfter = std::sqrt(sumSqZAfter / (float)zValsAfter.size());

    // 应用变换到全部点云
    m_session->pushUndo();
    CloudPtr leveledCloud(new Cloud());
    pcl::transformPointCloud(*cloud, *leveledCloud, tf.matrix());

    m_session->replaceCurrentDianYun(leveledCloud);
    m_session->setRawDianYunOnLoad(leveledCloud);  // 回正后同步，确保onHoleShibie使用回正点云
    m_session->applyZGradient();
    // 回正只改变点云本体，dianYunChanged 已统一负责整云刷新与状态栏更新。
    emit dianYunChanged();

    // 审计日志

    m_cloudName += "_leveled";

    ui->statusbar->showMessage(
        QStringLiteral("倾斜回正完成：倾斜角=%.3f° pitch校正=%.3f° roll校正=%.3f° 内点数=%1 内点率=%.1f%%")
            .arg(tiltAngleDeg, 0, 'f', 3)
            .arg(pitchDeg, 0, 'f', 3)
            .arg(rollDeg, 0, 'f', 3)
            .arg((int)inliers.indices.size())
            .arg(inlierRatio * 100.0f, 0, 'f', 1),
        10000);

    // 询问是否保存
    auto saveAns = QMessageBox::question(this, QStringLiteral("倾斜回正"),
        QStringLiteral("回正完成。是否保存为 _leveled.pcd 文件？"));
    if (saveAns == QMessageBox::Yes) {
        QString suggestedName = QString::fromStdString(m_cloudName) + ".pcd";
        const QString fileName = QFileDialog::getSaveFileName(this,
            QStringLiteral("保存回正点云"), suggestedName, "*.pcd");
        if (!fileName.isEmpty()) {
            QString err;
            if (dianYunIO::savePcd(fileName, leveledCloud, &err, true)) {
                ui->statusbar->showMessage(QStringLiteral("已保存回正点云：%1").arg(fileName), 5000);
            } else {
                xianShiJingGao(this, QStringLiteral("保存失败"),
                    err.isEmpty() ? QStringLiteral("写入文件失败。") : err);
            }
        }
    }
}

// ── 孔信息面板（悬浮在三维视图下端，不再占右侧整列）──
/** 【函数导航】
 * 作用：执行“ensureHolePanel”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::ensureHolePanel()
{
    if (m_holePanel) return;

    m_holePanel = new QWidget(ui->centralwidget);
    m_holePanel->setObjectName(QStringLiteral("holeInfoPanel"));
    m_holePanel->setAttribute(Qt::WA_StyledBackground, true);
    QPalette holePal = m_holePanel->palette();
    holePal.setColor(QPalette::Window, QColor(251, 252, 254));
    m_holePanel->setPalette(holePal);
    m_holePanel->setAutoFillBackground(true);
    auto* panel = m_holePanel;
    panel->setMinimumSize(0, 0);
    panel->setStyleSheet(QStringLiteral(
        "QWidget{font-family:'Microsoft YaHei UI';}"
        "QWidget#holeInfoPanel{background-color:rgba(251,252,254,97%);border:1px solid #cbd5e1;"
        "border-radius:8px;}"
        "QGroupBox{font-weight:600;border:1px solid #d7dde5;border-radius:7px;"
        "margin-top:9px;padding-top:8px;background:#fbfcfe;}"
        "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;color:#34495e;}"
        "QSlider::groove:horizontal{height:4px;background:#dfe6ee;border-radius:2px;}"
        "QSlider::handle:horizontal{width:13px;margin:-5px 0;border-radius:6px;background:#3182ce;}"));
    auto* v = new QVBoxLayout(panel);
    v->setContentsMargins(8, 8, 8, 8);
    v->setSpacing(6);

    auto* header = new QHBoxLayout();
    header->setContentsMargins(0, 0, 0, 0);

    m_btnHolePrev = new QToolButton(panel);
    m_btnHolePrev->setArrowType(Qt::LeftArrow);
    m_btnHolePrev->setAutoRaise(true);
    connect(m_btnHolePrev, &QToolButton::clicked, this, &ZhuChuangKouWindow::onHolePrev);

    m_btnHoleNext = new QToolButton(panel);
    m_btnHoleNext->setArrowType(Qt::RightArrow);
    m_btnHoleNext->setAutoRaise(true);
    connect(m_btnHoleNext, &QToolButton::clicked, this, &ZhuChuangKouWindow::onHoleNext);

    m_holeLabelTitle = new QLabel(QStringLiteral("孔 -/-"), panel);
    m_holeLabelTitle->setAlignment(Qt::AlignCenter);
    QFont f = m_holeLabelTitle->font();
    f.setBold(true);
    m_holeLabelTitle->setFont(f);

    header->addWidget(m_btnHolePrev);
    header->addWidget(m_holeLabelTitle, 1);
    header->addWidget(m_btnHoleNext);

    m_holeLabelInfo = new QLabel(panel);
    m_holeLabelInfo->setWordWrap(true);
    m_holeLabelInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_holeLabelInfo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_holeLabelInfo->setMinimumWidth(0);
    m_holeLabelInfo->setMaximumWidth(420);
    m_holeLabelInfo->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_holeLabelInfo->setStyleSheet(QStringLiteral(
        "QLabel{color:#44546a;background:#f4f7fb;border:1px solid #e1e7ef;"
        "border-radius:6px;padding:7px;}"));

    m_holeParamGroup = new QGroupBox(QStringLiteral("识别结果"), panel);
    auto* holeGrid = new QGridLayout(m_holeParamGroup);
    holeGrid->setContentsMargins(6, 8, 6, 6);
    holeGrid->setHorizontalSpacing(10);
    holeGrid->setVerticalSpacing(5);
    holeGrid->setColumnStretch(0, 0);
    holeGrid->setColumnStretch(1, 1);

    auto makeFieldLabel = [panel](const QString& text) -> QLabel* {
        auto* lbl = new QLabel(text, panel);
        lbl->setStyleSheet(QStringLiteral("color: #7f8c8d; font-size: 12px;"));
        return lbl;
    };
    auto makeValueLabel = [panel](const QString& text) -> QLabel* {
        auto* lbl = new QLabel(text, panel);
        QFont f = lbl->font();
        f.setBold(true);
        lbl->setFont(f);
        lbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        lbl->setWordWrap(true);
        lbl->setMinimumWidth(0);
        lbl->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        lbl->setStyleSheet(QStringLiteral("font-size:12px;color:#1f2937;"));
        return lbl;
    };

    int row = 0;
    holeGrid->addWidget(makeFieldLabel(QStringLiteral("孔型")), row, 0, Qt::AlignLeft);
    m_lblHoleTypeVal = makeValueLabel(QStringLiteral("-"));
    holeGrid->addWidget(m_lblHoleTypeVal, row, 1, Qt::AlignLeft);
    ++row;

    // XYZ 坐标使用“标题一行 + 数值跨两列两行”的布局，避免右侧面板较窄时第三项 Z 被裁掉。
    m_lblCenterXKey = makeFieldLabel(QStringLiteral("有效上口中心 XYZ(mm)"));
    holeGrid->addWidget(m_lblCenterXKey, row, 0, 1, 2, Qt::AlignLeft | Qt::AlignTop);
    ++row;
    m_lblCenterXVal = makeValueLabel(QStringLiteral("X: -    Y: -\nZ: -"));
    m_lblCenterXVal->setMinimumHeight(34);
    holeGrid->addWidget(m_lblCenterXVal, row, 0, 1, 2, Qt::AlignLeft | Qt::AlignTop);
    ++row;

    // 下口中心与上口中心使用完全一致的 XYZ 展示语义；Z 独立到第二行保证始终可见。
    m_lblCenterBotKey = makeFieldLabel(QStringLiteral("有效下端中心 XYZ(mm)"));
    holeGrid->addWidget(m_lblCenterBotKey, row, 0, 1, 2, Qt::AlignLeft | Qt::AlignTop);
    ++row;
    m_lblCenterBotVal = makeValueLabel(QStringLiteral("X: -    Y: -\nZ: 未估计"));
    m_lblCenterBotVal->setMinimumHeight(34);
    holeGrid->addWidget(m_lblCenterBotVal, row, 0, 1, 2, Qt::AlignLeft | Qt::AlignTop);
    ++row;

    m_lblRadiusTopKey = makeFieldLabel(QStringLiteral("R 有效上口半径(mm)"));
    holeGrid->addWidget(m_lblRadiusTopKey, row, 0, Qt::AlignLeft);
    m_lblRadiusTopVal = makeValueLabel(QStringLiteral("-"));
    holeGrid->addWidget(m_lblRadiusTopVal, row, 1, Qt::AlignLeft);
    ++row;

    m_lblRadiusBotKey = makeFieldLabel(QStringLiteral("r 有效下口半径(mm)"));
    holeGrid->addWidget(m_lblRadiusBotKey, row, 0, Qt::AlignLeft);
    m_lblRadiusBotVal = makeValueLabel(QStringLiteral("-"));
    holeGrid->addWidget(m_lblRadiusBotVal, row, 1, Qt::AlignLeft);
    ++row;

    QLabel* depthKey = makeFieldLabel(QStringLiteral("沿孔轴有效深度(mm)"));
    depthKey->setToolTip(QStringLiteral("从有效上口沿最终入孔轴方向到可靠下端的识别深度；不是单独的 Z 轴高度。"));
    holeGrid->addWidget(depthKey, row, 0, Qt::AlignLeft);
    m_lblDepthVal = makeValueLabel(QStringLiteral("-"));
    m_lblDepthVal->setToolTip(QStringLiteral("depth：沿最终孔轴测得的有效深度。"));
    holeGrid->addWidget(m_lblDepthVal, row, 1, Qt::AlignLeft);
    ++row;

    m_lblAngleKey = makeFieldLabel(QStringLiteral("孔壁半角(°)"));
    m_lblAngleKey->setToolTip(QStringLiteral(
        "孔壁半角描述锥壁收缩角度，不是孔轴相对坐标系的倾角。孔轴倾角显示在入孔方向一行。"));
    holeGrid->addWidget(m_lblAngleKey, row, 0, Qt::AlignLeft);
    m_lblAngleVal = makeValueLabel(QStringLiteral("-"));
    holeGrid->addWidget(m_lblAngleVal, row, 1, Qt::AlignLeft);
    ++row;

    holeGrid->addWidget(makeFieldLabel(QStringLiteral("入孔方向(nx,ny,nz)")), row, 0, Qt::AlignLeft);
    m_lblNormalVal = makeValueLabel(QStringLiteral("-"));
    holeGrid->addWidget(m_lblNormalVal, row, 1, Qt::AlignLeft);
    ++row;

    m_holeParamGroup->setLayout(holeGrid);
    m_holeParamGroup->hide();

    auto* details = new QWidget(panel);
    details->setMinimumSize(0, 0);
    details->setMaximumWidth(430);
    details->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    auto* detailsLayout = new QVBoxLayout(details);
    detailsLayout->setContentsMargins(0, 0, 0, 0);
    detailsLayout->setSpacing(7);

    m_holePreviewGroup = new QGroupBox(QStringLiteral("孔局部形态 · 1:1 点云"), details);
    m_holePreviewGroup->setToolTip(QStringLiteral(
        "按识别上口半径自动截取约 8~15 mm 的局部半径范围；几何坐标不缩放，只由小窗口相机放大显示。左键拖动可独立旋转。"));
    auto* previewLayout = new QVBoxLayout(m_holePreviewGroup);
    previewLayout->setContentsMargins(6, 8, 6, 6);
    previewLayout->setSpacing(4);
    m_holePreviewWidget = new QVTKOpenGLNativeWidget(m_holePreviewGroup);
    m_holePreviewWidget->setMinimumHeight(205);
    m_holePreviewWidget->setMaximumHeight(255);
    m_holePreviewWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    previewLayout->addWidget(m_holePreviewWidget);
    m_lblHolePreviewInfo = new QLabel(QStringLiteral("选择孔后显示局部点云。"), m_holePreviewGroup);
    m_lblHolePreviewInfo->setWordWrap(true);
    m_lblHolePreviewInfo->setStyleSheet(QStringLiteral(
        "QLabel{color:#526477;font-size:11px;padding:3px 2px;}"));
    previewLayout->addWidget(m_lblHolePreviewInfo);
    m_holePreviewRenderer = std::make_unique<dianYunView::DianYunXuanRanQi>(m_holePreviewWidget);
    if (m_renderer) {
        double br = 0.0, bg = 0.0, bb = 0.0;
        m_renderer->getBackgroundColor(br, bg, bb);
        m_holePreviewRenderer->setBackgroundColor(br, bg, bb);
        DianYunViewOptions previewOpt = m_renderer->cloudVizOptions();
        previewOpt.basePointSize = std::max(2.5, previewOpt.basePointSize);
        m_holePreviewRenderer->setDianYunViewOptions(previewOpt);
        m_holePreviewRenderer->setHeightColorMap(m_renderer->heightColorMap());
        m_holePreviewRenderer->setScreenSpaceRotationEnabled(true);
    }

    detailsLayout->addWidget(m_holePreviewGroup);
    detailsLayout->addWidget(m_holeLabelInfo);
    detailsLayout->addWidget(m_holeParamGroup);

    // 孔参数列表的正式输出入口。按钮只负责触发导出，文件格式逻辑不放在界面代码中。
    m_btnExportHoleParameters = new QPushButton(QStringLiteral("导出孔参数"), details);
    m_btnExportHoleParameters->setToolTip(QStringLiteral(
        "把当前规范孔列表导出为 CSV 表格。每行一个孔；直孔没有可靠下口时，深度和下口字段写为“无法测量”。"));
    m_btnExportHoleParameters->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_btnExportHoleParameters->setEnabled(false);
    connect(m_btnExportHoleParameters, &QPushButton::clicked,
            this, &ZhuChuangKouWindow::onShuchuHoleCanshu);
    detailsLayout->addWidget(m_btnExportHoleParameters);

    detailsLayout->addStretch(1);

    auto* scroll = new QScrollArea(panel);
    scroll->setObjectName(QStringLiteral("holeInfoScroll"));
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setMinimumSize(0, 100);
    scroll->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
    scroll->setWidget(details);

    v->addLayout(header);
    v->addWidget(scroll, 1);

    panel->setLayout(v);
    // 右侧形态（与原来接近），但只占三维视图这一行的高度：
    // 把 viewStack 与面板放进一个水平布局，下面控件行/按钮行保持整行。
    m_holePanel->setMinimumWidth(330);
    m_holePanel->setMaximumWidth(440);
    m_holePanel->hide();
    if (auto* vl = qobject_cast<QVBoxLayout*>(ui->centralwidget->layout())) {
        auto* hbox = new QHBoxLayout();
        hbox->setSpacing(4);
        hbox->addWidget(ui->viewStack, 1);
        hbox->addWidget(m_holePanel, 0);
        vl->insertLayout(0, hbox, 6);
    }
}

/** 【函数导航】
 * 作用：清理/重置“clearHoleView”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::clearHoleView(bool refreshRenderer)
{
    m_lastHoleResult = HoleShibieResult{};
    m_normalDisplayCandidates.clear();
    m_normalDisplaySeedNumbers.clear();
    m_holeIndex = -1;
    if (m_btnExportHoleParameters) m_btnExportHoleParameters->setEnabled(false);
    clearHoleLocalPreview();
    // Hole 几何与 seed 标记分开管理：清孔结果时保留仍有效的 seed 圆环/编号。
    if (m_sdHoleSeed.empty()) {
        m_renderer->clearDieJiaDianYunNoRefresh();
        m_renderer->clearSeedNumberLabels();
    }
    m_renderer->clearOverlayLines();
    m_renderer->setBaseFadeNoRefresh(false);
    if (refreshRenderer) m_renderer->refreshDieJiaOnly();
    if (m_holePanel) m_holePanel->hide();
}

// ═══════════════════════════════════════════════════════
// 设置孔检测结果并刷新显示
//
// 功能: 将HoleShibieResult存储到m_lastHoleResult，刷新覆盖层和孔信息面板。
// 流程：保存识别结果 → 汇总规范法向 → 设置显示模式 → 渲染孔几何 → 刷新参数面板。
// 输出：刷新规范孔列表、三维覆盖层和孔参数面板；文件导出由独立按钮触发。
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：接收一次正式 Hole 识别结果，更新缓存、结果面板和可视化 overlay。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::setHoleShibieResult(HoleShibieResult res)
{
    ensureHolePanel();
    m_lastHoleResult = std::move(res);
    m_normalDisplayCandidates.clear();
    m_normalDisplaySeedNumbers.clear();
    m_holeIndex = -1;
    m_holePanel->show();

    if (!m_lastHoleResult.descriptors.empty()) {
        // 按统一显示策略构建规范孔列表。
        auto canonIdx = holeXianshi::selectGuiFanNormalXianshiHouXuan(m_lastHoleResult.descriptors);
        std::vector<bool> seedNumberUsed(m_sdHoleSeed.size(), false);
        for (int ci : canonIdx) {
            const HoleMiaoshu& hole = m_lastHoleResult.descriptors[ci];
            m_normalDisplayCandidates.push_back(hole);

            // 结果显示编号继续沿用“点击顺序”而不是重新按识别结果计数。
            // 若个别 seed 未识别成功，其它孔仍会显示原来的点击编号，避免用户把孔对应错。
            int seedNumber = static_cast<int>(m_normalDisplayCandidates.size());
            if (!m_sdHoleSeed.empty()) {
                const Eigen::Vector3f center = (hole.rTop > 0.0f) ? hole.centerTop : hole.center;
                int bestSeed = -1;
                float bestD2 = std::numeric_limits<float>::max();
                for (std::size_t si = 0; si < m_sdHoleSeed.size(); ++si) {
                    if (seedNumberUsed[si]) continue;
                    const float d2 = (m_sdHoleSeed[si] - center).squaredNorm();
                    if (d2 < bestD2) { bestD2 = d2; bestSeed = static_cast<int>(si); }
                }
                if (bestSeed >= 0) {
                    seedNumberUsed[(std::size_t)bestSeed] = true;
                    seedNumber = bestSeed + 1;
                }
            }
            m_normalDisplaySeedNumbers.push_back(seedNumber);
        }
        if (m_btnExportHoleParameters)
            m_btnExportHoleParameters->setEnabled(!m_normalDisplayCandidates.empty());

        // 优先切到最近点击编号对应的孔；找不到时才回退到总览。
        int preferredIndex = -1;
        if (m_preferredHoleSeedNumber > 0) {
            for (std::size_t i = 0; i < m_normalDisplaySeedNumbers.size(); ++i) {
                if (m_normalDisplaySeedNumbers[i] == m_preferredHoleSeedNumber) {
                    preferredIndex = static_cast<int>(i);
                    break;
                }
            }
        }
        if (preferredIndex >= 0) showHoleAt(preferredIndex);
        else showHoleAt(-1);
        return;
    }

    clearHoleLocalPreview();
    // 识别为 0 个孔时仍保留用户的 seed 圆环和连续编号，便于直接修正选点；这里只清孔几何线。
    m_renderer->clearOverlayLines();
    m_renderer->setBaseFadeNoRefresh(false);
    m_renderer->refreshDieJiaOnly();
    if (m_holeLabelTitle) m_holeLabelTitle->setText(QStringLiteral("孔 0 / 0"));
    if (m_holeParamGroup) m_holeParamGroup->hide();
    if (m_holeLabelInfo) {
        m_holeLabelInfo->show();
        const QString msg = !m_lastHoleResult.error.isEmpty()
            ? QStringLiteral("结果：0 个孔\n原因：%1").arg(m_lastHoleResult.error)
            : QStringLiteral("结果：0 个孔\n未通过当前阈值，请尝试放宽初筛参数或减弱预处理。");
        m_holeLabelInfo->setText(msg);
    }
    if (m_btnHolePrev) m_btnHolePrev->setEnabled(false);
    if (m_btnHoleNext) m_btnHoleNext->setEnabled(false);
}

// ═══════════════════════════════════════════════════════
// 孔信息面板填充
//
// 功能: 根据孔描述符填充右侧停靠面板的孔型、中心、半径、深度、坡角等字段。
// 输入: index(m_normalDisplayCandidates中的索引)
// 显示逻辑: 按 physicalHoleTypeDisplay 区分锥孔/直孔，并显示生产参数：上下口中心、R/r、深度、坡角和入孔方向。
// ═══════════════════════════════════════════════════════

/** 【函数导航】
 * 作用：显示“showHoleInfo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::showHoleInfo(int index)
{
    const int total = (int)m_normalDisplayCandidates.size();
    if (total == 0) return;
    index = std::clamp(index, 0, total - 1);
    const HoleMiaoshu& d = m_normalDisplayCandidates[(size_t)index];

    const int seedNumber = (index >= 0 && index < (int)m_normalDisplaySeedNumbers.size())
        ? m_normalDisplaySeedNumbers[(std::size_t)index]
        : (index + 1);
    if (m_holeLabelTitle) {
        m_holeLabelTitle->setText(QStringLiteral("孔 #%1 · 结果 %2 / %3")
            .arg(seedNumber).arg(index + 1).arg(total));
    }
    if (m_btnHolePrev) m_btnHolePrev->setEnabled(index > 0);
    if (m_btnHoleNext) m_btnHoleNext->setEnabled(index + 1 < total);
    if (m_holeLabelInfo) m_holeLabelInfo->hide();

    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    const bool isCone = (d.physicalHoleTypeDisplay == PHT::Cone);
    const bool isStraight = (d.physicalHoleTypeDisplay == PHT::Straight
                             || d.physicalHoleTypeDisplay == PHT::Straight_DeformedWall);
    // Row 0: 孔型 — 仅来源于 physicalHoleTypeDisplay
    if (m_lblHoleTypeVal) {
        QString typeText;
        QString style;
        switch (d.physicalHoleTypeDisplay) {
        case PHT::Cone:
            typeText = QStringLiteral("锥孔");
            style = QStringLiteral("color:#006600;font-weight:bold;");
            break;
        case PHT::Straight:
            typeText = QStringLiteral("直孔");
            style = QStringLiteral("color:#0044cc;");
            break;
        case PHT::Straight_DeformedWall:
            typeText = QStringLiteral("直孔（壁面变形）");
            style = QStringLiteral("color:#cc6600;");
            break;
        case PHT::ConeLike_LowConfidence:
            typeText = QStringLiteral("直孔（壁面变形）");
            style = QStringLiteral("color:#cc6600;");
            break;
        case PHT::Artifact:
            typeText = QStringLiteral("假孔特征");
            style = QStringLiteral("color:#8c8c8c;");
            break;
        default:
            typeText = QStringLiteral("未知");
            style = QString();
            break;
        }
        m_lblHoleTypeVal->setText(typeText);
        m_lblHoleTypeVal->setStyleSheet(style);
    }

    // 上/下口中心统一用 XYZ 三元组展示，避免把 Z 单独解释成模糊的“高度”。
    const Eigen::Vector3f cTop = (d.rTop > 0.0f) ? d.centerTop : d.center;
    auto relSuffix = [](const std::string& rel) -> QString {
        if (rel == "Low") return QStringLiteral(" (低置信)");
        if (rel == "Medium") return QStringLiteral(" (待确认)");
        return {};
    };
    if (m_lblCenterXKey) {
        m_lblCenterXKey->setText((isCone || isStraight)
            ? QStringLiteral("有效上口中心 XYZ(mm)")
            : QStringLiteral("中心 XYZ(mm)"));
    }
    if (m_lblCenterXVal) {
        m_lblCenterXVal->setText(QStringLiteral("X: %1    Y: %2\nZ: %3%4")
            .arg(cTop.x(), 0, 'f', 2)
            .arg(cTop.y(), 0, 'f', 2)
            .arg(cTop.z(), 0, 'f', 2)
            .arg(relSuffix(d.centerReliability)));
        m_lblCenterXVal->setToolTip(QStringLiteral("X, Y, Z 三维坐标；第三项就是上口 Z，不再另设含义不清的高度字段。"));
    }

    if (m_lblCenterBotKey) m_lblCenterBotKey->setText(QStringLiteral("有效下端中心 XYZ(mm)"));
    if (m_lblCenterBotVal) {
        const float dx = d.centerBot.x() - cTop.x();
        const float dy = d.centerBot.y() - cTop.y();
        const float dz = d.centerBot.z() - cTop.z();
        const float dist2 = dx*dx + dy*dy + dz*dz;
        if (d.centerBot.allFinite() && dist2 > 0.001f * 0.001f) {
            m_lblCenterBotVal->setText(QStringLiteral("X: %1    Y: %2\nZ: %3")
                .arg(d.centerBot.x(), 0, 'f', 2)
                .arg(d.centerBot.y(), 0, 'f', 2)
                .arg(d.centerBot.z(), 0, 'f', 2));
        } else {
            m_lblCenterBotVal->setText(QStringLiteral("X: -    Y: -\nZ: 未估计"));
        }
    }

    // Row 4: 半径 — 标签随孔型变化
    if (m_lblRadiusTopKey) {
        if (isCone || isStraight) m_lblRadiusTopKey->setText(QStringLiteral("R 有效上口半径(mm)"));
        else m_lblRadiusTopKey->setText(QStringLiteral("粗半径(mm)"));
    }
    if (m_lblRadiusTopVal) {
        const float rShow = (d.rTop > 0.0f) ? d.rTop : d.radius;
        m_lblRadiusTopVal->setText(QStringLiteral("%1").arg(rShow, 0, 'f', 2));
    }

    // Row 5: 下口半径（锥孔专属）
    if (m_lblRadiusBotKey) {
        m_lblRadiusBotKey->setVisible(isCone);
        if (isCone) m_lblRadiusBotKey->setText(QStringLiteral("r 有效下口半径(mm)"));
    }
    if (m_lblRadiusBotVal) {
        m_lblRadiusBotVal->setVisible(isCone);
        if (isCone) {
            float rBotShow = d.rBotProfile;
            if (rBotShow <= 0.0f && d.rBot > 0.0f && d.rBot < d.rTop * 0.98f)
                rBotShow = d.rBot;  // 主流水线实测下口
            if (rBotShow <= 0.0f && d.wallDownTrackValid && d.wallDownTrackDepthMm > 1.5f
                && d.wallDownCombinedClass == "CONE_WALL_TRACKED")
                rBotShow = d.rTop * 0.65f;  // 墙下锥孔估计
            if (rBotShow > 0.0f)
                m_lblRadiusBotVal->setText(QStringLiteral("%1").arg(rBotShow, 0, 'f', 2));
            else
                m_lblRadiusBotVal->setText(QStringLiteral("未估计"));
        }
    }

    // Row 6: 当前识别深度
    if (m_lblDepthVal) {
        if (d.depth > 0.0f)
            m_lblDepthVal->setText(QStringLiteral("%1").arg(d.depth, 0, 'f', 2));
        else
            m_lblDepthVal->setText(QStringLiteral("未估计"));
    }
    // 锥角（锥孔/低置信锥孔显示，直孔隐藏）
    const bool showConeAngle = isCone || (d.physicalHoleTypeDisplay == PHT::ConeLike_LowConfidence);
    if (m_lblAngleKey) m_lblAngleKey->setVisible(showConeAngle);
    if (m_lblAngleVal) {
        m_lblAngleVal->setVisible(showConeAngle);
        if (showConeAngle) {
            if (d.slopeDeg > 0.0f)
                m_lblAngleVal->setText(QStringLiteral("%1°").arg(d.slopeDeg, 0, 'f', 1));
            else if (d.radiusProfileN >= 2 && d.radiusProfileR2 >= 0.3f && d.profileBasedSlope > 0.0f)
                m_lblAngleVal->setText(QStringLiteral("%1°").arg(d.profileBasedSlope, 0, 'f', 1));
            else
                m_lblAngleVal->setText(QStringLiteral("未估计"));
        }
    }

    if (m_lblNormalVal) {
        Eigen::Vector3f axis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
        if (d.holeAxisInValid && axis.allFinite() && axis.norm() > 1e-6f) {
            axis.normalize();
            m_lblNormalVal->setText(QStringLiteral("%1, %2, %3  (%4°)")
                .arg(axis.x(), 0, 'f', 3)
                .arg(axis.y(), 0, 'f', 3)
                .arg(axis.z(), 0, 'f', 3)
                .arg(d.holeAxisTiltDeg, 0, 'f', 1));
        } else {
            m_lblNormalVal->setText(QStringLiteral("方向未知"));
        }
    }

    if (m_holeParamGroup) m_holeParamGroup->show();
    updateHoleLocalPreview(index);
}

// ═══════════════════════════════════════════════════════
// 孔导航: 选中并高亮指定序号的候选
//
// 功能: 在视图中高亮显示指定序号的孔(index<0则显示全部)。
// 输入: index(m_normalDisplayCandidates中的索引, -1=全显示)
// 算法: clamp序号 → 刷新覆盖层(setOverlayHoles) → 单孔高亮时降低底图透明度(0.12)。
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：显示“showHoleAt”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::showHoleAt(int index)
{
    if (m_lastHoleResult.descriptors.empty()) {
        clearHoleView();
        return;
    }

    // 使用与参数面板、导出一致的规范孔列表。
    const int n = (int)m_normalDisplayCandidates.size();
    auto currentBaseOpacity = [this]() {
        if (ui->sliderBaseOpacity) {
            return std::clamp(ui->sliderBaseOpacity->value() / 100.0, 0.02, 1.0);
        }
        return std::clamp(m_renderer->cloudVizOptions().baseOpacityFaded, 0.02, 1.0);
    };
    if (index < 0) {
        m_holeIndex = -1;
        showHoleInfo(0);
        m_renderer->setBaseFadeNoRefresh(true, currentBaseOpacity());
        holeDieJia::applyHoleDieJia(*m_renderer, m_normalDisplayCandidates, -1);
        if (m_holeLabelTitle) {
            m_holeLabelTitle->setText(QStringLiteral("有效孔：%1 个").arg(n));
        }
        if (m_holeLabelInfo) {
            m_holeLabelInfo->show();
            m_holeLabelInfo->setTextFormat(Qt::PlainText);
            m_holeLabelInfo->setText(QStringLiteral("识别完成\n有效孔：%1\n使用左右箭头逐孔查看。").arg(n));
        }
        if (m_btnHolePrev) m_btnHolePrev->setEnabled(true);
        if (m_btnHoleNext) m_btnHoleNext->setEnabled(true);
        return;
    }

    index = std::clamp(index, 0, n - 1);
    m_holeIndex = index;
    if (index >= 0 && index < (int)m_normalDisplaySeedNumbers.size())
        m_preferredHoleSeedNumber = m_normalDisplaySeedNumbers[(std::size_t)index];

    showHoleInfo(index);
    m_renderer->setBaseFadeNoRefresh(true, currentBaseOpacity());
        holeDieJia::applyHoleDieJia(*m_renderer, m_normalDisplayCandidates, m_holeIndex);
}

/** 【函数导航】
 * 作用：执行“onHolePrev”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onHolePrev()
{
    const int n = (int)m_normalDisplayCandidates.size();
    if (n <= 0) return;
    int start = (m_holeIndex < 0) ? n - 1 : m_holeIndex - 1;
    if (start >= 0) { showHoleAt(start); return; }
    showHoleAt(-1);
}

/** 【函数导航】
 * 作用：执行“onHoleNext”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onHoleNext()
{
    const int n = (int)m_normalDisplayCandidates.size();
    if (n <= 0) return;
    int start = (m_holeIndex < 0) ? 0 : m_holeIndex + 1;
    if (start < n) { showHoleAt(start); return; }
    showHoleAt(-1);
}

/** 【函数导航】
 * 作用：执行“onFirstShow”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onFirstShow()
{
    auto cloud = m_session->currentDianYunPtr();
    if (cloud && !cloud->empty()) {
        ui->viewStack->setCurrentWidget(ui->vtkWidget);
        // dianYunChanged 已连接到 refreshDianYunView()/updateStatusBar()，避免同一整云连续刷新两次。
        emit dianYunChanged();
        return;
    }

    if (auto* welcome = ui->viewStack->findChild<QWidget*>(
            QStringLiteral("welcomePage"), Qt::FindDirectChildrenOnly)) {
        ui->viewStack->setCurrentWidget(welcome);
    }
    ui->statusbar->showMessage(
        QStringLiteral("界面已就绪 — 点击“打开点云文件”开始。"), 8000);
}

// ═══════════════════════════════════════════════════════
// 文件操作: 加载点云
//
// 功能: 通过文件对话框加载PCD/PLY/CSV点云文件。
// 按扩展名分发。PCD/PLY 直接读取；CSV 自动判断 CSV0/CSV1，只有 CSV1 再询问 XY 网格间距。
// PCD嵌入孔: 加载后自动解析头注释中的嵌入孔参数(jiaZaiPcdHoleCanshu)。
// 云名提取: 从文件名提取cloudName(去掉_yrev后缀)。
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：响应“打开点云文件”操作：弹出文件选择框，按格式读取，并在成功后更新会话、seed、视图和状态栏。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onOpenDianYunFile()
{
    const QString filter = QStringLiteral("点云文件 (*.pcd *.ply *.csv);;PCD (*.pcd);;PLY (*.ply);;CSV (*.csv);;所有文件 (*.*)");
    const QString fileName = QFileDialog::getOpenFileName(
        this, QStringLiteral("打开点云文件"), dianYunOpenChuShiDirectory(), filter);
    if (fileName.isEmpty()) return;

    clearHoleView();

    const QString ext = QFileInfo(fileName).suffix().toLower();

    auto finishOk = [&](const QString& loadedPath) {
        // 新点云与上一份点云的手动选孔坐标没有任何语义关系。
        // 仅在新文件真正加载成功后自动丢弃旧 seed；打开失败/取消时不破坏当前点云的选择。
        if (m_autoHoleRecognitionTimer) m_autoHoleRecognitionTimer->stop();
        m_autoRecognitionPending = false;
        m_sdHoleSeed.clear();
        m_preferredHoleSeedNumber = -1;
        ++m_seedRevision;
        shuaXinSdHoleSeedBiaoJi();

        // 标题栏只显示当前文件名，不显示完整路径，便于同时处理多份扫描数据时确认当前点云。
        setWindowTitle(QStringLiteral("点云孔识别 - %1").arg(QFileInfo(loadedPath).fileName()));
        ui->viewStack->setCurrentWidget(ui->vtkWidget);
        m_session->applyZGradient();
        refreshDianYunView();
        updateGaoduColorBar(true);   // 新点云：恢复默认等高距调色板
        // 云已设置后再重置视角，确保初始视图完整显示整片点云
        m_renderer->resetCamera();
        updateStatusBar();
        emit dianYunChanged();
    };

    if (ext == QStringLiteral("pcd")) {
        auto r = dianYunIO::loadPcd(fileName);
        if (r.cloud && !r.cloud->empty()) {
            m_session->setLoadedDianYun(r.cloud);
            clearEmbeddedHoleData();
            {   QString base = QFileInfo(fileName).completeBaseName();
                int up = base.indexOf(QStringLiteral("_yrev"));
                if (up >= 0) base = base.left(up);
                m_cloudName = base.toStdString();
            }
            jiaZaiPcdHoleCanshu(fileName);
            finishOk(fileName);
            if (m_hasEmbeddedHoles && !m_embeddedHoleResult.descriptors.empty())
                setHoleShibieResult(m_embeddedHoleResult);
        } else {
            xianShiJingGao(this, QStringLiteral("打开点云文件"),
                QStringLiteral("打开失败：%1").arg(r.error.isEmpty() ? fileName : r.error));
        }
        return;
    }

    if (ext == QStringLiteral("ply")) {
        auto r = dianYunIO::loadPly(fileName);
        if (r.cloud && !r.cloud->empty()) {
            m_session->setLoadedDianYun(r.cloud);
            clearEmbeddedHoleData();
            QString base = QFileInfo(fileName).completeBaseName();
            int up = base.indexOf(QStringLiteral("_yrev"));
            if (up >= 0) base = base.left(up);
            m_cloudName = base.toStdString();
            finishOk(fileName);
        } else {
            xianShiJingGao(this, QStringLiteral("打开点云文件"),
                QStringLiteral("打开失败：%1").arg(r.error.isEmpty() ? fileName : r.error));
        }
        return;
    }

    if (ext == QStringLiteral("csv")) {
        const dianYunIO::CsvGeshiXinxi info = dianYunIO::inspectCsvFormat(fileName);
        if (!info.error.isEmpty()
            || info.format == dianYunIO::CsvDianYunFormat::Unknown) {
            xianShiJingGao(this, QStringLiteral("打开 CSV 点云"),
                info.error.isEmpty()
                    ? QStringLiteral("无法自动识别 CSV0/CSV1 格式。")
                    : info.error);
            return;
        }

        dianYunIO::CsvWangGeCanshu opt;
        if (info.format == dianYunIO::CsvDianYunFormat::Csv1HeightMatrix) {
            if (!xuanZeCsv1WangGeCanShu(this, opt)) return;
        }

        auto r = dianYunIO::loadCsv(fileName, opt, info.format);
        if (r.cloud && !r.cloud->empty()) {
            m_session->setLoadedDianYun(r.cloud);
            clearEmbeddedHoleData();
            {   QString base = QFileInfo(fileName).completeBaseName();
                int up = base.indexOf(QStringLiteral("_yrev"));
                if (up >= 0) base = base.left(up);
                m_cloudName = base.toStdString();
            }
            finishOk(fileName);
            ui->statusbar->showMessage(
                QStringLiteral("已按 %1 打开：%2 点。%3")
                    .arg(dianYunIO::csvFormatXianshiName(r.csvFormat))
                    .arg(static_cast<qulonglong>(r.cloud->size()))
                    .arg(info.detail), 6000);
        } else {
            xianShiJingGao(this, QStringLiteral("打开点云文件"),
                QStringLiteral("打开失败：%1").arg(r.error.isEmpty() ? fileName : r.error));
        }
        return;
    }

    xianShiXinXi(this, QStringLiteral("打开点云文件"), QStringLiteral("暂不支持该格式：%1").arg(ext));
}

/** 【函数导航】
 * 作用：执行“onSavePcd”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onSavePcd()
{
    const QString filter = QStringLiteral(
        "PCD 点云 (*.pcd);;"
        "CSV0（三列 X,Y,Z） (*.csv);;"
        "CSV1（二维 Z 高度矩阵） (*.csv)");
    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(
        this, QStringLiteral("保存点云"), QStringLiteral("."), filter, &selectedFilter);
    if (fileName.isEmpty()) return;

    /** 【类型导航注释】
     * SaveLeixing：主窗口流程编排中的自定义 枚举。
     * 主要使用位置：ZhuChuangKou_Window.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class SaveLeixing { Pcd, Csv0, Csv1 };
    SaveLeixing kind = SaveLeixing::Pcd;
    if (selectedFilter.contains(QStringLiteral("CSV1"), Qt::CaseInsensitive))
        kind = SaveLeixing::Csv1;
    else if (selectedFilter.contains(QStringLiteral("CSV0"), Qt::CaseInsensitive)
        || fileName.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive))
        kind = SaveLeixing::Csv0;

    if (kind == SaveLeixing::Pcd) {
        if (!fileName.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
            fileName += QStringLiteral(".pcd");
    } else {
        if (!fileName.endsWith(QStringLiteral(".csv"), Qt::CaseInsensitive))
            fileName += QStringLiteral(".csv");
    }

    auto cloud = m_session->currentDianYunPtr();
    if (!cloud || cloud->empty()) {
        xianShiJingGao(this, QStringLiteral("保存点云"), QStringLiteral("当前没有可保存的点云。"));
        return;
    }

    const bool saveWithHoles = m_actSaveWithHoles && m_actSaveWithHoles->isChecked()
        && (!m_lastHoleResult.descriptors.empty() || m_hasEmbeddedHoles);
    QString err;

    if (kind == SaveLeixing::Pcd && saveWithHoles) {
        std::vector<HoleMiaoshu> holes;
        if (!m_lastHoleResult.descriptors.empty()) holes = m_lastHoleResult.descriptors;
        else if (m_hasEmbeddedHoles) holes = m_embeddedHoleResult.descriptors;
        if (!holeIO::savePcdWithEmbeddedHoles(fileName, cloud, holes, &err)) {
            xianShiJingGao(this, QStringLiteral("保存点云"),
                err.isEmpty() ? QStringLiteral("保存失败，请检查路径/权限。") : err);
        }
        return;
    }

    if (kind != SaveLeixing::Pcd && saveWithHoles) {
        ui->statusbar->showMessage(
            QStringLiteral("提示：孔参数嵌入只支持 PCD；本次 CSV 仅保存点云坐标/高度。"), 6000);
    }

    bool ok = false;
    if (kind == SaveLeixing::Csv0) {
        ok = dianYunIO::saveCsv0(fileName, cloud, &err);
    } else if (kind == SaveLeixing::Csv1) {
        dianYunIO::CsvWangGeCanshu opt;
        if (!xuanZeCsv1WangGeCanShu(this, opt, QStringLiteral("保存 CSV1 高度矩阵"))) return;
        ok = dianYunIO::saveCsv1(fileName, cloud, opt, &err);
    } else {
        ok = dianYunIO::savePcd(fileName, cloud, &err, false);
    }

    if (!ok) {
        xianShiJingGao(this, QStringLiteral("保存点云"),
            err.isEmpty() ? QStringLiteral("保存失败，请检查路径/权限。") : err);
        return;
    }

    const QString fmt = kind == SaveLeixing::Csv0
        ? QStringLiteral("CSV0（三列 XYZ）")
        : (kind == SaveLeixing::Csv1 ? QStringLiteral("CSV1（高度矩阵）") : QStringLiteral("PCD"));
    ui->statusbar->showMessage(QStringLiteral("已保存 %1：%2").arg(fmt, fileName), 6000);
}

// ═══════════════════════════════════════════════════════
// 体素按钮入口：当前界面将该按钮映射为 SOR 降噪处理。
// 槽函数名保持 Qt UI 绑定不变，实际处理逻辑统一转到 onDianYunJiangZao()。
// ═══════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════
// 去除Z<-100低位点
// ═══════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════
// 统计离群点降噪
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：执行“onDianYunJiangZao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onDianYunJiangZao()
{
    const CloudPtr in = m_session->currentDianYunPtr();
    if (!in || in->empty()) return;

    // 统计离群点降噪固定使用 K=50，使不同扫描姿态保持一致的邻域统计口径。
    // 降噪放在线程池执行，保持界面响应；不改变 SOR 的点筛选规则。
    constexpr int meanK = 50;
    const double stddevMult = 1.0;
    ui->btnDenoise->setEnabled(false);
    ui->statusbar->showMessage(
        QStringLiteral("正在进行统计离群降噪… 点数=%1，近邻K=%2")
            .arg(static_cast<qulonglong>(in->size())).arg(meanK));

    QPointer<ZhuChuangKouWindow> self(this);
    QThreadPool::globalInstance()->start([self, in, meanK, stddevMult]() {
        CloudPtr out = tongJiLiQun(in, meanK, stddevMult, 0.0f);

        if (!self) return;
        QMetaObject::invokeMethod(self.data(), [self, in, out, meanK]() {
            if (!self) return;
            self->ui->btnDenoise->setEnabled(true);

            if (!out || out->empty()) {
                self->ui->statusbar->clearMessage();
                xianShiJingGao(self.data(), QStringLiteral("SOR降噪"), QStringLiteral("处理失败或结果为空。"));
                return;
            }

            // 用户在后台计算期间如果已经打开/修改了另一份点云，就丢弃旧结果，避免覆盖新数据。
            if (self->m_session->currentDianYunPtr().get() != in.get()) {
                self->ui->statusbar->showMessage(
                    QStringLiteral("降噪结果已丢弃：处理期间当前点云已经变化。"), 5000);
                return;
            }

            // 随后 dianYunChanged 会用降噪后的新云统一刷新；这里清覆盖层但不先渲染旧云。
            self->clearHoleView(false);
            self->m_session->pushUndo();
            self->m_session->replaceCurrentDianYun(out);

            // 渲染器已经直接使用 HeightZ + LUT 显示高度色。SOR 会保留原 RGB，
            // 因此这里不再重复执行整云 applyZGradient()，避免一次无必要的分位统计和全点颜色写入。
            // dianYunChanged 已统一负责 refreshDianYunView() + updateStatusBar()。
            // 这里不再手动重复刷新，避免 SOR 完成后整云 VTK 更新和 min/max 扫描各执行两遍。
            emit self->dianYunChanged();

            xianShiXinXi(self.data(), QStringLiteral("SOR降噪完成"),
                QStringLiteral("处理后剩余：%1\n统计近邻：K=%2")
                    .arg(dianShuWenBen(static_cast<qulonglong>(out->size())))
                    .arg(meanK));
        }, Qt::QueuedConnection);
    });
}

/** 【函数导航】
 * 作用：更新/刷新“updateStatusBar”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::updateStatusBar()
{
    static const QString kBuildStamp = QStringLiteral("");
    auto c = m_session->currentDianYunPtr();
    if (!c || c->empty()) {
        ui->labelStatus->setText(kBuildStamp + QStringLiteral(" 点数:---  X:---  Y:---  Z:---"));
        return;
    }
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*c, minPt, maxPt);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    // 状态栏优先完整显示 XYZ 范围；dX/dY/dZ 可由范围直接得到，删除重复长文本避免 Z 被窗口裁掉。
    oss << "点数:" << c->size()
        << "  X:[" << minPt[0] << "," << maxPt[0] << "]"
        << "  Y:[" << minPt[1] << "," << maxPt[1] << "]"
        << "  Z:[" << minPt[2] << "," << maxPt[2] << "]";
    ui->labelStatus->setText(QString::fromStdString(oss.str()));
}

// ═══════════════════════════════════════════════════════
// 孔检测入口
//
// 功能: 只对手动选孔点附近局部区域执行当前孔识别。
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：响应 Hole 识别操作：收集当前点云和手动 seed，提交正式识别流程并展示结果。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::onHoleShibie()
{
    auto c = m_session->currentDianYunPtr();
    if (!c || c->empty()) c = m_session->rawDianYunOnLoad();
    if (!c || c->empty()) {
        xianShiXinXi(this, QStringLiteral("手动孔识别"), QStringLiteral("点云为空。"));
        return;
    }

    if (m_sdHoleSeed.empty()) {
        xianShiXinXi(this, QStringLiteral("孔识别"),
            QStringLiteral("请先在点云中左键点击一个或多个孔位；程序会自动编号并识别。"));
        return;
    }
    if (m_holeRecognitionRunning) {
        m_autoRecognitionPending = true;
        return;
    }
    if (m_autoHoleRecognitionTimer && m_autoHoleRecognitionTimer->isActive())
        m_autoHoleRecognitionTimer->stop();

    const float maxHoleRadiusMm = ui->spinMaxHoleRadius
        ? static_cast<float>(ui->spinMaxHoleRadius->value()) : 10.0f;
    // 用户只调整一个“最大孔半径 Rmax”。粗搜索半径从默认 25 mm 起，Rmax>10 mm 后严格按 2*Rmax+5 mm 联动：
    // Rmax=10 -> 25 mm；Rmax=12 -> 29 mm。这样最大孔即使 seed 落在孔沿外侧，
    // 仍能覆盖孔的远侧边缘；Rmax<=10 mm 时保持既有 25 mm 默认搜索，不因减小参数而缩坏稳定案例。
    // 这里只改变搜索空间，不放宽法向、圆拟合、空腔或孔型质量阈值。
    const float linkedSearchRadiusMm = std::max(25.0f, 2.0f * maxHoleRadiusMm + 5.0f);

    std::vector<ShouDongHoleSeed> seeds;
    seeds.reserve(m_sdHoleSeed.size());
    for (const auto& p : m_sdHoleSeed) {
        ShouDongHoleSeed s;
        s.point = p;
        s.maxRadiusMm = maxHoleRadiusMm;
        s.searchRadiusMm = linkedSearchRadiusMm;
        seeds.push_back(s);
    }

    CloudPtr sdCloud = c;
    const std::string cloudName = m_cloudName;
    const quint64 seedRevision = m_seedRevision;
    runHoleShibieAsync([sdCloud, seeds, cloudName]() {
                             // 实时 GUI 直接由 HoleMiaoshu 构建 VTK overlay；旧 dianYunView/holeVis 点云没有显示消费者，
                             // 因此正常识别链不再额外生成一套历史可视化点云。
                             return shouDongJuBuHoleShibie(sdCloud, seeds, cloudName.c_str());
                         },
                         QStringLiteral("按当前选孔自动识别中..."),
                         QStringLiteral("孔识别完成"),
                         seedRevision,
                         m_sdHoleSeed);
}

// ═══════════════════════════════════════════════════════
// 异步孔检测执行
//
/*
异步孔识别调度。runFn 在 Qt 线程池执行，界面线程只负责禁用按钮、显示状态和接收最终 HoleShibieResult。
runningText 是运行提示，donePrefix 是完成提示前缀；任何 QWidget 更新都必须回到主线程执行。
*/
// ═══════════════════════════════════════════════════════
/** 【函数导航】
 * 作用：把耗时 Hole 识别任务提交到后台线程，并在 GUI 线程接收结果、更新生产界面。
 * 所属模块：主窗口流程编排。
 * 主要引用/调用位置：ZhuChuangKou_Window.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void ZhuChuangKouWindow::runHoleShibieAsync(const std::function<HoleShibieResult()>& runFn,
                                       const QString& runningText,
                                       const QString& donePrefix,
                                       quint64 seedRevision,
                                       std::vector<Eigen::Vector3f> seedSnapshot)
{
    m_holeRecognitionRunning = true;
    m_autoRecognitionPending = false;
    ui->btnHole->setEnabled(false);
    if (ui->btnDenoise) ui->btnDenoise->setEnabled(false);
    ui->actionUndo->setEnabled(false);
    ui->statusbar->showMessage(runningText);
    // 识别在后台运行，但不再设置全局 WaitCursor；主三维视图应始终保持可旋转、可继续点孔。
    QPointer<ZhuChuangKouWindow> self(this);
    QThreadPool* recognitionPool = m_holeRecognitionPool ? m_holeRecognitionPool : QThreadPool::globalInstance();
    recognitionPool->start([self, runFn, donePrefix, seedRevision, seedSnapshot = std::move(seedSnapshot)]() mutable {
        HoleShibieResult res;
        try {
            res = runFn ? runFn() : HoleShibieResult{};
        } catch (...) {
            res.error = QStringLiteral("孔识别发生未知错误。");
        }
        auto payload = std::make_shared<HoleShibieResult>(std::move(res));

        QMetaObject::invokeMethod(QApplication::instance(),
            [self, payload, donePrefix, seedRevision, seedSnapshot = std::move(seedSnapshot)]() mutable {
                if (!self) return;
                ZhuChuangKouWindow* window = self.data();
                const bool ok = payload->ok();
                const QString error = payload->error;
                const bool stale = (seedRevision != window->m_seedRevision);

                // 若用户只是在识别期间继续“往后追加”新孔，则本轮前缀结果仍然有效，可以先显示，
                // 不必因为 revision 变化把已经算好的前几个孔全部藏起来；删除/改序时仍严格丢弃旧结果。
                bool samePrefix = window->m_sdHoleSeed.size() >= seedSnapshot.size();
                if (samePrefix) {
                    for (std::size_t i = 0; i < seedSnapshot.size(); ++i) {
                        if ((window->m_sdHoleSeed[i] - seedSnapshot[i]).squaredNorm() > 1e-8f) {
                            samePrefix = false;
                            break;
                        }
                    }
                }
                const bool canShowPrefixResult = stale && samePrefix && !seedSnapshot.empty();
                window->m_holeRecognitionRunning = false;

                if (!stale || canShowPrefixResult) {
                    window->setHoleShibieResult(std::move(*payload));
                    if (!ok && !canShowPrefixResult)
                        xianShiXinXi(window, QStringLiteral("孔识别结果"), error);
                }

                window->ui->btnHole->setEnabled(true);
                if (window->ui->btnDenoise) window->ui->btnDenoise->setEnabled(true);

                if (canShowPrefixResult) {
                    window->ui->statusbar->showMessage(
                        QStringLiteral("已先显示前 %1 个选孔的结果；新增孔正在继续识别…")
                            .arg((int)seedSnapshot.size()), 2500);
                } else if (stale) {
                    window->ui->statusbar->showMessage(
                        QStringLiteral("选孔已删除或改序，旧结果已丢弃，正在按最新顺序重新识别…"), 3000);
                } else if (ok) {
                    const int nEff = static_cast<int>(window->m_normalDisplayCandidates.size());
                    window->ui->statusbar->showMessage(
                        QStringLiteral("%1：有效孔 %2 个").arg(donePrefix).arg(nEff), 8000);
                } else {
                    window->ui->statusbar->showMessage(
                        QStringLiteral("%1：0 个孔。%2")
                            .arg(donePrefix)
                            .arg(error.isEmpty() ? QStringLiteral("请检查 seed 是否落在目标孔附近或适当调整最大孔半径。") : error),
                        8000);
                }

                if ((stale || window->m_autoRecognitionPending) && !window->m_sdHoleSeed.empty()) {
                    window->m_autoRecognitionPending = false;
                    window->scheduleAutoHoleShibie();
                }
            }, Qt::QueuedConnection);
    });
}

// ============================================================================
// GUI 内部参数对话框实现
// ============================================================================
/*
模块职责：
通用界面弹窗模块。

主要调用位置：
由 ZhuChuangKouWindow 在需要用户确认或输入时调用。

维护说明：
弹窗只负责交互，不保存生产算法状态；输入值应由真正使用它的模块校验。
*/
