/*
================================================================================
文件：DianYunXianshi_View.cpp
模块：点云显示实现

【主要职责】
实现基础点云 actor、相机交互、点选、Hole overlay、红蓝 seed 标记与高度调色板。

【主要调用关系】
由 ZhuChuangKouWindow 通过 DianYunXuanRanQi 调用。

【线程与状态】
GUI 主线程；不得在识别 worker 直接操作 VTK。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：实现点云主场景、覆盖层、拾取、视图旋转和高度颜色映射。
主要调用位置：ZhuChuangKouWindow。所有函数只影响显示或交互，不参与孔识别判定。
维护说明：渲染器可以缓存 VTK 对象，但不得拥有识别语义；点大小、透明度、颜色等调整只能改变显示结果。
*/
#include "DianYunXianshi_View.h"
#include "DianYunLeixing_Types.h"
#include <QVTKOpenGLNativeWidget.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkAreaPicker.h>
#include <vtkCellPicker.h>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyle.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkColorTransferFunction.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkTextProperty.h>
#include <vtkFloatArray.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dianYunView {
namespace {

/** 【函数导航】
 * 作用：执行“dianZYYouXiao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool dianZYYouXiao(float z)
{
    return std::isfinite(z) && z > -998.0f;
}

/** 【函数导航】
 * 作用：构建“makeDefaultHeightMap”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
GaoduColorMapSettings makeDefaultHeightMap(double zMin, double zMax)
{
    GaoduColorMapSettings s;
    s.enabled = true;
    s.stopCount = 4;
    s.zMin = zMin;
    s.zMax = zMax;
    const double span = zMax - zMin;
    s.stops = {
        { zMin,                      0x2C / 255.0, 0x7B / 255.0, 0xB6 / 255.0, true  },
        { zMin + span / 3.0,         0x00 / 255.0, 0xA6 / 255.0, 0xCA / 255.0, false },
        { zMin + span * 2.0 / 3.0,   0xF6 / 255.0, 0xC3 / 255.0, 0x44 / 255.0, false },
        { zMax,                      0xD7 / 255.0, 0x30 / 255.0, 0x27 / 255.0, true  },
    };
    return s;
}

/** 【函数导航】
 * 作用：执行“cloudToPolyData”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
vtkSmartPointer<vtkPolyData> cloudToPolyData(const Cloud& cloud, float zOffset)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("rgb");
    vtkSmartPointer<vtkFloatArray> hz = vtkSmartPointer<vtkFloatArray>::New();
    hz->SetName("HeightZ");
    hz->SetNumberOfComponents(1);

    points->SetDataTypeToFloat();
    points->Allocate((vtkIdType)cloud.size());
    colors->Allocate((vtkIdType)cloud.size() * 3);
    hz->Allocate((vtkIdType)cloud.size());

    for (const auto& p : cloud) {
        points->InsertNextPoint(p.x, p.y, p.z + zOffset);
        colors->InsertNextTuple3(p.r, p.g, p.b);
        hz->InsertNextValue(p.z);
    }

    vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
    verts->AllocateExact((vtkIdType)cloud.size(), 1);
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud.size()); ++i) {
        verts->InsertNextCell(1, &i);
    }

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetVerts(verts);
    poly->GetPointData()->SetScalars(colors);
    poly->GetPointData()->SetActiveScalars("rgb");
    poly->GetPointData()->AddArray(hz);
    return poly;
}

/** 【函数导航】
 * 作用：执行“markerDianYunToLinePolyData”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
vtkSmartPointer<vtkPolyData> markerDianYunToLinePolyData(const Cloud& cloud, double radius)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("rgb");

    auto addPoint = [&](double x, double y, double z,
                        unsigned char r, unsigned char g, unsigned char b) -> vtkIdType {
        vtkIdType id = points->InsertNextPoint(x, y, z);
        colors->InsertNextTuple3(r, g, b);
        return id;
    };
    auto addSegment = [&](double x0, double y0, double z0,
                          double x1, double y1, double z1,
                          unsigned char r, unsigned char g, unsigned char b) {
        const vtkIdType a = addPoint(x0, y0, z0, r, g, b);
        const vtkIdType c = addPoint(x1, y1, z1, r, g, b);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(a);
        lines->InsertCellPoint(c);
    };

    // 手动选孔 seed 使用红/蓝各半的高对比双色标记。
    // 单个 VTK point 无法稳定显示“半红半蓝”，因此把原来的圆环/十字几何分成独立线段，
    // 每段两端使用同一颜色，避免 mapper 在线段内部插值成紫色。
    constexpr unsigned char redR = 255, redG = 0, redB = 0;
    constexpr unsigned char blueR = 0, blueG = 96, blueB = 255;
    const int circleN = 32;
    const double zLift = 0.02;
    for (const auto& p : cloud) {
        const double x = p.x, y = p.y, z = p.z + zLift;

        // 十字的正向半边为红色，负向半边为蓝色，任何视角下都能同时看到两种颜色。
        addSegment(x, y, z, x + radius, y, z, redR, redG, redB);
        addSegment(x - radius, y, z, x, y, z, blueR, blueG, blueB);
        addSegment(x, y, z, x, y + radius, z, redR, redG, redB);
        addSegment(x, y - radius, z, x, y, z, blueR, blueG, blueB);
        addSegment(x, y, z, x, y, z + radius * 0.55, redR, redG, redB);
        addSegment(x, y, z - radius * 0.55, x, y, z, blueR, blueG, blueB);

        // 圆环严格按两个半圆着色：0..π 红，π..2π 蓝。
        for (int i = 0; i < circleN; ++i) {
            const double a0 = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(circleN);
            const double a1 = 2.0 * M_PI * static_cast<double>(i + 1) / static_cast<double>(circleN);
            const bool redHalf = (i < circleN / 2);
            const unsigned char r = redHalf ? redR : blueR;
            const unsigned char g = redHalf ? redG : blueG;
            const unsigned char b = redHalf ? redB : blueB;
            addSegment(x + radius * std::cos(a0), y + radius * std::sin(a0), z,
                       x + radius * std::cos(a1), y + radius * std::sin(a1), z,
                       r, g, b);
        }
    }

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetLines(lines);
    poly->GetPointData()->SetScalars(colors);
    poly->GetPointData()->SetActiveScalars("rgb");
    return poly;
}

}

// 左键“点击/旋转”交互由交互样式自身处理：按下、移动、松开都在同一对象
// 内接收，不会因为 VTK GrabFocus 独占事件而丢失松开状态。
// 左键拖动 = 场景绕点击点旋转；中键平移/右键缩放/滚轮保持基类行为。
class XuanZhuanInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static XuanZhuanInteractorStyle* New();
    vtkTypeMacro(XuanZhuanInteractorStyle, vtkInteractorStyleTrackballCamera);

    /** 【函数导航】
     * 作用：应用/设置“SetRendererHost”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示实现。
     * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void SetRendererHost(DianYunXuanRanQi* host) { m_host = host; }

    /** 【函数导航】
     * 作用：执行“OnLeftButtonDown”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示实现。
     * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void OnLeftButtonDown() override
    {
        if (m_host) m_host->onLeftPress();
        // 不调用基类：避免进入 VTKIS_ROTATE 相机旋转
    }

    /** 【函数导航】
     * 作用：执行“OnMouseMove”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示实现。
     * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void OnMouseMove() override
    {
        if (m_host && m_host->m_rotating)
        {
            m_host->onMouseMove();
            return;
        }
        if (m_host && m_host->m_rightPressed) m_host->onRightMouseMove();
        this->Superclass::OnMouseMove(); // 中键平移/右键缩放等仍走基类
    }

    /** 【函数导航】
     * 作用：执行“OnLeftButtonUp”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示实现。
     * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    void OnLeftButtonUp() override
    {
        if (m_host) m_host->onLeftRelease();
        this->Superclass::OnLeftButtonUp();
    }

    void OnRightButtonDown() override
    {
        if (m_host) m_host->onRightPress();
        this->Superclass::OnRightButtonDown();
    }

    void OnRightButtonUp() override
    {
        this->Superclass::OnRightButtonUp();
        if (m_host) m_host->onRightRelease();
    }

private:
    DianYunXuanRanQi* m_host = nullptr;
};
vtkStandardNewMacro(XuanZhuanInteractorStyle);

DianYunXuanRanQi::DianYunXuanRanQi(QVTKOpenGLNativeWidget* widget)
    : m_w(widget)
{
    m_win = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_ren = vtkSmartPointer<vtkRenderer>::New();
    m_renOverlay = vtkSmartPointer<vtkRenderer>::New();

    m_win->SetNumberOfLayers(2);
    m_ren->SetLayer(0);
    m_ren->SetBackground(m_bgR, m_bgG, m_bgB);
    m_renOverlay->SetLayer(1);
    m_renOverlay->SetInteractive(0);
    m_renOverlay->SetBackgroundAlpha(0.0);
    m_renOverlay->SetBackground(m_bgR, m_bgG, m_bgB);
    m_renOverlay->SetErase(false);
    m_renOverlay->PreserveColorBufferOn();

    m_win->AddRenderer(m_ren);
    m_win->AddRenderer(m_renOverlay);
    m_w->setRenderWindow(m_win.Get());

    m_om = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_orientationAxesActor = vtkSmartPointer<vtkAxesActor>::New();
    m_om->SetOrientationMarker(m_orientationAxesActor);
    m_om->SetViewport(0.0, 0.0, 0.25, 0.25);
    m_om->SetInteractor(m_win->GetInteractor());
    m_om->EnabledOn();
    m_om->InteractiveOff();

    m_axesActor = vtkSmartPointer<vtkAxesActor>::New();
    m_axesActor->SetTotalLength(10.0, 10.0, 10.0);
    m_axesActor->SetShaftTypeToCylinder();
    m_axesActor->SetCylinderRadius(0.003);
    m_axesActor->SetConeRadius(0.08);
    m_axesActor->SetAxisLabels(0);

    m_areaPicker = vtkSmartPointer<vtkAreaPicker>::New();
    m_cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_cellPicker->SetTolerance(0.002);
    m_rightCellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_rightCellPicker->SetTolerance(0.004);
    if (auto iren = m_win->GetInteractor()) {
        iren->SetPicker(m_cellPicker);
    }

    m_styleTrackball = vtkSmartPointer<XuanZhuanInteractorStyle>::New();
    m_styleTrackball->SetRendererHost(this);
    m_styleTrackball->SetCurrentRenderer(m_ren);
    if (auto iren = m_win->GetInteractor()) {
        iren->SetInteractorStyle(m_styleTrackball);
    }

    m_viewRot = vtkSmartPointer<vtkTransform>::New();
    m_viewRot->Identity();
    m_viewRotAtPress = vtkSmartPointer<vtkTransform>::New();
    m_viewRotAtPress->Identity();

    m_heightLut = vtkSmartPointer<vtkColorTransferFunction>::New();
    m_heightLut->SetColorSpaceToLab();
    m_heightMap = makeDefaultHeightMap(0.0, 1.0);

    m_pickCbCmd = vtkSmartPointer<vtkCallbackCommand>::New();
    m_pickCbCmd->SetClientData(this);
    m_pickCbCmd->SetCallback([](vtkObject* caller, unsigned long, void* cd, void*) {
        auto self = static_cast<DianYunXuanRanQi*>(cd);
        auto pk = vtkCellPicker::SafeDownCast(caller);
        if (!self || !pk || !self->m_pickCb) return;
        if (pk->GetCellId() < 0 && pk->GetPointId() < 0) return;
        double p[3];
        pk->GetPickPosition(p);
        // 拾取坐标是经过场景旋转后的世界坐标，映射回原始点云坐标
        vtkSmartPointer<vtkMatrix4x4> inv = vtkSmartPointer<vtkMatrix4x4>::New();
        vtkSmartPointer<vtkMatrix4x4> rot = vtkSmartPointer<vtkMatrix4x4>::New();
        rot->DeepCopy(self->m_viewRot->GetMatrix());
        vtkMatrix4x4::Invert(rot, inv);
        double h[4] = { p[0], p[1], p[2], 1.0 };
        double r[4] = { 0.0, 0.0, 0.0, 0.0 };
        inv->MultiplyPoint(h, r);
        if (std::fabs(r[3]) > 1e-12) {
            self->m_pickCb(r[0] / r[3], r[1] / r[3], r[2] / r[3]);
        }
    });
    m_cellPicker->AddObserver(vtkCommand::EndPickEvent, m_pickCbCmd);

}

DianYunXuanRanQi::~DianYunXuanRanQi()
{
    if (m_win) {
        if (auto iren = m_win->GetInteractor()) {
            iren->SetInteractorStyle(nullptr);
            iren->SetPicker(nullptr);
        }
    }
    if (m_w) m_w->setRenderWindow(static_cast<vtkGenericOpenGLRenderWindow*>(nullptr));
    m_cloudActor = nullptr;
    m_overlayActor = nullptr;
    m_overlayLineActor = nullptr;
    m_overlayLines = nullptr;
    m_win = nullptr;
}

/** 【函数导航】
 * 作用：应用/设置“setDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setDianYun(const CloudConstPtr& cloud)
{
    m_cloud = cloud;
}

/** 【函数导航】
 * 作用：更新/刷新“refresh”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::refresh()
{
    if (!m_win || !m_ren) return;
    m_ren->RemoveAllViewProps();
    if (m_renOverlay) m_renOverlay->RemoveAllViewProps();
    m_cloudActor = nullptr;
    m_overlayActor = nullptr;
    m_overlayLineActor = nullptr;

    if (m_cloud && !m_cloud->empty()) {

        m_validZMin = std::numeric_limits<double>::max();
        m_validZMax = std::numeric_limits<double>::lowest();
        m_hasValidHeight = false;
        for (const auto& p : *m_cloud) {
            if (!dianZYYouXiao(p.z)) continue;
            double z = p.z;
            if (m_useProjectedHeightScalar) {
                const Eigen::Vector3f delta(
                    p.x - m_heightScalarOrigin.x(),
                    p.y - m_heightScalarOrigin.y(),
                    p.z - m_heightScalarOrigin.z());
                z = static_cast<double>(delta.dot(m_heightScalarAxis));
            }
            if (!std::isfinite(z)) continue;
            if (z < m_validZMin) m_validZMin = z;
            if (z > m_validZMax) m_validZMax = z;
            m_hasValidHeight = true;
        }
        if (!m_hasValidHeight) { m_validZMin = 0.0; m_validZMax = 1.0; }

        auto poly = cloudToPolyData(*m_cloud, 0.0f);
        if (m_useProjectedHeightScalar) {
            if (auto* hz = vtkFloatArray::SafeDownCast(poly->GetPointData()->GetArray("HeightZ"))) {
                vtkIdType hi = 0;
                for (const auto& p : *m_cloud) {
                    const Eigen::Vector3f delta(
                        p.x - m_heightScalarOrigin.x(),
                        p.y - m_heightScalarOrigin.y(),
                        p.z - m_heightScalarOrigin.z());
                    hz->SetValue(hi++, delta.dot(m_heightScalarAxis));
                }
            }
        }
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(poly);

        m_cloudActor = vtkSmartPointer<vtkActor>::New();
        m_cloudActor->SetMapper(mapper);
        m_cloudActor->PickableOn();
        m_cloudActor->GetProperty()->SetRepresentationToPoints();
        m_cloudActor->GetProperty()->SetPointSize((float)std::clamp(m_opt.basePointSize, 1.0, 20.0));

        if (m_heightMap.enabled && m_hasValidHeight) {

            rebuildHeightLut();
            mapper->ScalarVisibilityOn();
            mapper->SelectColorArray("HeightZ");
            mapper->SetScalarModeToUsePointFieldData();
            mapper->SetScalarRange(m_validZMin, m_validZMax);
            mapper->SetLookupTable(m_heightLut);
            mapper->SetUseLookupTableScalarRange(true);
            m_cloudActor->GetProperty()->SetOpacity(std::clamp(m_baseOpacity, 0.02, 1.0));
        } else {

            mapper->ScalarVisibilityOn();
            mapper->SetScalarModeToUsePointData();
            mapper->SetColorModeToDirectScalars();
            m_cloudActor->GetProperty()->SetOpacity(std::clamp(m_baseOpacity, 0.02, 1.0));
        }
        m_ren->AddActor(m_cloudActor);

        double bounds[6];
        poly->GetBounds(bounds);
        const double dx = bounds[1] - bounds[0];
        const double dy = bounds[3] - bounds[2];
        const double dz = bounds[5] - bounds[4];
        const double span = std::max({ dx, dy, dz });
        const double len = span > 0 ? span * 0.14 : 10.0;
        m_axesActor->SetTotalLength(len, len, len);

        const double cx = 0.5 * (bounds[0] + bounds[1]);
        const double cy = 0.5 * (bounds[2] + bounds[3]);
        const double cz = 0.5 * (bounds[4] + bounds[5]);
        m_cloudCenter[0] = cx;
        m_cloudCenter[1] = cy;
        m_cloudCenter[2] = cz;
        m_axesActor->SetPosition(cx, cy, cz);
        m_ren->AddActor(m_axesActor);

        // 不自动回归中心：保持用户当前视角（仅打开新点云时由 resetCamera() 重置）。
        // 裁剪范围在 applyViewRotationToActors() 之后按“变换后的可见 actor bounds”统一重算。
        // 不能使用这里旋转前的 poly bounds，否则薄板转到侧视时近端/远端会同时被裁掉。
        if (m_styleTrackball) {
            m_styleTrackball->SetCurrentRenderer(m_ren);
        }
    }

    if (m_overlayCloud && !m_overlayCloud->empty() && m_renOverlay) {
        auto polyO = markerDianYunToLinePolyData(*m_overlayCloud, 1.8);
        auto mapperO = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapperO->SetInputData(polyO);

        m_overlayActor = vtkSmartPointer<vtkActor>::New();
        m_overlayActor->SetMapper(mapperO);
        m_overlayActor->PickableOff();
        m_overlayActor->GetProperty()->SetRepresentationToWireframe();
        m_overlayActor->GetProperty()->SetOpacity(std::clamp(m_opt.overlayOpacity, 0.02, 1.0));
        m_overlayActor->GetProperty()->SetLineWidth(6.0f);
        m_overlayActor->GetProperty()->LightingOff();
        m_overlayActor->GetProperty()->SetAmbient(1.0);
        m_overlayActor->GetProperty()->SetDiffuse(0.0);
        if (m_opt.overlayForceColor) {
            mapperO->ScalarVisibilityOff();
            m_overlayActor->GetProperty()->SetColor(
                std::clamp(m_opt.overlayColorR, 0.0, 1.0),
                std::clamp(m_opt.overlayColorG, 0.0, 1.0),
                std::clamp(m_opt.overlayColorB, 0.0, 1.0));
        } else {
            mapperO->ScalarVisibilityOn();
        }
        if (m_ren->GetActiveCamera()) {
            m_renOverlay->SetActiveCamera(m_ren->GetActiveCamera());
        }
        m_renOverlay->AddActor(m_overlayActor);
    }

    if (m_hasLineOverlay && m_overlayLines && m_renOverlay
        && m_overlayLines->GetNumberOfPoints() > 0) {
        auto mapperL = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapperL->SetInputData(m_overlayLines);
        if (m_lineForceColor) mapperL->ScalarVisibilityOff();
        else mapperL->ScalarVisibilityOn();

        m_overlayLineActor = vtkSmartPointer<vtkActor>::New();
        m_overlayLineActor->SetMapper(mapperL);
        m_overlayLineActor->PickableOff();
        m_overlayLineActor->GetProperty()->SetRepresentationToWireframe();
        m_overlayLineActor->GetProperty()->SetLineWidth((float)m_lineWidth);
        m_overlayLineActor->GetProperty()->LightingOff();
        m_overlayLineActor->GetProperty()->SetAmbient(1.0);
        m_overlayLineActor->GetProperty()->SetDiffuse(0.0);
        if (m_lineForceColor) {
            m_overlayLineActor->GetProperty()->SetColor(m_lineR, m_lineG, m_lineB);
        }
        m_overlayLineActor->GetProperty()->SetOpacity(std::clamp(m_lineOpacity, 0.02, 1.0));
        if (m_ren->GetActiveCamera()) {
            m_renOverlay->SetActiveCamera(m_ren->GetActiveCamera());
        }
        m_renOverlay->AddActor(m_overlayLineActor);
    }

    if (!m_hasEverResetCamera && m_cloud && !m_cloud->empty()) {
        m_ren->ResetCamera();
        m_hasEverResetCamera = true;
    }

    rebuildSeedNumberLabels();
    applyViewRotationToActors();
    renderWindowRender();
}

/** 【函数导航】
 * 作用：更新/刷新“refreshDieJiaOnly”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::refreshDieJiaOnly()
{
    if (!m_win || !m_renOverlay) return;
    // 主点云没有变化：只清理/重建 overlay renderer，保留 base actor、mapper、polydata、Z 范围和坐标轴。
    m_renOverlay->RemoveAllViewProps();
    m_overlayActor = nullptr;
    m_overlayLineActor = nullptr;

    if (m_overlayCloud && !m_overlayCloud->empty()) {
        auto polyO = markerDianYunToLinePolyData(*m_overlayCloud, 1.8);
        auto mapperO = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapperO->SetInputData(polyO);
        m_overlayActor = vtkSmartPointer<vtkActor>::New();
        m_overlayActor->SetMapper(mapperO);
        m_overlayActor->PickableOff();
        m_overlayActor->GetProperty()->SetRepresentationToWireframe();
        m_overlayActor->GetProperty()->SetOpacity(std::clamp(m_opt.overlayOpacity, 0.02, 1.0));
        m_overlayActor->GetProperty()->SetLineWidth(6.0f);
        m_overlayActor->GetProperty()->LightingOff();
        m_overlayActor->GetProperty()->SetAmbient(1.0);
        m_overlayActor->GetProperty()->SetDiffuse(0.0);
        if (m_opt.overlayForceColor) {
            mapperO->ScalarVisibilityOff();
            m_overlayActor->GetProperty()->SetColor(
                std::clamp(m_opt.overlayColorR, 0.0, 1.0),
                std::clamp(m_opt.overlayColorG, 0.0, 1.0),
                std::clamp(m_opt.overlayColorB, 0.0, 1.0));
        } else {
            mapperO->ScalarVisibilityOn();
        }
        if (m_ren->GetActiveCamera()) m_renOverlay->SetActiveCamera(m_ren->GetActiveCamera());
        m_renOverlay->AddActor(m_overlayActor);
    }

    if (m_hasLineOverlay && m_overlayLines && m_overlayLines->GetNumberOfPoints() > 0) {
        auto mapperL = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapperL->SetInputData(m_overlayLines);
        if (m_lineForceColor) mapperL->ScalarVisibilityOff();
        else mapperL->ScalarVisibilityOn();
        m_overlayLineActor = vtkSmartPointer<vtkActor>::New();
        m_overlayLineActor->SetMapper(mapperL);
        m_overlayLineActor->PickableOff();
        m_overlayLineActor->GetProperty()->SetRepresentationToWireframe();
        m_overlayLineActor->GetProperty()->SetLineWidth(static_cast<float>(m_lineWidth));
        m_overlayLineActor->GetProperty()->LightingOff();
        m_overlayLineActor->GetProperty()->SetAmbient(1.0);
        m_overlayLineActor->GetProperty()->SetDiffuse(0.0);
        if (m_lineForceColor) m_overlayLineActor->GetProperty()->SetColor(m_lineR, m_lineG, m_lineB);
        m_overlayLineActor->GetProperty()->SetOpacity(std::clamp(m_lineOpacity, 0.02, 1.0));
        if (m_ren->GetActiveCamera()) m_renOverlay->SetActiveCamera(m_ren->GetActiveCamera());
        m_renOverlay->AddActor(m_overlayLineActor);
    }

    rebuildSeedNumberLabels();
    applyViewRotationToActors();
    renderWindowRender();
}

/** 【函数导航】
 * 作用：清理/重置“resetCamera”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::resetCamera()
{
    if (!m_ren || !m_win) return;
    // 打开新点云时：清除场景旋转并回到默认视角
    if (m_viewRot) m_viewRot->Identity();
    applyViewRotationToActors();
    if (m_cloud && !m_cloud->empty()) m_ren->ResetCamera();
    m_hasEverResetCamera = true;
    renderWindowRender();
}

void DianYunXuanRanQi::setHoleInspectionView(const Eigen::Vector3f& centerTop,
                                              const Eigen::Vector3f& inwardAxis,
                                              double tiltDeg)
{
    if (!m_ren || !m_win || !m_cloud || m_cloud->empty()) return;

    Eigen::Vector3d axis = inwardAxis.cast<double>();
    if (!axis.allFinite() || axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitZ();
    axis.normalize();

    // 局部坐标：Z=入孔轴，X=全局 X 在孔轴正交平面上的投影；默认展示的是孔的局部 XZ 剖视。
    Eigen::Vector3d localX = Eigen::Vector3d::UnitX()
        - axis * axis.dot(Eigen::Vector3d::UnitX());
    if (localX.norm() < 1e-6) {
        localX = Eigen::Vector3d::UnitY()
            - axis * axis.dot(Eigen::Vector3d::UnitY());
    }
    if (localX.norm() < 1e-6) localX = axis.unitOrthogonal();
    localX.normalize();
    Eigen::Vector3d localY = axis.cross(localX);
    if (localY.norm() < 1e-9) localY = axis.unitOrthogonal();
    localY.normalize();

    const double kPi = 3.14159265358979323846;
    const double tilt = std::clamp(tiltDeg, -60.0, 60.0) * kPi / 180.0;
    // 先正视局部 XZ 平面（沿局部 Y 方向观察），再绕局部 X 轴向观察者方向轻微倾斜。
    const Eigen::Vector3d baseViewDirection = (-localY).normalized();
    const Eigen::AngleAxisd tiltRotation(-tilt, localX);
    const Eigen::Vector3d viewDirection = (tiltRotation * baseViewDirection).normalized();
    // 屏幕向上取“孔外方向”，使入孔轴/孔深默认朝屏幕下方。
    // 这等价于在既有正确 XZ 视角上做一次 180° roll，修正此前整体上下颠倒。
    Eigen::Vector3d viewUp = -(tiltRotation * axis).normalized();
    Eigen::Vector3d viewRight = viewDirection.cross(viewUp);
    if (!viewRight.allFinite() || viewRight.norm() < 1e-9) viewRight = localX;
    else viewRight.normalize();
    viewUp = viewRight.cross(viewDirection).normalized();

    // 切换孔时先清除上一个孔留下的手动旋转；点云、孔线框、坐标轴随后仍共用同一变换。
    if (m_viewRot) m_viewRot->Identity();
    applyViewRotationToActors();

    vtkCamera* camera = m_ren->GetActiveCamera();
    if (!camera) return;
    const Eigen::Vector3d topCenter = centerTop.allFinite()
        ? centerTop.cast<double>() : Eigen::Vector3d(m_cloudCenter[0], m_cloudCenter[1], m_cloudCenter[2]);

    // 以“上口中心”为基准，同时把关注中心沿孔轴轻微下移，让孔口在屏幕里略偏上，减少上方空旷感。
    double inwardMax = 0.0;
    double minU = std::numeric_limits<double>::max();
    double maxU = -std::numeric_limits<double>::max();
    double minV = std::numeric_limits<double>::max();
    double maxV = -std::numeric_limits<double>::max();
    double minW = std::numeric_limits<double>::max();
    double maxW = -std::numeric_limits<double>::max();
    for (const auto& p : *m_cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d pw((double)p.x, (double)p.y, (double)p.z);
        const Eigen::Vector3d d = pw - topCenter;
        inwardMax = std::max(inwardMax, d.dot(axis));
    }
    const double inwardBias = std::clamp(inwardMax * 0.18, 0.0, 3.0);
    const Eigen::Vector3d focusCenter = topCenter + axis * inwardBias;

    for (const auto& p : *m_cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d pw((double)p.x, (double)p.y, (double)p.z);
        const Eigen::Vector3d d = pw - focusCenter;
        const double u = d.dot(viewRight);
        const double v = d.dot(viewUp);
        const double w = d.dot(viewDirection);
        minU = std::min(minU, u); maxU = std::max(maxU, u);
        minV = std::min(minV, v); maxV = std::max(maxV, v);
        minW = std::min(minW, w); maxW = std::max(maxW, w);
    }

    if (!std::isfinite(minU) || !std::isfinite(maxU) || !std::isfinite(minV) || !std::isfinite(maxV)) return;

    const int* winSize = m_win->GetSize();
    const double aspect = (winSize && winSize[1] > 0)
        ? std::max(0.25, (double)winSize[0] / (double)winSize[1])
        : 1.0;
    const double halfU = std::max(std::fabs(minU), std::fabs(maxU));
    const double halfV = std::max(std::fabs(minV), std::fabs(maxV));
    const double halfW = std::max(std::fabs(minW), std::fabs(maxW));
    const double parallelScale = std::max(2.0, 1.10 * std::max(halfV, halfU / aspect));
    const double cameraDistance = std::max(80.0, 30.0 + halfW + parallelScale * 3.0);
    const Eigen::Vector3d cameraPos = focusCenter - viewDirection * cameraDistance;

    camera->SetFocalPoint(focusCenter.x(), focusCenter.y(), focusCenter.z());
    camera->SetPosition(cameraPos.x(), cameraPos.y(), cameraPos.z());
    camera->SetViewUp(viewUp.x(), viewUp.y(), viewUp.z());
    camera->OrthogonalizeViewUp();
    camera->ParallelProjectionOn();
    camera->SetParallelScale(parallelScale);
    camera->SetClippingRange(std::max(0.1, cameraDistance - halfW - 80.0), cameraDistance + halfW + 80.0);

    m_ren->ResetCameraClippingRange();
    m_hasEverResetCamera = true;
    renderWindowRender();
}

/** 【函数导航】
 * 作用：应用/设置“applyViewRotationToActors”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::applyViewRotationToActors()
{
    if (!m_viewRot) return;
    if (m_cloudActor) m_cloudActor->SetUserTransform(m_viewRot);
    if (m_axesActor) m_axesActor->SetUserTransform(m_viewRot);
    if (m_overlayActor) m_overlayActor->SetUserTransform(m_viewRot);
    if (m_overlayLineActor) m_overlayLineActor->SetUserTransform(m_viewRot);
    // vtkBillboardTextActor3D 会自己朝向相机，UserTransform 对它的位置更新并不可靠。
    // 因此编号位置直接由原始 seed 锚点经过当前场景矩阵重新计算；文字仍保持正对屏幕，
    // 但会和孔/seed 一起移动，不再轻微旋转后停在旧屏幕位置。
    vtkMatrix4x4* viewMat = m_viewRot->GetMatrix();
    const std::size_t labelCount = std::min(m_seedLabelActors.size(), m_seedLabelPositions.size());
    for (std::size_t i = 0; i < labelCount; ++i) {
        auto& labelActor = m_seedLabelActors[i];
        if (!labelActor || !viewMat) continue;
        const Eigen::Vector3f& seed = m_seedLabelPositions[i];
        double h0[4] = { seed.x(), seed.y(), seed.z() + 2.2, 1.0 };
        double h1[4] = { 0.0, 0.0, 0.0, 0.0 };
        viewMat->MultiplyPoint(h0, h1);
        if (std::fabs(h1[3]) > 1e-12)
            labelActor->SetPosition(h1[0] / h1[3], h1[1] / h1[3], h1[2] / h1[3]);
        labelActor->SetUserTransform(nullptr);
    }

    // OrientationMarkerWidget 本身只跟随主相机；本程序的拖拽是“相机不动、actor 旋转”，
    // 所以需要把场景旋转的 3x3 部分同步给左下角方向坐标轴。不要复制绕 pivot 产生的平移。
    if (m_orientationAxesActor) {
        vtkSmartPointer<vtkMatrix4x4> omMat = vtkSmartPointer<vtkMatrix4x4>::New();
        omMat->Identity();
        vtkMatrix4x4* src = m_viewRot->GetMatrix();
        if (src) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    omMat->SetElement(r, c, src->GetElement(r, c));
        }
        vtkSmartPointer<vtkTransform> omRot = vtkSmartPointer<vtkTransform>::New();
        omRot->SetMatrix(omMat);
        m_orientationAxesActor->SetUserTransform(omRot);
    }

    // VTK 的裁剪平面必须跟随当前可见 actor 的变换后边界。
    // 这里不改变相机位置/焦点，只更新 near/far clipping range。
    if (m_ren) m_ren->ResetCameraClippingRange();
}

/** 【函数导航】
 * 作用：执行“onLeftPress”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::onLeftPress()
{
    auto iren = m_win ? m_win->GetInteractor() : nullptr;
    if (!iren || !m_ren) return;
    int* pos = iren->GetEventPosition();

    // 旋转支点 = 当前画面中心射线上、位于点云空间中心深度处的点。
    // 这样既满足“绕当前窗口中心旋转”，又不会把支点定到焦点平面深处
    // （画面中心可能没有点，投影可能落到很远甚至无穷远）。
    bool hasPivot = (m_cloud && !m_cloud->empty());
    if (hasPivot) {
        double pivot[3] = { m_cloudCenter[0], m_cloudCenter[1], m_cloudCenter[2] };
        if (vtkCamera* cam = m_ren->GetActiveCamera()) {
            double pos[3], fp[3];
            cam->GetPosition(pos);
            cam->GetFocalPoint(fp);
            double d[3] = { fp[0] - pos[0], fp[1] - pos[1], fp[2] - pos[2] };
            const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (len > 1e-12) {
                d[0] /= len; d[1] /= len; d[2] /= len;
                const double t = (m_cloudCenter[0] - pos[0]) * d[0]
                               + (m_cloudCenter[1] - pos[1]) * d[1]
                               + (m_cloudCenter[2] - pos[2]) * d[2];
                pivot[0] = pos[0] + t * d[0];
                pivot[1] = pos[1] + t * d[1];
                pivot[2] = pos[2] + t * d[2];
            }
        }
        m_rotatePivot[0] = pivot[0];
        m_rotatePivot[1] = pivot[1];
        m_rotatePivot[2] = pivot[2];
    } else if (vtkCamera* cam = m_ren->GetActiveCamera()) {
        double fp[3];
        cam->GetFocalPoint(fp);
        m_rotatePivot[0] = fp[0];
        m_rotatePivot[1] = fp[1];
        m_rotatePivot[2] = fp[2];
    }
    m_rotatePivotValid = hasPivot;
    m_pressX = pos[0];
    m_pressY = pos[1];
    m_rotateAccumDeg = 0;
    m_rotateLastX = pos[0];
    m_rotateLastY = pos[1];
    m_viewRotAtPress->DeepCopy(m_viewRot);
    m_mousePressed = true;
    m_rotating = true;
}

/** 【函数导航】
 * 作用：执行“onMouseMove”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::onMouseMove()
{
    if (!m_rotating || !m_rotatePivotValid) return;
    auto iren = m_win ? m_win->GetInteractor() : nullptr;
    if (!iren) return;
    int* pos = iren->GetEventPosition();
    const int dx = pos[0] - m_rotateLastX;
    const int dy = pos[1] - m_rotateLastY;
    m_rotateLastX = pos[0];
    m_rotateLastY = pos[1];
    if (dx == 0 && dy == 0) return;

    const double yawDeg = dx * 0.3;
    const double pitchDeg = -dy * 0.3;
    m_rotateAccumDeg += std::fabs(yawDeg) + std::fabs(pitchDeg);
    const double p[3] = { m_rotatePivot[0], m_rotatePivot[1], m_rotatePivot[2] };
    const double kPi = 3.14159265358979323846;

    Eigen::Vector3d yawAxis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d pitchAxis = Eigen::Vector3d::UnitX();
    if (m_screenSpaceRotation) {
        // 右上局部窗口使用严格的屏幕坐标旋转。这样无论当前已经转到哪个姿态，
        // 左右拖动都只对应屏幕水平旋转，上下拖动都只对应屏幕垂直旋转。
        if (vtkCamera* cam = m_ren ? m_ren->GetActiveCamera() : nullptr) {
            double upRaw[3] = {0.0, 1.0, 0.0};
            double dirRaw[3] = {0.0, 0.0, -1.0};
            cam->GetViewUp(upRaw);
            cam->GetDirectionOfProjection(dirRaw);
            Eigen::Vector3d viewUp(upRaw[0], upRaw[1], upRaw[2]);
            Eigen::Vector3d viewDir(dirRaw[0], dirRaw[1], dirRaw[2]);
            if (viewUp.allFinite() && viewUp.norm() > 1e-9
                && viewDir.allFinite() && viewDir.norm() > 1e-9) {
                viewUp.normalize();
                viewDir.normalize();
                Eigen::Vector3d viewRight = viewDir.cross(viewUp);
                if (viewRight.allFinite() && viewRight.norm() > 1e-9) {
                    viewRight.normalize();
                    yawAxis = viewUp;
                    pitchAxis = viewRight;
                }
            }
        }
    }

    Eigen::Affine3d d = Eigen::Translation3d(p[0], p[1], p[2])
        * Eigen::AngleAxisd(yawDeg * kPi / 180.0, yawAxis)
        * Eigen::AngleAxisd(pitchDeg * kPi / 180.0, pitchAxis)
        * Eigen::Translation3d(-p[0], -p[1], -p[2]);
    vtkSmartPointer<vtkMatrix4x4> dm = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) dm->SetElement(r, c, d.matrix()(r, c));
    }
    vtkSmartPointer<vtkMatrix4x4> nm = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkMatrix4x4::Multiply4x4(dm, m_viewRot->GetMatrix(), nm);
    m_viewRot->SetMatrix(nm);
    applyViewRotationToActors();
    renderWindowRender();
}

/** 【函数导航】
 * 作用：执行“onLeftRelease”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::onLeftRelease()
{
    if (!m_mousePressed) return;
    m_mousePressed = false;
    const bool wasRotating = m_rotating;
    m_rotating = false;
    if (!wasRotating) return;
    if (m_rotateAccumDeg > 5.0) {
        // 旋转超过 5°：视为旋转操作；按下时未做拾取，因此没有点击副作用需要撤销。
    } else {
        // ≤5°：视为单击，恢复按下时的场景旋转（画面不动）
        m_viewRot->DeepCopy(m_viewRotAtPress);
        applyViewRotationToActors();

        // 单击才做拾取（选种子/ROI/孔定位）。硬件拾取会清空/破坏可见帧缓冲，
        // 所以在拾取结束后强制重绘一次，避免画面变黑。
        auto iren = m_win ? m_win->GetInteractor() : nullptr;
        if (iren && m_ren && m_cellPicker) {
            m_cellPicker->Pick(m_pressX, m_pressY, 0, m_ren);
        }
        renderWindowRender();
    }
}

/** 【函数导航】
 * 作用：应用/设置“setDieJiaDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::onRightPress()
{
    auto iren = m_win ? m_win->GetInteractor() : nullptr;
    if (!iren) return;
    int* pos = iren->GetEventPosition();
    m_rightPressed = true;
    m_rightPressX = pos[0];
    m_rightPressY = pos[1];
    m_rightMovePixels = 0.0;
}

void DianYunXuanRanQi::onRightMouseMove()
{
    if (!m_rightPressed) return;
    auto iren = m_win ? m_win->GetInteractor() : nullptr;
    if (!iren) return;
    int* pos = iren->GetEventPosition();
    const double dx = static_cast<double>(pos[0] - m_rightPressX);
    const double dy = static_cast<double>(pos[1] - m_rightPressY);
    m_rightMovePixels = std::max(m_rightMovePixels, std::sqrt(dx * dx + dy * dy));
}

void DianYunXuanRanQi::onRightRelease()
{
    if (!m_rightPressed) return;
    m_rightPressed = false;
    if (!m_rightPickCb || m_rightMovePixels > 5.0) return;
    auto iren = m_win ? m_win->GetInteractor() : nullptr;
    if (!iren || !m_ren || !m_rightCellPicker) return;
    int* pos = iren->GetEventPosition();

    // seed 常落在孔沿甚至空腔附近，右键位置不一定正好有底图点可拾取。
    // 因此先在屏幕坐标里直接命中可见 seed 标记；命中后把该 seed 的原始世界坐标回传。
    int bestSeed = -1;
    double bestSeedPx2 = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < m_seedLabelPositions.size(); ++i) {
        const Eigen::Vector3f& seed = m_seedLabelPositions[i];
        double h0[4] = { seed.x(), seed.y(), seed.z(), 1.0 };
        double h1[4] = { 0.0, 0.0, 0.0, 0.0 };
        m_viewRot->GetMatrix()->MultiplyPoint(h0, h1);
        if (std::fabs(h1[3]) <= 1e-12) continue;
        m_ren->SetWorldPoint(h1[0] / h1[3], h1[1] / h1[3], h1[2] / h1[3], 1.0);
        m_ren->WorldToDisplay();
        double disp[3] = {0.0, 0.0, 0.0};
        m_ren->GetDisplayPoint(disp);
        const double dx = disp[0] - static_cast<double>(pos[0]);
        const double dy = disp[1] - static_cast<double>(pos[1]);
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestSeedPx2) { bestSeedPx2 = d2; bestSeed = static_cast<int>(i); }
    }
    if (bestSeed >= 0 && bestSeedPx2 <= 28.0 * 28.0) {
        const Eigen::Vector3f& seed = m_seedLabelPositions[(std::size_t)bestSeed];
        m_rightPickCb(seed.x(), seed.y(), seed.z());
        return;
    }

    // 没有直接命中 seed 标记时再退化到底图三维拾取，方便用户在孔沿附近右键。
    m_rightCellPicker->Pick(pos[0], pos[1], 0, m_ren);
    if (m_rightCellPicker->GetCellId() < 0 && m_rightCellPicker->GetPointId() < 0) return;
    double p[3];
    m_rightCellPicker->GetPickPosition(p);

    vtkSmartPointer<vtkMatrix4x4> inv = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkSmartPointer<vtkMatrix4x4> rot = vtkSmartPointer<vtkMatrix4x4>::New();
    rot->DeepCopy(m_viewRot->GetMatrix());
    vtkMatrix4x4::Invert(rot, inv);
    double h[4] = { p[0], p[1], p[2], 1.0 };
    double r[4] = { 0.0, 0.0, 0.0, 0.0 };
    inv->MultiplyPoint(h, r);
    if (std::fabs(r[3]) <= 1e-12) return;
    m_rightPickCb(r[0] / r[3], r[1] / r[3], r[2] / r[3]);
}

void DianYunXuanRanQi::rebuildSeedNumberLabels()
{
    if (m_renOverlay) {
        for (const auto& actor : m_seedLabelActors) {
            if (actor) m_renOverlay->RemoveActor(actor);
        }
    }
    m_seedLabelActors.clear();
    if (!m_renOverlay || m_seedLabelPositions.empty()) return;
    if (m_ren && m_ren->GetActiveCamera()) m_renOverlay->SetActiveCamera(m_ren->GetActiveCamera());

    m_seedLabelActors.reserve(m_seedLabelPositions.size());
    for (std::size_t i = 0; i < m_seedLabelPositions.size(); ++i) {
        const Eigen::Vector3f& p = m_seedLabelPositions[i];
        auto actor = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        const std::string text = std::to_string(i + 1);
        actor->SetInput(text.c_str());
        actor->SetPosition(p.x(), p.y(), p.z() + 2.2f);
        actor->PickableOff();
        if (auto* tp = actor->GetTextProperty()) {
            tp->SetFontSize(20);
            tp->SetBold(1);
            tp->SetColor(1.0, 1.0, 1.0);
            tp->SetBackgroundColor(0.05, 0.08, 0.12);
            tp->SetBackgroundOpacity(0.82);
            tp->SetJustificationToCentered();
            tp->SetVerticalJustificationToCentered();
        }
        m_renOverlay->AddActor(actor);
        m_seedLabelActors.push_back(actor);
    }
}

void DianYunXuanRanQi::setDieJiaDianYun(const CloudConstPtr& cloud)
{
    m_overlayCloud = cloud;
    refreshDieJiaOnly();
}

/** 【函数导航】
 * 作用：应用/设置“setDieJiaDianYunNoRefresh”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setDieJiaDianYunNoRefresh(const CloudConstPtr& cloud)
{
    m_overlayCloud = cloud;
}

/** 【函数导航】
 * 作用：清理/重置“clearDieJiaDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::clearDieJiaDianYun()
{
    clearDieJiaDianYunNoRefresh();
    refreshDieJiaOnly();
}

/** 【函数导航】
 * 作用：清理/重置“clearDieJiaDianYunNoRefresh”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::clearDieJiaDianYunNoRefresh()
{
    m_overlayCloud.reset();
}

/** 【函数导航】
 * 作用：应用/设置“setOverlayLinePolyData”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setOverlayLinePolyData(const vtkSmartPointer<vtkPolyData>& poly)
{
    m_overlayLines = poly;
    m_hasLineOverlay = (poly && poly->GetNumberOfPoints() > 0);
}

/** 【函数导航】
 * 作用：应用/设置“setOverlayLineOptions”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setOverlayLineOptions(bool forceColor, double r, double g, double b,
                                               double opacity, double lineWidth)
{
    m_lineForceColor = forceColor;
    m_lineR = r; m_lineG = g; m_lineB = b;
    m_lineOpacity = opacity;
    m_lineWidth = lineWidth;
}

/** 【函数导航】
 * 作用：清理/重置“clearOverlayLines”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::clearOverlayLines()
{
    m_overlayLines = nullptr;
    m_hasLineOverlay = false;
}

/** 【函数导航】
 * 作用：应用/设置“setBaseFade”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setBaseFade(bool enable, double opacity)
{
    setBaseFadeNoRefresh(enable, opacity);
    if (m_cloudActor) renderWindowRender();
}

/** 【函数导航】
 * 作用：应用/设置“setBaseFadeNoRefresh”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setBaseFadeNoRefresh(bool enable, double opacity)
{
    m_baseOpacity = std::clamp(enable ? opacity : 1.0, 0.02, 1.0);
    m_fadeBase = (m_baseOpacity < 1.0);
    m_fadeOpacity = m_baseOpacity;
    if (m_cloudActor) m_cloudActor->GetProperty()->SetOpacity(m_baseOpacity);
}

/** 【函数导航】
 * 作用：应用/设置“setBaseOpacity”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setBaseOpacity(double opacity)
{
    m_baseOpacity = std::clamp(opacity, 0.02, 1.0);
    m_fadeBase = (m_baseOpacity < 1.0);
    m_fadeOpacity = m_baseOpacity;
    if (m_cloudActor) {
        m_cloudActor->GetProperty()->SetOpacity(m_baseOpacity);
        renderWindowRender();
    }
}

/** 【函数导航】
 * 作用：应用/设置“setDianYunViewOptions”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setDianYunViewOptions(const DianYunViewOptions& opt)
{
    m_opt = opt;
    m_baseOpacity = std::clamp(opt.baseOpacity, 0.02, 1.0);
}

/** 【函数导航】
 * 作用：执行“cloudVizOptions”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
DianYunViewOptions DianYunXuanRanQi::cloudVizOptions() const
{
    DianYunViewOptions o = m_opt;
    o.baseOpacity = m_baseOpacity;
    o.baseOpacityFaded = m_baseOpacity;
    return o;
}

/** 【函数导航】
 * 作用：执行“rebuildHeightLut”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::rebuildHeightLut()
{
    if (!m_heightLut) return;
    m_heightLut->RemoveAllPoints();
    m_heightLut->SetColorSpaceToLab();
    std::vector<GaoduColorStop> stops = m_heightMap.stops;
    if (stops.empty()) {
        m_heightMap = makeDefaultHeightMap(m_validZMin, m_validZMax);
        stops = m_heightMap.stops;
    }
    // 局部孔预览使用轴向投影标量时，仅复用主视图的“颜色与相对节点位置”，
    // 不复用主视图绝对世界 Z 数值。这样倾斜零件回正观看后不会留下斜向红蓝色带。
    if (m_useProjectedHeightScalar && m_hasValidHeight && m_validZMax > m_validZMin) {
        const double srcMin = m_heightMap.zMin;
        const double srcMax = m_heightMap.zMax;
        const double srcSpan = srcMax - srcMin;
        const double dstSpan = m_validZMax - m_validZMin;
        for (std::size_t i = 0; i < stops.size(); ++i) {
            double u = (stops.size() <= 1) ? 0.0
                : static_cast<double>(i) / static_cast<double>(stops.size() - 1);
            if (std::isfinite(srcSpan) && std::fabs(srcSpan) > 1e-12)
                u = std::clamp((stops[i].z - srcMin) / srcSpan, 0.0, 1.0);
            stops[i].z = m_validZMin + u * dstSpan;
        }
    }
    std::sort(stops.begin(), stops.end(),
              [](const GaoduColorStop& a, const GaoduColorStop& b) { return a.z < b.z; });
    for (const auto& st : stops) {
        m_heightLut->AddRGBPoint(st.z,
                                 std::clamp(st.r, 0.0, 1.0),
                                 std::clamp(st.g, 0.0, 1.0),
                                 std::clamp(st.b, 0.0, 1.0));
    }
    m_heightLut->Build();
}

/** 【函数导航】
 * 作用：应用/设置“applyHeightScalarToMapper”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::applyHeightScalarToMapper(vtkPolyDataMapper* mapper)
{
    if (!mapper) return;
    mapper->ScalarVisibilityOn();
    mapper->SelectColorArray("HeightZ");
    mapper->SetScalarModeToUsePointFieldData();
    mapper->SetScalarRange(m_validZMin, m_validZMax);
    mapper->SetLookupTable(m_heightLut);
    mapper->SetUseLookupTableScalarRange(true);
}

/** 【函数导航】
 * 作用：应用/设置“setHeightColorMap”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setHeightScalarProjection(
    const Eigen::Vector3f& origin, const Eigen::Vector3f& axis)
{
    m_heightScalarOrigin = origin.allFinite() ? origin : Eigen::Vector3f::Zero();
    m_heightScalarAxis = axis;
    if (!m_heightScalarAxis.allFinite() || m_heightScalarAxis.norm() < 1e-6f)
        m_heightScalarAxis = Eigen::Vector3f::UnitZ();
    else
        m_heightScalarAxis.normalize();
    m_useProjectedHeightScalar = true;
}

void DianYunXuanRanQi::clearHeightScalarProjection()
{
    m_useProjectedHeightScalar = false;
    m_heightScalarOrigin = Eigen::Vector3f::Zero();
    m_heightScalarAxis = Eigen::Vector3f::UnitZ();
}

void DianYunXuanRanQi::setHeightColorMap(const GaoduColorMapSettings& s)
{
    m_heightMap = s;
    if (m_heightMap.stopCount != 3 && m_heightMap.stopCount != 4) {
        m_heightMap.stopCount = 4;
    }
    if (!m_hasValidHeight) return;
    rebuildHeightLut();
    if (m_cloudActor && m_cloudActor->GetMapper()) {
        if (auto* mapper = vtkPolyDataMapper::SafeDownCast(m_cloudActor->GetMapper())) {
            applyHeightScalarToMapper(mapper);
        }
    }
    renderWindowRender();
}

/** 【函数导航】
 * 作用：清理/重置“resetHeightColorMap”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::resetHeightColorMap()
{
    if (!m_hasValidHeight) return;
    m_heightMap = makeDefaultHeightMap(m_validZMin, m_validZMax);
    setHeightColorMap(m_heightMap);
}

/** 【函数导航】
 * 作用：应用/设置“setHeightColoringEnabled”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setHeightColoringEnabled(bool enabled)
{
    m_heightMap.enabled = enabled;
    if (!m_cloudActor || !m_cloudActor->GetMapper()) return;
    auto* mapper = vtkPolyDataMapper::SafeDownCast(m_cloudActor->GetMapper());
    if (!mapper) return;
    if (enabled && m_hasValidHeight) {
        applyHeightScalarToMapper(mapper);
    } else {
        mapper->ScalarVisibilityOn();
        mapper->SetScalarModeToUsePointData();
        mapper->SetColorModeToDirectScalars();
    }
    renderWindowRender();
}

/** 【函数导航】
 * 作用：应用/设置“setBackgroundColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setBackgroundColor(double r, double g, double b)
{
    m_bgR = r; m_bgG = g; m_bgB = b;
    if (m_ren) m_ren->SetBackground(m_bgR, m_bgG, m_bgB);
    if (m_renOverlay) m_renOverlay->SetBackground(m_bgR, m_bgG, m_bgB);
    if (m_w) m_w->update();
}

/** 【函数导航】
 * 作用：执行“getBackgroundColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::getBackgroundColor(double& r, double& g, double& b) const
{
    r = m_bgR; g = m_bgG; b = m_bgB;
}

/** 【函数导航】
 * 作用：应用/设置“setFadePointColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setFadePointColor(double r, double g, double b)
{

    m_fadePtR = r; m_fadePtG = g; m_fadePtB = b;
}

/** 【函数导航】
 * 作用：执行“getFadePointColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::getFadePointColor(double& r, double& g, double& b) const
{
    r = m_fadePtR; g = m_fadePtG; b = m_fadePtB;
}

/** 【函数导航】
 * 作用：应用/设置“setPointPickCallback”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setPointPickCallback(std::function<void(double, double, double)> cb)
{
    m_pickCb = std::move(cb);
}

void DianYunXuanRanQi::setRightPointPickCallback(std::function<void(double, double, double)> cb)
{
    m_rightPickCb = std::move(cb);
}

void DianYunXuanRanQi::setSeedNumberLabels(const std::vector<Eigen::Vector3f>& positions)
{
    m_seedLabelPositions = positions;
    refreshDieJiaOnly();
}

void DianYunXuanRanQi::clearSeedNumberLabels()
{
    if (m_seedLabelPositions.empty() && m_seedLabelActors.empty()) return;
    m_seedLabelPositions.clear();
    refreshDieJiaOnly();
}

/** 【函数导航】
 * 作用：应用/设置“setPickCancelledCallback”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::setPickCancelledCallback(std::function<void()> cb)
{
    m_pickCancelledCb = std::move(cb);
}

/** 【函数导航】
 * 作用：执行“renderWindowRender”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void DianYunXuanRanQi::renderWindowRender()
{
    if (!m_win) return;
    m_win->Render();
}

}

// ============================================================================
// 高度色带控件实现
// ============================================================================
/*
模块职责：
高度色标界面控件。

主要调用位置：
由主窗口/点云显示模块创建，用于展示和调整高度着色范围。

维护说明：
颜色范围只影响显示；不得借此修改点云几何或孔识别输入。
*/
#include <QPainter>
#include <QMouseEvent>
#include <QColorDialog>
#include <QDoubleSpinBox>

namespace {
/** 【函数导航】
 * 作用：执行“srgbRelativeLuminance”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double srgbRelativeLuminance(const QColor& c)
{
    const auto lin = [](double v) {
        v /= 255.0;
        return (v <= 0.04045) ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4);
    };
    return 0.2126 * lin(c.red()) + 0.7152 * lin(c.green()) + 0.0722 * lin(c.blue());
}
}

namespace {
/** 【函数导航】
 * 作用：执行“stopColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QColor stopColor(const dianYunView::GaoduColorStop& s)
{
    return QColor::fromRgbF(std::clamp(s.r, 0.0, 1.0),
                            std::clamp(s.g, 0.0, 1.0),
                            std::clamp(s.b, 0.0, 1.0));
}

/** 【函数导航】
 * 作用：执行“lerpColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QColor lerpColor(const QColor& a, const QColor& b, double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return QColor::fromRgbF(a.redF() + (b.redF() - a.redF()) * t,
                            a.greenF() + (b.greenF() - a.greenF()) * t,
                            a.blueF() + (b.blueF() - a.blueF()) * t);
}
}

GaoduColorBarWidget::GaoduColorBarWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("heightColorBar"));

    setAutoFillBackground(false);

    m_btn3 = new QPushButton(QStringLiteral("3色"), this);
    m_btn4 = new QPushButton(QStringLiteral("4色"), this);
    m_btnEqual = new QPushButton(QStringLiteral("等分"), this);
    m_btnTop = new QPushButton(QStringLiteral("顶聚"), this);
    m_btnBottom = new QPushButton(QStringLiteral("底聚"), this);
    m_dzSpin = new QDoubleSpinBox(this);
    m_btn3->setObjectName(QStringLiteral("btnHeight3"));
    m_btn4->setObjectName(QStringLiteral("btnHeight4"));
    m_btnEqual->setObjectName(QStringLiteral("btnHeightEqual"));
    m_btnTop->setObjectName(QStringLiteral("btnHeightTop"));
    m_btnBottom->setObjectName(QStringLiteral("btnHeightBottom"));
    m_dzSpin->setObjectName(QStringLiteral("spinHeightDz"));
    m_dzSpin->setDecimals(1);
    m_dzSpin->setRange(0.1, 1.0e6);
    m_dzSpin->setValue(10.0);
    m_dzSpin->setPrefix(QStringLiteral("ΔZ "));
    m_dzSpin->setSuffix(QStringLiteral(" 单位"));

    for (QPushButton* b : { m_btn3, m_btn4, m_btnEqual, m_btnTop, m_btnBottom }) {
        b->setFixedWidth(24);
        b->setFixedHeight(20);
        b->setStyleSheet(QStringLiteral(
            "QPushButton{border:1px solid #94a3b8;border-radius:3px;background:#f1f5f9;color:#1e293b;font-size:10px;}"
            "QPushButton:hover{background:#e2e8f0;}"));
    }
    m_dzSpin->setFixedWidth(28);
    m_dzSpin->setFixedHeight(20);
    m_dzSpin->setStyleSheet(QStringLiteral(
        "QDoubleSpinBox{border:1px solid #94a3b8;border-radius:3px;background:#f8fafc;color:#1e293b;font-size:10px;}"));

    connect(m_btn3, &QPushButton::clicked, this, [this]() { setStopCount(3); });
    connect(m_btn4, &QPushButton::clicked, this, [this]() { setStopCount(4); });
    connect(m_btnEqual, &QPushButton::clicked, this, [this]() { makeEqualSpacing(); });
    connect(m_btnTop, &QPushButton::clicked, this, [this]() {
        if (m_rangeValid) makeTopFocus(m_dzSpin->value());
    });
    connect(m_btnBottom, &QPushButton::clicked, this, [this]() {
        if (m_rangeValid) makeBottomFocus(m_dzSpin->value());
    });

    if (parent) {
        parent->installEventFilter(this);
        layoutGeometry();
    }
    setRangeValid(false);
}

/** 【函数导航】
 * 作用：执行“eventFilter”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool GaoduColorBarWidget::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == parent() && ev->type() == QEvent::Resize) {
        layoutGeometry();
    }
    return QWidget::eventFilter(obj, ev);
}

/** 【函数导航】
 * 作用：执行“layoutGeometry”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::layoutGeometry()
{
    if (!parentWidget()) return;
    const int ph = parentWidget()->height();
    const int margin = 8;
    const int reservedBottom = std::max(132, ph * 22 / 100);
    const int w = 118;
    const int h = std::max(190, ph - reservedBottom - margin);
    setGeometry(margin, margin, w, h);
    layoutControls();
}

/** 【函数导航】
 * 作用：执行“layoutControls”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::layoutControls()
{
    m_controlsRect = QRect(4, height() - 50, width() - 8, 46);
    const int y1 = m_controlsRect.top();
    const int y2 = y1 + 23;
    const int x = m_controlsRect.left();
    // 高度配色工具保持两行布局：第一行是色阶/等分/ΔZ，第二行是顶聚和底聚。
    m_btn3->setGeometry(x, y1, 24, 20);
    m_btn4->setGeometry(x + 27, y1, 24, 20);
    m_btnEqual->setGeometry(x + 54, y1, 24, 20);
    m_dzSpin->setGeometry(x + 82, y1, 28, 20);
    m_btnTop->setGeometry(x, y2, 24, 20);
    m_btnBottom->setGeometry(x + 27, y2, 24, 20);
    m_gradientRect = QRect(10, 24, 20, m_controlsRect.top() - 34);
}

/** 【函数导航】
 * 作用：应用/设置“setChangJingBackgroundColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::setChangJingBackgroundColor(const QColor& c)
{
    m_sceneBg = c;
    m_fg = (srgbRelativeLuminance(c) > 0.179) ? QColor(0, 0, 0) : QColor(255, 255, 255);
    update();
}

/** 【函数导航】
 * 作用：应用/设置“setRangeValid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::setRangeValid(bool valid)
{
    m_rangeValid = valid;
    update();
}

/** 【函数导航】
 * 作用：应用/设置“setRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::setRange(double zMin, double zMax, bool resetToDefaults)
{
    if (!std::isfinite(zMin) || !std::isfinite(zMax) || zMax <= zMin) {
        setRangeValid(false);
        return;
    }
    const double oldMin = m_settings.zMin;
    const double oldMax = m_settings.zMax;
    const bool hadOldRange = m_rangeValid && oldMax > oldMin;
    m_rangeValid = true;
    m_settings.zMin = zMin;
    m_settings.zMax = zMax;

    if (resetToDefaults || !hadOldRange || m_nodes.empty()) {
        makeEqualSpacing();
        return;
    }

    const double oldSpan = oldMax - oldMin;
    const double newSpan = zMax - zMin;
    for (std::size_t i = 0; i < m_settings.stops.size(); ++i) {
        auto& st = m_settings.stops[i];
        if (i == 0) {
            st.z = zMin;
            st.positionLocked = true;
        } else if (i + 1 == m_settings.stops.size()) {
            st.z = zMax;
            st.positionLocked = true;
        } else {
            const double norm = (st.z - oldMin) / oldSpan;
            st.z = zMin + norm * newSpan;
        }
    }
    rebuildNodes();
    emitChanged();
    update();
}

/** 【函数导航】
 * 作用：应用/设置“applySettings”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::applySettings(const dianYunView::GaoduColorMapSettings& s)
{
    m_settings = s;
    if (m_settings.stopCount != 3 && m_settings.stopCount != 4) m_settings.stopCount = 4;
    rebuildNodes();
    update();
}

/** 【函数导航】
 * 作用：应用/设置“setSettings”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::setSettings(const dianYunView::GaoduColorMapSettings& s)
{
    applySettings(s);
}

/** 【函数导航】
 * 作用：执行“rebuildNodes”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::rebuildNodes()
{
    m_nodes.clear();
    for (const auto& st : m_settings.stops) {
        Node n;
        n.z = st.z;
        n.color = stopColor(st);
        n.locked = st.positionLocked;
        m_nodes.push_back(n);
    }
    m_dragIndex = -1;
}

/** 【函数导航】
 * 作用：执行“emitChanged”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::emitChanged()
{
    emit settingsChanged(m_settings);
}

/** 【函数导航】
 * 作用：应用/设置“setStopCount”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::setStopCount(int n)
{
    if (n != 3 && n != 4) return;
    m_settings.stopCount = n;
    makeEqualSpacing();
}

/** 【函数导航】
 * 作用：构建“makeEqualSpacing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::makeEqualSpacing()
{
    if (!m_rangeValid) return;
    const double zMin = m_settings.zMin;
    const double zMax = m_settings.zMax;
    const int n = m_settings.stopCount;
    const double span = zMax - zMin;
    const QColor low(0x2C, 0x7B, 0xB6);
    const QColor high(0xD7, 0x30, 0x27);
    const QColor mid4a(0x00, 0xA6, 0xCA);
    const QColor mid4b(0xF6, 0xC3, 0x44);
    const QColor mid3(0xF6, 0xC3, 0x44);

    std::vector<dianYunView::GaoduColorStop> stops;
    auto add = [&](double z, const QColor& c, bool locked) {
        dianYunView::GaoduColorStop st;
        st.z = z;
        st.r = c.redF(); st.g = c.greenF(); st.b = c.blueF();
        st.positionLocked = locked;
        stops.push_back(st);
    };
    if (n == 4) {
        add(zMin, low, true);
        add(zMin + span / 3.0, mid4a, false);
        add(zMin + span * 2.0 / 3.0, mid4b, false);
        add(zMax, high, true);
    } else {
        add(zMin, low, true);
        add(zMin + span * 0.5, mid3, false);
        add(zMax, high, true);
    }
    m_settings.stops = stops;
    rebuildNodes();
    emitChanged();
    update();
}

/** 【函数导航】
 * 作用：构建“makeTopFocus”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::makeTopFocus(double dz)
{
    if (!m_rangeValid || dz <= 0.0) return;
    const double zMin = m_settings.zMin;
    const double zMax = m_settings.zMax;
    const double span = zMax - zMin;
    dz = std::clamp(dz, span * 0.02, span * 0.98);
    const int n = m_settings.stopCount;
    const QColor low(0x2C, 0x7B, 0xB6);
    const QColor high(0xD7, 0x30, 0x27);
    const QColor midA(0x00, 0xA6, 0xCA);
    const QColor midB(0xF6, 0xC3, 0x44);

    std::vector<dianYunView::GaoduColorStop> stops;
    auto add = [&](double z, const QColor& c, bool locked) {
        dianYunView::GaoduColorStop st;
        st.z = std::clamp(z, zMin, zMax);
        st.r = c.redF(); st.g = c.greenF(); st.b = c.blueF();
        st.positionLocked = locked;
        stops.push_back(st);
    };
    if (n == 4) {
        add(zMin, low, true);
        add(zMax - dz, midA, false);
        add(zMax - dz / 3.0, midB, false);
        add(zMax, high, true);
    } else {
        add(zMin, low, true);
        add(zMax - dz * 0.5, midB, false);
        add(zMax, high, true);
    }
    m_settings.stops = stops;
    rebuildNodes();
    emitChanged();
    update();
}

/** 【函数导航】
 * 作用：构建“makeBottomFocus”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::makeBottomFocus(double dz)
{
    if (!m_rangeValid || dz <= 0.0) return;
    const double zMin = m_settings.zMin;
    const double zMax = m_settings.zMax;
    const double span = zMax - zMin;
    dz = std::clamp(dz, span * 0.02, span * 0.98);
    const int n = m_settings.stopCount;
    const QColor low(0x2C, 0x7B, 0xB6);
    const QColor high(0xD7, 0x30, 0x27);
    const QColor midA(0x00, 0xA6, 0xCA);
    const QColor midB(0xF6, 0xC3, 0x44);

    std::vector<dianYunView::GaoduColorStop> stops;
    auto add = [&](double z, const QColor& c, bool locked) {
        dianYunView::GaoduColorStop st;
        st.z = std::clamp(z, zMin, zMax);
        st.r = c.redF(); st.g = c.greenF(); st.b = c.blueF();
        st.positionLocked = locked;
        stops.push_back(st);
    };
    if (n == 4) {
        add(zMin, low, true);
        add(zMin + dz / 3.0, midA, false);
        add(zMin + dz, midB, false);
        add(zMax, high, true);
    } else {
        add(zMin, low, true);
        add(zMin + dz * 0.5, midB, false);
        add(zMax, high, true);
    }
    m_settings.stops = stops;
    rebuildNodes();
    emitChanged();
    update();
}

/** 【函数导航】
 * 作用：执行“yToZ”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double GaoduColorBarWidget::yToZ(int y) const
{
    const QRect r = m_gradientRect;
    if (r.height() <= 1) return m_settings.zMin;
    const double t = 1.0 - (y - r.top()) / double(r.height());
    return m_settings.zMin + t * (m_settings.zMax - m_settings.zMin);
}

/** 【函数导航】
 * 作用：执行“zToY”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int GaoduColorBarWidget::zToY(double z) const
{
    const QRect r = m_gradientRect;
    const double span = m_settings.zMax - m_settings.zMin;
    const double t = span > 0.0 ? (z - m_settings.zMin) / span : 0.0;
    return r.top() + int((1.0 - std::clamp(t, 0.0, 1.0)) * r.height());
}

/** 【函数导航】
 * 作用：执行“hitNode”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int GaoduColorBarWidget::hitNode(const QPoint& pos) const
{
    const int cy = pos.y();
    int best = -1;
    int bestD = 12;
    for (std::size_t i = 0; i < m_nodes.size(); ++i) {
        const int ny = zToY(m_nodes[i].z);
        const int d = std::abs(ny - cy);
        if (d < bestD && pos.x() >= m_gradientRect.left() - 8 && pos.x() <= m_gradientRect.right() + 70) {
            bestD = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“mousePressEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::mousePressEvent(QMouseEvent* ev)
{
    if (ev->button() == Qt::LeftButton) {
        const int idx = hitNode(ev->pos());
        m_selectedIndex = idx;
        if (idx >= 0 && !m_nodes[static_cast<std::size_t>(idx)].locked) {
            m_dragIndex = idx;
        }
        update();
    }
    QWidget::mousePressEvent(ev);
}

/** 【函数导航】
 * 作用：执行“mouseMoveEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if (m_dragIndex >= 0 && m_rangeValid && m_nodes.size() >= 3) {
        const std::size_t i = static_cast<std::size_t>(m_dragIndex);
        const double zMin = (i == 0) ? m_settings.zMin : m_nodes[i - 1].z;
        const double zMax = (i + 1 >= m_nodes.size()) ? m_settings.zMax : m_nodes[i + 1].z;
        const double z = std::clamp(yToZ(ev->pos().y()), zMin + 1e-6, zMax - 1e-6);
        m_nodes[i].z = z;
        m_settings.stops[i].z = z;
        emitChanged();
        update();
    }
    QWidget::mouseMoveEvent(ev);
}

/** 【函数导航】
 * 作用：执行“mouseReleaseEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::mouseReleaseEvent(QMouseEvent* ev)
{
    m_dragIndex = -1;
    QWidget::mouseReleaseEvent(ev);
}

/** 【函数导航】
 * 作用：执行“mouseDoubleClickEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::mouseDoubleClickEvent(QMouseEvent* ev)
{
    const int idx = hitNode(ev->pos());
    if (idx >= 0) {
        openColorDialog(idx);
        return;
    }
    QWidget::mouseDoubleClickEvent(ev);
}

/** 【函数导航】
 * 作用：执行“openColorDialog”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::openColorDialog(int nodeIndex)
{
    if (nodeIndex < 0 || nodeIndex >= static_cast<int>(m_nodes.size())) return;
    const std::size_t i = static_cast<std::size_t>(nodeIndex);
    const QColor c = QColorDialog::getColor(m_nodes[i].color, this, QStringLiteral("锚点颜色"));
    if (!c.isValid()) return;
    m_nodes[i].color = c;
    m_settings.stops[i].r = c.redF();
    m_settings.stops[i].g = c.greenF();
    m_settings.stops[i].b = c.blueF();
    emitChanged();
    update();
}

/** 【函数导航】
 * 作用：执行“paintEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    if (!m_rangeValid) {
        p.setPen(m_fg);
        p.drawText(rect().adjusted(6, 8, -6, -8), Qt::AlignLeft | Qt::AlignTop,
                   QStringLiteral("无有效Z\n范围"));
        return;
    }

    const QRect g = m_gradientRect;
    if (g.height() > 1) {
        for (int y = g.top(); y <= g.bottom(); ++y) {
            const double z = yToZ(y);
            QColor col = m_nodes.front().color;
            for (std::size_t i = 1; i < m_nodes.size(); ++i) {
                if (z <= m_nodes[i].z) {
                    const double a = m_nodes[i - 1].z;
                    const double b = m_nodes[i].z;
                    const double t = (b > a) ? (z - a) / (b - a) : 0.0;
                    col = lerpColor(m_nodes[i - 1].color, m_nodes[i].color, t);
                    break;
                }
            }
            p.fillRect(g.left(), y, g.width(), 1, col);
        }
        p.setPen(m_fg);
        p.drawRect(g.adjusted(0, 0, -1, -1));
    }

    for (std::size_t i = 0; i < m_nodes.size(); ++i) {
        const int ny = zToY(m_nodes[i].z);
        const QColor& c = m_nodes[i].color;
        const bool sel = (static_cast<int>(i) == m_selectedIndex);
        p.setBrush(c);
        p.setPen(sel ? m_fg : m_fg);
        p.drawEllipse(QPoint(g.right() - 1, ny), sel ? 7 : 5, sel ? 7 : 5);
        if (sel) {
            p.setPen(m_fg);
            p.drawEllipse(QPoint(g.right() - 1, ny), 9, 9);
        }

        p.setPen(m_fg);
        QFont f = p.font();
        f.setPointSize(8);
        p.setFont(f);
        p.drawText(g.right() + 10, ny - 9, width() - g.right() - 16, 18,
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QString::number(m_nodes[i].z, 'f', 2));
        if (m_nodes[i].locked) {
            p.setPen(m_fg);
            p.drawText(g.right() + 10, ny + 8, width() - g.right() - 16, 12,
                       Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("锁"));
        }
    }

    p.setPen(m_fg);
    p.drawText(g.left(), g.top() - 14, g.width(), 12, Qt::AlignCenter, QStringLiteral("高Z"));
    p.drawText(g.left(), g.bottom() + 4, g.width(), 12, Qt::AlignCenter, QStringLiteral("低Z"));
}

/** 【函数导航】
 * 作用：执行“resizeEvent”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void GaoduColorBarWidget::resizeEvent(QResizeEvent* ev)
{
    layoutControls();
    QWidget::resizeEvent(ev);
}


// ============================================================================
// 功能分区：孔结果显示实现
// ============================================================================
/*
模块职责：
孔结果显示实现。

维护说明：
本文件按功能整合生产实现。各分区通过明确职责组织，算法阈值和候选顺序集中在对应分区维护。
*/
// 孔显示实现已合并到本文件。

// ============================================================================
// 功能分区：孔显示策略实现
// ============================================================================
/*
模块职责：
实现规范孔列表筛选、同孔候选替换和点击最近孔选择，保证界面显示、点击定位与孔参数导出使用同一套孔集合。

主要调用位置：
ZhuChuangKouWindow 设置孔结果、孔覆盖层和导出孔参数时调用；本模块只做无状态选择，不渲染也不修改 HoleMiaoshu。

维护说明：
这里的可见性门和候选优先级属于生产显示语义。整理代码时可以改善结构和命名，但不能为了“多显示一些孔”放宽假孔过滤条件。
*/

namespace holeXianshi {

/** 【函数导航】
 * 作用：执行“isNormalXianshiHole”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool isNormalXianshiHole(const HoleMiaoshu& d)
{
    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    // 排除 Artifact 类型
    if (d.physicalHoleTypeDisplay == PHT::Artifact) return false;
    // 空腔真实性检查：核心区域没有形成有效空洞时，候选不作为真实孔显示
    if (d.roiVoidnessDataValid && d.roiCoreVoidRatio <= 0.01f
        && d.roiWallToCoreDensityRatio <= 1.05f) return false;
    // 排除 FalseHole
    if (d.finalHoleJudgment == HoleMiaoshu::FinalHolePanDing::FalseHole) return false;
    // 强锥壁证据可以提升候选显示优先级，但仍必须通过孔型审核
    // 必须排除core-filled/BossInner等artifact来源
    bool strongConePromoted = (d.physicalHoleTypeDisplay == PHT::Cone
        && d.coneWallCompletenessScore >= 0.60f
        && d.wallDownTrackDepthMm > 2.0f
        && d.wallDownCombinedClass == "CONE_WALL_TRACKED"
        && d.roiVoidnessDataValid && d.roiCoreVoidRatio > 0.30f);
    // 排除隐藏/仅查看（strong cone除外）
    if (!strongConePromoted
        && (d.guiDisplayStatus == "CandidateReview"
            || d.guiDisplayStatus == "HiddenByWeakScanline"
            || d.guiDisplayStatus == "DowngradedByMZShallowNoRing")) return false;
    // ConeLike_LowConfidence: 需有空洞 + 孔壁支撑
    // 高空洞(>0.5)放宽深度要求（扫描限制场景）
    if (d.physicalHoleTypeDisplay == PHT::ConeLike_LowConfidence) {
        if (!d.roiVoidnessDataValid) return false;
        if (d.roiCoreVoidRatio <= 0.05f) return false;
        if (d.roiWallToCoreDensityRatio <= 1.10f) return false;
        float depthMin = (d.roiCoreVoidRatio > 0.5f) ? 0.5f : 2.0f;
        if (d.wallDownTrackDepthMm < depthMin) return false;
        if (d.wallDownTrackLayerCount < 4) return false;
        return true;
    }
    // 仅接受正向类型
    return (d.physicalHoleTypeDisplay == PHT::Cone
            || d.physicalHoleTypeDisplay == PHT::Straight
            || d.physicalHoleTypeDisplay == PHT::Straight_DeformedWall
            || (d.physicalHoleTypeDisplay == PHT::Unknown
                && d.guiDisplayStatus == "Visible"));
}

/** 【函数导航】
 * 作用：执行“hasRealHoleExplanation”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool hasRealHoleExplanation(const HoleMiaoshu& d)
{
    // 核心必须有足够的空腔
    if (d.roiVoidnessDataValid) {
        if (d.roiCoreVoidRatio <= 0.01f && d.roiWallToCoreDensityRatio <= 1.05f)
            return false;
    }
    // 排除 FalseHole
    if (d.finalHoleJudgment == HoleMiaoshu::FinalHolePanDing::FalseHole)
        return false;
    // guiDisplayStatus过滤已在isNormalXianshiHole中统一处理；
    // hasRealHoleExplanation只负责几何证据验证，不做二次状态过滤。
    // 根据physicalType做最终判断
    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    switch (d.physicalHoleTypeDisplay) {
    case PHT::Cone:
        return d.roiVoidnessDataValid && d.roiCoreVoidRatio > 0.05f;
    case PHT::Straight:
    case PHT::Straight_DeformedWall:
        return d.roiVoidnessDataValid && d.roiCoreVoidRatio > 0.05f;
    case PHT::ConeLike_LowConfidence:
        // 直孔和低置信锥形候选使用各自的可见性条件，不套用完整锥孔深度条件
        if (!d.roiVoidnessDataValid || d.roiCoreVoidRatio <= 0.05f) return false;
        if (d.roiWallToCoreDensityRatio <= 1.10f) return false;
        if (d.wallDownTrackLayerCount < 4) return false;
        return true;
    case PHT::Unknown:
        return d.guiDisplayStatus == "Visible"
            && d.roiVoidnessDataValid && d.roiCoreVoidRatio > 0.05f;
    default:
        return false;
    }
}


/** 【函数导航】
 * 作用：选择“selectGuiFanNormalXianshiHouXuan”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<int> selectGuiFanNormalXianshiHouXuan(
    const std::vector<HoleMiaoshu>& candidates)
{
    using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
    const int n = (int)candidates.size();

    // 先建baseNormal
    std::vector<int> baseNormal;
    for (int i = 0; i < n; ++i)
        if (isNormalXianshiHole(candidates[i]))
            baseNormal.push_back(i);

    // 对每个baseNormal候选，查找同孔更强候选做替换
    std::vector<int> result;
    for (int bi : baseNormal) {
        int bestIdx = bi; int bestPri = 0;
        const auto& bn = candidates[bi];

        // 找同孔邻近候选（只replace，不add）
        for (int j = 0; j < n; ++j) {
            if (j == bi) continue;
            const auto& h = candidates[j];
            float dx = bn.centerTop.x() - h.centerTop.x();
            float dy = bn.centerTop.y() - h.centerTop.y();
            float dist = std::sqrt(dx*dx + dy*dy);
            float rMin = std::min(bn.rTop > 0 ? bn.rTop : bn.radius,
                                  h.rTop > 0 ? h.rTop : h.radius);
            if (dist > std::max(1.5f, 0.5f * rMin)) continue;
            float rMax = std::max(bn.rTop, h.rTop);
            float rMin2 = std::min(bn.rTop, h.rTop);
            if (rMin2 <= 0 || rMax / rMin2 >= 2.0f) continue;
            // 排除artifact/FalseHole
            if (h.physicalHoleTypeDisplay == PHT::Artifact) continue;
            if (h.finalHoleJudgment == HoleMiaoshu::FinalHolePanDing::FalseHole) continue;
            // 更强候选？
            int pri = 0;
            bool isCone = (h.physicalHoleTypeDisplay == PHT::Cone);
            bool strongCone = isCone && h.coneWallCompletenessScore >= 0.60f
                && h.wallDownTrackDepthMm > 2.0f && h.signedRadiusTrend < -0.01f;
            if (strongCone) pri = 6;
            else if (isCone) pri = 5;
            else if (h.physicalHoleTypeDisplay == PHT::Straight) pri = 4;
            else if (h.physicalHoleTypeDisplay == PHT::Straight_DeformedWall) pri = 3;
            if (pri > bestPri) { bestPri = pri; bestIdx = j; }
        }
        result.push_back(bestIdx);
    }
    return result;
}

/** 【函数导航】
 * 作用：选择“selectDisplaySet”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
XianshiSelection selectDisplaySet(const std::vector<HoleMiaoshu>& holes,
                                  int selectedIndex)
{
    XianshiSelection sel;
    const auto canonIdx = selectGuiFanNormalXianshiHouXuan(holes);
    for (int ci : canonIdx) {
        if (ci == selectedIndex)
            sel.selectedPosition = static_cast<int>(sel.displayIndices.size());
        sel.displayIndices.push_back(ci);
    }
    return sel;
}

/** 【函数导航】
 * 作用：检测/搜索“findNearestHoleIndex”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int findNearestHoleIndex(const std::vector<HoleMiaoshu>& holes,
                         const Eigen::Vector3f& clickWorld,
                         float maxDistanceMm)
{
    int best = -1;
    float bestD2 = maxDistanceMm * maxDistanceMm;
    for (std::size_t i = 0; i < holes.size(); ++i) {
        const HoleMiaoshu& d = holes[i];
        const Eigen::Vector3f c =
            (d.rTop > 0.0f && d.centerTop.allFinite()) ? d.centerTop : d.center;
        const float dx = c.x() - clickWorld.x();
        const float dy = c.y() - clickWorld.y();
        const float dz = c.z() - clickWorld.z();
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < bestD2) {
            bestD2 = d2;
            best = static_cast<int>(i);
        }
    }
    return best;
}

}


// ============================================================================
// 功能分区：孔叠加几何实现
// ============================================================================
/*
模块职责：
把 HoleMiaoshu 转换为 VTK 线框，并通过统一渲染接口显示孔口、孔轴和当前选中孔。

主要调用位置：
ZhuChuangKouWindow 在识别完成、切换孔或加载内嵌孔参数后调用；输入孔列表已经由 HoleDisplayPolicy 规范化。

维护说明：
本模块只负责显示几何。颜色、线宽和透明度可以调整，但不得在这里重新估计中心、半径、深度或孔形。
*/
#include "HoleLeixing_Types.h"


namespace holeDieJia {

/** 【函数导航】
 * 作用：构建“buildLineGeometry”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
namespace {
vtkSmartPointer<vtkPolyData> buildLineGeometryImpl(
    const std::vector<HoleMiaoshu>& holes, int selectedIndex, double zLift, bool previewAxisAligned)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("rgb");

    auto addPoint = [&](double x, double y, double z, unsigned char r, unsigned char g, unsigned char b) -> vtkIdType {
        vtkIdType id = points->InsertNextPoint(x, y, z);
        colors->InsertNextTuple3(r, g, b);
        return id;
    };

    auto addPolyline = [&](const std::vector<vtkIdType>& ids) {
        if (ids.size() < 2) return;
        lines->InsertNextCell(static_cast<vtkIdType>(ids.size()));
        for (vtkIdType id : ids) lines->InsertCellPoint(id);
    };

    auto validCenter = [](const Eigen::Vector3f& v) {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    };

    const int N = 144;

    for (size_t hi = 0; hi < holes.size(); ++hi) {
        if (selectedIndex >= 0 && static_cast<int>(hi) != selectedIndex) continue;
        const HoleMiaoshu& d = holes[hi];

        // 显示模式过滤：非单孔查看时按统一规则过滤
        if (selectedIndex < 0) {
            if (!holeXianshi::isNormalXianshiHole(d)) continue;
        }

        const float rTop = (d.rTop > 0.0f && std::isfinite(d.rTop)) ? d.rTop : d.radius;
        if (!(rTop > 0.0f) || !std::isfinite(rTop)) continue;

        Eigen::Vector3f cTop = validCenter(d.centerTop) ? d.centerTop : d.center;
        if (!validCenter(cTop)) continue;

        Eigen::Vector3f normal(d.localPlaneNx, d.localPlaneNy, d.localPlaneNz);
        if (!std::isfinite(normal.x()) || !std::isfinite(normal.y()) || !std::isfinite(normal.z())
            || normal.norm() < 0.5f) {
            normal = Eigen::Vector3f(0, 0, 1);
        } else {
            normal.normalize();
        }
        const bool polarityKnown = HoleJiXing::known(d.inwardPolarity);
        const float inwardSign = HoleJiXing::signOrZero(d.inwardPolarity);
        const Eigen::Vector3f surfaceInwardAxis = normal * inwardSign;
        Eigen::Vector3f finalAxis(d.holeAxisInNx, d.holeAxisInNy, d.holeAxisInNz);
        if (!d.holeAxisInValid || !finalAxis.allFinite() || finalAxis.norm() < 1e-6f)
            finalAxis = surfaceInwardAxis;
        if (!finalAxis.allFinite() || finalAxis.norm() < 1e-6f) finalAxis = -normal;
        finalAxis.normalize();

        // 显示层统一采用最终孔轴 holeAxisIn 作为上下口截面法向。
        // 上口、下口、中心十字和孔轴线必须属于同一套最终孔几何，不能主视图仍使用局部表面法向、
        // 局部预览再改用孔轴法向；否则倾斜锥孔（4.1最明显）会出现圆环角度与参数栏孔轴方向不一致。
        // previewAxisAligned 参数保留仅为接口兼容，当前主视图和局部预览都执行轴对齐显示。
        (void)previewAxisAligned;
        Eigen::Vector3f axisU = finalAxis.unitOrthogonal();
        if (!axisU.allFinite() || axisU.norm() < 1e-6f) axisU = Eigen::Vector3f::UnitX();
        axisU.normalize();
        Eigen::Vector3f axisV = finalAxis.cross(axisU);
        if (!axisV.allFinite() || axisV.norm() < 1e-6f) axisV = Eigen::Vector3f::UnitY();
        axisV.normalize();
        const Eigen::Vector3f topU = axisU;
        const Eigen::Vector3f topV = axisV;
        const Eigen::Vector3f bottomU = axisU;
        const Eigen::Vector3f bottomV = axisV;

        // 颜色只由最终物理孔型决定，置信度信息通过文字状态单独表达
        unsigned char rt, gt, bt;
        if (selectedIndex >= 0) {
            rt = 255; gt = 40; bt = 40;  // 单孔高亮红
        } else {
            using PHT = HoleMiaoshu::WuLiHoleLeixingXianshi;
            switch (d.physicalHoleTypeDisplay) {
            case PHT::Cone:
                rt = 0;   gt = 220; bt = 0;   break;  // 绿色：锥孔
            case PHT::Straight:
                rt = 0;   gt = 140; bt = 255; break;  // 蓝色：直孔
            case PHT::Straight_DeformedWall:
                rt = 255; gt = 140; bt = 0;   break;  // 橙色：直孔壁面变形
            case PHT::ConeLike_LowConfidence:
                rt = 0;   gt = 180; bt = 180; break;  // 青色：低置信锥孔
            default:
                rt = 180; gt = 180; bt = 180; break;  // 灰色：未知
            }
        }

        // 最终 GUI 上口示意圆只表达结果栏的规范 rTop。
        // 真实物理孔口交线可能因支撑面倾斜/圆角入口表现为椭圆或半径随角度变化，
        // 它继续保存在 HoleMiaoshu 作为内部几何证据，但不替代主示意圆，避免数值与图形语义不一致。
        std::vector<vtkIdType> ringTop;
        ringTop.reserve(N + 1);
        for (int i = 0; i <= N; ++i) {
            const double a = 2.0 * M_PI * (double)(i % N) / (double)N;
            Eigen::Vector3f rp = cTop
                + topU * static_cast<float>(rTop * std::cos(a))
                + topV * static_cast<float>(rTop * std::sin(a));
            ringTop.push_back(addPoint(
                rp.x(), rp.y(), rp.z() + zLift, rt, gt, bt));
        }
        addPolyline(ringTop);

        // 中心十字线（小十字，不再使用重叠高亮点）
        const double cr = std::max(1.2, std::min(3.0, (double)rTop * 0.25));
        Eigen::Vector3f pUx0 = cTop - topU * static_cast<float>(cr);
        Eigen::Vector3f pUx1 = cTop + topU * static_cast<float>(cr);
        Eigen::Vector3f pVy0 = cTop - topV * static_cast<float>(cr);
        Eigen::Vector3f pVy1 = cTop + topV * static_cast<float>(cr);
        addPolyline({
            addPoint(pUx0.x(), pUx0.y(), pUx0.z() + zLift, 255, 220, 0),
            addPoint(pUx1.x(), pUx1.y(), pUx1.z() + zLift, 255, 220, 0)
        });
        addPolyline({
            addPoint(pVy0.x(), pVy0.y(), pVy0.z() + zLift, 255, 220, 0),
            addPoint(pVy1.x(), pVy1.y(), pVy1.z() + zLift, 255, 220, 0)
        });

        // 轴线方向必须与参数栏显示的最终 holeAxisIn 完全一致。
        // centerBot 只用于提供可视长度，不再用 (centerBot-centerTop) 直接决定显示方向，
        // 避免下口中心存在少量横向拟合偏差时把孔轴线画歪。
        Eigen::Vector3f cEnd = cTop;
        const bool centerBotAxisUsable = validCenter(d.centerBot)
            && (d.centerBot - cTop).norm() > 0.3f;
        float axisDisplayLen = 0.0f;
        if (d.depth > 0.0f && std::isfinite(d.depth)) {
            axisDisplayLen = d.depth;
        } else if (centerBotAxisUsable) {
            axisDisplayLen = (d.centerBot - cTop).norm();
        } else if (d.wallDownTrackValid && d.wallDownTrackDepthMm > 0.5f) {
            axisDisplayLen = d.wallDownTrackDepthMm;
        } else if (polarityKnown || d.holeAxisInValid) {
            axisDisplayLen = std::max(2.0f, rTop * 0.5f);
        }
        const bool drawAxis = axisDisplayLen > 0.0f && std::isfinite(axisDisplayLen);
        if (drawAxis)
            cEnd = cTop + finalAxis * axisDisplayLen;
        if (drawAxis) {
            addPolyline({
                addPoint(cTop.x(), cTop.y(), cTop.z() + zLift, 0, 220, 255),
                addPoint(cEnd.x(), cEnd.y(), cEnd.z() + zLift, 0, 220, 255)
            });
        }

        // 显示孔型与下口几何分开处理，避免不可观测下口影响孔型显示
        // isDisplayCone由physicalHoleTypeDisplay决定，不用type决定
        // bottom geometry从真实数据(rBot/centerBot/wallDown)提取
        // 下口示意必须与参数栏/CSV使用同一套“最终测量值”语义。
        // 最终复核可能只更新 rBotProfile；若这里仍优先旧 rBot，会出现
        // “数值是最终下口半径、图形却仍画旧半径”的单孔显示错位（4.1锥孔已复现）。
        float rBotDraw = 0.0f;
        if (d.reliableBottomRadius && d.rBotProfile > 0.0f && std::isfinite(d.rBotProfile))
            rBotDraw = d.rBotProfile;
        else if (d.reliableBottomRadius && d.rBot > 0.0f && std::isfinite(d.rBot))
            rBotDraw = d.rBot;
        bool drawLowerRing = false;
        Eigen::Vector3f cBotDraw = (validCenter(d.centerBot)) ? d.centerBot : cTop;

        bool isDisplayCone = (d.physicalHoleTypeDisplay
            == HoleMiaoshu::WuLiHoleLeixingXianshi::Cone)
            || d.type == 2
            || d.finalGeometryType == HoleMiaoshu::FinalJiheType::Cone;
        if (isDisplayCone) {
            if (rBotDraw > 0.0f && validCenter(d.centerBot)
                && rBotDraw < rTop * 0.95f) {
                drawLowerRing = true;
            } else if (polarityKnown && d.depth > 0.3f && std::isfinite(d.depth)
                       && d.slopeDeg > 1.0f && std::isfinite(d.slopeDeg)) {
                const float inferred = rTop - d.depth * std::tan(
                    d.slopeDeg * static_cast<float>(M_PI) / 180.0f);
                if (inferred > 0.05f && inferred < rTop * 0.95f) {
                    rBotDraw = inferred;
                    cBotDraw = cTop + finalAxis * d.depth;
                    drawLowerRing = true;
                }
            } else if (polarityKnown && d.wallDownTrackValid
                       && d.wallDownTrackDepthMm > 1.5f
                       && d.wallDownCombinedClass == "CONE_WALL_TRACKED") {
                rBotDraw = rTop * 0.65f;
                cBotDraw = cTop + finalAxis * d.wallDownTrackDepthMm;
                drawLowerRing = true;
            }
        }

        if (drawLowerRing) {
            std::vector<vtkIdType> ringBot;
            ringBot.reserve(N + 1);
            for (int i = 0; i <= N; ++i) {
                const double a = 2.0 * M_PI * (double)(i % N) / (double)N;
                Eigen::Vector3f rp = cBotDraw
                    + bottomU * static_cast<float>(rBotDraw * std::cos(a))
                    + bottomV * static_cast<float>(rBotDraw * std::sin(a));
                ringBot.push_back(addPoint(
                    rp.x(),
                    rp.y(),
                    rp.z() + zLift,
                    0, 220, 80));
            }
            addPolyline(ringBot);
        }
    }

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetLines(lines);
    poly->GetPointData()->SetScalars(colors);
    poly->GetPointData()->SetActiveScalars("rgb");
    return poly;
}
} // namespace

vtkSmartPointer<vtkPolyData> buildLineGeometry(
    const std::vector<HoleMiaoshu>& holes, int selectedIndex)
{
    return buildLineGeometryImpl(holes, selectedIndex, 0.02, false);
}

vtkSmartPointer<vtkPolyData> buildLineGeometryForPreview(
    const std::vector<HoleMiaoshu>& holes, int selectedIndex)
{
    return buildLineGeometryImpl(holes, selectedIndex, 0.0, true);
}

/** 【函数导航】
 * 作用：应用/设置“applyHoleDieJia”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云显示实现。
 * 主要引用/调用位置：DianYunXianshi_View.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void applyHoleDieJia(dianYunView::DianYunXuanRanQi& renderer,
                      const std::vector<HoleMiaoshu>& holes,
                      int selectedIndex)
{
    const auto sel = holeXianshi::selectDisplaySet(holes, selectedIndex);
    std::vector<HoleMiaoshu> renderHoles;
    renderHoles.reserve(sel.displayIndices.size());
    for (int ci : sel.displayIndices) renderHoles.push_back(holes[ci]);

    renderer.setDieJiaDianYunNoRefresh(nullptr);
    const auto polyL = buildLineGeometry(renderHoles, sel.selectedPosition);
    if (polyL && polyL->GetNumberOfPoints() > 0) {
        renderer.setOverlayLinePolyData(polyL);
        renderer.setOverlayLineOptions(false, 1.0, 0.0, 0.0, 1.0,
                                       selectedIndex >= 0 ? 6.0 : 4.0);
    } else {
        renderer.clearOverlayLines();
    }
    renderer.refreshDieJiaOnly();
}

}


