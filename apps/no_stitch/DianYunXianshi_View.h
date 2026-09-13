/*
================================================================================
文件：DianYunXianshi_View.h
模块：点云显示接口

【主要职责】
声明 VTK/PCL 渲染器、点选、overlay、颜色映射与显示统计类型。

【主要调用关系】
主窗口创建 DianYunXuanRanQi 并通过其接口刷新基础点云和 Hole 叠加。

【线程与状态】
VTK/Qt 对象只在 GUI 主线程操作。

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
点云可视化数据结构与 VTK/PCL 场景渲染接口。

主要调用位置：
ZhuChuangKou_Window.cpp 创建 dianYunView::DianYunXuanRanQi 和 GaoduColorBarWidget；
孔结果显示适配也复用本文件中的渲染接口。

维护说明：
本文件按功能合并相关子模块。头文件必须自包含：凡是在类声明中直接使用的 Qt/VTK/PCL 类型，
必须在第一次使用前完成包含或前置声明，不能依赖某个 .cpp 或其他头文件“碰巧先包含”。
*/

#include "DianYunLeixing_Types.h"

#include <QWidget>
#include <QColor>
#include <QPoint>
#include <QRect>
#include <vtkSmartPointer.h>
#include <Eigen/Core>

#include <array>
#include <functional>
#include <utility>
#include <vector>

class QObject;
class QEvent;
class QPaintEvent;
class QMouseEvent;
class QResizeEvent;
class QDoubleSpinBox;
class QPushButton;
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;
class vtkOrientationMarkerWidget;
class vtkAxesActor;
class vtkAreaPicker;
class vtkCellPicker;
class vtkCallbackCommand;
class vtkTransform;
class vtkInteractorStyleTrackballCamera;
class vtkActor;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkColorTransferFunction;
class vtkBillboardTextActor3D;

// ============================================================================
// 功能分区：可视化数据类型
// ============================================================================
/*
模块职责：
集中定义 VTK 点云显示的颜色映射、底图/覆盖层样式和渲染统计数据。这里只描述“怎么显示”，不拥有点云，也不参与孔识别。

主要调用位置：
DianYunXuanRanQi 持有并应用这些设置；ZhuChuangKouWindow 只在用户切换显示或突出孔结果时调整选项。

维护说明：
透明度范围为 0~1，点大小使用 VTK 屏幕点大小语义。修改这些默认值只应改变视觉效果；如果出现识别结果变化，说明模块边界被破坏。
*/

namespace dianYunView {

// 高度颜色映射中的一个控制点。
struct GaoduColorStop
{
    double z = 0.0;          // 点云数据坐标中的 Z；单位沿用输入点云，不在显示层强制假定为毫米
    double r = 0.0, g = 0.0, b = 0.0;
    bool positionLocked = false;  // 端点是否锁定在当前有效高度范围的最小值或最大值
};

// 仅用于显示的高度颜色设置，不允许修改任何孔识别结果。
struct GaoduColorMapSettings
{
    bool enabled = true;              // 是否启用按高度着色；关闭只影响显示，不改变点坐标。
    int stopCount = 4;                 // 当前界面允许 3 或 4 个颜色控制点。
    double zMin = 0.0;                 // 颜色映射的最低高度；通常由当前点云范围刷新。
    double zMax = 1.0;                 // 颜色映射的最高高度；必须大于 zMin 才有有效渐变范围。
    std::vector<GaoduColorStop> stops;
};

struct DianYunViewOptions
{
    double baseOpacityFaded = 0.38;    // 突出孔结果时底图透明度；越小孔覆盖层越醒目。
    double baseOpacity = 1.0;         // 正常底图透明度 0~1，与颜色模式独立。
    double basePointSize = 2.0;       // 底图点大小；增大会更易观察，也会增加遮挡。
    double overlayPointSize = 14.0;   // 选点/孔标记的点大小；只影响可见性。
    double overlayOpacity = 1.0;      // 覆盖层透明度 0~1。
    bool overlayRenderAsSpheres = true; // true 时覆盖点按球形点渲染，视觉更明显但可能增加显示开销。
    bool overlayForceColor = true;    // 是否强制使用下方 RGB 颜色覆盖点云自身颜色。
    double overlayColorR = 1.0;       // 覆盖层红色分量 0~1。
    double overlayColorG = 0.0;       // 覆盖层绿色分量 0~1。
    double overlayColorB = 0.0;       // 覆盖层蓝色分量 0~1。
};


}

// ============================================================================
// 功能分区：点云场景渲染
// ============================================================================
/*
模块职责：
封装主三维场景的 VTK 对象、点云显示、孔覆盖层、拾取、旋转和高度颜色映射。
识别算法只产生数据，本模块只把数据画出来，不参与孔型、半径、深度和法向判定。

主要调用位置：
ZhuChuangKouWindow 创建并持有 DianYunXuanRanQi；DianYunHuiHua 的当前点云通过 refreshDianYunView() 送入本模块。

维护说明：
以下设置只允许影响显示：
setBaseFade() 的透明度、DianYunViewOptions 的点大小和 GaoduColorMapSettings 只影响显示；
这些参数不能被识别算法读取，避免“显示设置改变识别结果”。
*/


namespace dianYunView {

class XuanZhuanInteractorStyle;

class DianYunXuanRanQi
{
public:
    explicit DianYunXuanRanQi(QVTKOpenGLNativeWidget* widget);
    ~DianYunXuanRanQi();
    DianYunXuanRanQi(const DianYunXuanRanQi&) = delete;
    DianYunXuanRanQi& operator=(const DianYunXuanRanQi&) = delete;

    // 设置当前显示点云，使用共享所有权，不做整云深拷贝。
    void setDianYun(const CloudConstPtr& cloud);
    /** 【函数导航】
     * 作用：执行“cloud”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    CloudConstPtr cloud() const { return m_cloud; }

    // 完整刷新：仅在主点云本体发生变化时重建底图。
    void refresh();
    // 轻量刷新：主点云不变时只重建 overlay 并 Render，不做整云 PCL->VTK 转换。
    void refreshDieJiaOnly();

    // 新点云打开后显式重置相机，使整片点云进入初始视野。
    void resetCamera();
    // 单孔微观检查默认视角：以最终入孔轴回正，再绕孔局部 X 轴倾斜指定角度；
    // 使用正交投影并按当前局部点云 bounds 自动取景，保留坐标轴且允许后续手动旋转。
    void setHoleInspectionView(const Eigen::Vector3f& centerTop,
                               const Eigen::Vector3f& inwardAxis,
                               double tiltDeg = 18.0);

    // 点标记覆盖层：把标记点转换成十字/圆环等显示几何。
    void setDieJiaDianYun(const CloudConstPtr& cloud);
    void setDieJiaDianYunNoRefresh(const CloudConstPtr& cloud);
    void clearDieJiaDianYun();
    void clearDieJiaDianYunNoRefresh();

    // 通用线覆盖层：孔显示适配模块负责构造几何，本类只负责绘制。
    void setOverlayLinePolyData(const vtkSmartPointer<vtkPolyData>& poly);
    void setOverlayLineOptions(bool forceColor, double r, double g, double b,
                               double opacity, double lineWidth);
    void clearOverlayLines();

    void setBaseFade(bool enable, double opacity = 0.18);
    void setBaseFadeNoRefresh(bool enable, double opacity = 0.18);
    /** 【函数导航】
     * 作用：执行“baseFade”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool baseFade() const { return m_fadeBase; }
    // 底图真实透明度，与高度着色是否开启相互独立。
    void setBaseOpacity(double opacity);
    /** 【函数导航】
     * 作用：执行“baseOpacity”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    double baseOpacity() const { return m_baseOpacity; }

    // 可调 Z 高度颜色映射；交互修改只更新颜色映射表，不重建点数组。
    void setHeightColorMap(const GaoduColorMapSettings& s);
    /** 【函数导航】
     * 作用：执行“heightColorMap”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    GaoduColorMapSettings heightColorMap() const { return m_heightMap; }
    void resetHeightColorMap();
    void setHeightColoringEnabled(bool enabled);
    // 局部孔预览可把颜色标量改为“沿指定轴的投影高度”，几何坐标本身不变。
    // 典型用法：origin=上口中心，axis=-holeAxisIn，使上口偏红、向孔内逐渐偏蓝，
    // 避免原始零件整体倾斜时按世界 Z 上色造成误导性的斜向色带。
    void setHeightScalarProjection(const Eigen::Vector3f& origin, const Eigen::Vector3f& axis);
    void clearHeightScalarProjection();
    /** 【函数导航】
     * 作用：执行“heightColoringEnabled”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool heightColoringEnabled() const { return m_heightMap.enabled; }
    /** 【函数导航】
     * 作用：执行“validHeightRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    std::pair<double, double> validHeightRange() const { return { m_validZMin, m_validZMax }; }
    /** 【函数导航】
     * 作用：执行“hasValidHeightRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool hasValidHeightRange() const { return m_hasValidHeight; }

    void setBackgroundColor(double r, double g, double b);
    void getBackgroundColor(double& r, double& g, double& b) const;

    void setFadePointColor(double r, double g, double b);
    void getFadePointColor(double& r, double& g, double& b) const;

    void setDianYunViewOptions(const DianYunViewOptions& opt);
    DianYunViewOptions cloudVizOptions() const;

    void setPointPickCallback(std::function<void(double, double, double)> cb);
    // 右键短按拾取：与右键拖动缩放并存，只有移动量很小时才触发。
    void setRightPointPickCallback(std::function<void(double, double, double)> cb);
    // 手动 seed 的连续编号标签；位置使用原始点云世界坐标，并跟随场景旋转。
    void setSeedNumberLabels(const std::vector<Eigen::Vector3f>& positions);
    void clearSeedNumberLabels();
    // 小窗口可启用屏幕坐标旋转：水平拖动始终绕屏幕竖直轴，垂直拖动始终绕屏幕水平轴。
    void setScreenSpaceRotationEnabled(bool enabled) { m_screenSpaceRotation = enabled; }
    // 鼠标左键释放时若判定为旋转手势，则通知上层回滚按下瞬间产生的拾取效果，用来区分“单击”和“拖拽旋转”。
    void setPickCancelledCallback(std::function<void()> cb);


private:
    friend class XuanZhuanInteractorStyle;

    void renderWindowRender();
    void applyViewRotationToActors();
    void rebuildHeightLut();
    void applyHeightScalarToMapper(vtkPolyDataMapper* mapper);
    void onLeftPress();
    void onMouseMove();
    void onLeftRelease();
    void onRightPress();
    void onRightMouseMove();
    void onRightRelease();
    void rebuildSeedNumberLabels();

    QVTKOpenGLNativeWidget* m_w = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_win;
    vtkSmartPointer<vtkRenderer> m_ren;
    vtkSmartPointer<vtkRenderer> m_renOverlay;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_om;
    vtkSmartPointer<vtkAxesActor> m_orientationAxesActor; // 左下角方向坐标轴；跟随场景手动旋转
    vtkSmartPointer<vtkAxesActor> m_axesActor;
    vtkSmartPointer<vtkAreaPicker> m_areaPicker;
    vtkSmartPointer<vtkCellPicker> m_cellPicker;
    vtkSmartPointer<vtkCellPicker> m_rightCellPicker;
    vtkSmartPointer<vtkCallbackCommand> m_pickCbCmd;
    vtkSmartPointer<XuanZhuanInteractorStyle> m_styleTrackball;
    vtkSmartPointer<vtkActor> m_cloudActor;
    vtkSmartPointer<vtkActor> m_overlayActor;
    vtkSmartPointer<vtkActor> m_overlayLineActor;
    std::vector<vtkSmartPointer<vtkBillboardTextActor3D>> m_seedLabelActors;
    std::vector<Eigen::Vector3f> m_seedLabelPositions;

    CloudConstPtr m_cloud;
    CloudConstPtr m_overlayCloud;
    vtkSmartPointer<vtkPolyData> m_overlayLines;
    bool m_hasLineOverlay = false;
    bool m_lineForceColor = true;
    double m_lineR = 1.0, m_lineG = 0.0, m_lineB = 0.0;
    double m_lineOpacity = 1.0;
    double m_lineWidth = 4.0;

    bool m_hasEverResetCamera = false;
    bool m_mousePressed = false;
    // 场景级“点击点旋转”：相机不动，模型绕点击点旋转（视图中心不移动）。
    vtkSmartPointer<vtkTransform> m_viewRot;        // 累计场景旋转（原始坐标->显示坐标）
    vtkSmartPointer<vtkTransform> m_viewRotAtPress; // 按下时的旋转，用于 <5° 单击回滚
    bool m_rotating = false;
    bool m_screenSpaceRotation = false;
    bool m_rotatePivotValid = false;
    double m_rotatePivot[3] = { 0, 0, 0 };
    double m_cloudCenter[3] = { 0, 0, 0 };
    double m_rotateAccumDeg = 0;
    int m_rotateLastX = 0;
    int m_rotateLastY = 0;
    int m_pressX = 0;
    int m_pressY = 0;
    bool m_rightPressed = false;
    int m_rightPressX = 0;
    int m_rightPressY = 0;
    double m_rightMovePixels = 0.0;
    bool m_fadeBase = false;
    double m_fadeOpacity = 0.18;
    double m_baseOpacity = 1.0;
    GaoduColorMapSettings m_heightMap;
    vtkSmartPointer<vtkColorTransferFunction> m_heightLut;
    double m_validZMin = 0.0;
    double m_validZMax = 1.0;
    bool m_hasValidHeight = false;
    bool m_useProjectedHeightScalar = false;
    Eigen::Vector3f m_heightScalarOrigin = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_heightScalarAxis = Eigen::Vector3f::UnitZ();
    DianYunViewOptions m_opt;
    double m_bgR = 0.0, m_bgG = 0.0, m_bgB = 0.0;
    double m_fadePtR = 1.0, m_fadePtG = 1.0, m_fadePtB = 0.0;
    std::function<void(double, double, double)> m_pickCb;
    std::function<void(double, double, double)> m_rightPickCb;
    std::function<void()> m_pickCancelledCb;
};

}

// ============================================================================
// 高度色带控件：根据当前高度映射显示颜色刻度。
// ============================================================================
/*
模块职责：
高度色标界面控件。

主要调用位置：
由主窗口/点云显示模块创建，用于展示和调整高度着色范围。

维护说明：
颜色范围只影响显示；不得借此修改点云几何或孔识别输入。
*/

class GaoduColorBarWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GaoduColorBarWidget(QWidget* parent = nullptr);

    void setRange(double zMin, double zMax, bool resetToDefaults);
    void setRangeValid(bool valid);
    /** 【函数导航】
     * 作用：执行“hasValidRange”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool hasValidRange() const { return m_rangeValid; }

    void setChangJingBackgroundColor(const QColor& c);
    /** 【函数导航】
     * 作用：执行“foregroundColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    QColor foregroundColor() const { return m_fg; }
    /** 【函数导航】
     * 作用：执行“sceneBackgroundColor”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    QColor sceneBackgroundColor() const { return m_sceneBg; }

    /** 【函数导航】
     * 作用：应用/设置“settings”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云显示接口。
     * 主要引用/调用位置：DianYunXianshi_View.h（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    dianYunView::GaoduColorMapSettings settings() const { return m_settings; }
    void setSettings(const dianYunView::GaoduColorMapSettings& s);

signals:
    void settingsChanged(const dianYunView::GaoduColorMapSettings& s);

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;
    void resizeEvent(QResizeEvent*) override;

private:
    /** 【类型导航注释】
     * Node：点云显示接口中的自定义 结构体。
     * 主要使用位置：DianYunXianshi_View.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct Node {
        double z = 0.0;
        QColor color;
        bool locked = false;
    };

    void layoutGeometry();
    void rebuildNodes();
    void applySettings(const dianYunView::GaoduColorMapSettings& s);
    void emitChanged();
    void makeEqualSpacing();
    void makeTopFocus(double dz);
    void makeBottomFocus(double dz);
    void setStopCount(int n);
    void openColorDialog(int nodeIndex);
    double yToZ(int y) const;
    int zToY(double z) const;
    int hitNode(const QPoint& pos) const;
    void layoutControls();

    dianYunView::GaoduColorMapSettings m_settings;
    std::vector<Node> m_nodes;
    bool m_rangeValid = false;
    QColor m_sceneBg = QColor(0, 0, 0);
    QColor m_fg = QColor(255, 255, 255);
    int m_dragIndex = -1;
    int m_selectedIndex = -1;
    QRect m_gradientRect;
    QRect m_controlsRect;
    QDoubleSpinBox* m_dzSpin = nullptr;
    QPushButton* m_btn3 = nullptr;
    QPushButton* m_btn4 = nullptr;
    QPushButton* m_btnEqual = nullptr;
    QPushButton* m_btnTop = nullptr;
    QPushButton* m_btnBottom = nullptr;
};


// ============================================================================
// 功能分区：孔结果显示与列表选择
// ============================================================================
/*
模块职责：
把孔识别结果转换成主界面列表、最近孔选择和 VTK 覆盖几何。与通用点云显示放在同一模块，避免显示逻辑分散。

主要调用位置：
ZhuChuangKouWindow 在刷新孔列表、定位孔和绘制孔口/孔轴时调用。

维护说明：
这里允许改变“怎么显示”，不允许改 HoleMiaoshu 内的识别数值；导出和列表筛选必须复用同一规范孔选择规则。
*/
/*
模块职责：
孔识别结果的显示策略和几何叠加绘制。

维护说明：
本文件按职责合并相互紧密的子模块。维护时请按中文功能分区定位逻辑；同一职责优先在现有分区内扩展，避免把连续算法拆成过细文件。
*/

// ============================================================================
// 功能分区：孔显示策略
// ============================================================================
/*
模块职责：
根据 HoleMiaoshu 决定哪些结果属于生产界面应显示的规范孔列表，并提供最近孔等纯选择逻辑。
本模块不创建 VTK 对象、不修改孔参数，也不依赖 ZhuChuangKouWindow。

主要调用位置：
ZhuChuangKouWindow 设置识别结果、点击定位和孔参数导出时都使用同一套规范孔选择规则，保证“看到的孔”和“导出的孔”一致。

维护说明：
这里的筛选条件属于生产显示语义。新增导出或列表功能必须复用本模块，不能各自复制一套孔过滤条件。
*/
#include "HoleLeixing_Types.h"

namespace holeXianshi {

// 判断一个孔是否属于生产界面正常显示范围。
bool isNormalXianshiHole(const HoleMiaoshu& d);

// 判断候选是否有足够的真实几何解释，避免仅靠标签把候选对象显示成正式孔。
bool hasRealHoleExplanation(const HoleMiaoshu& d);

// 从原始 descriptors 中选出规范生产孔，返回原数组索引，便于调用方保持数据来源可追溯。
std::vector<int> selectGuiFanNormalXianshiHouXuan(
    const std::vector<HoleMiaoshu>& candidates);

// 规范孔列表加“当前选中孔”映射后的结果。
/** 【类型导航注释】
 * XianshiSelection：点云显示接口中的自定义 结构体。
 * 主要使用位置：DianYunXianshi_View.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct XianshiSelection {
    std::vector<int> displayIndices;  // 输入孔数组中的索引，顺序与规范显示列表一致。
    int selectedPosition = -1;        // 选中孔在 displayIndices 中的位置；未命中时为 -1。
};

// 按规范显示规则生成界面使用的列表，并把原始 selectedIndex 映射到规范列表位置。
XianshiSelection selectDisplaySet(const std::vector<HoleMiaoshu>& holes,
                                  int selectedIndex);

// 在给定孔列表中寻找最靠近点击位置的孔；maxDistanceMm 限制最大可接受距离。
int findNearestHoleIndex(const std::vector<HoleMiaoshu>& holes,
                         const Eigen::Vector3f& clickWorld,
                         float maxDistanceMm = 20.0f);

}

// ============================================================================
// 功能分区：孔叠加几何接口
// ============================================================================
/*
模块职责：
把孔结果转换成通用的点/线覆盖几何，再交给 DianYunXuanRanQi 绘制。
它是孔语义与通用渲染器之间的适配层，不重新计算孔半径、孔型、深度或法向。

主要调用位置：
ZhuChuangKouWindow 显示全部孔或高亮单孔时调用本模块。

维护说明：
OverlayStyle 只允许调整显示颜色、透明度和线宽；任何识别半径或深度修正都不能放在本模块。
*/


namespace holeDieJia {

// 通过 DianYunXuanRanQi 应用的通用线覆盖样式。
/** 【类型导航注释】
 * OverlayStyle：点云显示接口中的自定义 结构体。
 * 主要使用位置：DianYunXianshi_View.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct OverlayStyle {
    bool forceColor = false;  // true 时强制所有孔使用下面的固定颜色；只影响显示。
    double r = 1.0, g = 0.0, b = 0.0;  // RGB 分量，范围 0~1；默认红色。
    double opacity = 1.0;     // 线框透明度，范围 0~1；降低只让覆盖层更透明。
    double lineWidth = 4.0;   // VTK 线宽，单位为屏幕像素附近的显示尺度；不改变真实孔尺寸。
};

// 根据显示孔列表构造圆环、孔轴和十字标记的线几何。
// 保持既有显示语义，不在这里重新计算识别结果。
vtkSmartPointer<vtkPolyData> buildLineGeometry(
    const std::vector<HoleMiaoshu>& holes, int selectedIndex = -1);
// 局部孔预览专用：几何不再做世界 Z 抬升，避免侧视时与真实点云看起来错位。
vtkSmartPointer<vtkPolyData> buildLineGeometryForPreview(
    const std::vector<HoleMiaoshu>& holes, int selectedIndex = -1);

// 显示桥接：规范孔选择 → 通用线几何 → 当前渲染器。
// 整个过程只改变显示，不改变 HoleMiaoshu。
void applyHoleDieJia(dianYunView::DianYunXuanRanQi& renderer,
                      const std::vector<HoleMiaoshu>& holes,
                      int selectedIndex = -1);

}
