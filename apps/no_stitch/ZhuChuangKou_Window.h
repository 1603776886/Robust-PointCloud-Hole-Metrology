/*
================================================================================
文件：ZhuChuangKou_Window.h
模块：主窗口接口

【主要职责】
声明 GUI 状态、槽函数、异步识别入口和结果面板状态。

【主要调用关系】
main.cpp 创建；Qt 信号/按钮/菜单调用其槽函数。

【线程与状态】
GUI 主线程保存状态；后台识别通过 Qt 全局 QThreadPool 执行，完成后把结果安全回送 GUI 主线程。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
主窗口负责 GUI 流程编排：接收用户操作，把点云交给读写、显示、降噪和孔识别模块，再把结果展示或导出。
主窗口本身不实现孔几何算法，也不保存第二套点云业务状态。

主要调用位置：
main.cpp 创建 ZhuChuangKouWindow；菜单、按钮和 VTK 点选事件进入各个槽函数。
孔参数导出由 onShuchuHoleCanshu() 调用独立输出接口。

维护说明：
新增功能优先放入现有功能模块，ZhuChuangKouWindow 只保留信号连接、输入收集和结果展示。
影响孔半径、深度、孔型或法向的参数必须放回对应算法模块，并在参数旁写清单位和调整影响。
*/
#pragma once

#include <QMainWindow>
#include <QDialog>
#include <QString>
#include <QLabel>
#include <QVector>
#include <functional>
#include <vector>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "DianYunJichu_Core.h"
#include "HoleShibie_Recognition.h"
#include "DianYunXianshi_View.h"

class QAction;
class QDockWidget;
class QToolButton;
class QSlider;
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QPushButton;
class QStackedWidget;
class QTimer;
class QThreadPool;
class QVTKOpenGLNativeWidget;
class QEvent;
class GaoduColorBarWidget;
class DianYunHuiHua;

// ============================================================================
// ZhuChuangKouWindow：主界面、单点云操作、孔识别触发与结果展示
// ============================================================================
namespace Ui { class ZhuChuangKouWindow; }

class ZhuChuangKouWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit ZhuChuangKouWindow(QWidget* parent = nullptr);
    ~ZhuChuangKouWindow();

private slots:
    void onOpenDianYunFile();
    void onSavePcd();
    void onDianYunJiangZao();
    void onHoleShibie();
    void undo();
    void updateStatusBar();
    void onFirstShow();
    void onShuchuHoleCanshu();
    void onHolePrev();
    void onHoleNext();
    void onXyzPingYi();
    void onFanZhuanX();
    void onFanZhuanY();
    void onDianYunShuiPing();
    // 主窗口到渲染器的唯一刷新桥接：从 DianYunHuiHua 取当前点云并交给渲染器。
    void refreshDianYunView();

signals:
    void dianYunChanged();

private:
    Ui::ZhuChuangKouWindow* ui;
    QLabel* m_labelPicked = nullptr;

    // 点击即选孔：左键按点击顺序加入 seed，右键短按删除最近 seed；编号始终由 vector 顺序连续生成。
    std::vector<Eigen::Vector3f> m_sdHoleSeed;
    QTimer* m_autoHoleRecognitionTimer = nullptr;
    QThreadPool* m_holeRecognitionPool = nullptr; // 单线程常驻：连续识别复用 recognition 内 thread_local 热缓存
    bool m_holeRecognitionRunning = false;
    bool m_autoRecognitionPending = false;
    quint64 m_seedRevision = 0;

    // 孔结果面板和覆盖层管理：负责显示当前识别结果，不参与几何判定。
    void ensureHolePanel();
    void clearHoleView(bool refreshRenderer = true);
    void setHoleShibieResult(HoleShibieResult res);
    void showHoleInfo(int index);
    QString holeShuchuSuggestedPath(const QString& fileName) const;
    void jiLuHoleShuchuDirectory(const QString& fileName);
    void showHoleAt(int index);
    void shuaXinSdHoleSeedBiaoJi();
    void scheduleAutoHoleShibie();
    void updateHoleLocalPreview(int index);
    void clearHoleLocalPreview();
    int findNearestDisplayedHoleIndex(const Eigen::Vector3f& worldPt, float maxDistMm = -1.0f) const;
    void updateGaoduColorBar(bool resetToDefaults);

    QWidget* m_holePanel = nullptr;
    QLabel* m_holeLabelTitle = nullptr;
    QLabel* m_holeLabelInfo = nullptr;
    QGroupBox* m_holeParamGroup = nullptr;
    QGroupBox* m_holePreviewGroup = nullptr;
    QLabel* m_lblHolePreviewInfo = nullptr;
    QVTKOpenGLNativeWidget* m_holePreviewWidget = nullptr;
    std::unique_ptr<dianYunView::DianYunXuanRanQi> m_holePreviewRenderer;
    QPushButton* m_btnExportHoleParameters = nullptr;
    QLabel* m_lblHoleTypeVal = nullptr;
    QLabel* m_lblCenterXVal = nullptr;
    QLabel* m_lblRadiusTopVal = nullptr;
    QLabel* m_lblRadiusBotVal = nullptr;
    QLabel* m_lblDepthVal = nullptr;
    QLabel* m_lblAngleVal = nullptr;
    QLabel* m_lblNormalVal = nullptr;
    // 孔型感知参数面板：根据直孔/锥孔自动切换字段标签与可测状态
    QLabel* m_lblRadiusTopKey = nullptr;    // 根据孔形显示“半径”“上口半径”或“粗半径”
    QLabel* m_lblRadiusBotKey = nullptr;    // 锥孔专属行标签
    QLabel* m_lblAngleKey = nullptr;        // 锥孔专属行标签
    QLabel* m_lblCenterXKey = nullptr;      // 根据孔形显示“中心X”或“上口中心X”
    QLabel* m_lblCenterBotKey = nullptr;    // 下口中心标签，仅锥孔显示
    QLabel* m_lblCenterBotVal = nullptr;    // 下口三维坐标合并显示
    QToolButton* m_btnHolePrev = nullptr;
    QToolButton* m_btnHoleNext = nullptr;
    HoleShibieResult m_lastHoleResult;
    
    int m_holeIndex = -1;
    std::vector<HoleMiaoshu> m_normalDisplayCandidates;  // 规范生产孔列表；显示、点击定位和导出共同使用这一份数据。
    std::vector<int> m_normalDisplaySeedNumbers;      // 与上方列表一一对应的点击编号（1-based），便于结果面板保持选孔顺序语义。
    int m_preferredHoleSeedNumber = -1;               // 自动识别完成后优先切到这个点击编号对应的孔。


    std::string m_cloudName;
    // Hole 参数导出的最近目录。
    QString m_holeShuchuDirectory;

    void runHoleShibieAsync(const std::function<HoleShibieResult()>& runFn,
                               const QString& runningText,
                               const QString& donePrefix,
                               quint64 seedRevision,
                               std::vector<Eigen::Vector3f> seedSnapshot);

    std::unique_ptr<DianYunHuiHua> m_session;
    std::unique_ptr<dianYunView::DianYunXuanRanQi> m_renderer;
    GaoduColorBarWidget* m_heightBar = nullptr;

    // PCD 孔数据嵌入：孔参数作为 PCD 头注释保存/加载
    bool m_hasEmbeddedHoles = false;
    HoleShibieResult m_embeddedHoleResult;
    QAction* m_actSaveWithHoles = nullptr;
    QAction* m_actShowHoleDock = nullptr;

    // 孔参数嵌入 PCD：从文件头注释读 / 写到文件头注释。
    void clearEmbeddedHoleData();
    void jiaZaiPcdHoleCanshu(const QString& filePath);
    void baoCunDaiHole(const QString& fileName);
};
