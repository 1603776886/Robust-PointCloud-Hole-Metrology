/*
================================================================================
文件：main.cpp
模块：程序入口

【主要职责】
创建 QApplication 和主窗口对象，完成应用级初始化后进入 Qt 事件循环。

【主要调用关系】
由操作系统启动；下游只创建 ZhuChuangKouWindow。

【线程与状态】
GUI 主线程。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
程序唯一入口。初始化 Qt/VTK 图形环境并创建主窗口，最终发布版只保留图形界面生产入口。

主要调用位置：
操作系统启动程序后进入 main()，随后所有业务都由 ZhuChuangKouWindow 的菜单、按钮和点选事件驱动。

维护说明：
这里仅允许放应用生命周期和图形平台初始化。孔识别、点云读写和降噪逻辑必须留在对应模块。
*/
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include "ZhuChuangKou_Window.h"

#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#if _WIN32
#pragma execution_character_set("utf-8")
#endif

namespace {
/** 【函数导航】
 * 作用：执行“chuShiHuaTuXingHuanJing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：程序入口。
 * 主要引用/调用位置：main.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void chuShiHuaTuXingHuanJing()
{
    if (qEnvironmentVariableIsEmpty("QT_OPENGL"))
        qputenv("QT_OPENGL", QByteArray("desktop"));

    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
#endif

    QSurfaceFormat format = QVTKOpenGLNativeWidget::defaultFormat();
    format.setSamples(0);
    QSurfaceFormat::setDefaultFormat(format);
}
}

/** 【函数导航】
 * 作用：执行“main”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：程序入口。
 * 主要引用/调用位置：main.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
int main(int argc, char* argv[])
{
    chuShiHuaTuXingHuanJing();
    QApplication app(argc, argv);
    ZhuChuangKouWindow window;
    window.show();
    return app.exec();
}
