/*
================================================================================
文件：DianYun_IO.h
模块：点云文件接口

【主要职责】
声明 PCD/PLY/CSV 读取保存以及 PCD 内嵌 Hole 参数读写。

【主要调用关系】
主窗口文件操作调用；不弹文件对话框、不拥有当前点云。

【线程与状态】
同步 I/O；调用者决定线程。

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
PCD/PLY/CSV 等点云读写，以及 PCD 内嵌孔参数读写。

维护说明：
本文件按职责合并相互紧密的子模块。维护时请按中文功能分区定位逻辑；同一职责优先在现有分区内扩展，避免把连续算法拆成过细文件。
*/

// ============================================================================
// 功能分区：通用点云文件读写接口
// ============================================================================
/*
模块职责：
负责点云文件的读取、格式判断和保存，是界面与 PCL 文件接口之间唯一的常规持久化边界。
这里不拥有当前点云，也不弹出对话框；调用者负责选择文件并把读取结果交给 DianYunHuiHua。

主要调用位置：
ZhuChuangKouWindow 的点云打开与保存。

维护说明：
以下是文件格式层真正可调的参数：
CsvWangGeCanshu::xStepMm 和 yStepMm 只用于 CSV1 高度矩阵恢复 XY 坐标，单位为毫米；
CSV0 是普通 XYZ 三列表，不使用网格步距。修改 CSV1 步距会直接改变点云 XY 几何，必须由用户明确输入。
*/
#include "DianYunJichu_Core.h"
#include <QString>
#include <string>

namespace dianYunIO {

/*
CSV0：普通点表。标准保存格式第一行为 x,y,z，之后每行一个点。
CSV1：零原点高度矩阵。行号映射 X，反向列号映射 Y，单元格数值映射 Z；XY 间距由 CsvWangGeCanshu 指定。
自动判断时，无表头文件只有“恰好三列纯数字”才按 CSV0 处理，避免窄幅高度矩阵被误认成 XYZ 点表。
*/
/** 【类型导航注释】
 * CsvDianYunFormat：点云文件接口中的自定义 枚举。
 * 主要使用位置：ZhuChuangKou_Window.cpp、DianYun_IO.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
enum class CsvDianYunFormat {
    Auto = 0,
    Csv0Xyz,
    Csv1HeightMatrix,
    Unknown
};

/** 【类型导航注释】
 * CsvWangGeCanshu：点云文件接口中的自定义 结构体。
 * 主要使用位置：ZhuChuangKou_Window.cpp、DianYun_IO.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CsvWangGeCanshu {
    double xStepMm = 0.25;  // CSV1 的 X 网格间距，单位毫米；减小会让点在 X 方向更密集。
    double yStepMm = 0.30;  // CSV1 的 Y 网格间距，单位毫米；减小会让点在 Y 方向更密集。
};

/** 【类型导航注释】
 * CsvGeshiXinxi：点云文件接口中的自定义 结构体。
 * 主要使用位置：ZhuChuangKou_Window.cpp、DianYun_IO.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CsvGeshiXinxi {
    CsvDianYunFormat format = CsvDianYunFormat::Unknown;
    int sampledRows = 0;
    int minColumns = 0;
    int maxColumns = 0;
    bool hasXyzHeader = false;
    QString detail;
    QString error;
};

/** 【类型导航注释】
 * LoadJieGuo：点云文件接口中的自定义 结构体。
 * 主要使用位置：DianYun_IO.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct LoadJieGuo {
    CloudPtr cloud;
    QString error;
    CsvDianYunFormat csvFormat = CsvDianYunFormat::Unknown;
};

// 读取 PCD，兼容二进制、压缩二进制和 ASCII；Unicode 路径在模块内部转换。
LoadJieGuo loadPcd(const QString& path);

// 读取 PLY。优先读取 XYZRGB；只有 XYZ 时统一补白色，保持已有显示行为。
LoadJieGuo loadPly(const QString& path);

// 只读取足够判断 CSV0/CSV1 的文本，不把整份大文件重复载入内存。
CsvGeshiXinxi inspectCsvFormat(const QString& path);
QString csvFormatXianshiName(CsvDianYunFormat format);

// 读取 CSV。Auto 与 inspectCsvFormat() 使用同一套判断规则；界面已判断格式时也可以显式传入。
LoadJieGuo loadCsv(const QString& path, const CsvWangGeCanshu& options,
                   CsvDianYunFormat format = CsvDianYunFormat::Auto);

// 保存 PCD。compressed=true 使用压缩二进制；false 使用普通二进制。
bool savePcd(const QString& path, const CloudConstPtr& cloud,
             QString* error = nullptr, bool compressed = true);


// 保存 CSV0：写 x,y,z 表头，每行一个点。
bool saveCsv0(const QString& path, const CloudConstPtr& cloud, QString* error = nullptr);

/*
保存 CSV1：要求 XY 坐标能落在以零为原点、指定步距的规则网格上。
缺失单元格写 -999，与既有文件格式约定一致；如果点云已经平移、旋转或不再规则，函数应拒绝保存，
不能为了凑成矩阵而悄悄修改几何。
*/
bool saveCsv1(const QString& path, const CloudConstPtr& cloud,
              const CsvWangGeCanshu& options, QString* error = nullptr);

// 未显式指定 CSV 形式时按 CSV0 保存。
bool saveCsv(const QString& path, const CloudConstPtr& cloud, QString* error = nullptr);

// 提供 CSV 读取后直接写成二进制 PCD 的便捷接口。
bool csvToPcd(const QString& csvPath, const QString& pcdPath,
              const CsvWangGeCanshu& options, QString* error = nullptr);

// 给仍使用 std::string 的 PCL/底层库调用提供 UTF-8 路径转换。
std::string utf8Path(const QString& path);

}

// ============================================================================
// 功能分区：带孔参数 PCD 读写接口
// ============================================================================
/*
模块职责：
负责把孔参数嵌入 PCD 头注释，以及从 PCD 头恢复这些参数。它只处理持久化，不参与孔识别和界面显示。

主要调用位置：
ZhuChuangKouWindow 打开带孔 PCD 时读取；保存点云并选择“同时保存孔数据”时写入。

维护说明：
这是兼容已有 PCD 文件的格式边界。字段顺序或文本键名的修改会影响旧文件兼容，必须保留向后读取能力。
*/
#include "HoleLeixing_Types.h"
#include <vector>

namespace holeIO {

// 从 PCD 头部读取以“# HOLE”开头的孔参数行，遇到 DATA 行后停止解析。
std::vector<HoleMiaoshu> readEmbeddedHoles(const QString& pcdPath,
                                              QString* error = nullptr);

// 保存压缩二进制 PCD，并在 DATA 行之前插入孔参数。holes 为空时等价于普通压缩 PCD 保存。
bool savePcdWithEmbeddedHoles(const QString& path,
                              const CloudConstPtr& cloud,
                              const std::vector<HoleMiaoshu>& holes,
                              QString* error = nullptr);

}

