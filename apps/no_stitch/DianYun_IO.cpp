/*
================================================================================
文件：DianYun_IO.cpp
模块：点云文件实现

【主要职责】
实现格式判断、Unicode 路径处理、PCD/PLY/CSV 解析与 Hole 参数持久化。

【主要调用关系】
由 DianYun_IO.h 的公开函数进入。

【线程与状态】
同步 I/O；不触碰 GUI 控件。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
点云文件读写实现。

维护说明：
本文件按功能整合生产实现。各分区通过明确职责组织，算法阈值和候选顺序集中在对应分区维护。
*/
#include "DianYun_IO.h"

// ============================================================================
// 功能分区：通用点云文件读写实现
// ============================================================================
/*
模块职责：实现 PCD、PLY、CSV0 和 CSV1 的读取、自动识别与保存。
主要调用位置：ZhuChuangKouWindow。
维护说明：PCD/PLY/CSV 解析与保存必须在本模块内保持读写契约一致；CSV1 的 XY 步距会改变几何，不能自行猜测。
*/
#include "DianYunLeixing_Types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QTemporaryFile>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace dianYunIO {
namespace {

/** 【函数导航】
 * 作用：执行“asciiPath”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool asciiPath(const QString& path)
{
    const QByteArray latin = path.toLatin1();
    return QString::fromLatin1(latin) == path;
}

class LinShiFileShouHu
{
public:
    /** 【函数导航】
     * 作用：执行“LinShiFileShouHu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云文件实现。
     * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    explicit LinShiFileShouHu(const QString& ext)
    {
        QTemporaryFile tf(QDir::temp().filePath(QStringLiteral("sb24_io_XXXXXX") + ext));
        tf.setAutoRemove(false);
        if (tf.open()) {
            m_path = tf.fileName();
            tf.close();
        }
    }
    ~LinShiFileShouHu() { if (!m_path.isEmpty()) QFile::remove(m_path); }
    LinShiFileShouHu(const LinShiFileShouHu&) = delete;
    LinShiFileShouHu& operator=(const LinShiFileShouHu&) = delete;
    /** 【函数导航】
     * 作用：执行“valid”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云文件实现。
     * 主要引用/调用位置：HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、ShouDongHole_Manual.h。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool valid() const { return !m_path.isEmpty(); }
    /** 【函数导航】
     * 作用：执行“path”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云文件实现。
     * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    const QString& path() const { return m_path; }

    /** 【函数导航】
     * 作用：执行“moveTo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：点云文件实现。
     * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool moveTo(const QString& target)
    {
        if (m_path.isEmpty()) return false;
        if (QFile::exists(target)) {
            if (!QFile::remove(target)) return false;
        }
        if (!QFile::rename(m_path, target)) return false;
        m_path.clear();
        return true;
    }

private:
    QString m_path;
};

/** 【函数导航】
 * 作用：应用/设置“setError”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void setError(QString* error, const QString& msg)
{
    if (error) *error = msg;
}

/** 【函数导航】
 * 作用：执行“trimStr”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::string trimStr(const std::string& s)
{
    const std::size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::string();
    const std::size_t last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

/** 【函数导航】
 * 作用：执行“lowerAscii”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::string lowerAscii(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

/** 【函数导航】
 * 作用：读取/解析“parseFloatStrict”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool parseFloatStrict(const std::string& text, float& value)
{
    const std::string t = trimStr(text);
    if (t.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const float v = std::strtof(t.c_str(), &end);
    if (end == t.c_str() || errno == ERANGE || !std::isfinite(v)) return false;
    while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end != '\0') return false;
    value = v;
    return true;
}

using CsvRows = std::vector<std::vector<std::string>>;

/** 【函数导航】
 * 作用：读取/解析“parseCsvRows”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CsvRows parseCsvRows(const QByteArray& bytes, std::size_t maxRows = 0)
{
    std::istringstream fin(std::string(bytes.constData(), static_cast<std::size_t>(bytes.size())));
    CsvRows rows;
    std::string line;
    while (std::getline(fin, line)) {
        if (maxRows > 0 && rows.size() >= maxRows) break;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trimStr(line).empty()) continue;
        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(ss, field, ',')) fields.push_back(field);
        if (fields.empty()) continue;
        if (rows.empty() && !fields[0].empty()
            && static_cast<unsigned char>(fields[0][0]) == 0xEF
            && fields[0].size() >= 3
            && static_cast<unsigned char>(fields[0][1]) == 0xBB
            && static_cast<unsigned char>(fields[0][2]) == 0xBF) {
            fields[0].erase(0, 3);
        }
        rows.push_back(std::move(fields));
    }
    return rows;
}

/** 【函数导航】
 * 作用：执行“xyzHeader”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool xyzHeader(const std::vector<std::string>& row)
{
    if (row.size() < 3) return false;
    return lowerAscii(trimStr(row[0])) == "x"
        && lowerAscii(trimStr(row[1])) == "y"
        && lowerAscii(trimStr(row[2])) == "z";
}

/** 【函数导航】
 * 作用：读取/解析“inspectRows”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CsvGeshiXinxi inspectRows(const CsvRows& rows)
{
    CsvGeshiXinxi info;
    if (rows.empty()) {
        info.error = QStringLiteral("CSV 没有可读取的数据行");
        return info;
    }

    info.sampledRows = static_cast<int>(rows.size());
    info.minColumns = std::numeric_limits<int>::max();
    info.maxColumns = 0;
    for (const auto& row : rows) {
        info.minColumns = std::min(info.minColumns, static_cast<int>(row.size()));
        info.maxColumns = std::max(info.maxColumns, static_cast<int>(row.size()));
    }
    if (info.minColumns == std::numeric_limits<int>::max()) info.minColumns = 0;

    info.hasXyzHeader = xyzHeader(rows.front());
    if (info.hasXyzHeader) {
        info.format = CsvDianYunFormat::Csv0Xyz;
        info.detail = QStringLiteral("检测到 x,y,z 表头");
        return info;
    }

    if (info.minColumns == 3 && info.maxColumns == 3) {
        int numericRows = 0;
        for (const auto& row : rows) {
            if (row.size() != 3) continue;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (parseFloatStrict(row[0], x)
                && parseFloatStrict(row[1], y)
                && parseFloatStrict(row[2], z)
                && std::abs(z) < 1.0e6f) {
                ++numericRows;
            }
        }
        if (numericRows * 5 >= static_cast<int>(rows.size()) * 4) {
            info.format = CsvDianYunFormat::Csv0Xyz;
            info.detail = QStringLiteral("检测到三列数值 XYZ 点表");
            return info;
        }
    }

    info.format = CsvDianYunFormat::Csv1HeightMatrix;
    info.detail = QStringLiteral("检测为二维高度矩阵");
    return info;
}

/** 【函数导航】
 * 作用：读取/解析“parseCsv0Rows”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr parseCsv0Rows(const CsvRows& rows)
{
    auto cloud = CloudPtr(new Cloud());
    cloud->reserve(rows.size());
    for (const auto& fields : rows) {
        if (fields.size() < 3 || xyzHeader(fields)) continue;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        if (!parseFloatStrict(fields[0], x)
            || !parseFloatStrict(fields[1], y)
            || !parseFloatStrict(fields[2], z)) continue;

        pcl::PointXYZRGB p;
        p.x = x; p.y = y; p.z = z;
        p.r = p.g = p.b = 128;
        if (fields.size() >= 4) {
            float v = 0.0f;
            if (parseFloatStrict(fields[3], v))
                p.r = static_cast<std::uint8_t>(std::clamp(static_cast<int>(v), 0, 255));
        }
        if (fields.size() >= 5) {
            float v = 0.0f;
            if (parseFloatStrict(fields[4], v))
                p.g = static_cast<std::uint8_t>(std::clamp(static_cast<int>(v), 0, 255));
        }
        if (fields.size() >= 6) {
            float v = 0.0f;
            if (parseFloatStrict(fields[5], v))
                p.b = static_cast<std::uint8_t>(std::clamp(static_cast<int>(v), 0, 255));
        }
        if (p.r == 0 && p.g == 0 && p.b == 0) p.r = p.g = p.b = 128;
        cloud->push_back(p);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    return cloud;
}

/** 【函数导航】
 * 作用：读取/解析“parseCsv1Rows”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CloudPtr parseCsv1Rows(const CsvRows& rows, const CsvWangGeCanshu& options)
{
    static constexpr float kInvalidHeightSentinel = -999.0f;
    if (!(options.xStepMm > 0.0) || !(options.yStepMm > 0.0)) return CloudPtr();

    int maxCols = 0;
    for (const auto& row : rows) maxCols = std::max(maxCols, static_cast<int>(row.size()));
    if (maxCols <= 0) return CloudPtr();

    auto cloud = CloudPtr(new Cloud());
    std::size_t reserveCount = 0;
    for (const auto& row : rows) reserveCount += row.size();
    cloud->reserve(reserveCount);

    for (std::size_t row = 0; row < rows.size(); ++row) {
        const float x = static_cast<float>(static_cast<double>(row) * options.xStepMm);
        const auto& fields = rows[row];
        for (std::size_t col = 0; col < fields.size(); ++col) {
            float z = 0.0f;
            if (!parseFloatStrict(fields[col], z)) continue;
            if (z <= kInvalidHeightSentinel) continue;
            pcl::PointXYZRGB p;
            p.x = x;
            p.y = static_cast<float>(
                static_cast<double>(maxCols - 1 - static_cast<int>(col)) * options.yStepMm);
            p.z = z;
            p.r = static_cast<std::uint8_t>(std::clamp(p.x * 50.0f, 0.0f, 255.0f));
            p.g = static_cast<std::uint8_t>(std::clamp(p.y * 50.0f, 0.0f, 255.0f));
            p.b = static_cast<std::uint8_t>(std::clamp(p.z * 50.0f, 0.0f, 255.0f));
            cloud->push_back(p);
        }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    return cloud;
}

/** 【函数导航】
 * 作用：保存/输出“writePcdTo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool writePcdTo(const QString& path, const CloudConstPtr& cloud, bool compressed, bool ascii, QString* error)
{
    if (!cloud || cloud->empty()) {
        setError(error, QStringLiteral("empty cloud"));
        return false;
    }
    if (asciiPath(path)) {
        const int code = ascii
            ? pcl::io::savePCDFileASCII(path.toStdString(), *cloud)
            : (compressed ? pcl::io::savePCDFileBinaryCompressed(path.toStdString(), *cloud)
                          : pcl::io::savePCDFileBinary(path.toStdString(), *cloud));
        if (code == 0) return true;
        setError(error, QStringLiteral("PCD save failed: %1").arg(path));
        return false;
    }
    LinShiFileShouHu tmp(QStringLiteral(".pcd"));
    if (!tmp.valid()) { setError(error, QStringLiteral("temp file unavailable")); return false; }
    const int code = ascii
        ? pcl::io::savePCDFileASCII(tmp.path().toStdString(), *cloud)
        : (compressed ? pcl::io::savePCDFileBinaryCompressed(tmp.path().toStdString(), *cloud)
                      : pcl::io::savePCDFileBinary(tmp.path().toStdString(), *cloud));
    if (code != 0) { setError(error, QStringLiteral("PCD save failed: %1").arg(path)); return false; }
    if (!tmp.moveTo(path)) { setError(error, QStringLiteral("cannot move temp PCD to %1").arg(path)); return false; }
    return true;
}

}

/** 【函数导航】
 * 作用：执行“utf8Path”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::string utf8Path(const QString& path)
{
    return path.toUtf8().constData();
}

/** 【函数导航】
 * 作用：读取 PCD 点云文件并返回点云或错误信息。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
LoadJieGuo loadPcd(const QString& path)
{
    LoadJieGuo r;
    if (path.isEmpty()) { setError(&r.error, QStringLiteral("empty path")); return r; }
    if (asciiPath(path)) {
        auto cloud = CloudPtr(new Cloud());
        if (pcl::io::loadPCDFile(path.toStdString(), *cloud) == 0 && !cloud->empty()) {
            r.cloud = cloud;
        } else {
            setError(&r.error, QStringLiteral("PCD load failed: %1").arg(path));
        }
        return r;
    }
    LinShiFileShouHu tmp(QStringLiteral(".pcd"));
    if (!tmp.valid()) {
        setError(&r.error, QStringLiteral("cannot stage PCD: %1").arg(path));
        return r;
    }
    QFile::remove(tmp.path());
    if (!QFile::copy(path, tmp.path())) {
        setError(&r.error, QStringLiteral("cannot stage PCD: %1").arg(path));
        return r;
    }
    auto cloud = CloudPtr(new Cloud());
    if (pcl::io::loadPCDFile(tmp.path().toStdString(), *cloud) == 0 && !cloud->empty()) {
        r.cloud = cloud;
    } else {
        setError(&r.error, QStringLiteral("PCD load failed: %1").arg(path));
    }
    return r;
}

/** 【函数导航】
 * 作用：读取 PLY 点云文件并返回点云或错误信息。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
LoadJieGuo loadPly(const QString& path)
{
    LoadJieGuo r;
    if (path.isEmpty()) { setError(&r.error, QStringLiteral("empty path")); return r; }

    const bool direct = asciiPath(path);
    QString usePath = path;
    LinShiFileShouHu tmp(QStringLiteral(".ply"));
    if (!direct) {
        if (!tmp.valid()) {
            setError(&r.error, QStringLiteral("cannot stage PLY: %1").arg(path));
            return r;
        }
        QFile::remove(tmp.path());
        if (!QFile::copy(path, tmp.path())) {
            setError(&r.error, QStringLiteral("cannot stage PLY: %1").arg(path));
            return r;
        }
        usePath = tmp.path();
    }
    const std::string sPath = usePath.toStdString();

    auto cloudRgb = CloudPtr(new Cloud());
    if (pcl::io::loadPLYFile(sPath, *cloudRgb) == 0 && cloudRgb && !cloudRgb->empty()) {
        r.cloud = cloudRgb;
        return r;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyz(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(sPath, *cloudXyz) == 0 && cloudXyz && !cloudXyz->empty()) {
        auto out = CloudPtr(new Cloud());
        out->reserve(cloudXyz->size());
        for (const auto& p : cloudXyz->points) {
            pcl::PointXYZRGB q;
            q.x = p.x; q.y = p.y; q.z = p.z;
            q.r = q.g = q.b = 255;
            out->push_back(q);
        }
        out->width = static_cast<std::uint32_t>(out->size());
        out->height = 1;
        r.cloud = out;
        return r;
    }
    setError(&r.error, QStringLiteral("PLY load failed: %1").arg(path));
    return r;
}

/** 【函数导航】
 * 作用：读取/解析“inspectCsvFormat”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
CsvGeshiXinxi inspectCsvFormat(const QString& path)
{
    CsvGeshiXinxi info;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        info.error = QStringLiteral("CSV open failed: %1").arg(path);
        return info;
    }

    const QByteArray bytes = f.read(256 * 1024);
    f.close();
    return inspectRows(parseCsvRows(bytes, 128));
}

/** 【函数导航】
 * 作用：执行“csvFormatXianshiName”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString csvFormatXianshiName(CsvDianYunFormat format)
{
    switch (format) {
    case CsvDianYunFormat::Csv0Xyz:
        return QStringLiteral("CSV0（三列 XYZ 点表）");
    case CsvDianYunFormat::Csv1HeightMatrix:
        return QStringLiteral("CSV1（二维 Z 高度矩阵）");
    case CsvDianYunFormat::Auto:
        return QStringLiteral("CSV（自动识别）");
    default:
        return QStringLiteral("CSV（未知）");
    }
}

/** 【函数导航】
 * 作用：读取 CSV0/CSV1 点云并按当前网格规则恢复坐标。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
LoadJieGuo loadCsv(const QString& path, const CsvWangGeCanshu& options,
                   CsvDianYunFormat requestedFormat)
{
    LoadJieGuo r;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        setError(&r.error, QStringLiteral("CSV open failed: %1").arg(path));
        return r;
    }
    const QByteArray bytes = f.readAll();
    f.close();
    const CsvRows rows = parseCsvRows(bytes);
    const CsvGeshiXinxi detected = inspectRows(rows);
    CsvDianYunFormat format = requestedFormat;
    if (format == CsvDianYunFormat::Auto) format = detected.format;
    if (format != CsvDianYunFormat::Csv0Xyz
        && format != CsvDianYunFormat::Csv1HeightMatrix) {
        setError(&r.error, detected.error.isEmpty()
            ? QStringLiteral("无法识别 CSV 点云格式") : detected.error);
        return r;
    }

    r.cloud = (format == CsvDianYunFormat::Csv0Xyz)
        ? parseCsv0Rows(rows)
        : parseCsv1Rows(rows, options);
    r.csvFormat = format;
    if (!r.cloud || r.cloud->empty()) {
        setError(&r.error, QStringLiteral("%1 解析后没有有效点：%2")
            .arg(csvFormatXianshiName(format), path));
        r.cloud.reset();
    }
    return r;
}

/** 【函数导航】
 * 作用：保存 PCD 点云文件。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool savePcd(const QString& path, const CloudConstPtr& cloud, QString* error, bool compressed)
{
    return writePcdTo(path, cloud, compressed, false, error);
}

/** 【函数导航】
 * 作用：保存/输出“saveCsv0”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool saveCsv0(const QString& path, const CloudConstPtr& cloud, QString* error)
{
    if (!cloud || cloud->empty()) { setError(error, QStringLiteral("empty cloud")); return false; }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        setError(error, QStringLiteral("CSV0 open failed: %1").arg(path));
        return false;
    }
    QTextStream ts(&f);
    ts.setRealNumberNotation(QTextStream::SmartNotation);
    ts.setRealNumberPrecision(9);
    ts << "x,y,z\n";
    for (const auto& p : *cloud) ts << p.x << ',' << p.y << ',' << p.z << '\n';
    ts.flush();
    const bool ok = f.error() == QFileDevice::NoError;
    f.close();
    if (!ok) setError(error, QStringLiteral("CSV0 write failed: %1").arg(path));
    return ok;
}

/** 【函数导航】
 * 作用：保存/输出“saveCsv1”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool saveCsv1(const QString& path, const CloudConstPtr& cloud,
              const CsvWangGeCanshu& options, QString* error)
{
    static constexpr float kSentinel = -999.0f;
    if (!cloud || cloud->empty()) { setError(error, QStringLiteral("empty cloud")); return false; }
    if (!(options.xStepMm > 0.0) || !(options.yStepMm > 0.0)) {
        setError(error, QStringLiteral("CSV1 X/Y 间距必须大于 0"));
        return false;
    }

    /** 【类型导航注释】
     * YingShePoint：点云文件实现中的自定义 结构体。
     * 主要使用位置：DianYun_IO.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct YingShePoint { int xi = 0; int yi = 0; float z = 0.0f; };
    std::vector<YingShePoint> mapped;
    mapped.reserve(cloud->size());
    int maxXi = 0;
    int maxYi = 0;
    const double tolX = std::max(0.001, options.xStepMm * 0.002);
    const double tolY = std::max(0.001, options.yStepMm * 0.002);

    for (const auto& p : *cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z <= kSentinel) {
            setError(error, QStringLiteral("CSV1 无法保存 Z<=-999 的有效点，因为 -999 是缺失值标记"));
            return false;
        }
        const long long xi64 = std::llround(static_cast<double>(p.x) / options.xStepMm);
        const long long yi64 = std::llround(static_cast<double>(p.y) / options.yStepMm);
        if (xi64 < 0 || yi64 < 0
            || xi64 > std::numeric_limits<int>::max()
            || yi64 > std::numeric_limits<int>::max()) {
            setError(error, QStringLiteral("CSV1 要求 X/Y 为从 0 开始的非负规则网格坐标"));
            return false;
        }
        const double expectedX = static_cast<double>(xi64) * options.xStepMm;
        const double expectedY = static_cast<double>(yi64) * options.yStepMm;
        if (std::abs(static_cast<double>(p.x) - expectedX) > tolX
            || std::abs(static_cast<double>(p.y) - expectedY) > tolY) {
            setError(error, QStringLiteral(
                "CSV1 要求点云位于从 0 开始的规则 X/Y 网格；当前点 (%1,%2) 与间距 (%3,%4) 不匹配")
                .arg(p.x, 0, 'g', 8).arg(p.y, 0, 'g', 8)
                .arg(options.xStepMm, 0, 'g', 8).arg(options.yStepMm, 0, 'g', 8));
            return false;
        }
        const int xi = static_cast<int>(xi64);
        const int yi = static_cast<int>(yi64);
        mapped.push_back({xi, yi, p.z});
        maxXi = std::max(maxXi, xi);
        maxYi = std::max(maxYi, yi);
    }
    if (mapped.empty()) {
        setError(error, QStringLiteral("CSV1 没有可保存的有限点"));
        return false;
    }

    const std::uint64_t rows = static_cast<std::uint64_t>(maxXi) + 1ULL;
    const std::uint64_t cols = static_cast<std::uint64_t>(maxYi) + 1ULL;
    const std::uint64_t cells = rows * cols;
    if (rows == 0 || cols == 0 || cells > 50000000ULL) {
        setError(error, QStringLiteral("CSV1 网格过大（%1 x %2），请检查 X/Y 间距")
            .arg(static_cast<qulonglong>(rows)).arg(static_cast<qulonglong>(cols)));
        return false;
    }

    std::vector<float> matrix(static_cast<std::size_t>(cells), kSentinel);
    std::vector<unsigned char> used(static_cast<std::size_t>(cells), 0);
    for (const YingShePoint& p : mapped) {

        const int col = maxYi - p.yi;
        const std::size_t index = static_cast<std::size_t>(p.xi)
            * static_cast<std::size_t>(cols) + static_cast<std::size_t>(col);
        if (used[index]) {
            if (std::abs(matrix[index] - p.z) > 1.0e-5f) {
                setError(error, QStringLiteral("CSV1 同一 X/Y 网格单元存在多个不同 Z，无法无损保存"));
                return false;
            }
            continue;
        }
        used[index] = 1;
        matrix[index] = p.z;
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        setError(error, QStringLiteral("CSV1 open failed: %1").arg(path));
        return false;
    }
    QTextStream ts(&f);
    ts.setRealNumberNotation(QTextStream::SmartNotation);
    ts.setRealNumberPrecision(9);
    for (int row = 0; row <= maxXi; ++row) {
        for (int col = 0; col <= maxYi; ++col) {
            if (col) ts << ',';
            const float z = matrix[static_cast<std::size_t>(row)
                * static_cast<std::size_t>(cols) + static_cast<std::size_t>(col)];
            ts << z;
        }
        ts << '\n';
    }
    ts.flush();
    const bool ok = f.error() == QFileDevice::NoError;
    f.close();
    if (!ok) setError(error, QStringLiteral("CSV1 write failed: %1").arg(path));
    return ok;
}

/** 【函数导航】
 * 作用：保存 CSV 点云文件。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool saveCsv(const QString& path, const CloudConstPtr& cloud, QString* error)
{
    return saveCsv0(path, cloud, error);
}

/** 【函数导航】
 * 作用：执行“csvToPcd”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool csvToPcd(const QString& csvPath, const QString& pcdPath,
              const CsvWangGeCanshu& options, QString* error)
{
    auto r = loadCsv(csvPath, options, CsvDianYunFormat::Auto);
    if (!r.cloud) {
        if (error && !r.error.isEmpty()) *error = r.error;
        else setError(error, QStringLiteral("CSV load failed"));
        return false;
    }
    return savePcd(pcdPath, r.cloud, error, false);
}

}


// ============================================================================
// 功能分区：带孔参数 PCD 读写实现
// ============================================================================
/*
模块职责：实现 PCD 头部孔参数的读取和写入，保持已有文件格式兼容。
主要调用位置：ZhuChuangKouWindow。该模块只负责序列化，不修改孔识别结果。
维护说明：PCD 头部键名属于持久化文件格式；增加字段时必须保持既有文件可读，不应随意修改已发布键名。
*/
#include <QByteArray>
#include <QStringList>
#include <optional>

namespace holeIO {
namespace {

// 把一个 HoleMiaoshu 序列化成稳定的“# HOLE ...”头部记录。
/** 【函数导航】
 * 作用：执行“holeWenBen”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
QString holeWenBen(const HoleMiaoshu& d)
{
    auto f = [](float v) { return QString::number(v, 'f', 4); };
    return QStringLiteral("# HOLE type=%1 cxt=%2 cyt=%3 czt=%4 rtop=%5 rbot=%6 cx=%7 cy=%8 cz=%9 depth=%10 slope=%11 support=%12 complete=%13")
        .arg(d.type)
        .arg(f(d.centerTop.x())).arg(f(d.centerTop.y())).arg(f(d.centerTop.z()))
        .arg(f(d.rTop)).arg(f(d.rBot))
        .arg(f(d.center.x())).arg(f(d.center.y())).arg(f(d.center.z()))
        .arg(f(d.depth)).arg(f(d.slopeDeg))
        .arg(d.supportPts).arg(f(d.completeness));
}

// 把“# HOLE ...”键值行恢复成 HoleMiaoshu，继续兼容既有 PCD 文件。
/** 【函数导航】
 * 作用：执行“wenBenJieXiHole”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::optional<HoleMiaoshu> wenBenJieXiHole(const QString& line)
{
    if (!line.startsWith(QStringLiteral("# HOLE "))) return std::nullopt;
    HoleMiaoshu d;
    const QString payload = line.mid(6);
    const QStringList pairs = payload.split(' ', Qt::SkipEmptyParts);
    for (const auto& p : pairs) {
        int eq = p.indexOf('=');
        if (eq < 0) continue;
        const QString key = p.left(eq);
        const float val = p.mid(eq + 1).toFloat();
        if (key == QStringLiteral("type")) d.type = (int)val;
        else if (key == QStringLiteral("cxt")) d.centerTop.x() = val;
        else if (key == QStringLiteral("cyt")) d.centerTop.y() = val;
        else if (key == QStringLiteral("czt")) d.centerTop.z() = val;
        else if (key == QStringLiteral("rtop")) d.rTop = val;
        else if (key == QStringLiteral("rbot")) d.rBot = val;
        else if (key == QStringLiteral("cx")) d.center.x() = val;
        else if (key == QStringLiteral("cy")) d.center.y() = val;
        else if (key == QStringLiteral("cz")) d.center.z() = val;
        else if (key == QStringLiteral("depth")) d.depth = val;
        else if (key == QStringLiteral("slope")) d.slopeDeg = val;
        else if (key == QStringLiteral("support")) d.supportPts = (int)val;
        else if (key == QStringLiteral("complete")) d.completeness = val;
    }
    if (d.rTop <= 0.0f) d.rTop = d.radius;
    else d.radius = d.rTop;
    return d;
}

}

/** 【函数导航】
 * 作用：读取/解析“readEmbeddedHoles”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<HoleMiaoshu> readEmbeddedHoles(const QString& pcdPath, QString* error)
{
    std::vector<HoleMiaoshu> holes;
    QFile file(pcdPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) *error = QStringLiteral("cannot open PCD: %1").arg(pcdPath);
        return holes;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        const QString line = in.readLine().trimmed();
        if (line.startsWith(QStringLiteral("DATA "))) break;
        auto opt = wenBenJieXiHole(line);
        if (opt.has_value()) holes.push_back(opt.value());
    }
    file.close();
    return holes;
}

/** 【函数导航】
 * 作用：保存/输出“savePcdWithEmbeddedHoles”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：点云文件实现。
 * 主要引用/调用位置：DianYun_IO.h、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool savePcdWithEmbeddedHoles(const QString& path,
                              const CloudConstPtr& cloud,
                              const std::vector<HoleMiaoshu>& holes,
                              QString* error)
{
    if (!cloud || cloud->empty()) {
        if (error) *error = QStringLiteral("empty cloud");
        return false;
    }
    QStringList holeLines;
    for (const auto& d : holes) holeLines.append(holeWenBen(d));
    if (holeLines.isEmpty()) {
        return dianYunIO::savePcd(path, cloud, error, true);
    }

    // 先保存到唯一临时文件，在 DATA 之前插入孔参数块，最后原子替换目标文件，避免写到一半留下坏文件。
    const QString tmp = QStringLiteral("sb24_hole_XXXXXX.pcd");
    QString tmpPath;
    {
        QTemporaryFile tf(QDir::temp().filePath(tmp));
        tf.setAutoRemove(false);
        if (!tf.open()) {
            if (error) *error = QStringLiteral("temp file unavailable");
            return false;
        }
        tmpPath = tf.fileName();
        tf.close();
        // 临时文件对象在此释放，但文件保留，后面继续注入孔参数。
    }

    if (!dianYunIO::savePcd(tmpPath, cloud, error, true)) {
        QFile::remove(tmpPath);
        return false;
    }
    {
        QFile tmpFile(tmpPath);
        if (!tmpFile.open(QIODevice::ReadOnly)) {
            QFile::remove(tmpPath);
            if (error) *error = QStringLiteral("cannot read staged PCD");
            return false;
        }
        const QByteArray raw = tmpFile.readAll();
        tmpFile.close();

        const QString holeBlock = holeLines.join(QStringLiteral("\n")) + QStringLiteral("\n");
        const QByteArray holeBytes = holeBlock.toUtf8();
        const int dataPos = raw.indexOf("DATA ");
        if (dataPos < 0) {
            QFile::remove(tmpPath);
            if (error) *error = QStringLiteral("PCD DATA line not found");
            return false;
        }
        QByteArray out;
        out.reserve(raw.size() + holeBytes.size() + 64);
        out.append(raw.left(dataPos));
        out.append(holeBytes);
        out.append(raw.mid(dataPos));

        QFile outFile(path);
        if (!outFile.open(QIODevice::WriteOnly)) {
            QFile::remove(tmpPath);
            if (error) *error = QStringLiteral("cannot write %1").arg(path);
            return false;
        }
        outFile.write(out);
        outFile.close();
        QFile::remove(tmpPath);
        return true;
    }
}

}


