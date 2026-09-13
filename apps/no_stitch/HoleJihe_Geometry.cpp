/*
================================================================================
文件：HoleJihe_Geometry.cpp
模块：Hole 几何实现

【主要职责】
实现局部坐标几何、半径精修、表面剖面、内壁/锥壁证据和最终几何评估。

【主要调用关系】
由识别编排和分析策略调用。

【线程与状态】
纯计算；关键阈值直接影响识别结果。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
孔几何计算实现。

维护说明：
本文件按功能整合生产实现。各分区通过明确职责组织，算法阈值和候选顺序集中在对应分区维护。
*/
#include "HoleJihe_Geometry.h"

// ============================================================================
// 功能分区：几何公共工具实现
// ============================================================================
/*
模块职责：
孔几何公共工具。

主要调用位置：
由 HoleShibie_Recognition.cpp 及孔识别子模块复用，提供不持有状态的几何计算。

维护说明：
这里只放可复用纯计算；阈值应由调用模块显式传入或在敏感参数旁说明。
*/
#include "DianYunJichu_Core.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace holeJihe {

/** 【函数导航】
 * 作用：执行“kasaZhongWeiShu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float kasaZhongWeiShu(std::vector<float>& v)
{
    if (v.empty()) return 0.0f;
    size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float m = v[mid];
    if ((v.size() % 2) == 0 && mid > 0) {
        std::nth_element(v.begin(), v.begin() + mid - 1, v.end());
        m = 0.5f * (m + v[mid - 1]);
    }
    return m;
}
/** 【函数导航】
 * 作用：执行“kasaNiHeYuan”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool kasaNiHeYuan(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius)
{
    int n = (int)pts.size();
    if (n < 8) return false;
    float sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, szx = 0, szy = 0, sz = 0;
    for (const auto& p : pts) {
        float x = p.x(), y = p.y(); float z = x * x + y * y;
        sxx += x * x; sxy += x * y; sx += x; syy += y * y; sy += y;
        szx += z * x; szy += z * y; sz += z;
    }
    Eigen::Matrix3f A;
    A << 4 * sxx, 4 * sxy, -2 * sx,
         4 * sxy, 4 * syy, -2 * sy,
         -2 * sx, -2 * sy, (float)n;
    Eigen::Vector3f b(2 * szx, 2 * szy, -sz);
    Eigen::Vector3f sol = A.ldlt().solve(b);
    center = Eigen::Vector2f(sol(0), sol(1));
    float c = sol(2);
    radius = std::sqrt(std::max(0.0f, center.x() * center.x() + center.y() * center.y() - c));
    return std::isfinite(radius) && std::isfinite(center.x()) && std::isfinite(center.y());
}
/** 【函数导航】
 * 作用：执行“kasaJiaoJunBianJie”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void kasaJiaoJunBianJie(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,     float cx, float cy, float cz, float r, float zHalf, int bins,     std::vector<Eigen::Vector2f>& outPts, bool quZuiYuan)
{
    outPts.clear();
    if (bins < 8 || r <= 0.0f) return;
    /** 【类型导航注释】
     * DianWuCha：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct DianWuCha { float wuCha; float x, y; };
    std::vector<std::vector<DianWuCha>> tong((size_t)bins);
    const float zLo = cz - zHalf, zHi = cz + zHalf;
    const float rLo = r * 0.60f, rHi = r * 1.45f;
    const float angStep = 2.0f * (float)M_PI / (float)bins;
    for (const auto& p : cloud) {
        if (p.z < zLo || p.z > zHi) continue;
        float dx = p.x - cx, dy = p.y - cy;
        float d = std::sqrt(dx * dx + dy * dy);
        if (d < rLo || d > rHi) continue;
        float ang = std::atan2(dy, dx);
        int bin = (int)((ang + (float)M_PI) / angStep + 0.5f) % bins;
        if (bin < 0) bin += bins;
        tong[(size_t)bin].push_back({ std::abs(d - r), p.x, p.y });
    }
    for (auto& td : tong) {
        if (td.empty()) continue;
        size_t sel = 0;
        for (size_t i = 1; i < td.size(); ++i)
            if (quZuiYuan ? (td[i].wuCha > td[sel].wuCha) : (td[i].wuCha < td[sel].wuCha)) sel = i;
        outPts.emplace_back(td[sel].x, td[sel].y);
    }
}
/** 【函数导航】
 * 作用：执行“kasaJieDuanNiHe”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool kasaJieDuanNiHe(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius, float& medErr)
{
    if (pts.size() < 8) return false;
    std::vector<Eigen::Vector2f> cur = pts;
    Eigen::Vector2f c; float r = 0.0f; float lastErr = 1e9f; medErr = 1e9f;
    for (int it = 0; it < 4; ++it) {
        if (!kasaNiHeYuan(cur, c, r)) return false;
        std::vector<std::pair<float, int>> errs; errs.reserve(pts.size());
        for (int i = 0; i < (int)pts.size(); ++i)
            errs.emplace_back(std::abs((pts[(size_t)i] - c).norm() - r), i);
        int keepN = std::max(8, (int)std::floor((float)pts.size() * 0.70f));
        if (keepN >= (int)errs.size()) keepN = (int)errs.size();
        std::nth_element(errs.begin(), errs.begin() + (keepN - 1), errs.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        errs.resize((size_t)keepN);
        std::vector<float> eOnly; std::vector<Eigen::Vector2f> next;
        eOnly.reserve((size_t)keepN); next.reserve((size_t)keepN);
        for (auto& ei : errs) { eOnly.push_back(ei.first); next.push_back(pts[(size_t)ei.second]); }
        medErr = kasaZhongWeiShu(eOnly);
        cur.swap(next);
        if (std::abs(lastErr - medErr) < 0.01f) break;
        lastErr = medErr;
    }
    if (!kasaNiHeYuan(cur, center, radius)) return false;
    return std::isfinite(center.x()) && std::isfinite(center.y()) && std::isfinite(radius) && radius > 0.5f;
}
/** 【函数导航】
 * 作用：执行“ransacNiHeYuanSanDian”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool ransacNiHeYuanSanDian(const std::vector<Eigen::Vector2f>& pts,     Eigen::Vector2f& center, float& radius, float inlierDist, int maxIter )
{
    const int n = (int)pts.size();
    if (n < 8) return false;
    // 预计算每个点的扇区(8扇区)
    std::vector<uint8_t> sectors((size_t)n);
    for (int i = 0; i < n; ++i) {
        float ang = std::atan2(pts[(size_t)i].y(), pts[(size_t)i].x());
        if (ang < 0) ang += 2.0f * (float)M_PI;
        sectors[(size_t)i] = (uint8_t)std::min(7, (int)(ang / (2.0f*(float)M_PI) * 8));
    }
    int bestCnt = 0; float bestR = 0; Eigen::Vector2f bestC(0,0);
    for (int iter = 0; iter < maxIter; ++iter) {
        int i1 = rand() % n;
        int i2 = rand() % n; while (i2 == i1) i2 = rand() % n;
        int i3 = rand() % n; while (i3 == i1 || i3 == i2) i3 = rand() % n;
        // RC约束：三点必须来自3个不同的扇区
        uint8_t s1 = sectors[(size_t)i1], s2 = sectors[(size_t)i2], s3 = sectors[(size_t)i3];
        if (s1 == s2 || s1 == s3 || s2 == s3) continue;
        const auto& a = pts[(size_t)i1]; const auto& b = pts[(size_t)i2]; const auto& c = pts[(size_t)i3];
        float da = a.x()*a.x() + a.y()*a.y();
        float db = b.x()*b.x() + b.y()*b.y();
        float dc = c.x()*c.x() + c.y()*c.y();
        float A00 = 2.0f*(b.x()-a.x()), A01 = 2.0f*(b.y()-a.y()), B0 = db - da;
        float A10 = 2.0f*(c.x()-a.x()), A11 = 2.0f*(c.y()-a.y()), B1 = dc - da;
        float det = A00*A11 - A01*A10;
        if (std::abs(det) < 1e-8f) continue;
        float cx = (B0*A11 - B1*A01) / det;
        float cy = (A00*B1 - A10*B0) / det;
        float r = std::sqrt((a.x()-cx)*(a.x()-cx) + (a.y()-cy)*(a.y()-cy));
        if (r < 1.0f || r > 50.0f) continue;
        int cnt = 0;
        for (const auto& p : pts) {
            float d = std::abs(std::sqrt((p.x()-cx)*(p.x()-cx)+(p.y()-cy)*(p.y()-cy)) - r);
            if (d < inlierDist) ++cnt;
        }
        if (cnt > bestCnt) { bestCnt = cnt; bestC = Eigen::Vector2f(cx,cy); bestR = r; }
    }
    if (bestCnt < std::max(6, n/4)) return false;
    std::vector<Eigen::Vector2f> inls;
    for (const auto& p : pts) {
        float d = std::abs(std::sqrt((p.x()-bestC.x())*(p.x()-bestC.x())+(p.y()-bestC.y())*(p.y()-bestC.y())) - bestR);
        if (d < inlierDist*1.5f) inls.push_back(p);
    }
    if (inls.size() < 6) return false;
    if (!kasaNiHeYuan(inls, center, radius)) return false;
    return std::isfinite(center.x()) && std::isfinite(center.y()) && std::isfinite(radius) && radius > 0.5f;
}
/** 【函数导航】
 * 作用：执行“ransacNiHeSanWeiYuan”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool ransacNiHeSanWeiYuan(const std::vector<Eigen::Vector3f>& pts,     Eigen::Vector3f& center, Eigen::Vector3f& normal, float& radius,     float inlierDist, int maxIter)
{
    const int n = (int)pts.size();
    if (n < 12) return false;
    int bestCnt = 0; float bestR = 0;
    Eigen::Vector3f bestC(0,0,0), bestN(0,0,1);

    for (int iter = 0; iter < maxIter; ++iter) {
        int i1 = rand() % n;
        int i2 = rand() % n; while (i2 == i1) i2 = rand() % n;
        int i3 = rand() % n; while (i3 == i1 || i3 == i2) i3 = rand() % n;
        const Eigen::Vector3f& a = pts[(size_t)i1];
        const Eigen::Vector3f& b = pts[(size_t)i2];
        const Eigen::Vector3f& c = pts[(size_t)i3];

        // 3点确定平面，法向=ab×ac
        Eigen::Vector3f ab = b - a, ac = c - a;
        Eigen::Vector3f pn = ab.cross(ac);
        if (pn.norm() < 1e-8f) continue;
        pn.normalize(); if (pn.z() < 0) pn = -pn;

        // 在平面内建2D坐标系
        Eigen::Vector3f uu = (std::abs(pn.x()) < 0.9f) ?
            pn.cross(Eigen::Vector3f(1,0,0)).normalized() :
            pn.cross(Eigen::Vector3f(0,1,0)).normalized();
        Eigen::Vector3f vv = pn.cross(uu);

        // 3点投影到2D
        auto to2D = [&](const Eigen::Vector3f& p) -> Eigen::Vector2f {
            Eigen::Vector3f d = p - a;
            return Eigen::Vector2f(d.dot(uu), d.dot(vv));
        };
        Eigen::Vector2f a2 = to2D(a); // (0,0)
        Eigen::Vector2f b2 = to2D(b);
        Eigen::Vector2f c2 = to2D(c);

        // 2D三点圆拟合
        float da2 = b2.x()*b2.x() + b2.y()*b2.y();
        float db2 = c2.x()*c2.x() + c2.y()*c2.y();
        float A00 = 2.0f*b2.x(), A01 = 2.0f*b2.y(), B0 = da2;
        float A10 = 2.0f*c2.x(), A11 = 2.0f*c2.y(), B1 = db2;
        float det = A00*A11 - A01*A10;
        if (std::abs(det) < 1e-8f) continue;
        float cx2 = (B0*A11 - B1*A01) / det;
        float cy2 = (A00*B1 - A10*B0) / det;
        float r2 = std::sqrt(cx2*cx2 + cy2*cy2);
        if (r2 < 1.0f || r2 > 50.0f) continue;

        // 3D圆心
        Eigen::Vector3f c3 = a + cx2 * uu + cy2 * vv;

        // 计内点
        int cnt = 0;
        for (const auto& p : pts) {
            Eigen::Vector3f d = p - c3;
            float distToPlane = std::abs(d.dot(pn));
            float distInPlane = std::abs(std::sqrt(d.squaredNorm() - distToPlane*distToPlane) - r2);
            if (distToPlane < inlierDist && distInPlane < inlierDist) ++cnt;
        }
        if (cnt > bestCnt) { bestCnt = cnt; bestC = c3; bestN = pn; bestR = r2; }
    }
    if (bestCnt < std::max(8, n/5)) return false;

    // 用内点做Kasa精修
    std::vector<Eigen::Vector3f> inls;
    for (const auto& p : pts) {
        Eigen::Vector3f d = p - bestC;
        float dp = std::abs(d.dot(bestN));
        float di = std::abs(std::sqrt(d.squaredNorm() - dp*dp) - bestR);
        if (dp < inlierDist*1.5f && di < inlierDist*1.5f) inls.push_back(p);
    }
    if (inls.size() < 8) return false;

    // 内点PCA精修法向，再用内点平均圆心
    Eigen::Vector3f ic(0,0,0);
    for (const auto& p : inls) ic += p;
    ic /= (float)inls.size();
    Eigen::Matrix3f cv = Eigen::Matrix3f::Zero();
    for (const auto& p : inls) {
        Eigen::Vector3f d = p - ic; cv += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cv);
    if (es.info() != Eigen::Success) return false;
    Eigen::Vector3f rn = es.eigenvectors().col(0).normalized();
    if (rn.z() < 0) rn = -rn;

    // 投影内点到精修平面,用Kasa拟合2D圆得最终圆心和半径
    Eigen::Vector3f iu = (std::abs(rn.x())<0.9f)? rn.cross(Eigen::Vector3f(1,0,0)).normalized() : rn.cross(Eigen::Vector3f(0,1,0)).normalized();
    Eigen::Vector3f iv = rn.cross(iu);
    std::vector<Eigen::Vector2f> i2d;
    for (const auto& p : inls) {
        Eigen::Vector3f d = p - ic;
        i2d.emplace_back(d.dot(iu), d.dot(iv));
    }
    Eigen::Vector2f kc; float kr;
    if (!kasaNiHeYuan(i2d, kc, kr)) return false;
    if (!std::isfinite(kc.x()) || !std::isfinite(kc.y()) || !std::isfinite(kr) || kr < 0.5f) return false;

    center = ic + kc.x() * iu + kc.y() * iv;
    normal = rn;
    radius = kr;
    return true;
}
/** 【函数导航】
 * 作用：执行“shouDongZhongWeiShu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongZhongWeiShu(std::vector<float>& values)
{
    if (values.empty()) return 0.0f;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
}
/** 【函数导航】
 * 作用：执行“shouDongFenWeiShu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongFenWeiShu(std::vector<float>& values, float q)
{
    if (values.empty()) return 0.0f;
    q = std::clamp(q, 0.0f, 1.0f);
    const size_t suoYin = std::min(values.size() - 1,
        static_cast<size_t>(std::round(q * static_cast<float>(values.size() - 1))));
    std::nth_element(values.begin(), values.begin() + suoYin, values.end());
    return values[suoYin];
}

}


// ============================================================================
// 功能分区：支撑面估计实现
// ============================================================================
/*
模块职责：
孔口支撑面计算子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，为孔口中心、法向和深度提供局部参考面。

维护说明：
平面内点距离、迭代次数和采样范围属于敏感参数，应注明毫米单位及增减后果。
*/

namespace holeZhicheng {

/** 【函数导航】
 * 作用：执行“shouDongZhuZaiFaXiangGao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongZhuZaiFaXiangGao(const CloudPtr& localCloud)
{
    if (!localCloud || localCloud->empty()) return 0.0f;

    float zMin = std::numeric_limits<float>::max();
    float zMax = -std::numeric_limits<float>::max();
    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.z)) continue;
        zMin = std::min(zMin, p.z);
        zMax = std::max(zMax, p.z);
    }
    if (!(zMax > zMin)) return 0.0f;

    const float binW = 0.35f;
    const int nBins = std::max(3, std::min(800, static_cast<int>((zMax - zMin) / binW) + 2));
    std::vector<int> hist(nBins, 0);
    for (const auto& p : *localCloud) {
        int bi = static_cast<int>((p.z - zMin) / binW);
        if (bi < 0) bi = 0;
        if (bi >= nBins) bi = nBins - 1;
        hist[(size_t)bi]++;
    }
    int peak = 0;
    for (int i = 1; i < nBins; ++i) {
        if (hist[(size_t)i] > hist[(size_t)peak]) peak = i;
    }
    return zMin + (peak + 0.5f) * binW;
}
/** 【函数导航】
 * 作用：执行“shouDongJinLinZhuZaiFaXiangGao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongJinLinZhuZaiFaXiangGao(const CloudPtr& localCloud, float cx, float cy, float radius)
{
    if (!localCloud || localCloud->empty() || radius <= 0.0f) return shouDongZhuZaiFaXiangGao(localCloud);

    std::vector<float> zs;
    zs.reserve(localCloud->size());
    const float rLo = std::max(0.5f, radius * 0.90f);
    const float rHi = std::max(rLo + 0.5f, radius * 2.15f + 1.0f);
    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (rr >= rLo && rr <= rHi) zs.push_back(p.z);
    }
    if (zs.size() < 12) return shouDongZhuZaiFaXiangGao(localCloud);

    auto mm = std::minmax_element(zs.begin(), zs.end());
    const float zMin = *mm.first;
    const float zMax = *mm.second;
    if (!(zMax > zMin)) return zMin;

    const float binW = 0.30f;
    const int nBins = std::max(3, std::min(500, static_cast<int>((zMax - zMin) / binW) + 2));
    std::vector<int> hist((size_t)nBins, 0);
    for (float z : zs) {
        int bi = static_cast<int>((z - zMin) / binW);
        if (bi < 0) bi = 0;
        if (bi >= nBins) bi = nBins - 1;
        hist[(size_t)bi]++;
    }
    int peak = 0;
    for (int i = 1; i < nBins; ++i) {
        if (hist[(size_t)i] > hist[(size_t)peak]) peak = i;
    }
    return zMin + (peak + 0.5f) * binW;
}
/** 【函数导航】
 * 作用：执行“shouDongJinLinZhiChengMianFaXiangGao”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongJinLinZhiChengMianFaXiangGao(     const CloudPtr& localCloud,     float cx,     float cy,     float radius,     bool& outPlaneUsed,     int& outSupportSectors,     float& outResidual)
{
    outPlaneUsed = false;
    outSupportSectors = 0;
    outResidual = 0.0f;
    const float tuoDi = shouDongJinLinZhuZaiFaXiangGao(localCloud, cx, cy, radius);
    if (!localCloud || localCloud->empty() || radius <= 0.0f) return tuoDi;

    /** 【类型导航注释】
     * S：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct S { float x, y, z, a; };
    std::vector<S> raw;
    raw.reserve(localCloud->size());
    const bool largeRadiusForSupport = radius >= 4.80f;
    const float rLo = largeRadiusForSupport
        ? radius + std::clamp(radius * 0.20f, 1.20f, 2.05f)
        : std::max(radius * 1.12f, radius + std::clamp(radius * 0.16f, 0.55f, 1.10f));
    const float rHi = largeRadiusForSupport
        ? rLo + std::clamp(radius * 0.48f, 2.80f, 4.80f)
        : std::max(rLo + 1.20f, radius * 2.20f + 1.00f);
    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (rr < rLo || rr > rHi) continue;
        float a = std::atan2(dy, dx);
        if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
        raw.push_back({p.x, p.y, p.z, a});
    }
    if (raw.size() < 30) return tuoDi;

    std::array<std::vector<S>, 16> sectors;
    for (const auto& s : raw) {
        int bi = std::min(15, static_cast<int>(s.a / (2.0 * M_PI) * 16));
        sectors[(size_t)bi].push_back(s);
    }

    std::vector<Eigen::Vector3f> support;
    support.reserve(128);
    for (auto& bucket : sectors) {
        if (bucket.size() < 3) continue;
        std::sort(bucket.begin(), bucket.end(), [](const S& a, const S& b) { return a.z < b.z; });
        const float zMid = bucket[bucket.size() / 2].z;
        const float sectorBand = largeRadiusForSupport
            ? std::clamp(radius * 0.085f, 0.45f, 0.78f)
            : std::clamp(radius * 0.130f, 0.65f, 1.15f);
        std::sort(bucket.begin(), bucket.end(), [zMid](const S& a, const S& b) {
            return std::abs(a.z - zMid) < std::abs(b.z - zMid);
        });
        int kept = 0;
        const int perSectorLimit = largeRadiusForSupport ? 10 : 8;
        for (const auto& s : bucket) {
            if (std::abs(s.z - zMid) > sectorBand) continue;
            support.emplace_back(s.x, s.y, s.z);
            kept++;
            if (kept >= perSectorLimit) break;
        }
        if (kept > 0) outSupportSectors++;
    }
    if (support.size() < 24 || outSupportSectors < 5) return tuoDi;

    Eigen::Matrix3f ata = Eigen::Matrix3f::Zero();
    Eigen::Vector3f atb = Eigen::Vector3f::Zero();
    for (const auto& s : support) {
        Eigen::Vector3f row(s.x(), s.y(), 1.0f);
        ata += row * row.transpose();
        atb += row * s.z();
    }
    Eigen::Vector3f abc = ata.ldlt().solve(atb);
    if (!std::isfinite(abc.x()) || !std::isfinite(abc.y()) || !std::isfinite(abc.z())) return tuoDi;

    std::vector<float> residuals;
    residuals.reserve(support.size());
    for (const auto& s : support) {
        residuals.push_back(std::abs((abc.x() * s.x() + abc.y() * s.y() + abc.z()) - s.z()));
    }
    const size_t mid = residuals.size() / 2;
    std::nth_element(residuals.begin(), residuals.begin() + mid, residuals.end());
    outResidual = residuals[mid];
    const float residualGate = largeRadiusForSupport
        ? std::clamp(radius * 0.085f, 0.38f, 0.68f)
        : std::clamp(radius * 0.130f, 0.62f, 1.10f);
    if (outResidual > residualGate) return tuoDi;

    const float zAtCenter = abc.x() * cx + abc.y() * cy + abc.z();
    const float planeShift = zAtCenter - tuoDi;
    const float shiftLo = largeRadiusForSupport ? -2.80f : -4.00f;
    const float shiftHi = largeRadiusForSupport ? 5.20f : 8.00f;
    if (planeShift < shiftLo || planeShift > shiftHi) return tuoDi;
    outPlaneUsed = true;
    return zAtCenter;
}

}


// ============================================================================
// 功能分区：孔深与孔壁实现
// ============================================================================
/*
模块职责：
孔深与孔壁几何子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，处理沿孔轴方向的深度/内孔证据。

维护说明：
深度搜索范围、层间距和半径阈值会直接改变测量结果，不能在纯整理轮中改默认值。
*/

namespace holeShendu {
using holeJihe::shouDongFenWeiShu;
using holeJihe::shouDongZhongWeiShu;

/** 【函数导航】
 * 作用：执行“shouDongFenCengBanJing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool shouDongFenCengBanJing(     const CloudPtr& localCloud,     float cx,     float cy,     float seedR,     float z,     float halfThickness,     ShouDongFenCengTongJi& out,     float zCeil )
{
    if (!localCloud || seedR <= 0.0f) return false;

    std::vector<float> rs;
    rs.reserve(128);
    std::array<uint8_t, 16> sectors{};

    const float rLo = std::max(0.4f, seedR * 0.45f);
    const float rHi = seedR * 1.45f + 0.9f;
    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z > zCeil) continue;
        if (std::abs(p.z - z) > halfThickness) continue;
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (rr < rLo || rr > rHi) continue;
        rs.push_back(rr);

        float a = std::atan2(dy, dx);
        if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
        const int bi = std::min(15, static_cast<int>(a / (2.0 * M_PI) * 16));
        sectors[(size_t)bi] = 1;
    }

    int sectorCount = 0;
    for (uint8_t s : sectors) sectorCount += s ? 1 : 0;
    if (rs.size() < 8 || sectorCount < 3) return false;

    int maxGap = 0;
    if (sectorCount > 0) {
        for (int start = 0; start < 16; ++start) {
            int gap = 0;
            while (gap < 16 && sectors[(size_t)((start + gap) % 16)] == 0) ++gap;
            maxGap = std::max(maxGap, gap);
        }
    }

    const size_t mid = rs.size() / 2;
    std::nth_element(rs.begin(), rs.begin() + mid, rs.end());
    const float rMed = rs[mid];
    const size_t qInner = std::min(rs.size() - 1,
        static_cast<size_t>(std::round(static_cast<float>(rs.size() - 1) * 0.38f)));
    std::nth_element(rs.begin(), rs.begin() + qInner, rs.end());
    const float rInner = rs[qInner];
    const size_t qOuter = std::min(rs.size() - 1,
        static_cast<size_t>(std::round(static_cast<float>(rs.size() - 1) * 0.76f)));
    std::nth_element(rs.begin(), rs.begin() + qOuter, rs.end());
    const float rOuter = rs[qOuter];
    out.z = z;
    out.radius = rMed;
    out.radiusInner = rInner;
    out.radiusOuter = rOuter;
    out.pts = static_cast<int>(rs.size());
    out.sectors = sectorCount;
    out.maxSectorGap = maxGap;
    return true;
}
/** 【函数导航】
 * 作用：执行“shouDongZhongWeiBanJing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongZhongWeiBanJing(const std::vector<ShouDongFenCengTongJi>& layers, size_t begin, size_t end, int radiusMode )
{
    if (begin >= end || begin >= layers.size()) return 0.0f;
    end = std::min(end, layers.size());
    std::vector<float> rs;
    rs.reserve(end - begin);
    for (size_t i = begin; i < end; ++i) {
        float r = layers[i].radius;
        if (radiusMode == 1 && layers[i].radiusOuter > 0.0f) r = layers[i].radiusOuter;
        else if (radiusMode == 2 && layers[i].radiusInner > 0.0f) r = layers[i].radiusInner;
        if (r > 0.0f && std::isfinite(r))
            rs.push_back(r);
    }
    if (rs.empty()) return 0.0f;
    const size_t mid = rs.size() / 2;
    std::nth_element(rs.begin(), rs.begin() + mid, rs.end());
    return rs[mid];
}
/** 【函数导航】
 * 作用：执行“shouDongJinLinShenDuBanJing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
float shouDongJinLinShenDuBanJing(const std::vector<ShouDongFenCengTongJi>& layers, float targetT, float band, int radiusMode )
{
    if (layers.empty()) return 0.0f;
    std::vector<float> rs;
    rs.reserve(layers.size());
    for (const auto& l : layers) {
        if (std::abs(l.t - targetT) > band) continue;
        float r = l.radius;
        if (radiusMode == 1 && l.radiusOuter > 0.0f) r = l.radiusOuter;
        else if (radiusMode == 2 && l.radiusInner > 0.0f) r = l.radiusInner;
        if (r > 0.0f && std::isfinite(r)) rs.push_back(r);
    }

    if (rs.empty()) {
        size_t nearest = 0;
        float bestDt = std::numeric_limits<float>::max();
        for (size_t i = 0; i < layers.size(); ++i) {
            const float dt = std::abs(layers[i].t - targetT);
            if (dt < bestDt) {
                bestDt = dt;
                nearest = i;
            }
        }
        const size_t begin = nearest > 0 ? nearest - 1 : nearest;
        const size_t end = std::min(layers.size(), nearest + 2);
// 备忘：这个函数从多层半径中取稳健代表值，避免单层异常值直接影响结果。
        return shouDongZhongWeiBanJing(layers, begin, end, radiusMode);
    }

    const size_t mid = rs.size() / 2;
    std::nth_element(rs.begin(), rs.begin() + mid, rs.end());
    return rs[mid];
}
/** 【函数导航】
 * 作用：执行“shouDongJingXiangShenDu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShouDongJingXiangShenDu shouDongJingXiangShenDu(     const CloudPtr& localCloud,     float cx,     float cy,     float radius,     float zCeil )
{
    ShouDongJingXiangShenDu prof;
    if (!localCloud || radius <= 0.0f) return prof;

    std::vector<float> innerZ;
    std::vector<float> ringZ;
    std::vector<float> outerZ;
    std::array<uint8_t, 16> ringSector{};
    innerZ.reserve(256);
    ringZ.reserve(512);
    outerZ.reserve(1024);

    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z > zCeil) continue;
        const float dx = p.x - cx;
        const float dy = p.y - cy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (rr < radius * 0.62f) {
            innerZ.push_back(p.z);
        } else if (rr >= radius * 0.82f && rr <= radius * 1.36f) {
            ringZ.push_back(p.z);
            float a = std::atan2(dy, dx);
            if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
            ringSector[(size_t)std::min(15, static_cast<int>(a / (2.0 * M_PI) * 16))] = 1;
        } else if (rr >= radius * 1.42f && rr <= radius * 2.25f + 1.5f) {
            outerZ.push_back(p.z);
        }
    }

    prof.innerPts = static_cast<int>(innerZ.size());
    prof.ringPts = static_cast<int>(ringZ.size());
    prof.outerPts = static_cast<int>(outerZ.size());
    int ringBins = 0;
    for (uint8_t b : ringSector) ringBins += b ? 1 : 0;
    int maxGap = 0;
    if (ringBins > 0) {
        for (int start = 0; start < 16; ++start) {
            int gap = 0;
            while (gap < 16 && ringSector[(size_t)((start + gap) % 16)] == 0) ++gap;
            maxGap = std::max(maxGap, gap);
        }
    }
    prof.ringSectorCount = ringBins;
    prof.ringMaxSectorGap = maxGap;
    prof.ringCoverage = static_cast<float>(ringBins) / 16.0f;
    if (innerZ.size() < 8 || ringZ.size() < 16) return prof;

    const float innerLow = shouDongFenWeiShu(innerZ, 0.30f);
    prof.innerMed = shouDongZhongWeiShu(innerZ);
    prof.ringMed = shouDongZhongWeiShu(ringZ);
    prof.outerMed = outerZ.empty() ? prof.ringMed : shouDongZhongWeiShu(outerZ);
    prof.contrast = std::max(0.0f, std::max(prof.ringMed - innerLow,
                                            prof.outerMed - innerLow));
    prof.valid = true;
    return prof;
}

}


// ============================================================================
// 功能分区：孔口轮廓实现
// ============================================================================
/*
模块职责：
孔口轮廓计算子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，提取和整理物理孔口轮廓证据。

维护说明：
半径、覆盖率和轮廓门限会影响最终孔口几何，修改时必须保持正式 comparator 约束。
*/
#include <pcl/kdtree/kdtree_flann.h>

namespace holeKou {
using holeJihe::shouDongZhongWeiShu;
using holeShendu::shouDongZhongWeiBanJing;
using holeShendu::ShouDongFenCengTongJi;

/** 【函数导航】
 * 作用：执行“shouDongGuJiZhuiHoleKouBuBaoLuo”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShouDongZhuiHoleKouBaoLuoJingXiu shouDongGuJiZhuiHoleKouBuBaoLuo(     const std::vector<ShouDongFenCengTongJi>& layers,     float currentRTop,     float seedMaxRadius)
{
    ShouDongZhuiHoleKouBaoLuoJingXiu out;
    if (layers.size() < 3 || currentRTop <= 0.0f) return out;

    std::vector<float> headMed;
    std::vector<float> headOuter;
    std::vector<float> headInner;
    headMed.reserve(4);
    headOuter.reserve(4);
    headInner.reserve(4);

    float coverageSum = 0.0f;
    const size_t headEnd = std::min<size_t>(4, layers.size());
    for (size_t i = 0; i < headEnd; ++i) {
        const auto& l = layers[i];
        if (l.pts < 8 || l.sectors < 5 || l.maxSectorGap > 12) continue;
        if (l.radius <= 0.0f || l.radiusOuter <= 0.0f || l.radiusInner <= 0.0f) continue;
        headMed.push_back(l.radius);
        headOuter.push_back(l.radiusOuter);
        headInner.push_back(l.radiusInner);
        coverageSum += std::clamp(static_cast<float>(l.sectors) / 16.0f, 0.0f, 1.0f);
    }
    if (headOuter.size() < 2) return out;

    std::vector<float> medCopy = headMed;
    std::vector<float> outerCopy = headOuter;
    std::vector<float> innerCopy = headInner;
    const float medR = shouDongZhongWeiShu(medCopy);
    const float outerR = shouDongZhongWeiShu(outerCopy);
    const float innerR = shouDongZhongWeiShu(innerCopy);
    if (outerR <= 0.0f || medR <= 0.0f || innerR <= 0.0f) return out;

    const float spread = outerR - medR;
    const float innerSpread = medR - innerR;
    const float grow = outerR - currentRTop;
    if (grow <= 0.12f) return out;
    if (outerR > seedMaxRadius) return out;

    const float spreadGate = std::max(0.42f, std::min(0.92f, medR * 0.17f));
    if (spread < 0.05f || spread > spreadGate) return out;
    if (innerSpread > std::max(1.05f, medR * 0.24f)) return out;

    const float tailR = shouDongZhongWeiBanJing(
        layers,
        layers.size() > 3 ? layers.size() - 3 : 0,
        layers.size());
    const float shrink = (tailR > 0.0f) ? (medR - tailR) : 0.0f;
    const float coverage = coverageSum / static_cast<float>(headOuter.size());

    out.layers = static_cast<int>(headOuter.size());
    out.radius = outerR;
    out.grow = grow;
    out.spread = spread;
    out.shrink = shrink;
    out.coverage = coverage;
    out.score = std::clamp(
        coverage * 0.42f
        + std::clamp(spread / spreadGate, 0.0f, 1.0f) * 0.26f
        + std::clamp(shrink / std::max(0.60f, currentRTop * 0.14f), 0.0f, 1.0f) * 0.22f
        + std::clamp(1.0f - grow / std::max(0.70f, currentRTop * 0.18f), 0.0f, 1.0f) * 0.10f,
        0.0f, 1.0f);
    out.usable = out.score >= 0.58f;
    return out;
}
/** 【函数导航】
 * 作用：执行“shouDongGuJiZhuiBiKouBuWaiTui”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShouDongZhuiBiKouBuWaiTui shouDongGuJiZhuiBiKouBuWaiTui(     const std::vector<ShouDongFenCengTongJi>& layers,     float currentOutputRTop,     float seedMaxRadius)
{
    ShouDongZhuiBiKouBuWaiTui out;
    if (layers.size() < 4 || currentOutputRTop <= 0.0f) return out;

    /** 【类型导航注释】
     * NiHeYangBen：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct NiHeYangBen { float t, r, w; };
    std::vector<NiHeYangBen> samples;
    samples.reserve(8);
    for (const auto& l : layers) {
        if (l.t > 5.6f) break;
        if (l.t < 0.55f) continue;
        if (l.pts < 8 || l.sectors < 5 || l.maxSectorGap > 12) continue;
        if (l.radius <= 0.0f || l.radiusOuter <= 0.0f) continue;
        const float outerLift = std::clamp(l.radiusOuter - l.radius, 0.0f,
            std::min(0.82f, std::max(0.30f, l.radius * 0.15f)));
        const float wallR = l.radius + outerLift * 0.60f;
        if (wallR <= 0.0f || wallR > seedMaxRadius + 0.20f) continue;
        const float w = std::clamp(static_cast<float>(l.sectors) / 16.0f, 0.25f, 1.0f);
        samples.push_back({l.t, wallR, w});
    }
    if (samples.size() < 3) return out;

    float sw = 0.0f, st = 0.0f, sr = 0.0f;
    for (const auto& s : samples) {
        sw += s.w;
        st += s.w * s.t;
        sr += s.w * s.r;
    }
    if (sw <= 0.0f) return out;
    const float mt = st / sw;
    const float mr = sr / sw;
    float num = 0.0f, den = 0.0f;
    for (const auto& s : samples) {
        const float dt = s.t - mt;
        num += s.w * dt * (s.r - mr);
        den += s.w * dt * dt;
    }
    if (den <= 1e-4f) return out;
    const float slope = num / den;
    const float intercept = mr - slope * mt;
    if (!std::isfinite(intercept) || !std::isfinite(slope)) return out;

    double err = 0.0;
    for (const auto& s : samples) {
        const float pred = intercept + slope * s.t;
        const float e = pred - s.r;
        err += static_cast<double>(s.w) * e * e;
    }
    const float rms = static_cast<float>(std::sqrt(err / std::max(0.001f, sw)));
    const float grow = intercept - currentOutputRTop;

    if (slope > -0.065f) return out;
    if (rms > std::max(0.45f, currentOutputRTop * 0.105f)) return out;
    if (grow <= 0.15f || grow > std::max(1.20f, currentOutputRTop * 0.24f)) return out;
    if (intercept > seedMaxRadius || intercept < currentOutputRTop * 0.82f) return out;

    out.layers = static_cast<int>(samples.size());
    out.radius = intercept;
    out.grow = grow;
    out.slope = slope;
    out.rms = rms;
    out.score = std::clamp(
        std::clamp((-slope) / 0.42f, 0.0f, 1.0f) * 0.36f
        + std::clamp(1.0f - rms / std::max(0.45f, currentOutputRTop * 0.105f), 0.0f, 1.0f) * 0.34f
        + std::clamp(static_cast<float>(samples.size()) / 5.0f, 0.0f, 1.0f) * 0.18f
        + std::clamp(1.0f - grow / std::max(0.90f, currentOutputRTop * 0.18f), 0.0f, 1.0f) * 0.12f,
        0.0f, 1.0f);
    out.usable = out.score >= 0.58f;
    return out;
}
/** 【函数导航】
 * 作用：估计“guJiWangGeHoleKouYinYing”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShouDongWangGeHoleKou guJiWangGeHoleKouYinYing(     const CloudPtr& localCloud,     float seedCx,     float seedCy,     float seedR,     float mouthZ,     const ShouDongHoleSeed& seed)
{
    ShouDongWangGeHoleKou out;
    out.executed = true;
    out.cx = seedCx;
    out.cy = seedCy;
    if (!localCloud || localCloud->size() < 30 || seedR <= 0.0f) return out;

    const float cell = 0.50f;
    const float extent = std::min(std::max(seedR * 2.8f + 4.0f, 10.0f),
                                  std::max(10.0f, seed.searchRadiusMm * 0.92f));
    const int n = static_cast<int>(std::ceil((extent * 2.0f) / cell)) + 1;
    if (n < 12 || n > 96) return out;
    const int total = n * n;
    const float x0 = seedCx - extent;
    const float y0 = seedCy - extent;

    /** 【类型导航注释】
     * WangGeDanYuan：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct WangGeDanYuan {
        int count = 0;
        float zMax = -std::numeric_limits<float>::max();
        bool top = false;
        bool low = false;
        bool seen = false;
    };
    std::vector<WangGeDanYuan> grid(static_cast<size_t>(total));
    auto suoYin = [&](int ix, int iy) { return iy * n + ix; };
    auto shiFouZaiNei = [&](int ix, int iy) { return ix >= 0 && iy >= 0 && ix < n && iy < n; };
    auto geZiZhongXin = [&](int ix, int iy) {
        return Eigen::Vector2f(x0 + (static_cast<float>(ix) + 0.5f) * cell,
                               y0 + (static_cast<float>(iy) + 0.5f) * cell);
    };

    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z > mouthZ + 3.0f || p.z < mouthZ - 12.0f) continue;
        const int ix = static_cast<int>(std::floor((p.x - x0) / cell));
        const int iy = static_cast<int>(std::floor((p.y - y0) / cell));
        if (!shiFouZaiNei(ix, iy)) continue;
        WangGeDanYuan& c = grid[static_cast<size_t>(suoYin(ix, iy))];
        c.count++;
        c.zMax = std::max(c.zMax, p.z);
    }

    const float topGate = mouthZ - std::max(0.70f, seedR * 0.10f);
    const float lowGate = mouthZ - std::max(1.10f, seedR * 0.20f);
    for (int iy = 0; iy < n; ++iy) {
        for (int ix = 0; ix < n; ++ix) {
            WangGeDanYuan& c = grid[static_cast<size_t>(suoYin(ix, iy))];
            c.top = c.count >= 1 && c.zMax >= topGate;
            c.low = c.count == 0 || c.zMax <= lowGate;
        }
    }

    int start = -1;
    float bestSeedDist = std::numeric_limits<float>::max();
    const float startSearchR = std::max(seedR * 1.10f, 2.2f);
    for (int iy = 0; iy < n; ++iy) {
        for (int ix = 0; ix < n; ++ix) {
            const int id = suoYin(ix, iy);
            if (!grid[static_cast<size_t>(id)].low) continue;
            const Eigen::Vector2f cc = geZiZhongXin(ix, iy);
            const float d = (cc - Eigen::Vector2f(seedCx, seedCy)).norm();
            if (d > startSearchR || d >= bestSeedDist) continue;
            bool nearTop = false;
            for (int oy = -1; oy <= 1 && !nearTop; ++oy) {
                for (int ox = -1; ox <= 1; ++ox) {
                    if (ox == 0 && oy == 0) continue;
                    const int nx = ix + ox, ny = iy + oy;
                    if (shiFouZaiNei(nx, ny) && grid[static_cast<size_t>(suoYin(nx, ny))].top) {
                        nearTop = true;
                        break;
                    }
                }
            }
            if (!nearTop) continue;
            start = id;
            bestSeedDist = d;
        }
    }
    if (start < 0) return out;

    std::vector<int> comp;
    comp.reserve(512);
    std::queue<int> q;
    q.push(start);
    grid[static_cast<size_t>(start)].seen = true;
    const float compLimitR = std::min(extent * 0.86f, std::max(seedR * 1.95f, 5.0f));
    const int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    while (!q.empty()) {
        const int id = q.front();
        q.pop();
        comp.push_back(id);
        const int ix = id % n;
        const int iy = id / n;
        for (const auto& dxy : dirs4) {
            const int nx = ix + dxy[0], ny = iy + dxy[1];
            if (!shiFouZaiNei(nx, ny)) continue;
            const int nid = suoYin(nx, ny);
            WangGeDanYuan& nc = grid[static_cast<size_t>(nid)];
            if (nc.seen || !nc.low) continue;
            const Eigen::Vector2f cc = geZiZhongXin(nx, ny);
            if ((cc - Eigen::Vector2f(seedCx, seedCy)).norm() > compLimitR) continue;
            nc.seen = true;
            q.push(nid);
        }
        if (comp.size() > 1800) break;
    }
    out.voidCells = static_cast<int>(comp.size());
    if (comp.size() < 6) return out;

    std::vector<Eigen::Vector2f> edgePts;
    edgePts.reserve(256);
    std::array<uint8_t, 16> sector{};
    for (int id : comp) {
        const int ix = id % n;
        const int iy = id / n;
        for (int oy = -1; oy <= 1; ++oy) {
            for (int ox = -1; ox <= 1; ++ox) {
                if (ox == 0 && oy == 0) continue;
                const int nx = ix + ox, ny = iy + oy;
                if (!shiFouZaiNei(nx, ny)) continue;
                const WangGeDanYuan& tc = grid[static_cast<size_t>(suoYin(nx, ny))];
                if (!tc.top) continue;
                const Eigen::Vector2f ep = geZiZhongXin(nx, ny);
                const float rr = (ep - Eigen::Vector2f(seedCx, seedCy)).norm();
                if (rr < std::max(0.55f, seed.minRadiusMm * 0.45f)
                    || rr > std::min(seed.maxRadiusMm * 1.18f, extent * 0.92f)) continue;
                edgePts.push_back(ep);
            }
        }
    }
    if (edgePts.size() < 8) return out;

    std::sort(edgePts.begin(), edgePts.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        if (a.x() == b.x()) return a.y() < b.y();
        return a.x() < b.x();
    });
    edgePts.erase(std::unique(edgePts.begin(), edgePts.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return std::abs(a.x() - b.x()) < 1e-4f && std::abs(a.y() - b.y()) < 1e-4f;
    }), edgePts.end());
    out.edgeCells = static_cast<int>(edgePts.size());
    if (edgePts.size() < 8) return out;

    Eigen::Matrix3f ata = Eigen::Matrix3f::Zero();
    Eigen::Vector3f atb = Eigen::Vector3f::Zero();
    for (const auto& p : edgePts) {
        Eigen::Vector3f row(p.x(), p.y(), 1.0f);
        ata += row * row.transpose();
        atb += row * (-(p.x() * p.x() + p.y() * p.y()));
    }
    const Eigen::Vector3f sol = ata.ldlt().solve(atb);
    if (!std::isfinite(sol.x()) || !std::isfinite(sol.y()) || !std::isfinite(sol.z())) return out;
    const float cx = -0.5f * sol.x();
    const float cy = -0.5f * sol.y();
    const float rr2 = cx * cx + cy * cy - sol.z();
    if (rr2 <= 0.0f || !std::isfinite(rr2)) return out;
    const float radius = std::sqrt(rr2);
    if (radius < std::max(0.6f, seed.minRadiusMm * 0.55f) || radius > seed.maxRadiusMm * 1.12f) return out;

    std::vector<float> residuals;
    residuals.reserve(edgePts.size());
    for (const auto& p : edgePts) {
        residuals.push_back(std::abs((p - Eigen::Vector2f(cx, cy)).norm() - radius));
        float a = std::atan2(p.y() - cy, p.x() - cx);
        if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
        sector[static_cast<size_t>(std::min(15, static_cast<int>(a / (2.0 * M_PI) * 16)))] = 1;
    }
    const size_t residualMid = residuals.size() / 2;
    std::nth_element(residuals.begin(), residuals.begin() + residualMid, residuals.end());
    out.residual = residuals[residualMid];
    int sectors = 0;
    for (uint8_t s : sector) sectors += s ? 1 : 0;
    out.sectors = sectors;
    out.cx = cx;
    out.cy = cy;
    out.radius = radius;
    out.centerShift = std::sqrt((cx - seedCx) * (cx - seedCx) + (cy - seedCy) * (cy - seedCy));

    const float sectorScore = std::min(1.0f, static_cast<float>(sectors) / 12.0f);
    const float edgeScore = std::min(1.0f, static_cast<float>(out.edgeCells) / 36.0f);
    const float residualScore = std::clamp(1.0f - out.residual / std::max(0.75f, radius * 0.20f), 0.0f, 1.0f);
    const float shiftScore = std::clamp(1.0f - out.centerShift / std::max(2.8f, seedR * 0.80f), 0.0f, 1.0f);
    out.score = std::clamp(0.34f * sectorScore + 0.24f * edgeScore
        + 0.24f * residualScore + 0.18f * shiftScore, 0.0f, 1.0f);
    out.stable = out.sectors >= 8
        && out.edgeCells >= 16
        && out.residual <= std::max(0.95f, radius * 0.22f)
        && out.centerShift <= std::max(4.2f, seedR * 1.05f)
        && out.score >= 0.58f;
    return out;
}
/** 【函数导航】
 * 作用：估计“guJiDaBanJingHoleKouWaiBianJie”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ShouDongWangGeHoleKou guJiDaBanJingHoleKouWaiBianJie(     const CloudPtr& localCloud,     float seedCx,     float seedCy,     float seedR,     float mouthZ,     const ShouDongHoleSeed& seed)
{
    ShouDongWangGeHoleKou out;
    out.executed = true;
    out.cx = seedCx;
    out.cy = seedCy;
    if (!localCloud || localCloud->size() < 80 || seedR <= 0.0f) return out;
    if (seed.maxRadiusMm <= seed.minRadiusMm) return out;

    // 备忘：这版不再用“第一处空缺后重现表面”作为唯一边界。
    // 大锥孔的真实上口往往是径向高度曲线的外侧高点；
    // 如果只找第一处表面重现，容易落在内圈低点/缺测区边界，半径就会偏小。
    constexpr int kRadialBins = 34;
    constexpr int kSectors = 36;
    const float rLo = std::max(seed.minRadiusMm * 0.55f, seedR * 0.50f);
    const float rHi = std::min(seed.searchRadiusMm * 0.72f, seed.maxRadiusMm * 1.08f);
    if (rHi <= rLo + std::max(0.8f, seed.maxRadiusMm * 0.10f)) return out;
    const float binW = (rHi - rLo) / static_cast<float>(kRadialBins);
    if (!(binW > 1e-4f)) return out;

    /** 【类型导航注释】
     * JingXiangFenXiang：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct JingXiangFenXiang {
        std::vector<float> z;
        std::array<uint8_t, kSectors> sectors{};
        int count = 0;
    };
    std::array<JingXiangFenXiang, kRadialBins> bins;

    auto fenWei = [](std::vector<float> values, float q) -> float {
        if (values.empty()) return 0.0f;
        q = std::clamp(q, 0.0f, 1.0f);
        const size_t id = std::min(values.size() - 1,
            static_cast<size_t>(std::round(q * static_cast<float>(values.size() - 1))));
        std::nth_element(values.begin(), values.begin() + id, values.end());
        return values[id];
    };

    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float dx = p.x - seedCx;
        const float dy = p.y - seedCy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (rr < rLo || rr > rHi) continue;
        const int ri = std::clamp(static_cast<int>((rr - rLo) / binW), 0, kRadialBins - 1);
        float a = std::atan2(dy, dx);
        if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
        const int si = std::min(kSectors - 1, static_cast<int>(a / (2.0 * M_PI) * kSectors));
        bins[(size_t)ri].z.push_back(p.z);
        bins[(size_t)ri].sectors[(size_t)si] = 1;
        bins[(size_t)ri].count++;
    }

    std::array<float, kRadialBins> zRep{};
    std::array<float, kRadialBins> cov{};
    std::array<uint8_t, kRadialBins> ok{};
    std::vector<float> validZ;
    validZ.reserve(kRadialBins);
    for (int i = 0; i < kRadialBins; ++i) {
        int sec = 0;
        for (uint8_t s : bins[(size_t)i].sectors) sec += s ? 1 : 0;
        cov[(size_t)i] = static_cast<float>(sec) / static_cast<float>(kSectors);
        if (bins[(size_t)i].z.size() < 8 || sec < 3) continue;
        // 用较高分位代表该半径环的上表面/孔口边界高度。
        // 中位数太容易被孔壁斜面拉低，最大值又容易受飞点影响，所以取 70% 分位。
        zRep[(size_t)i] = fenWei(bins[(size_t)i].z, 0.70f);
        ok[(size_t)i] = 1;
        validZ.push_back(zRep[(size_t)i]);
    }
    if (validZ.size() < 6) return out;

    std::vector<float> zForRange = validZ;
    const float z10 = fenWei(zForRange, 0.10f);
    zForRange = validZ;
    const float z90 = fenWei(zForRange, 0.90f);
    const float zRange = std::max(0.001f, z90 - z10);

    int bestI = -1;
    float bestScore = -std::numeric_limits<float>::max();
    for (int i = 1; i + 1 < kRadialBins; ++i) {
        if (!ok[(size_t)i]) continue;
        float innerMin = std::numeric_limits<float>::max();
        int innerN = 0;
        for (int j = 0; j < i; ++j) {
            if (!ok[(size_t)j]) continue;
            innerMin = std::min(innerMin, zRep[(size_t)j]);
            innerN++;
        }
        std::vector<float> outerVals;
        for (int j = i + 1; j < kRadialBins; ++j) {
            if (ok[(size_t)j]) outerVals.push_back(zRep[(size_t)j]);
        }
        if (innerN < 2 || outerVals.size() < 2) continue;
        const float outerMed = fenWei(outerVals, 0.50f);
        const float rise = std::max(0.0f, zRep[(size_t)i] - innerMin);
        const float outerDrop = std::max(0.0f, zRep[(size_t)i] - outerMed);
        const float riseScore = std::clamp(rise / zRange, 0.0f, 1.0f);
        const float dropScore = std::clamp(outerDrop / zRange, 0.0f, 1.0f);
        const float radius = rLo + (static_cast<float>(i) + 0.5f) * binW;
        const float radiusNearMax = std::clamp(radius / std::max(seed.maxRadiusMm, 1e-3f), 0.0f, 1.25f);
        // 半径项不是补偿，只是避免把很靠内的第一圈低面边界当成上口。
        const float score = cov[(size_t)i] * 0.30f
            + riseScore * 0.38f
            + dropScore * 0.18f
            + std::min(1.0f, radiusNearMax) * 0.14f;
        if (score > bestScore) {
            bestScore = score;
            bestI = i;
        }
    }
    if (bestI < 0) return out;

    const float baseR = rLo + (static_cast<float>(bestI) + 0.5f) * binW;
    const float peakZ = zRep[(size_t)bestI];
    std::array<std::vector<float>, kSectors> sectorR;
    const float radialWindow = std::max(binW * 1.8f, seed.maxRadiusMm * 0.055f);
    const float zWindow = std::max(zRange * 0.18f, binW);
    for (const auto& p : *localCloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float dx = p.x - seedCx;
        const float dy = p.y - seedCy;
        const float rr = std::sqrt(dx * dx + dy * dy);
        if (std::abs(rr - baseR) > radialWindow) continue;
        if (std::abs(p.z - peakZ) > zWindow) continue;
        float a = std::atan2(dy, dx);
        if (a < 0.0f) a += static_cast<float>(2.0 * M_PI);
        const int si = std::min(kSectors - 1, static_cast<int>(a / (2.0 * M_PI) * kSectors));
        sectorR[(size_t)si].push_back(rr);
    }

    std::vector<float> radii;
    radii.reserve(kSectors);
    for (auto& rs : sectorR) {
        if (rs.size() < 2) continue;
        radii.push_back(fenWei(rs, 0.50f));
    }
    if (radii.size() < 9) return out;

    std::vector<float> radiiCopy = radii;
    float radius = fenWei(radiiCopy, 0.50f);
    if (radius < seed.minRadiusMm * 0.70f || radius > seed.maxRadiusMm * 1.08f) return out;
    radius = std::min(seed.maxRadiusMm, radius);

    std::vector<float> absDev;
    absDev.reserve(radii.size());
    for (float r : radii) absDev.push_back(std::abs(r - radius));
    const float residual = fenWei(absDev, 0.50f);
    out.cx = seedCx;
    out.cy = seedCy;
    out.radius = radius;
    out.residual = residual;
    out.centerShift = 0.0f;
    out.edgeCells = static_cast<int>(radii.size());
    out.sectors = static_cast<int>(radii.size());
    out.voidCells = bestI;

    const float sectorScore = std::min(1.0f, static_cast<float>(radii.size()) / 18.0f);
    const float residualScore = std::clamp(1.0f - residual / std::max(binW * 1.4f, radius * 0.075f), 0.0f, 1.0f);
    const float peakScore = std::clamp((zRep[(size_t)bestI] - z10) / zRange, 0.0f, 1.0f);
    out.score = std::clamp(sectorScore * 0.38f + residualScore * 0.28f
        + peakScore * 0.22f + bestScore * 0.12f, 0.0f, 1.0f);
    out.stable = radii.size() >= 10
        && residual <= std::max(binW * 1.5f, radius * 0.085f)
        && out.score >= 0.58f;
    return out;
}

}


// ============================================================================
// 功能分区：表面与截面基础几何
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/
#include <array>
#include <cstddef>
#include <utility>

namespace HoleJiheSurfacePouMian {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSurfaceSectors = 48;
constexpr int kProfileSectors = 36;
constexpr double kDepthStep = 0.40;
constexpr double kLayerHalfThickness = 0.20;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double x) noexcept { return std::isfinite(x); }

/** 【函数导航】
 * 作用：执行“quantile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double quantile(std::vector<double> values, double q)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    q = std::clamp(q, 0.0, 1.0);
    std::sort(values.begin(), values.end());
    const double p = q * static_cast<double>(values.size() - 1);
    const std::size_t i0 = static_cast<std::size_t>(std::floor(p));
    const std::size_t i1 = static_cast<std::size_t>(std::ceil(p));
    const double a = p - static_cast<double>(i0);
    return values[i0] * (1.0 - a) + values[i1] * a;
}

double median(std::vector<double> values) { return quantile(std::move(values), 0.5); }

/** 【类型导航注释】
 * Point2：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Point2 { double x = 0.0; double y = 0.0; };

/** 【类型导航注释】
 * SurfaceGuJi：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct SurfaceGuJi {
    bool valid = false;
    double centerU = 0.0;
    double centerV = 0.0;
    double radius = 0.0;
    double mad = 0.0;
    int sectors = 0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：评估/审核“evaluateSurfaceAt”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
SurfaceGuJi evaluateSurfaceAt(
    const std::vector<Point2>& topPoints,
    double centerU,
    double centerV,
    double originU,
    double originV,
    double centerShiftPenalty)
{
    std::array<std::vector<double>, kSurfaceSectors> sectorRadii;
    for (const Point2& point : topPoints) {
        const double du = point.x - centerU;
        const double dv = point.y - centerV;
        const double radius = std::hypot(du, dv);
        if (radius < 0.20 || radius > 14.0) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        int sector = static_cast<int>(std::floor(
            angle / (2.0 * kPi) * static_cast<double>(kSurfaceSectors)));
        sector = std::clamp(sector, 0, kSurfaceSectors - 1);
        sectorRadii[static_cast<std::size_t>(sector)].push_back(radius);
    }

    std::vector<double> edges;
    edges.reserve(kSurfaceSectors);
    for (auto& values : sectorRadii) {
        if (values.size() < 3) continue;
        std::sort(values.begin(), values.end());
        std::size_t begin = 0;
        bool found = false;
        for (std::size_t end = 1; end <= values.size(); ++end) {
            const bool cut = end == values.size()
                || values[end] - values[end - 1] > 0.55;
            if (!cut) continue;
            const std::size_t count = end - begin;
            const double span = count > 0 ? values[end - 1] - values[begin] : 0.0;
            if (count >= 3 && span >= 1.00) {

                edges.push_back(values[begin] + std::min(0.15, 0.05 * span));
                found = true;
                break;
            }
            begin = end;
        }
        (void)found;
    }

    SurfaceGuJi estimate;
    if (edges.size() < 12) return estimate;
    const double radius = median(edges);
    std::vector<double> deviations;
    deviations.reserve(edges.size());
    for (double edge : edges) deviations.push_back(std::abs(edge - radius));
    const double mad = median(std::move(deviations));
    if (!finite(radius) || !finite(mad) || radius < 0.50 || radius > 11.0) return estimate;

    estimate.valid = true;
    estimate.centerU = centerU;
    estimate.centerV = centerV;
    estimate.radius = radius;
    estimate.mad = mad;
    estimate.sectors = static_cast<int>(edges.size());
    const double shift = std::hypot(centerU - originU, centerV - originV);
    estimate.score = static_cast<double>(estimate.sectors)
        - 20.0 * estimate.mad
        - centerShiftPenalty * shift;
    return estimate;
}

/** 【函数导航】
 * 作用：检测/搜索“searchSurface”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
SurfaceGuJi searchSurface(
    const std::vector<Sample>& samples,
    const Input& input)
{
    std::vector<Point2> topPoints;
    topPoints.reserve(samples.size());
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        if (std::abs(sample.w - input.topW) <= 0.24) {
            topPoints.push_back({sample.u, sample.v});
        }
    }
    if (topPoints.size() < 50) {
        for (const Sample& sample : samples) {
            if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
            if (std::abs(sample.w - input.topW) <= 0.36) {
                topPoints.push_back({sample.u, sample.v});
            }
        }
    }
    if (topPoints.size() < 50) return {};

    SurfaceGuJi best;

    const double maxShift = std::clamp(input.maxCenterShift, 0.0, 6.0);
    const double coarseStep = maxShift <= 1.25 ? 0.25 : (maxShift <= 2.5 ? 0.50 : 1.0);
    for (double du = -maxShift; du <= maxShift + 1e-9; du += coarseStep) {
        for (double dv = -maxShift; dv <= maxShift + 1e-9; dv += coarseStep) {
            if (std::hypot(du, dv) > maxShift + 1e-9) continue;
            const SurfaceGuJi candidate = evaluateSurfaceAt(
                topPoints, input.topU + du, input.topV + dv,
                input.topU, input.topV, input.centerShiftPenalty);
            if (candidate.valid && (!best.valid || candidate.score > best.score)) best = candidate;
        }
    }
    if (!best.valid) return best;

    for (const double step : {0.25, 0.10}) {
        const double nominalRange = step == 0.25 ? 0.80 : 0.30;
        const double remaining = std::max(0.0, maxShift
            - std::hypot(best.centerU - input.topU, best.centerV - input.topV));
        const double range = std::min(nominalRange, remaining);
        SurfaceGuJi refined = best;
        for (double du = -range; du <= range + 1e-9; du += step) {
            for (double dv = -range; dv <= range + 1e-9; dv += step) {
                const double candidateU = best.centerU + du;
                const double candidateV = best.centerV + dv;
                if (std::hypot(candidateU - input.topU, candidateV - input.topV)
                    > maxShift + 1e-9) continue;
                const SurfaceGuJi candidate = evaluateSurfaceAt(
                    topPoints, candidateU, candidateV,
                    input.topU, input.topV, input.centerShiftPenalty);
                if (candidate.valid && candidate.score > refined.score) refined = candidate;
            }
        }
        best = refined;
    }
    return best;
}

/** 【类型导航注释】
 * JingXiangPoint：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JingXiangPoint {
    double radius = 0.0;
    double angle = 0.0;
};

/** 【函数导航】
 * 作用：执行“measureLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool measureLayer(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    double surfaceRadius,
    int polarity,
    double depth,
    Layer& layer)
{
    std::vector<JingXiangPoint> radial;
    radial.reserve(256);
    const double maxRadius = std::min(14.0, surfaceRadius + 5.0);
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double pointDepth = static_cast<double>(polarity) * (sample.w - topW);
        if (std::abs(pointDepth - depth) > kLayerHalfThickness) continue;
        const double du = sample.u - centerU;
        const double dv = sample.v - centerV;
        const double radius = std::hypot(du, dv);
        if (radius < std::max(0.30, 0.08 * surfaceRadius) || radius > maxRadius) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        radial.push_back({radius, angle});
    }
    if (radial.size() < 5) return false;
    std::sort(radial.begin(), radial.end(), [](const JingXiangPoint& a, const JingXiangPoint& b) {
        return a.radius < b.radius;
    });

    std::size_t begin = 0;
    for (std::size_t end = 1; end <= radial.size(); ++end) {
        const bool cut = end == radial.size()
            || radial[end].radius - radial[end - 1].radius > 0.60;
        if (!cut) continue;
        const std::size_t count = end - begin;
        if (count >= 5) {
            std::array<bool, kProfileSectors> used{};
            std::vector<double> radii;
            radii.reserve(count);
            for (std::size_t i = begin; i < end; ++i) {
                int sector = static_cast<int>(std::floor(
                    radial[i].angle / (2.0 * kPi) * static_cast<double>(kProfileSectors)));
                sector = std::clamp(sector, 0, kProfileSectors - 1);
                used[static_cast<std::size_t>(sector)] = true;
                radii.push_back(radial[i].radius);
            }
            int sectors = 0;
            for (bool value : used) if (value) ++sectors;
            if (sectors >= 5) {
                const double radius = median(radii);
                const double span = radial[end - 1].radius - radial[begin].radius;
                const double maximumSpan = std::max(1.60, 0.32 * surfaceRadius);
                if (radius <= surfaceRadius + std::max(0.50, 0.08 * surfaceRadius)
                    && span <= maximumSpan) {
                    layer.depth = depth;
                    layer.radius = radius;
                    layer.coverage = static_cast<double>(sectors)
                        / static_cast<double>(kProfileSectors);
                    layer.span = span;
                    layer.points = static_cast<int>(count);
                    layer.sectors = sectors;
                    return true;
                }
            }
        }
        begin = end;
    }
    return false;
}

/** 【函数导航】
 * 作用：执行“robustSlope”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double robustSlope(const std::vector<Layer>& layers)
{
    std::vector<double> slopes;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        for (std::size_t j = i + 1; j < layers.size(); ++j) {
            const double dd = layers[j].depth - layers[i].depth;
            if (dd >= 0.35) slopes.push_back((layers[j].radius - layers[i].radius) / dd);
        }
    }
    return slopes.empty() ? 0.0 : median(std::move(slopes));
}

/** 【函数导航】
 * 作用：执行“robustLine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::pair<double, double> robustLine(const std::vector<Layer>& layers)
{
    if (layers.empty()) return {0.0, 0.0};
    const double slope = robustSlope(layers);
    std::vector<double> intercepts;
    intercepts.reserve(layers.size());
    for (const Layer& layer : layers) intercepts.push_back(layer.radius - slope * layer.depth);
    return {median(std::move(intercepts)), slope};
}

/** 【类型导航注释】
 * DirectionPouMian：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct DirectionPouMian {
    bool valid = false;
    int polarity = 0;
    std::vector<Layer> layers;
    double slope = 0.0;
    double shrink = 0.0;
    double monotonicRatio = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：构建“buildDirection”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
DirectionPouMian buildDirection(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    double surfaceRadius,
    int polarity)
{
    const double maxDepth = std::min(14.0, std::max(4.0, 1.8 * surfaceRadius + 2.0));
    std::vector<Layer> measured;
    for (double depth = kDepthStep; depth <= maxDepth + 1e-9; depth += kDepthStep) {
        Layer layer;
        if (measureLayer(samples, centerU, centerV, topW, surfaceRadius,
                         polarity, depth, layer)) {
            measured.push_back(layer);
        }
    }

    DirectionPouMian best;
    best.polarity = polarity;
    for (std::size_t start = 0; start < measured.size(); ++start) {
        if (measured[start].depth > 2.0) continue;
        std::vector<Layer> run{measured[start]};
        for (std::size_t i = start + 1; i < measured.size(); ++i) {
            const Layer& previous = run.back();
            const Layer& current = measured[i];
            if (current.depth - previous.depth > 1.20) break;
            if (current.radius > previous.radius + std::max(0.55, 0.10 * surfaceRadius)) break;
            if (previous.radius - current.radius > std::max(2.20, 0.40 * surfaceRadius)) break;
            run.push_back(current);
        }
        if (run.empty()) continue;

        const double slope = robustSlope(run);
        const double shrink = run.front().radius - run.back().radius;
        int monotonic = 0;
        double coverage = 0.0;
        for (std::size_t i = 0; i < run.size(); ++i) {
            coverage += run[i].coverage;
            if (i > 0 && run[i].radius <= run[i - 1].radius + 0.20) ++monotonic;
        }
        const double monotonicRatio = run.size() > 1
            ? static_cast<double>(monotonic) / static_cast<double>(run.size() - 1)
            : 0.0;
        const double meanCoverage = coverage / static_cast<double>(run.size());
        const double score = 3.0 * static_cast<double>(run.size())
            + 2.0 * meanCoverage
            - run.front().depth
            + (slope < -0.20 ? 2.0 : 0.0);
        if (!best.valid || score > best.score) {
            best.valid = true;
            best.layers = std::move(run);
            best.slope = slope;
            best.shrink = shrink;
            best.monotonicRatio = monotonicRatio;
            best.score = score;
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：检测/搜索“findConeTailStart”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::size_t findConeTailStart(const std::vector<Layer>& layers, double topRadius)
{
    if (layers.size() < 4) return layers.size();
    for (std::size_t split = 2; split + 1 < layers.size(); ++split) {
        const std::size_t end = std::min(layers.size(), split + 4);
        std::vector<double> tail;
        for (std::size_t i = split; i < end; ++i) tail.push_back(layers[i].radius);
        const auto minmax = std::minmax_element(tail.begin(), tail.end());
        const double range = *minmax.second - *minmax.first;
        std::vector<Layer> prefix(layers.begin(), layers.begin() + split + 1);
        if (range <= std::max(0.35, 0.06 * topRadius)
            && robustSlope(prefix) < -0.25) {
            return split;
        }
    }
    return layers.size();
}

}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    Result result;
    result.initialRadius = input.initialTopRadius;
    result.centerU = input.topU;
    result.centerV = input.topV;
    if (samples.size() < 30 || !finite(input.topU) || !finite(input.topV)
        || !finite(input.topW)) {
        result.reason = "SurfaceBoundary_INVALID_INPUT";
        return result;
    }

    const SurfaceGuJi surface = searchSurface(samples, input);
    if (!surface.valid) {
        result.reason = "SurfaceBoundary_NO_PHYSICAL_SURFACE_BOUNDARY";
        return result;
    }
    result.surfaceValid = true;
    result.centerU = surface.centerU;
    result.centerV = surface.centerV;
    result.centerShift = std::hypot(result.centerU - input.topU, result.centerV - input.topV);
    result.surfaceRadius = surface.radius;
    result.surfaceMad = surface.mad;
    result.surfaceSectors = surface.sectors;

    DirectionPouMian negative = buildDirection(samples, result.centerU, result.centerV,
                                                input.topW, result.surfaceRadius, -1);
    DirectionPouMian positive = buildDirection(samples, result.centerU, result.centerV,
                                                input.topW, result.surfaceRadius, 1);
    DirectionPouMian best;
    if (!negative.valid) best = positive;
    else if (!positive.valid) best = negative;
    else best = negative.score >= positive.score ? negative : positive;

    result.valid = true;
    if (!best.valid || best.layers.empty()) {
        result.holeType = 1;
        result.topRadius = result.surfaceRadius;
        result.bottomRadius = result.topRadius;
        result.inwardPolarity = 0;
        result.confidence = std::clamp(0.35 + 0.01 * surface.sectors - 0.50 * surface.mad,
                                       0.20, 0.70);
        result.reason = "SurfaceBoundary_STRAIGHT_SURFACE_ONLY_LOW_CONFIDENCE";
        return result;
    }

    result.profileValid = true;
    result.inwardPolarity = best.polarity;
    result.layers = best.layers;
    result.validLayers = static_cast<int>(best.layers.size());
    result.radiusSlope = best.slope;
    result.radiusShrink = best.shrink;
    result.monotonicRatio = best.monotonicRatio;

    const bool persistentCone = best.layers.size() >= 3
        && best.slope < -0.25
        && best.shrink >= std::max(0.50, 0.07 * result.surfaceRadius)
        && best.monotonicRatio >= 0.60;
    const bool shortStrongCone = best.layers.size() >= 2
        && best.slope < -0.75
        && best.shrink >= std::max(0.65, 0.12 * result.surfaceRadius)
        && best.monotonicRatio >= 0.90;
    const bool cone = persistentCone || shortStrongCone;

    if (cone) {
        const std::size_t tailStart = findConeTailStart(best.layers, result.surfaceRadius);
        const std::size_t coneEnd = tailStart < best.layers.size()
            ? tailStart : best.layers.size() - 1;
        std::vector<Layer> coneLayers(best.layers.begin(), best.layers.begin() + coneEnd + 1);
        const auto line = robustLine(coneLayers);
        result.topRadius = std::max(result.surfaceRadius, line.first);
        result.topRadius = std::clamp(result.topRadius, 0.50, 11.0);
        result.holeType = 2;

        if (tailStart < best.layers.size()) {
            std::vector<double> tailRadii;
            const std::size_t tailEnd = std::min(best.layers.size(), tailStart + 3);
            for (std::size_t i = tailStart; i < tailEnd; ++i) {
                tailRadii.push_back(best.layers[i].radius);
            }
            result.bottomRadius = median(std::move(tailRadii));
            result.depth = best.layers[tailStart].depth;
        } else {
            const Layer& tail = best.layers.back();
            result.bottomRadius = tail.radius;
            result.depth = tail.depth;
        }
        result.bottomValid = result.bottomRadius > 0.20
            && result.bottomRadius < 0.96 * result.topRadius
            && result.depth >= 0.80;
        if (!result.bottomValid) {
            result.bottomRadius = 0.0;
            result.depth = 0.0;
        }
        result.reason = result.bottomValid
            ? "SurfaceBoundary_CONE_FIRST_COMPONENT_PROFILE"
            : "SurfaceBoundary_CONE_BOTTOM_UNSUPPORTED";
    } else {
        result.holeType = 1;
        const Layer& first = best.layers.front();
        double weightedRadius = first.radius;
        double weight = std::max(0.10, first.coverage);
        if (best.layers.size() >= 2) {
            const Layer& second = best.layers[1];
            if (std::abs(second.radius - first.radius)
                <= std::max(0.30, 0.06 * result.surfaceRadius)) {
                const double secondWeight = std::max(0.10, second.coverage);
                weightedRadius = (weightedRadius * weight + second.radius * secondWeight)
                    / (weight + secondWeight);
            }
        }
        result.topRadius = std::clamp(weightedRadius, 0.40, result.surfaceRadius + 0.20);
        result.bottomRadius = result.topRadius;
        result.depth = 0.0;
        result.reason = "SurfaceBoundary_STRAIGHT_FIRST_INNER_WALL_COMPONENT";
    }

    result.confidence = std::clamp(
        0.45
            + 0.035 * static_cast<double>(result.validLayers)
            + 0.16 * result.monotonicRatio
            + 0.006 * static_cast<double>(result.surfaceSectors)
            - 0.35 * result.surfaceMad,
        0.0, 0.98);
    return result;
}

}


// ============================================================================
// 功能分区：内壁连续跟踪
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheNeiWallTrack {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSectors = 36;
constexpr double kFirstDepth = 0.25;
constexpr double kDepthStep = 0.25;
constexpr double kLayerHalfThickness = 0.10;
constexpr double kRadialClusterGap = 0.30;
constexpr double kMaxDepth = 12.0;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“quantile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double quantile(std::vector<double> values, double q)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    q = std::clamp(q, 0.0, 1.0);
    std::sort(values.begin(), values.end());
    const double position = q * static_cast<double>(values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(position));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(position));
    const double alpha = position - static_cast<double>(lo);
    return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

double median(std::vector<double> values) { return quantile(std::move(values), 0.5); }

/** 【类型导航注释】
 * TopPlaneGuJi：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct TopPlaneGuJi {
    bool valid = false;
    double w = 0.0;
    double mad = 0.0;
    int support = 0;
};

/** 【函数导航】
 * 作用：估计“estimateLocalTopPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
TopPlaneGuJi estimateLocalTopPlane(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double initialW)
{
    constexpr double binWidth = 0.05;
    constexpr double range = 1.20;
    constexpr int binCount = 49;
    std::array<int, binCount> histogram{};
    std::vector<double> values;
    values.reserve(samples.size() / 3);
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double radius = std::hypot(sample.u - centerU, sample.v - centerV);
        if (radius < 10.0 || radius > 18.0) continue;
        const double delta = sample.w - initialW;
        if (delta < -range || delta > range) continue;
        const int index = std::clamp(
            static_cast<int>(std::floor((delta + range) / binWidth)), 0, binCount - 1);
        ++histogram[static_cast<std::size_t>(index)];
        values.push_back(sample.w);
    }
    TopPlaneGuJi estimate;
    if (values.size() < 40) return estimate;
    int bestIndex = 0;
    int bestCount = -1;
    for (int index = 0; index < binCount; ++index) {
        const int count = histogram[static_cast<std::size_t>(index)]
            + (index > 0 ? histogram[static_cast<std::size_t>(index - 1)] : 0)
            + (index + 1 < binCount ? histogram[static_cast<std::size_t>(index + 1)] : 0);
        if (count > bestCount) {
            bestCount = count;
            bestIndex = index;
        }
    }
    const double modeW = initialW - range
        + (static_cast<double>(bestIndex) + 0.5) * binWidth;
    std::vector<double> support;
    support.reserve(values.size());
    for (double value : values) {
        if (std::abs(value - modeW) <= 0.13) support.push_back(value);
    }
    if (support.size() < 30) return estimate;
    const double planeW = median(support);
    std::vector<double> deviations;
    deviations.reserve(support.size());
    for (double value : support) deviations.push_back(std::abs(value - planeW));
    const double mad = median(std::move(deviations));
    if (!finite(planeW) || !finite(mad) || mad > 0.12
        || std::abs(planeW - initialW) > 1.0) return estimate;
    estimate.valid = true;
    estimate.w = planeW;
    estimate.mad = mad;
    estimate.support = static_cast<int>(support.size());
    return estimate;
}

/** 【类型导航注释】
 * JingXiangPoint：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JingXiangPoint {
    double radius = 0.0;
    double angle = 0.0;
};

/** 【函数导航】
 * 作用：执行“measureLayerCandidates”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<Layer> measureLayerCandidates(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    int polarity,
    double depth)
{
    std::vector<JingXiangPoint> radial;
    radial.reserve(384);
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double pointDepth = static_cast<double>(polarity) * (sample.w - topW);
        if (std::abs(pointDepth - depth) > kLayerHalfThickness) continue;
        const double du = sample.u - centerU;
        const double dv = sample.v - centerV;
        const double radius = std::hypot(du, dv);
        if (radius < 0.25 || radius > 12.0) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        radial.push_back({radius, angle});
    }

    std::vector<Layer> candidates;
    candidates.reserve(16);
    if (radial.size() < 4) return candidates;
    std::sort(radial.begin(), radial.end(), [](const JingXiangPoint& left, const JingXiangPoint& right) {
        return left.radius < right.radius;
    });

    std::size_t begin = 0;
    for (std::size_t end = 1; end <= radial.size(); ++end) {
        const bool cut = end == radial.size()
            || radial[end].radius - radial[end - 1].radius > kRadialClusterGap;
        if (!cut) continue;

        const std::size_t count = end - begin;
        if (count >= 4) {
            std::array<std::vector<double>, kSectors> sectorRadii;
            for (std::size_t index = begin; index < end; ++index) {
                int sector = static_cast<int>(std::floor(
                    radial[index].angle / (2.0 * kPi) * static_cast<double>(kSectors)));
                sector = std::clamp(sector, 0, kSectors - 1);
                sectorRadii[static_cast<std::size_t>(sector)].push_back(radial[index].radius);
            }

            std::vector<double> sectorMedians;
            sectorMedians.reserve(kSectors);
            for (auto& values : sectorRadii) {
                if (!values.empty()) sectorMedians.push_back(median(std::move(values)));
            }
            const int sectors = static_cast<int>(sectorMedians.size());
            if (sectors >= 3) {
                const double radius = median(sectorMedians);
                std::vector<double> deviations;
                deviations.reserve(sectorMedians.size());
                for (double value : sectorMedians) deviations.push_back(std::abs(value - radius));
                const double radialMad = median(std::move(deviations));
                const double span = radial[end - 1].radius - radial[begin].radius;
                const double maximumSpan = std::max(1.65, 0.35 * radius);
                if (finite(radius) && finite(radialMad)
                    && radius >= 0.40 && radius <= 11.0
                    && span <= maximumSpan) {
                    Layer layer;
                    layer.depth = depth;
                    layer.radius = radius;
                    layer.coverage = static_cast<double>(sectors) / static_cast<double>(kSectors);
                    layer.span = span;
                    layer.radialMad = radialMad;
                    layer.points = static_cast<int>(count);
                    layer.sectors = sectors;
                    candidates.push_back(layer);
                }
            }
        }
        begin = end;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Layer& left, const Layer& right) {
        return left.radius < right.radius;
    });
    return candidates;
}

/** 【函数导航】
 * 作用：执行“robustSlope”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double robustSlope(const std::vector<Layer>& layers)
{
    std::vector<double> slopes;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        for (std::size_t j = i + 1; j < layers.size(); ++j) {
            const double deltaDepth = layers[j].depth - layers[i].depth;
            if (deltaDepth >= kDepthStep - 1e-9) {
                slopes.push_back((layers[j].radius - layers[i].radius) / deltaDepth);
            }
        }
    }
    return slopes.empty() ? 0.0 : median(std::move(slopes));
}

/** 【函数导航】
 * 作用：执行“robustLine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::pair<double, double> robustLine(const std::vector<Layer>& layers)
{
    if (layers.empty()) return {0.0, 0.0};
    const double slope = robustSlope(layers);
    std::vector<double> intercepts;
    intercepts.reserve(layers.size());
    for (const Layer& layer : layers) intercepts.push_back(layer.radius - slope * layer.depth);
    return {median(std::move(intercepts)), slope};
}

/** 【函数导航】
 * 作用：执行“modelResidual”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double modelResidual(const std::vector<Layer>& layers, double intercept, double slope)
{
    std::vector<double> residuals;
    residuals.reserve(layers.size());
    for (const Layer& layer : layers) {
        residuals.push_back(std::abs(layer.radius - (intercept + slope * layer.depth)));
    }
    return residuals.empty() ? 0.0 : median(std::move(residuals));
}

/** 【类型导航注释】
 * GuiJi：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct GuiJi {
    int polarity = 0;
    std::vector<Layer> layers;
    bool cone = false;
    double slope = 0.0;
    double intercept = 0.0;
    double shrink = 0.0;
    double monotonicRatio = 0.0;
    double residual = 0.0;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：执行“analyseTrack”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
GuiJi analyseTrack(int polarity, std::vector<Layer> layers)
{
    GuiJi track;
    if (layers.empty()) return track;
    track.polarity = polarity;
    track.layers = std::move(layers);
    track.slope = robustSlope(track.layers);
    track.shrink = track.layers.front().radius - track.layers.back().radius;

    int monotonic = 0;
    for (std::size_t index = 1; index < track.layers.size(); ++index) {
        if (track.layers[index].radius <= track.layers[index - 1].radius + 0.10) ++monotonic;
    }
    track.monotonicRatio = track.layers.size() > 1
        ? static_cast<double>(monotonic) / static_cast<double>(track.layers.size() - 1)
        : 0.0;

    track.cone = track.layers.size() >= 4
        && track.slope < -0.22
        && track.shrink >= 0.50
        && track.monotonicRatio >= 0.65;

    if (track.cone) {
        const auto line = robustLine(track.layers);
        track.intercept = line.first;
        track.slope = line.second;
        track.residual = modelResidual(track.layers, track.intercept, track.slope);
    } else {
        const std::size_t take = std::min<std::size_t>(3, track.layers.size());
        std::vector<double> radii;
        radii.reserve(take);
        for (std::size_t index = 0; index < take; ++index) radii.push_back(track.layers[index].radius);
        track.intercept = median(std::move(radii));
        track.residual = modelResidual(track.layers, track.intercept, 0.0);
    }

    double sectors = 0.0;
    double points = 0.0;
    double span = 0.0;
    double mad = 0.0;
    for (const Layer& layer : track.layers) {
        sectors += static_cast<double>(layer.sectors);
        points += static_cast<double>(layer.points);
        span += layer.span;
        mad += layer.radialMad;
    }
    const double count = static_cast<double>(track.layers.size());
    track.score = 10.0 * count
        + 0.15 * (sectors / count)
        + 0.003 * (points / count)
        - 5.0 * track.residual
        - 0.35 * (span / count)
        - 1.8 * (mad / count)
        - 0.50 * track.layers.front().depth
        - 0.08 * median([&]() {
            std::vector<double> radii;
            radii.reserve(track.layers.size());
            for (const Layer& layer : track.layers) radii.push_back(layer.radius);
            return radii;
        }());
    return track;
}

/** 【函数导航】
 * 作用：构建“buildTracks”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<GuiJi> buildTracks(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    int polarity,
    int& candidateCount)
{
    std::vector<std::vector<Layer>> byDepth;
    byDepth.reserve(static_cast<std::size_t>(std::floor((kMaxDepth - kFirstDepth) / kDepthStep)) + 1U);
    for (double depth = kFirstDepth; depth <= kMaxDepth + 1e-9; depth += kDepthStep) {
        byDepth.push_back(measureLayerCandidates(samples, centerU, centerV, topW, polarity, depth));
    }

    candidateCount = 0;
    for (const auto& layerCandidates : byDepth) {
        candidateCount += static_cast<int>(layerCandidates.size());
    }

    std::vector<GuiJi> tracks;
    tracks.reserve(64);
    const std::size_t startLimit = std::min<std::size_t>(8, byDepth.size());
    for (std::size_t startIndex = 0; startIndex < startLimit; ++startIndex) {
        for (const Layer& start : byDepth[startIndex]) {
            std::vector<Layer> layers;
            layers.reserve(byDepth.size() - startIndex);
            layers.push_back(start);
            std::size_t lastIndex = startIndex;
            for (std::size_t nextIndex = startIndex + 1; nextIndex < byDepth.size(); ++nextIndex) {
                const std::size_t gapSteps = nextIndex - lastIndex;
                if (gapSteps > 3) break;

                const Layer& previous = layers.back();
                double predictedRadius = previous.radius;
                if (layers.size() >= 2) {
                    const Layer& before = layers[layers.size() - 2];
                    const double localSlope = std::clamp(
                        (previous.radius - before.radius) / (previous.depth - before.depth),
                        -3.0, 0.50);
                    predictedRadius += localSlope *
                        (byDepth[nextIndex].empty()
                            ? static_cast<double>(gapSteps) * kDepthStep
                            : byDepth[nextIndex].front().depth - previous.depth);
                }

                const Layer* chosen = nullptr;
                double chosenCost = std::numeric_limits<double>::infinity();
                for (const Layer& candidate : byDepth[nextIndex]) {
                    const double scale = static_cast<double>(gapSteps);
                    const double delta = candidate.radius - previous.radius;
                    if (delta > 0.35 * scale || -delta > 0.90 * scale) continue;
                    const double cost = std::abs(candidate.radius - predictedRadius)
                        + 0.15 * candidate.span
                        + 0.60 * candidate.radialMad
                        - 0.025 * static_cast<double>(candidate.sectors)
                        - 0.001 * static_cast<double>(candidate.points);
                    if (cost < chosenCost) {
                        chosen = &candidate;
                        chosenCost = cost;
                    }
                }
                if (chosen) {
                    layers.push_back(*chosen);
                    lastIndex = nextIndex;
                }
            }
            tracks.push_back(analyseTrack(polarity, std::move(layers)));
        }
    }
    return tracks;
}

/** 【函数导航】
 * 作用：执行“medianRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
double medianRadius(const GuiJi& track)
{
    std::vector<double> radii;
    radii.reserve(track.layers.size());
    for (const Layer& layer : track.layers) radii.push_back(layer.radius);
    return median(std::move(radii));
}

/** 【函数导航】
 * 作用：执行“betterCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool betterCone(const GuiJi& left, const GuiJi& right)
{
    if (left.layers.size() != right.layers.size()) return left.layers.size() > right.layers.size();
    return left.score > right.score;
}

/** 【函数导航】
 * 作用：执行“betterStraight”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool betterStraight(const GuiJi& left, const GuiJi& right, std::size_t maximumLength)
{
    const bool leftNearLongest = left.layers.size() + 1 >= maximumLength;
    const bool rightNearLongest = right.layers.size() + 1 >= maximumLength;
    if (leftNearLongest != rightNearLongest) return leftNearLongest;
    const double leftRadius = medianRadius(left);
    const double rightRadius = medianRadius(right);
    if (std::abs(leftRadius - rightRadius) > 0.12) return leftRadius < rightRadius;
    if (left.layers.size() != right.layers.size()) return left.layers.size() > right.layers.size();
    return left.score > right.score;
}

/** 【函数导航】
 * 作用：选择“selectDirectionTrack”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool selectDirectionTrack(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    int polarity,
    GuiJi& selected,
    int& candidateCount)
{
    std::vector<GuiJi> tracks = buildTracks(
        samples, centerU, centerV, topW, polarity, candidateCount);
    if (tracks.empty()) return false;

    bool foundCone = false;
    GuiJi bestCone;
    for (const GuiJi& track : tracks) {
        if (!track.cone) continue;
        if (!foundCone || betterCone(track, bestCone)) {
            bestCone = track;
            foundCone = true;
        }
    }
    if (foundCone) {
        selected = std::move(bestCone);
        return true;
    }

    std::size_t maximumLength = 0;
    for (const GuiJi& track : tracks) maximumLength = std::max(maximumLength, track.layers.size());

    bool foundMulti = false;
    GuiJi bestMulti;
    for (const GuiJi& track : tracks) {
        if (track.layers.size() < 2) continue;
        if (!foundMulti || betterStraight(track, bestMulti, maximumLength)) {
            bestMulti = track;
            foundMulti = true;
        }
    }
    if (foundMulti) {
        selected = std::move(bestMulti);
        return true;
    }

    bool foundSingle = false;
    GuiJi bestSingle;
    for (GuiJi& track : tracks) {
        if (track.layers.size() != 1 || track.layers.front().sectors < 6) continue;
        if (!foundSingle
            || track.layers.front().radius < bestSingle.layers.front().radius - 0.10
            || (std::abs(track.layers.front().radius - bestSingle.layers.front().radius) <= 0.10
                && track.score > bestSingle.score)) {
            bestSingle = track;
            foundSingle = true;
        }
    }
    if (!foundSingle) return false;
    selected = std::move(bestSingle);
    return true;
}

/** 【函数导航】
 * 作用：执行“betterAcrossDirections”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool betterAcrossDirections(const GuiJi& left, const GuiJi& right)
{
    if (left.cone != right.cone) return left.cone;
    if (left.layers.size() != right.layers.size()) return left.layers.size() > right.layers.size();
    if (!left.cone) {
        const double leftRadius = medianRadius(left);
        const double rightRadius = medianRadius(right);
        if (std::abs(leftRadius - rightRadius) > 0.15) return leftRadius < rightRadius;
    }
    return left.score > right.score;
}

/** 【类型导航注释】
 * ZhongXinTanZhen：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhongXinTanZhen {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    int polarity = 0;
    int layers = 0;
};

/** 【函数导航】
 * 作用：执行“probeCenter”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ZhongXinTanZhen probeCenter(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW)
{
    ZhongXinTanZhen best;
    constexpr std::array<double, 4> depths{{0.25, 0.50, 0.75, 1.00}};
    for (int polarity : {-1, 1}) {
        std::vector<Layer> chosen;
        const Layer* previous = nullptr;
        for (double depth : depths) {
            std::vector<Layer> candidates = measureLayerCandidates(
                samples, centerU, centerV, topW, polarity, depth);
            candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const Layer& layer) {
                return layer.sectors < 4;
            }), candidates.end());
            const Layer* selected = nullptr;
            if (!previous) {
                if (!candidates.empty()) selected = &candidates.front();
            } else {
                double bestCost = std::numeric_limits<double>::infinity();
                for (const Layer& candidate : candidates) {
                    const double delta = candidate.radius - previous->radius;
                    if (delta > 0.35 || -delta > 0.90) continue;
                    const double cost = std::abs(delta)
                        + 0.15 * candidate.span
                        + 0.60 * candidate.radialMad
                        - 0.025 * static_cast<double>(candidate.sectors);
                    if (cost < bestCost) {
                        bestCost = cost;
                        selected = &candidate;
                    }
                }
            }
            if (selected) {
                chosen.push_back(*selected);
                previous = &chosen.back();
            }
        }
        if (chosen.empty()) continue;
        double sectors = 0.0;
        double points = 0.0;
        double mad = 0.0;
        double span = 0.0;
        double radius = 0.0;
        for (const Layer& layer : chosen) {
            sectors += static_cast<double>(layer.sectors);
            points += static_cast<double>(layer.points);
            mad += layer.radialMad;
            span += layer.span;
            radius += layer.radius;
        }
        const double count = static_cast<double>(chosen.size());
        const double score = 3.0 * count
            + 0.80 * (sectors / count)
            + 0.010 * (points / count)
            - 5.0 * (mad / count)
            - 1.20 * (span / count)
            - 0.20 * (radius / count);
        if (!best.valid || score > best.score) {
            best.valid = true;
            best.score = score;
            best.polarity = polarity;
            best.layers = static_cast<int>(chosen.size());
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：精修“refineCenterIfNeeded”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::pair<double, double> refineCenterIfNeeded(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW)
{
    const ZhongXinTanZhen initial = probeCenter(samples, centerU, centerV, topW);
    if (initial.valid && initial.score >= 12.0) return {centerU, centerV};

    double bestU = centerU;
    double bestV = centerV;
    double bestScore = initial.valid ? initial.score : -std::numeric_limits<double>::infinity();
    for (int du = -4; du <= 4; ++du) {
        for (int dv = -4; dv <= 4; ++dv) {
            const double candidateU = centerU + static_cast<double>(du);
            const double candidateV = centerV + static_cast<double>(dv);
            const ZhongXinTanZhen probe = probeCenter(samples, candidateU, candidateV, topW);
            if (!probe.valid) continue;
            const double shift = std::hypot(static_cast<double>(du), static_cast<double>(dv));
            const double score = probe.score - 0.12 * shift;
            if (score > bestScore) {
                bestScore = score;
                bestU = candidateU;
                bestV = candidateV;
            }
        }
    }

    if (initial.valid && bestScore < initial.score + 2.0) return {centerU, centerV};

    double refinedU = bestU;
    double refinedV = bestV;
    double refinedScore = bestScore;
    for (int iu = -3; iu <= 3; ++iu) {
        for (int iv = -3; iv <= 3; ++iv) {
            const double candidateU = bestU + 0.25 * static_cast<double>(iu);
            const double candidateV = bestV + 0.25 * static_cast<double>(iv);
            const ZhongXinTanZhen probe = probeCenter(samples, candidateU, candidateV, topW);
            if (!probe.valid) continue;
            const double shift = std::hypot(candidateU - centerU, candidateV - centerV);
            const double score = probe.score - 0.12 * shift;
            if (score > refinedScore) {
                refinedScore = score;
                refinedU = candidateU;
                refinedV = candidateV;
            }
        }
    }
    return {refinedU, refinedV};
}

/** 【函数导航】
 * 作用：检测/搜索“findConeTailStart”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::size_t findConeTailStart(const std::vector<Layer>& layers, double topRadius)
{
    if (layers.size() < 5) return layers.size();
    for (std::size_t split = 2; split + 2 < layers.size(); ++split) {
        const std::size_t end = std::min(layers.size(), split + 4);
        std::vector<double> tail;
        for (std::size_t index = split; index < end; ++index) tail.push_back(layers[index].radius);
        const auto minmax = std::minmax_element(tail.begin(), tail.end());
        const double range = *minmax.second - *minmax.first;
        std::vector<Layer> prefix(layers.begin(), layers.begin() + split + 1);
        if (range <= std::max(0.28, 0.045 * topRadius)
            && robustSlope(prefix) < -0.22) {
            return split;
        }
    }
    return layers.size();
}

}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    Result result;
    result.initialRadius = input.initialTopRadius;
    result.centerU = input.topU;
    result.centerV = input.topV;
    result.centerShift = 0.0;
    result.seedDistance = std::hypot(input.topU - input.seedU, input.topV - input.seedV);
    result.topW = input.topW;

    if (samples.size() < 24 || !finite(input.topU) || !finite(input.topV)
        || !finite(input.topW) || !finite(input.seedU) || !finite(input.seedV)) {
        result.reason = "PersistentInnerWall_INVALID_INPUT";
        return result;
    }

    const TopPlaneGuJi topPlane = estimateLocalTopPlane(
        samples, input.topU, input.topV, input.topW);
    const double workingTopW = topPlane.valid ? topPlane.w : input.topW;
    result.surfaceValid = topPlane.valid;
    result.topW = workingTopW;
    result.axialShift = workingTopW - input.topW;
    result.surfaceMad = topPlane.mad;
    result.surfaceSectors = topPlane.support;

    auto selectAt = [&](double centerU, double centerV, GuiJi& selectedTrack,
                        int& totalCandidates) {
        GuiJi negative;
        GuiJi positive;
        int negativeCandidates = 0;
        int positiveCandidates = 0;
        const bool negativeValid = selectDirectionTrack(
            samples, centerU, centerV, workingTopW, -1, negative, negativeCandidates);
        const bool positiveValid = selectDirectionTrack(
            samples, centerU, centerV, workingTopW, 1, positive, positiveCandidates);
        totalCandidates = negativeCandidates + positiveCandidates;
        if (!negativeValid && !positiveValid) return false;
        if (!negativeValid) selectedTrack = positive;
        else if (!positiveValid) selectedTrack = negative;
        else selectedTrack = betterAcrossDirections(negative, positive) ? negative : positive;
        return !selectedTrack.layers.empty();
    };

    GuiJi selected;
    int selectedCandidateCount = 0;
    bool selectedValid = selectAt(input.topU, input.topV, selected, selectedCandidateCount);

    auto suspiciousOuterTrack = [](const GuiJi& track) {
        if (track.layers.empty()) return true;
        const double radius = medianRadius(track);
        double meanSectors = 0.0;
        for (const Layer& layer : track.layers) meanSectors += static_cast<double>(layer.sectors);
        meanSectors /= static_cast<double>(track.layers.size());
        return radius > 8.0
            && (track.layers.front().sectors < 12 || meanSectors < 9.0);
    };

    result.centerU = input.topU;
    result.centerV = input.topV;
    result.centerShift = 0.0;

    if (!selectedValid || suspiciousOuterTrack(selected)) {
        const auto refinedCenter = refineCenterIfNeeded(
            samples, input.topU, input.topV, workingTopW);
        const double shift = std::hypot(
            refinedCenter.first - input.topU, refinedCenter.second - input.topV);
        if (shift > 0.10) {
            GuiJi refinedTrack;
            int refinedCandidates = 0;
            if (selectAt(refinedCenter.first, refinedCenter.second,
                    refinedTrack, refinedCandidates)
                && !suspiciousOuterTrack(refinedTrack)) {
                selected = std::move(refinedTrack);
                selectedCandidateCount = refinedCandidates;
                selectedValid = true;
                result.centerU = refinedCenter.first;
                result.centerV = refinedCenter.second;
                result.centerShift = shift;
            }
        }
    }

    if (!selectedValid) {
        result.reason = "PersistentInnerWall_NO_PERSISTENT_INNER_WALL";
        return result;
    }

    result.candidateCount = selectedCandidateCount;

    if (selected.layers.empty()) {
        result.reason = "PersistentInnerWall_EMPTY_TRACK";
        return result;
    }

    result.valid = true;
    result.profileValid = true;
    result.inwardPolarity = selected.polarity;
    result.layers = selected.layers;
    result.validLayers = static_cast<int>(selected.layers.size());
    result.radiusSlope = selected.slope;
    result.radiusShrink = selected.shrink;
    result.monotonicRatio = selected.monotonicRatio;

    if (selected.cone) {
        result.holeType = 2;
        const auto line = robustLine(selected.layers);
        const double intercept = line.first;
        const double slope = line.second;

        const double lineTop = intercept + std::abs(slope) * (kLayerHalfThickness + 0.05);
        const double firstTop = selected.layers.front().radius
            + std::abs(slope) * (selected.layers.front().depth + kLayerHalfThickness);
        result.topRadius = std::clamp(0.5 * (lineTop + firstTop), 0.40, 11.0);

        const std::size_t tailStart = findConeTailStart(selected.layers, result.topRadius);
        const Layer& bottomLayer = tailStart < selected.layers.size()
            ? selected.layers[tailStart]
            : selected.layers.back();
        result.bottomRadius = bottomLayer.radius;
        result.depth = bottomLayer.depth;
        result.bottomValid = result.bottomRadius > 0.20
            && result.bottomRadius < 0.97 * result.topRadius
            && result.depth >= kFirstDepth;
        if (!result.bottomValid) {
            result.bottomRadius = 0.0;
            result.depth = 0.0;
        }
        result.reason = result.bottomValid
            ? "PersistentInnerWall_CONE_INNER_PERSISTENT_WALL"
            : "PersistentInnerWall_CONE_BOTTOM_UNSUPPORTED";
    } else {
        result.holeType = 1;
        const std::size_t take = std::min<std::size_t>(3, selected.layers.size());
        std::vector<double> radii;
        radii.reserve(take);
        for (std::size_t index = 0; index < take; ++index) {
            radii.push_back(selected.layers[index].radius);
        }
        result.topRadius = std::clamp(median(std::move(radii)), 0.40, 11.0);
        result.bottomRadius = result.topRadius;
        result.depth = 0.0;
        result.bottomValid = false;
        result.reason = selected.layers.size() >= 2
            ? "PersistentInnerWall_STRAIGHT_INNER_PERSISTENT_WALL"
            : "PersistentInnerWall_STRAIGHT_INNER_SINGLE_WALL";
    }

    result.surfaceRadius = result.topRadius;
    if (!topPlane.valid) {
        result.surfaceSectors = selected.layers.front().sectors;
        result.surfaceMad = selected.layers.front().radialMad;
    }

    double meanCoverage = 0.0;
    double meanMad = 0.0;
    for (const Layer& layer : selected.layers) {
        meanCoverage += layer.coverage;
        meanMad += layer.radialMad;
    }
    meanCoverage /= static_cast<double>(selected.layers.size());
    meanMad /= static_cast<double>(selected.layers.size());
    result.confidence = std::clamp(
        0.50
            + 0.055 * static_cast<double>(result.validLayers)
            + 0.22 * meanCoverage
            - 0.75 * meanMad
            + (selected.cone ? 0.08 * result.monotonicRatio : 0.0),
        0.0, 0.98);
    return result;
}

}


// ============================================================================
// 功能分区：锥孔保守补充判定
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/

namespace HoleJiheZhuiBuJiu {
namespace {

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“strictConeHuiFu”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool strictConeHuiFu(
    const HoleJiheSurfacePouMian::Result& base,
    const HoleJiheNeiWallTrack::Result& huiFu)
{
    if (!base.valid || base.holeType != 1) return false;
    if (!huiFu.valid || huiFu.holeType != 2 || !huiFu.profileValid
        || !huiFu.bottomValid) return false;

    const double baseScale = std::max(base.topRadius, base.surfaceRadius);
    if (!finite(baseScale) || baseScale < 0.50) return false;

    const bool baseLeansCone = base.profileValid && base.validLayers >= 2
        && base.radiusSlope < -0.12
        && base.radiusShrink >= std::max(0.35, 0.05 * baseScale)
        && base.monotonicRatio >= 0.50;
    const bool surfaceOnlyNearScale = !base.profileValid
        && huiFu.topRadius >= 0.85 * baseScale
        && huiFu.topRadius <= 1.45 * baseScale;
    if (!baseLeansCone && !surfaceOnlyNearScale) return false;

    const bool strongHuiFu = huiFu.validLayers >= 6
        && huiFu.radiusSlope < -0.35
        && huiFu.radiusShrink >= std::max(1.20, 0.18 * baseScale)
        && huiFu.monotonicRatio >= 0.80
        && huiFu.confidence >= 0.72
        && huiFu.depth >= 1.20
        && huiFu.bottomRadius > 0.20
        && huiFu.bottomRadius < 0.58 * baseScale;
    if (!strongHuiFu) return false;

    const double radiusDelta = huiFu.topRadius - baseScale;
    if (!finite(radiusDelta) || radiusDelta < -0.35
        || radiusDelta > std::max(1.60, 0.30 * baseScale)) {
        return false;
    }
    if (huiFu.centerShift > 0.60) return false;
    return true;
}

/** 【函数导航】
 * 作用：执行“convertLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Layer convertLayer(const HoleJiheSurfacePouMian::Layer& layer)
{
    return {layer.depth, layer.radius, layer.coverage, layer.span,
        layer.points, layer.sectors};
}

}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    Result result;
    result.initialRadius = input.initialTopRadius;
    result.seedDistance = std::hypot(input.topU - input.seedU, input.topV - input.seedV);

    std::vector<HoleJiheSurfacePouMian::Sample> surfaceBoundarySamples;
    std::vector<HoleJiheNeiWallTrack::Sample> persistentInnerWallSamples;
    surfaceBoundarySamples.reserve(samples.size());
    persistentInnerWallSamples.reserve(samples.size());
    for (const Sample& sample : samples) {
        surfaceBoundarySamples.push_back({sample.u, sample.v, sample.w});
        persistentInnerWallSamples.push_back({sample.u, sample.v, sample.w});
    }

    HoleJiheSurfacePouMian::Input surfaceBoundaryInput;
    surfaceBoundaryInput.topU = input.topU;
    surfaceBoundaryInput.topV = input.topV;
    surfaceBoundaryInput.topW = input.topW;
    surfaceBoundaryInput.initialTopRadius = input.initialTopRadius;
    surfaceBoundaryInput.maxCenterShift = input.baselineMaxCenterShift;
    surfaceBoundaryInput.centerShiftPenalty = input.baselineCenterShiftPenalty;
    const HoleJiheSurfacePouMian::Result base =
        HoleJiheSurfacePouMian::evaluate(surfaceBoundarySamples, surfaceBoundaryInput);

    if (!base.valid) {
        result.reason = std::string("GeometryReview_BASELINE_REJECT:") + base.reason;
        return result;
    }

    result.valid = true;
    result.surfaceValid = base.surfaceValid;
    result.profileValid = base.profileValid;
    result.bottomValid = base.bottomValid;
    result.holeType = base.holeType;
    result.inwardPolarity = base.inwardPolarity;
    result.baselineType = base.holeType;
    result.centerU = base.centerU;
    result.centerV = base.centerV;
    result.centerShift = base.centerShift;
    result.surfaceRadius = base.surfaceRadius;
    result.surfaceMad = base.surfaceMad;
    result.surfaceSectors = base.surfaceSectors;
    result.topRadius = base.topRadius;
    result.bottomRadius = base.bottomRadius;
    result.depth = base.depth;
    result.radiusSlope = base.radiusSlope;
    result.radiusShrink = base.radiusShrink;
    result.monotonicRatio = base.monotonicRatio;
    result.confidence = base.confidence;
    result.validLayers = base.validLayers;
    result.layers.reserve(base.layers.size());
    for (const auto& layer : base.layers) result.layers.push_back(convertLayer(layer));
    result.reason = std::string("GeometryReview_SurfaceBoundary_BASELINE:") + base.reason;

    HoleJiheNeiWallTrack::Input persistentInnerWallInput;
    persistentInnerWallInput.topU = base.centerU;
    persistentInnerWallInput.topV = base.centerV;
    persistentInnerWallInput.topW = input.topW;
    persistentInnerWallInput.seedU = input.seedU;
    persistentInnerWallInput.seedV = input.seedV;
    persistentInnerWallInput.initialTopRadius = base.topRadius;
    const HoleJiheNeiWallTrack::Result huiFu =
        HoleJiheNeiWallTrack::evaluate(persistentInnerWallSamples, persistentInnerWallInput);

    result.huiFuType = huiFu.holeType;
    result.huiFuLayers = huiFu.validLayers;
    result.huiFuTopRadius = huiFu.topRadius;
    result.huiFuBottomRadius = huiFu.bottomRadius;
    result.huiFuDepth = huiFu.depth;
    result.huiFuSlope = huiFu.radiusSlope;
    result.huiFuShrink = huiFu.radiusShrink;
    result.huiFuMonotonicRatio = huiFu.monotonicRatio;
    result.huiFuConfidence = huiFu.confidence;
    result.huiFuRadiusDelta = huiFu.topRadius - base.topRadius;

    if (!strictConeHuiFu(base, huiFu)) return result;

    result.coneHuiFuUsed = true;
    result.holeType = 2;
    result.inwardPolarity = huiFu.inwardPolarity;
    const double baseScale = std::max(base.topRadius, base.surfaceRadius);

    result.topRadius = std::clamp(
        0.80 * huiFu.topRadius + 0.20 * baseScale, 0.50, 11.0);
    result.surfaceRadius = result.topRadius;
    result.bottomValid = true;
    result.bottomRadius = std::clamp(
        huiFu.bottomRadius, 0.20, 0.96 * result.topRadius);
    result.depth = huiFu.depth;
    result.confidence = std::clamp(
        std::max(base.confidence, huiFu.confidence), 0.0, 0.98);
    result.reason = "GeometryReview_SurfaceBoundary_BASE_STRICT_PersistentInnerWall_CONE_RESCUE";
    return result;
}

}


// ============================================================================
// 功能分区：中心与半径细化
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/
#include <string>

namespace HoleJiheRadiusJingXiu {
namespace {

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

/** 【函数导航】
 * 作用：执行“convertLayer”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Layer convertLayer(const HoleJiheZhuiBuJiu::Layer& layer)
{
    return {layer.depth, layer.radius, layer.coverage, layer.span,
        layer.points, layer.sectors};
}

/** 【函数导航】
 * 作用：选择“selectOrderedRadius”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool selectOrderedRadius(
    const HoleJiheZhuiBuJiu::Result& base,
    const HoleJiheSurfacePouMian::Result& local,
    double& selectedRadius,
    double& adjustedSurfaceRadius,
    std::string& mode)
{
    if (!base.valid || !local.valid || !local.surfaceValid) return false;
    if (!finite(base.topRadius) || !finite(local.topRadius)
        || !finite(local.surfaceRadius)) return false;
    if (base.topRadius < 0.40 || base.topRadius > 11.0
        || local.topRadius < 0.40 || local.topRadius > 11.0
        || local.surfaceRadius < 0.40 || local.surfaceRadius > 11.0) return false;
    if (local.centerShift > 0.400001 || local.surfaceSectors < 12) return false;

    const double edgeAllowance = std::clamp(
        0.06 * local.surfaceRadius, 0.18, 0.35);
    adjustedSurfaceRadius = std::max(0.40,
        local.surfaceRadius - edgeAllowance);

    if (base.holeType == 2) {

        const double localObservation = std::max(
            local.topRadius, adjustedSurfaceRadius);
        if (localObservation <= base.topRadius) {
            const double blended = 0.75 * base.topRadius
                + 0.25 * localObservation;
            selectedRadius = std::max(0.84 * base.topRadius, blended);
            mode = "RadiusCenterRefine_CONE_BOUNDED_INWARD";
        } else {
            const double blended = 0.75 * base.topRadius
                + 0.25 * localObservation;
            selectedRadius = std::min(1.12 * base.topRadius, blended);
            mode = "RadiusCenterRefine_CONE_BOUNDED_OUTWARD";
        }
        if (base.bottomValid) {
            selectedRadius = std::max(selectedRadius,
                std::max(base.bottomRadius + 0.20,
                         1.08 * base.bottomRadius));
        }
        selectedRadius = std::clamp(selectedRadius, 0.50, 11.0);
        return std::abs(selectedRadius - base.topRadius) >= 0.05;
    }

    std::array<double, 3> candidates{
        base.topRadius, local.topRadius, adjustedSurfaceRadius};
    std::sort(candidates.begin(), candidates.end());
    selectedRadius = candidates[1];
    mode = "RadiusCenterRefine_STRAIGHT_ORDERED_MEDIAN";
    selectedRadius = std::clamp(selectedRadius, 0.40, 11.0);
    return std::abs(selectedRadius - base.topRadius) >= 0.05;
}
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    Result result;
    result.initialRadius = input.initialTopRadius;
    result.seedDistance = std::hypot(input.topU - input.seedU,
                                     input.topV - input.seedV);

    static thread_local std::vector<HoleJiheZhuiBuJiu::Sample> geometryReviewSamples;
    static thread_local std::vector<HoleJiheSurfacePouMian::Sample> surfaceBoundarySamples;
    geometryReviewSamples.clear();
    surfaceBoundarySamples.clear();
    if (geometryReviewSamples.capacity() < samples.size()) geometryReviewSamples.reserve(samples.size());
    if (surfaceBoundarySamples.capacity() < samples.size()) surfaceBoundarySamples.reserve(samples.size());
    for (const Sample& sample : samples) {
        geometryReviewSamples.push_back({sample.u, sample.v, sample.w});
        surfaceBoundarySamples.push_back({sample.u, sample.v, sample.w});
    }

    HoleJiheZhuiBuJiu::Input geometryReviewInput;
    geometryReviewInput.topU = input.topU;
    geometryReviewInput.topV = input.topV;
    geometryReviewInput.topW = input.topW;
    geometryReviewInput.seedU = input.seedU;
    geometryReviewInput.seedV = input.seedV;
    geometryReviewInput.initialTopRadius = input.initialTopRadius;
    geometryReviewInput.baselineMaxCenterShift = input.baselineMaxCenterShift;
    geometryReviewInput.baselineCenterShiftPenalty = input.baselineCenterShiftPenalty;
    const HoleJiheZhuiBuJiu::Result base =
        HoleJiheZhuiBuJiu::evaluate(geometryReviewSamples, geometryReviewInput);
    if (!base.valid) {
        result.reason = std::string("RadiusCenterRefine_GeometryReview_REJECT:") + base.reason;
        return result;
    }

    result.valid = true;
    result.surfaceValid = base.surfaceValid;
    result.profileValid = base.profileValid;
    result.bottomValid = base.bottomValid;
    result.coneHuiFuUsed = base.coneHuiFuUsed;
    result.holeType = base.holeType;
    result.inwardPolarity = base.inwardPolarity;
    result.baselineType = base.baselineType;
    result.huiFuType = base.huiFuType;
    result.centerU = input.topU;
    result.centerV = input.topV;
    result.centerShift = 0.0;
    result.surfaceRadius = base.surfaceRadius;
    result.surfaceMad = base.surfaceMad;
    result.surfaceSectors = base.surfaceSectors;
    result.topRadius = base.topRadius;
    result.bottomRadius = base.bottomRadius;
    result.depth = base.depth;
    result.radiusSlope = base.radiusSlope;
    result.radiusShrink = base.radiusShrink;
    result.monotonicRatio = base.monotonicRatio;
    result.confidence = base.confidence;
    result.validLayers = base.validLayers;
    result.huiFuLayers = base.huiFuLayers;
    result.huiFuTopRadius = base.huiFuTopRadius;
    result.huiFuBottomRadius = base.huiFuBottomRadius;
    result.huiFuDepth = base.huiFuDepth;
    result.huiFuSlope = base.huiFuSlope;
    result.huiFuShrink = base.huiFuShrink;
    result.huiFuMonotonicRatio = base.huiFuMonotonicRatio;
    result.huiFuConfidence = base.huiFuConfidence;
    result.huiFuRadiusDelta = base.huiFuRadiusDelta;
    result.layers.reserve(base.layers.size());
    for (const auto& layer : base.layers) result.layers.push_back(convertLayer(layer));
    result.reason = std::string("RadiusCenterRefine_GeometryReview_BASE:") + base.reason;
    result.radiusBeforeRefine = base.topRadius;

    HoleJiheSurfacePouMian::Input localInput;
    localInput.topU = input.topU;
    localInput.topV = input.topV;
    localInput.topW = input.topW;
    localInput.initialTopRadius = input.initialTopRadius;
    localInput.maxCenterShift = 0.35;
    localInput.centerShiftPenalty = 2.00;
    const HoleJiheSurfacePouMian::Result local =
        HoleJiheSurfacePouMian::evaluate(surfaceBoundarySamples, localInput);

    result.refineType = local.holeType;
    result.refinedRadius = local.topRadius;
    result.refineCenterShift = local.centerShift;
    result.refineSurfaceRadius = local.surfaceRadius;
    result.refineSurfaceMad = local.surfaceMad;
    result.refineSurfaceSectors = local.surfaceSectors;
    result.refineLayers = local.validLayers;
    result.refineSlope = local.radiusSlope;
    result.refineShrink = local.radiusShrink;
    result.refineMonotonicRatio = local.monotonicRatio;

    const bool localCenterReliable = local.valid && local.surfaceValid
        && local.centerShift <= 0.400001
        && local.surfaceSectors >= 12;
    if (localCenterReliable) {
        result.centerU = local.centerU;
        result.centerV = local.centerV;
        result.centerShift = std::hypot(result.centerU - input.topU,
                                        result.centerV - input.topV);
        result.centerRefineUsed = result.centerShift > 1e-6;
        result.surfaceRadius = local.surfaceRadius;
        result.surfaceMad = local.surfaceMad;
        result.surfaceSectors = local.surfaceSectors;
    }

    double selectedRadius = base.topRadius;
    double adjustedSurfaceRadius = 0.0;
    std::string radiusMode;
    result.adjustedSurfaceRadius = adjustedSurfaceRadius;
    if (selectOrderedRadius(base, local, selectedRadius, adjustedSurfaceRadius, radiusMode)) {
        result.adjustedSurfaceRadius = adjustedSurfaceRadius;
        result.radiusMode = radiusMode;
        result.radiusRefineUsed = true;
        result.topRadius = selectedRadius;
        if (result.holeType == 2 && result.bottomValid) {
            result.bottomRadius = std::min(result.bottomRadius,
                0.96 * result.topRadius);
        } else {
            result.bottomRadius = result.topRadius;
        }
        result.confidence = std::clamp(
            std::max(base.confidence, local.confidence), 0.0, 0.98);
        result.reason = radiusMode + ":" + base.reason;
    }

    const double finalShift = std::hypot(result.centerU - input.topU,
                                         result.centerV - input.topV);
    if (!finite(finalShift) || finalShift > 0.400001) {
        result.centerU = input.topU;
        result.centerV = input.topV;
        result.centerShift = 0.0;
        result.centerRefineUsed = false;
        result.reason += ":CENTER_RESET_TO_MouthSupportPlane";
    } else {
        result.centerShift = finalShift;
    }
    return result;
}

}


// ============================================================================
// 功能分区：最终孔几何汇总
// ============================================================================
/*
模块职责：
孔几何估计子模块。

主要调用位置：
由正式孔识别兼容链调用，负责中心、半径、法向或局部几何证据。

维护说明：
本分区负责独立的几何步骤；阈值会直接影响中心、半径或孔形判断，调整时应结合相邻分区一起检查。
*/
#include <queue>

namespace HoleJiheFinal {
namespace {

constexpr double kPi = 3.14159265358979323846;

/** 【函数导航】
 * 作用：执行“finite”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleFenxi_Analysis.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool finite(double value) noexcept { return std::isfinite(value); }

double median(std::vector<double> values)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 != 0
        ? values[middle]
        : 0.5 * (values[middle - 1] + values[middle]);
}

/** 【函数导航】
 * 作用：执行“medianArray”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
template <std::size_t N>
double medianArray(std::array<double, N> values, std::size_t count)
{
    if (count == 0 || count > N) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count));
    const std::size_t middle = count / 2;
    return (count & 1U) ? values[middle]
                        : 0.5 * (values[middle - 1] + values[middle]);
}

/** 【类型导航注释】
 * ZhouXiangPingMian：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ZhouXiangPingMian {
    bool valid = false;
    double w = 0.0;
    double mad = 0.0;
    int support = 0;
};

/** 【函数导航】
 * 作用：估计“estimateOuterPlane”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
ZhouXiangPingMian estimateOuterPlane(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double initialW)
{
    constexpr double kRange = 3.0;
    constexpr double kBinWidth = 0.06;
    constexpr int kBins = 101;
    std::array<int, kBins> histogram{};
    std::vector<double> values;
    values.reserve(samples.size() / 3);

    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double radius = std::hypot(sample.u - centerU, sample.v - centerV);
        if (radius < 9.0 || radius > 19.0) continue;
        const double delta = sample.w - initialW;
        if (delta < -kRange || delta > kRange) continue;
        const int index = std::clamp(
            static_cast<int>(std::floor((delta + kRange) / kBinWidth)),
            0, kBins - 1);
        ++histogram[static_cast<std::size_t>(index)];
        values.push_back(sample.w);
    }

    ZhouXiangPingMian result;
    if (values.size() < 45) return result;

    int bestIndex = 0;
    int bestCount = -1;
    for (int index = 0; index < kBins; ++index) {
        int count = histogram[static_cast<std::size_t>(index)];
        if (index > 0) count += histogram[static_cast<std::size_t>(index - 1)];
        if (index + 1 < kBins) count += histogram[static_cast<std::size_t>(index + 1)];
        if (count > bestCount) {
            bestCount = count;
            bestIndex = index;
        }
    }

    const double modeW = initialW - kRange
        + (static_cast<double>(bestIndex) + 0.5) * kBinWidth;
    std::vector<double> support;
    support.reserve(values.size());
    for (double value : values) {
        if (std::abs(value - modeW) <= 0.16) support.push_back(value);
    }
    if (support.size() < 36) return result;

    const double planeW = median(support);
    std::vector<double> deviations;
    deviations.reserve(support.size());
    for (double value : support) deviations.push_back(std::abs(value - planeW));
    const double mad = median(std::move(deviations));
    const double shift = planeW - initialW;
    if (!finite(planeW) || !finite(mad) || mad > 0.15
        || std::abs(shift) > 2.60) {
        return result;
    }

    result.valid = true;
    result.w = planeW;
    result.mad = mad;
    result.support = static_cast<int>(support.size());
    return result;
}

/** 【类型导航注释】
 * KongDongYuan：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct KongDongYuan {
    bool valid = false;
    double centerU = 0.0;
    double centerV = 0.0;
    double radius = 0.0;
    double circularity = 0.0;
    double minDistance = 0.0;
    double seedBoundaryError = 0.0;
    double canonicalCenterError = 0.0;
    int areaCells = 0;
    bool touchesGrid = false;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：估计“estimateCircularTopVoids”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
std::vector<KongDongYuan> estimateCircularTopVoids(
    const std::vector<Sample>& samples,
    double originU,
    double originV,
    double topW,
    double seedU,
    double seedV,
    bool canonicalMode,
    double canonicalCenterU,
    double canonicalCenterV,
    bool relaxedSeedGate)
{
    constexpr double kCell = 0.20;
    constexpr double kExtent = 16.0;
    constexpr double kBand = 0.38;
    constexpr int kGrid = 160;
    constexpr int kCellCount = kGrid * kGrid;

    std::vector<unsigned char> occupied(static_cast<std::size_t>(kCellCount), 0);
    int topPointCount = 0;
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        if (std::abs(sample.w - topW) > kBand) continue;
        const double du = sample.u - originU;
        const double dv = sample.v - originV;
        if (du < -kExtent || du >= kExtent || dv < -kExtent || dv >= kExtent) continue;
        const int x = static_cast<int>(std::floor((du + kExtent) / kCell));
        const int y = static_cast<int>(std::floor((dv + kExtent) / kCell));
        if (x < 0 || x >= kGrid || y < 0 || y >= kGrid) continue;
        occupied[static_cast<std::size_t>(y * kGrid + x)] = 1;
        ++topPointCount;
    }
    if (topPointCount < 65) return {};

    std::vector<unsigned char> plate = occupied;
    for (int y = 0; y < kGrid; ++y) {
        for (int x = 0; x < kGrid; ++x) {
            if (!occupied[static_cast<std::size_t>(y * kGrid + x)]) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    const int yy = y + dy;
                    if (xx >= 0 && xx < kGrid && yy >= 0 && yy < kGrid) {
                        plate[static_cast<std::size_t>(yy * kGrid + xx)] = 1;
                    }
                }
            }
        }
    }

    std::vector<int> label(static_cast<std::size_t>(kCellCount), -1);
    int nextLabel = 0;
    std::vector<KongDongYuan> candidates;
    const std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    const std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};

    for (int startY = 0; startY < kGrid; ++startY) {
        for (int startX = 0; startX < kGrid; ++startX) {
            const int startIndex = startY * kGrid + startX;
            if (plate[static_cast<std::size_t>(startIndex)]
                || label[static_cast<std::size_t>(startIndex)] >= 0) continue;

            std::queue<int> queue;
            queue.push(startIndex);
            label[static_cast<std::size_t>(startIndex)] = nextLabel;

            int area = 0;
            bool touches = false;
            double sumU = 0.0;
            double sumV = 0.0;
            double sumUU = 0.0;
            double sumVV = 0.0;
            double sumUV = 0.0;
            double minimumDistance = std::numeric_limits<double>::infinity();

            while (!queue.empty()) {
                const int index = queue.front();
                queue.pop();
                const int y = index / kGrid;
                const int x = index - y * kGrid;
                const double u = (static_cast<double>(x) + 0.5) * kCell - kExtent;
                const double v = (static_cast<double>(y) + 0.5) * kCell - kExtent;
                ++area;
                sumU += u;
                sumV += v;
                sumUU += u * u;
                sumVV += v * v;
                sumUV += u * v;
                minimumDistance = std::min(minimumDistance, std::hypot(u, v));
                touches = touches || x == 0 || y == 0
                    || x + 1 == kGrid || y + 1 == kGrid;

                for (std::size_t neighbor = 0; neighbor < dx.size(); ++neighbor) {
                    const int xx = x + dx[neighbor];
                    const int yy = y + dy[neighbor];
                    if (xx < 0 || xx >= kGrid || yy < 0 || yy >= kGrid) continue;
                    const int next = yy * kGrid + xx;
                    if (plate[static_cast<std::size_t>(next)]
                        || label[static_cast<std::size_t>(next)] >= 0) continue;
                    label[static_cast<std::size_t>(next)] = nextLabel;
                    queue.push(next);
                }
            }
            ++nextLabel;
            if (area < 35) continue;

            const double count = static_cast<double>(area);
            const double localCenterU = sumU / count;
            const double localCenterV = sumV / count;
            const double varU = std::max(0.0, sumUU / count - localCenterU * localCenterU);
            const double varV = std::max(0.0, sumVV / count - localCenterV * localCenterV);
            const double covUV = sumUV / count - localCenterU * localCenterV;
            const double trace = varU + varV;
            const double discriminant = std::sqrt(std::max(
                0.0, (varU - varV) * (varU - varV) + 4.0 * covUV * covUV));
            const double lambdaMaximum = 0.5 * (trace + discriminant);
            const double lambdaMinimum = 0.5 * (trace - discriminant);
            const double circularity = lambdaMaximum > 1e-9
                ? lambdaMinimum / lambdaMaximum : 0.0;
            const double radius = std::sqrt(count * kCell * kCell / kPi)
                + std::sqrt(2.0) * kCell;
            const double centerDistance = std::hypot(localCenterU, localCenterV);
            const double centerU = originU + localCenterU;
            const double centerV = originV + localCenterV;
            const double canonicalCenterError = std::hypot(
                canonicalCenterU - centerU, canonicalCenterV - centerV);
            double seedBoundaryError = 0.0;
            if (!canonicalMode) {
                const double seedDistance = std::hypot(seedU - centerU, seedV - centerV);
                seedBoundaryError = std::abs(seedDistance - radius);
            }

            if (!finite(radius) || radius < 1.0 || radius > 10.8) continue;
            if (circularity < 0.52 || centerDistance > 13.0) continue;
            if (minimumDistance > radius + 1.0) continue;
            if (!canonicalMode) {
                const double seedErrorLimit = relaxedSeedGate
                    ? std::max(6.5, 1.15 * radius)
                    : std::max(3.0, 0.65 * radius);
                if (seedBoundaryError > seedErrorLimit) continue;
            }
            if (touches && (circularity < 0.82 || centerDistance > 11.5
                            || radius > 7.0)) continue;

            KongDongYuan candidate;
            candidate.valid = true;
            candidate.centerU = centerU;
            candidate.centerV = centerV;
            candidate.radius = radius;
            candidate.circularity = circularity;
            candidate.minDistance = minimumDistance;
            candidate.seedBoundaryError = seedBoundaryError;
            candidate.canonicalCenterError = canonicalCenterError;
            candidate.areaCells = area;
            candidate.touchesGrid = touches;
            candidate.score = 5.0 * circularity
                + 0.25 * radius
                - (canonicalMode ? 0.18 * canonicalCenterError
                                 : 0.60 * seedBoundaryError)
                - 0.05 * centerDistance
                - (touches ? 0.25 : 0.0);
            candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const KongDongYuan& a, const KongDongYuan& b) {
        return a.score > b.score;
    });
    std::vector<KongDongYuan> unique;
    for (const KongDongYuan& candidate : candidates) {
        bool duplicate = false;
        for (const KongDongYuan& existing : unique) {
            if (std::hypot(candidate.centerU - existing.centerU,
                           candidate.centerV - existing.centerV) < 0.8
                && std::abs(candidate.radius - existing.radius) < 0.5) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) unique.push_back(candidate);
        if (unique.size() >= 8) break;
    }
    return unique;
}

/** 【类型导航注释】
 * HoleBiZhengJu：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleBiZhengJu {
    bool valid = false;
    bool cone = false;
    bool stable = false;
    int polarity = 0;
    int layers = 0;
    double slope = 0.0;
    double shrink = 0.0;
    double monotonic = 0.0;
    double firstRadius = 0.0;
    double lastRadius = 0.0;
    double lastDepth = 0.0;
};

/** 【函数导航】
 * 作用：执行“measureWallEvidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleBiZhengJu measureWallEvidence(
    const std::vector<Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    double mouthRadius)
{
    HoleBiZhengJu best;
    double bestScore = -std::numeric_limits<double>::infinity();
    constexpr int kSectors = 48;
    constexpr int kLayers = 10;
    constexpr double kLayerStep = 0.25;
    constexpr double kHalfSlab = 0.20;

    /** 【类型导航注释】
     * YuChuLiHoleBiYangBen：Hole 几何实现中的自定义 结构体。
     * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    struct YuChuLiHoleBiYangBen {
        double w = 0.0;
        double radius = 0.0;
        int sector = 0;
    };
    std::vector<YuChuLiHoleBiYangBen> prepared;
    prepared.reserve(samples.size());
    for (const Sample& sample : samples) {
        if (!finite(sample.u) || !finite(sample.v) || !finite(sample.w)) continue;
        const double du = sample.u - centerU;
        const double dv = sample.v - centerV;
        const double radius = std::hypot(du, dv);
        if (radius < 0.45 || radius > mouthRadius + 1.25) continue;
        double angle = std::atan2(dv, du);
        if (angle < 0.0) angle += 2.0 * kPi;
        const int sector = std::clamp(
            static_cast<int>(angle * static_cast<double>(kSectors) / (2.0 * kPi)),
            0, kSectors - 1);
        prepared.push_back({sample.w, radius, sector});
    }

    for (int polarity : {-1, 1}) {
        std::array<double, kLayers> depths{};
        std::array<double, kLayers> radii{};
        std::size_t layerCount = 0;
        for (int layerIndex = 1; layerIndex <= kLayers; ++layerIndex) {
            const double depth = kLayerStep * static_cast<double>(layerIndex);
            std::array<double, kSectors> minimumRadius;
            minimumRadius.fill(std::numeric_limits<double>::infinity());
            for (const YuChuLiHoleBiYangBen& sample : prepared) {
                const double axial = static_cast<double>(polarity) * (sample.w - topW);
                if (std::abs(axial - depth) > kHalfSlab) continue;
                minimumRadius[static_cast<std::size_t>(sample.sector)] = std::min(
                    minimumRadius[static_cast<std::size_t>(sample.sector)], sample.radius);
            }
            std::array<double, kSectors> sectorRadii{};
            std::size_t sectorCount = 0;
            for (double radius : minimumRadius) {
                if (finite(radius)) sectorRadii[sectorCount++] = radius;
            }
            if (sectorCount < 5) continue;
            const double layerRadius = medianArray(sectorRadii, sectorCount);
            std::array<double, kSectors> deviations{};
            for (std::size_t i = 0; i < sectorCount; ++i)
                deviations[i] = std::abs(sectorRadii[i] - layerRadius);
            const double mad = medianArray(deviations, sectorCount);
            if (!finite(layerRadius) || !finite(mad) || mad > 1.0) continue;
            depths[layerCount] = depth;
            radii[layerCount] = layerRadius;
            ++layerCount;
        }
        if (layerCount < 2) continue;

        double depthSum = 0.0;
        double radiusSum = 0.0;
        for (std::size_t i = 0; i < layerCount; ++i) {
            depthSum += depths[i];
            radiusSum += radii[i];
        }
        const double meanDepth = depthSum / static_cast<double>(layerCount);
        const double meanRadius = radiusSum / static_cast<double>(layerCount);
        double numerator = 0.0;
        double denominator = 0.0;
        for (std::size_t i = 0; i < layerCount; ++i) {
            numerator += (depths[i] - meanDepth) * (radii[i] - meanRadius);
            denominator += (depths[i] - meanDepth) * (depths[i] - meanDepth);
        }
        const double slope = denominator > 1e-9 ? numerator / denominator : 0.0;
        const double shrink = radii[0] - radii[layerCount - 1];
        int monotonicCount = 0;
        for (std::size_t i = 1; i < layerCount; ++i) {
            if (radii[i] <= radii[i - 1] + 0.10) ++monotonicCount;
        }
        const double monotonic = layerCount > 1
            ? static_cast<double>(monotonicCount) / static_cast<double>(layerCount - 1)
            : 0.0;
        const bool cone = (layerCount >= 3 && slope < -0.14
                           && shrink >= 0.18 && monotonic >= 0.65)
            || (layerCount >= 2 && slope < -0.24
                && shrink >= 0.16 && monotonic >= 0.99);
        const bool stable = layerCount >= 3
            && std::abs(slope) <= 0.16
            && std::abs(shrink) <= 0.28;
        const double score = (cone ? 100.0 : 0.0)
            + (stable ? 80.0 : 0.0)
            + 2.0 * static_cast<double>(layerCount)
            + (cone ? 10.0 * shrink : -2.0 * std::abs(shrink));
        if (score > bestScore) {
            bestScore = score;
            best.valid = true;
            best.cone = cone;
            best.stable = stable;
            best.polarity = polarity;
            best.layers = static_cast<int>(layerCount);
            best.slope = slope;
            best.shrink = shrink;
            best.monotonic = monotonic;
            best.firstRadius = radii[0];
            best.lastRadius = radii[layerCount - 1];
            best.lastDepth = depths[layerCount - 1];
        }
    }
    return best;
}

/** 【函数导航】
 * 作用：执行“measuredCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool measuredCone(const HoleJiheRadiusJingXiu::Result& candidate)
{
    return candidate.valid && candidate.holeType == 2
        && candidate.bottomValid
        && candidate.validLayers >= 3
        && candidate.bottomRadius > 0.20
        && candidate.depth >= 0.40
        && candidate.bottomRadius < 0.98 * candidate.topRadius;
}

/** 【函数导航】
 * 作用：执行“strongConeProfile”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool strongConeProfile(const HoleJiheRadiusJingXiu::Result& candidate)
{
    return candidate.valid && candidate.profileValid
        && candidate.validLayers >= 3
        && candidate.radiusSlope < -0.18
        && candidate.radiusShrink >= 0.20
        && candidate.monotonicRatio >= 0.75;
}

/** 【函数导航】
 * 作用：执行“weakPersistentCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool weakPersistentCone(const HoleJiheRadiusJingXiu::Result& base)
{
    return base.holeType == 1
        && base.profileValid
        && base.validLayers >= 3
        && base.radiusSlope < -0.18
        && base.radiusShrink >= 0.18
        && base.monotonicRatio >= 0.80;
}

/** 【函数导航】
 * 作用：执行“strongPersistentInnerWallCone”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
bool strongPersistentInnerWallCone(const HoleJiheRadiusJingXiu::Result& base)
{
    return base.holeType == 1
        && base.huiFuType == 2
        && base.huiFuLayers >= 5
        && base.huiFuSlope < -0.28
        && base.huiFuShrink >= 0.45
        && base.huiFuMonotonicRatio >= 0.85;
}

/** 【类型导航注释】
 * JiheHouXuan：Hole 几何实现中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.cpp（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct JiheHouXuan {
    bool valid = false;
    HoleJiheRadiusJingXiu::Result geometry;
    double topW = 0.0;
    double centerU = 0.0;
    double centerV = 0.0;
    double axialOffset = 0.0;
    bool fromVoid = false;
    KongDongYuan voidCircle;
    HoleBiZhengJu wall;
    double score = -std::numeric_limits<double>::infinity();
};

/** 【函数导航】
 * 作用：执行“runRadiusCenterRefine”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
HoleJiheRadiusJingXiu::Result runRadiusCenterRefine(
    const std::vector<HoleJiheRadiusJingXiu::Sample>& samples,
    double centerU,
    double centerV,
    double topW,
    double seedU,
    double seedV,
    double initialRadius,
    bool canonicalMode)
{
    HoleJiheRadiusJingXiu::Input input;
    input.topU = centerU;
    input.topV = centerV;
    input.topW = topW;
    input.seedU = seedU;
    input.seedV = seedV;
    input.initialTopRadius = initialRadius;
    if (canonicalMode) {
        input.baselineMaxCenterShift = 0.20;
        input.baselineCenterShiftPenalty = 1.00;
    }
    return HoleJiheRadiusJingXiu::evaluate(samples, input);
}

/** 【函数导航】
 * 作用：执行“copyBaseToResult”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.cpp（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
void copyBaseToResult(Result& result, const JiheHouXuan& selected, const Input& input)
{
    result.valid = selected.geometry.valid;
    result.surfaceValid = selected.geometry.surfaceValid;
    result.profileValid = selected.geometry.profileValid;
    result.bottomValid = selected.geometry.bottomValid;
    result.holeType = selected.geometry.holeType;
    result.inwardPolarity = selected.geometry.inwardPolarity;
    result.centerU = selected.geometry.centerU;
    result.centerV = selected.geometry.centerV;
    result.centerShift = std::hypot(result.centerU - input.topU,
                                    result.centerV - input.topV);
    result.topW = selected.topW;
    result.topRadius = selected.geometry.topRadius;
    result.bottomRadius = selected.geometry.bottomRadius;
    result.depth = selected.geometry.depth;
    result.confidence = selected.geometry.confidence;
}

}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 几何实现。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleFenxi_Analysis.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
Result evaluate(const std::vector<Sample>& samples, const Input& input)
{
    Result result;
    result.inputTopW = input.topW;
    result.topW = input.topW;
    result.initialRadius = input.initialTopRadius;
    result.canonicalMode = input.canonicalMode;
    result.rawSeedGateUsed = !input.canonicalMode;
    result.rawSeedScoreUsed = !input.canonicalMode;

    if (samples.size() < 30 || !finite(input.topU) || !finite(input.topV)
        || !finite(input.topW) || !finite(input.seedU) || !finite(input.seedV)
        || (input.canonicalMode
            && (!finite(input.canonicalCenterU) || !finite(input.canonicalCenterV)))) {
        result.reason = "FinalGeometry_INVALID_INPUT";
        return result;
    }

    static thread_local std::vector<HoleJiheRadiusJingXiu::Sample> radiusCenterRefineSamples;
    radiusCenterRefineSamples.clear();
    if (radiusCenterRefineSamples.capacity() < samples.size()) radiusCenterRefineSamples.reserve(samples.size());
    for (const Sample& sample : samples) {
        radiusCenterRefineSamples.push_back({sample.u, sample.v, sample.w});
    }

    const double effectiveSeedU = input.canonicalMode
        ? input.canonicalCenterU : input.seedU;
    const double effectiveSeedV = input.canonicalMode
        ? input.canonicalCenterV : input.seedV;

    JiheHouXuan baseCandidate;
    baseCandidate.geometry = runRadiusCenterRefine(radiusCenterRefineSamples,
        input.topU, input.topV, input.topW,
        effectiveSeedU, effectiveSeedV, input.initialTopRadius, input.canonicalMode);
    baseCandidate.valid = baseCandidate.geometry.valid;
    baseCandidate.topW = input.topW;
    baseCandidate.centerU = input.topU;
    baseCandidate.centerV = input.topV;
    const bool baseRejected = !baseCandidate.valid;

    const ZhouXiangPingMian plane = estimateOuterPlane(
        samples, input.topU, input.topV, input.topW);
    result.axialAnchorValid = plane.valid;
    result.axialMad = plane.mad;
    result.axialSupport = plane.support;
    result.axialShift = plane.valid ? plane.w - input.topW : 0.0;
    const double physicalTopW = plane.valid ? plane.w : input.topW;

    std::vector<KongDongYuan> voids = estimateCircularTopVoids(samples,
        input.topU, input.topV, physicalTopW, input.seedU, input.seedV,
        input.canonicalMode, input.canonicalCenterU, input.canonicalCenterV, false);
    const bool baseUndersizedStraight = baseCandidate.geometry.valid
        && baseCandidate.geometry.holeType == 1
        && input.initialTopRadius >= 2.5
        && input.initialTopRadius - baseCandidate.geometry.topRadius > 0.75;
    if (voids.empty() && baseUndersizedStraight) {

        voids = estimateCircularTopVoids(samples,
            input.topU, input.topV, physicalTopW, input.seedU, input.seedV,
            input.canonicalMode, input.canonicalCenterU, input.canonicalCenterV, true);
        result.voidRelaxedSearch = !voids.empty();
    }
    result.voidCandidateCount = static_cast<int>(voids.size());
    result.voidValid = !voids.empty();
    KongDongYuan targetVoid;
    if (!voids.empty()) targetVoid = voids.front();

    // canonicalMode 的语义是：上游已经通过统一 mouth-search 锁定了一个物理 Hole 口，
    // 本函数负责进一步“测量”中心、半径、孔壁和孔型，而不是重新决定这个 Hole 是否存在。
    // 因此把锁定的机械孔口本身作为候选集合中的基础成员。RadiusCenterRefine、二维 void、
    // 锥壁和多层轮廓只要得到更充分的测量证据，仍会正常覆盖它；如果精细测量因为圆弧缺失、
    // 降噪或局部遮挡没有形成完整几何，canonical mouth 仍可继续交给后面的统一物理空腔审核。
    JiheHouXuan canonicalMouthCandidate;
    if (input.canonicalMode && input.initialTopRadius > 1.0
        && finite(input.canonicalCenterU) && finite(input.canonicalCenterV)) {
        canonicalMouthCandidate.valid = true;
        canonicalMouthCandidate.topW = physicalTopW;
        canonicalMouthCandidate.centerU = input.canonicalCenterU;
        canonicalMouthCandidate.centerV = input.canonicalCenterV;
        canonicalMouthCandidate.geometry.valid = true;
        canonicalMouthCandidate.geometry.surfaceValid = plane.valid;
        canonicalMouthCandidate.geometry.profileValid = false;
        canonicalMouthCandidate.geometry.bottomValid = false;
        canonicalMouthCandidate.geometry.holeType = 1;
        canonicalMouthCandidate.geometry.inwardPolarity = 0;
        canonicalMouthCandidate.geometry.centerU = input.canonicalCenterU;
        canonicalMouthCandidate.geometry.centerV = input.canonicalCenterV;
        canonicalMouthCandidate.geometry.centerShift = std::hypot(
            input.canonicalCenterU - input.topU, input.canonicalCenterV - input.topV);
        canonicalMouthCandidate.geometry.initialRadius = input.initialTopRadius;
        canonicalMouthCandidate.geometry.surfaceRadius = input.initialTopRadius;
        canonicalMouthCandidate.geometry.topRadius = input.initialTopRadius;
        canonicalMouthCandidate.geometry.bottomRadius = input.initialTopRadius;
        canonicalMouthCandidate.geometry.depth = 0.0;
        canonicalMouthCandidate.geometry.confidence = 0.50;
        canonicalMouthCandidate.geometry.reason = "RadiusCenterRefine_CANONICAL_MOUTH_IDENTITY";
        canonicalMouthCandidate.geometry.radiusMode = "CanonicalMouthIdentity";
    }

    JiheHouXuan selected = baseCandidate.valid ? baseCandidate : canonicalMouthCandidate;
    if (!baseCandidate.valid && canonicalMouthCandidate.valid)
        result.takeoverMode = "FinalGeometry_CANONICAL_MOUTH_IDENTITY";
    JiheHouXuan voidCandidate;

    if (targetVoid.valid) {
        result.voidCenterU = targetVoid.centerU;
        result.voidCenterV = targetVoid.centerV;
        result.voidCenterShift = std::hypot(
            targetVoid.centerU - input.topU, targetVoid.centerV - input.topV);
        result.voidRadius = targetVoid.radius;
        result.voidCircularity = targetVoid.circularity;
        result.voidMinDistance = targetVoid.minDistance;
        result.voidSeedBoundaryError = targetVoid.seedBoundaryError;
        result.canonicalCenterError = targetVoid.canonicalCenterError;
        result.voidAreaCells = targetVoid.areaCells;
        result.voidTouchesGrid = targetVoid.touchesGrid;

        const bool highQualityVoid = !targetVoid.touchesGrid
            && (input.canonicalMode
                ? targetVoid.circularity >= 0.60
                : (result.voidRelaxedSearch
                    ? (targetVoid.circularity >= 0.65
                       && targetVoid.seedBoundaryError <= std::max(6.0, 1.10 * targetVoid.radius))
                    : (targetVoid.circularity >= 0.60
                       && targetVoid.seedBoundaryError <= std::max(3.6, 0.72 * targetVoid.radius))));
        const bool largeSupportedMouth = targetVoid.radius >= 5.70
            && targetVoid.areaCells >= 2300
            && targetVoid.circularity >= 0.62;
        const bool baseStrongCone = measuredCone(baseCandidate.geometry)
            && baseCandidate.geometry.validLayers >= 4
            && baseCandidate.geometry.radiusShrink >= 0.35;
        const bool baseWeakCone = baseCandidate.geometry.holeType == 2 && !baseStrongCone;
        const double radiusDifference = std::abs(
            targetVoid.radius - baseCandidate.geometry.topRadius);

        HoleBiZhengJu wall;
        if (highQualityVoid && !baseStrongCone
            && (baseRejected || baseCandidate.geometry.holeType == 1 || baseWeakCone)) {
            const double wallMouthRadius = baseCandidate.geometry.valid
                ? std::max(targetVoid.radius, baseCandidate.geometry.topRadius)
                : targetVoid.radius;
            wall = measureWallEvidence(samples,
                targetVoid.centerU, targetVoid.centerV, physicalTopW, wallMouthRadius);
            result.wallEvidenceValid = wall.valid;
            result.wallEvidenceLayers = wall.layers;
            result.wallEvidencePolarity = wall.polarity;
            result.wallEvidenceSlope = wall.slope;
            result.wallEvidenceShrink = wall.shrink;
            result.wallEvidenceMonotonic = wall.monotonic;
            result.wallEvidenceCone = wall.cone;
            result.wallEvidenceStable = wall.stable;
        }

        const bool baseConeEvidence = weakPersistentCone(baseCandidate.geometry)
            || strongPersistentInnerWallCone(baseCandidate.geometry);
        const bool preStrongWallCone = wall.valid && wall.cone
            && wall.layers >= 4
            && wall.slope < -0.30
            && wall.shrink >= 0.35
            && wall.monotonic >= 0.75;
        const bool fastStraightConsensus = highQualityVoid
            && baseCandidate.geometry.valid
            && baseCandidate.geometry.holeType == 1
            && !largeSupportedMouth
            && !baseConeEvidence
            && !preStrongWallCone
            && radiusDifference <= 0.35;
        if (fastStraightConsensus) {

            selected = baseCandidate;
            result.voidRecoveryUsed = true;
            result.takeoverMode = "FinalGeometry_FAST_STRAIGHT_CONSENSUS";
        }

        const bool needsOffsetGeometry = highQualityVoid && !baseStrongCone
            && !result.voidRecoveryUsed
            && !largeSupportedMouth
            && !baseConeEvidence
            && (baseRejected || baseWeakCone || radiusDifference > 0.45);
        if (needsOffsetGeometry) {

            constexpr std::array<double, 3> kOffsets{{-0.06, 0.0, 0.06}};
            double bestScore = -std::numeric_limits<double>::infinity();
            for (double offset : kOffsets) {
                JiheHouXuan candidate;
                candidate.geometry = runRadiusCenterRefine(radiusCenterRefineSamples,
                    targetVoid.centerU, targetVoid.centerV, physicalTopW + offset,
                    effectiveSeedU, effectiveSeedV, targetVoid.radius, input.canonicalMode);
                ++result.axialCandidateCount;
                if (!candidate.geometry.valid) continue;
                candidate.valid = true;
                candidate.fromVoid = true;
                candidate.voidCircle = targetVoid;
                candidate.topW = physicalTopW + offset;
                candidate.centerU = targetVoid.centerU;
                candidate.centerV = targetVoid.centerV;
                const bool fullConeCandidate = measuredCone(candidate.geometry);
                const bool coneProfile = strongConeProfile(candidate.geometry);
                const double radiusAgreement = std::abs(
                    candidate.geometry.topRadius - targetVoid.radius);
                candidate.score = (fullConeCandidate ? 180.0 : 0.0)
                    + (coneProfile ? 100.0 : 0.0)
                    + (candidate.geometry.holeType == 1 ? 50.0 : 20.0)
                    + 5.0 * candidate.geometry.confidence
                    + 0.5 * static_cast<double>(candidate.geometry.validLayers)
                    - 1.0 * radiusAgreement
                    - 0.8 * std::abs(offset);
                if (!voidCandidate.valid || candidate.score > bestScore) {
                    bestScore = candidate.score;
                    voidCandidate = candidate;
                }
            }
            if (voidCandidate.valid) voidCandidate.wall = wall;
        }

        const bool fullCone = voidCandidate.valid && measuredCone(voidCandidate.geometry);
        const bool stableStraight = voidCandidate.valid
            && voidCandidate.geometry.holeType == 1
            && wall.valid && wall.stable;
        const bool strongWallCone = wall.valid && wall.cone
            && wall.layers >= 4
            && wall.slope < -0.30
            && wall.shrink >= 0.35
            && wall.monotonic >= 0.75;

        if (fullCone && highQualityVoid) {
            selected = voidCandidate;
            result.voidRecoveryUsed = true;
            result.takeoverMode = "FinalGeometry_VOID_CENTER_MEASURED_CONE";
        } else if (strongWallCone && highQualityVoid) {

            selected = voidCandidate.valid ? voidCandidate : baseCandidate;
            selected.valid = true;
            selected.topW = physicalTopW;
            selected.centerU = targetVoid.centerU;
            selected.centerV = targetVoid.centerV;
            selected.geometry.valid = true;
            selected.geometry.surfaceValid = true;
            selected.geometry.profileValid = true;
            selected.geometry.holeType = 2;
            selected.geometry.inwardPolarity = wall.polarity;
            selected.geometry.centerU = targetVoid.centerU;
            selected.geometry.centerV = targetVoid.centerV;
            const double wideRadius = baseCandidate.geometry.valid
                ? baseCandidate.geometry.topRadius : targetVoid.radius;
            selected.geometry.topRadius = 0.5 * (wideRadius + targetVoid.radius);
            selected.geometry.bottomRadius = wall.lastRadius;
            selected.geometry.depth = wall.lastDepth;
            selected.geometry.bottomValid = wall.lastRadius > 0.20
                && wall.lastRadius < 0.96 * selected.geometry.topRadius
                && wall.lastDepth >= 0.50;
            selected.geometry.validLayers = wall.layers;
            selected.geometry.radiusSlope = wall.slope;
            selected.geometry.radiusShrink = wall.shrink;
            selected.geometry.monotonicRatio = wall.monotonic;
            selected.geometry.confidence = 0.78;
            result.voidRecoveryUsed = true;
            result.takeoverMode = "FinalGeometry_STRONG_WALL_CONE_BEFORE_VOID_STRAIGHT";
        } else if (stableStraight && highQualityVoid
                   && (baseRejected || baseCandidate.geometry.holeType == 1 || baseWeakCone)
                   && (radiusDifference > 0.35
                       || result.voidCenterShift > 0.40
                       || baseWeakCone)) {
            selected = voidCandidate;
            result.voidRecoveryUsed = true;
            result.takeoverMode = baseWeakCone
                ? "FinalGeometry_VOID_STABLE_STRAIGHT_OVERRIDE_WEAK_CONE"
                : "FinalGeometry_VOID_STABLE_STRAIGHT";
        } else if ((baseRejected || baseCandidate.geometry.holeType == 1)
                   && largeSupportedMouth && highQualityVoid
                   && !weakPersistentCone(baseCandidate.geometry)
                   && !strongPersistentInnerWallCone(baseCandidate.geometry)) {

            selected = voidCandidate.valid
                ? voidCandidate
                : (baseCandidate.valid ? baseCandidate : canonicalMouthCandidate);
            if (!selected.geometry.valid) {
                selected.geometry.valid = true;
                selected.geometry.surfaceValid = true;
                selected.geometry.profileValid = false;
                selected.geometry.confidence = 0.66;
            }
            selected.valid = true;
            selected.topW = physicalTopW;
            selected.centerU = targetVoid.centerU;
            selected.centerV = targetVoid.centerV;
            if (baseCandidate.geometry.valid)
                selected.geometry = baseCandidate.geometry;
            selected.geometry.valid = true;
            selected.geometry.centerU = targetVoid.centerU;
            selected.geometry.centerV = targetVoid.centerV;
            selected.geometry.topRadius = std::max(
                baseCandidate.geometry.topRadius, targetVoid.radius);
            selected.geometry.holeType = 1;
            result.voidRecoveryUsed = true;
            result.takeoverMode = "FinalGeometry_LARGE_SUPPORTED_VISIBLE_MOUTH";
        } else if ((baseRejected || baseCandidate.geometry.holeType == 1)
                   && highQualityVoid && radiusDifference > 0.55) {

            selected = voidCandidate.valid
                ? voidCandidate
                : (baseCandidate.valid ? baseCandidate : canonicalMouthCandidate);
            if (!selected.geometry.valid) {
                selected.geometry.valid = true;
                selected.geometry.surfaceValid = true;
                selected.geometry.profileValid = false;
                selected.geometry.confidence = 0.66;
            }
            selected.valid = true;
            selected.topW = physicalTopW;
            selected.centerU = targetVoid.centerU;
            selected.centerV = targetVoid.centerV;
            if (baseCandidate.geometry.valid)
                selected.geometry = baseCandidate.geometry;
            selected.geometry.valid = true;
            selected.geometry.centerU = targetVoid.centerU;
            selected.geometry.centerV = targetVoid.centerV;
            selected.geometry.holeType = 1;
            selected.geometry.topRadius = targetVoid.radius;
            selected.geometry.bottomRadius = targetVoid.radius;
            selected.geometry.bottomValid = false;
            selected.geometry.depth = 0.0;
            result.voidRecoveryUsed = true;
            result.takeoverMode = "FinalGeometry_DIRECT_CIRCULAR_VOID_STRAIGHT";
        }
    }

    if (!result.voidRecoveryUsed && plane.valid
        && (baseRejected || baseCandidate.geometry.holeType == 1)) {
        constexpr std::array<double, 3> kOffsets{{-0.06, 0.0, 0.06}};
        for (double offset : kOffsets) {
            JiheHouXuan candidate;
            candidate.geometry = runRadiusCenterRefine(radiusCenterRefineSamples,
                input.topU, input.topV, physicalTopW + offset,
                effectiveSeedU, effectiveSeedV, input.initialTopRadius, input.canonicalMode);
            ++result.axialCandidateCount;
            if (!candidate.geometry.valid || !measuredCone(candidate.geometry)) continue;
            candidate.valid = true;
            candidate.topW = physicalTopW + offset;
            candidate.centerU = input.topU;
            candidate.centerV = input.topV;
            selected = candidate;
            result.axialAnchorUsed = true;
            result.axialCandidateOffset = offset;
            result.takeoverMode = "FinalGeometry_AXIAL_MEASURED_CONE";
            break;
        }
    }

    if (!result.voidRecoveryUsed
        && !targetVoid.valid
        && (baseRejected || selected.geometry.holeType == 1
            || selected.geometry.topRadius < 3.80)
        && input.initialTopRadius <= 4.80) {
        static thread_local std::vector<HoleJiheSurfacePouMian::Sample> surfaceBoundarySamples;
        surfaceBoundarySamples.clear();
        if (surfaceBoundarySamples.capacity() < samples.size()) surfaceBoundarySamples.reserve(samples.size());
        for (const Sample& sample : samples) {
            surfaceBoundarySamples.push_back({sample.u, sample.v, sample.w});
        }
        HoleJiheSurfacePouMian::Result bestCone;
        double bestConeTopW = physicalTopW;
        double bestConeScore = -std::numeric_limits<double>::infinity();

        const std::array<double, 4> requestedConeTopLevels{{
            input.topW, physicalTopW - 0.03, physicalTopW, physicalTopW + 0.03}};
        std::vector<double> coneTopLevels;
        coneTopLevels.reserve(requestedConeTopLevels.size());
        for (double level : requestedConeTopLevels) {
            const bool duplicate = std::any_of(coneTopLevels.begin(), coneTopLevels.end(),
                [level](double existing) { return std::abs(existing - level) <= 0.012; });
            if (!duplicate) coneTopLevels.push_back(level);
        }
        for (double coneTopW : coneTopLevels) {
            HoleJiheSurfacePouMian::Input coneInput;
            coneInput.topU = input.topU;
            coneInput.topV = input.topV;
            coneInput.topW = coneTopW;
            coneInput.initialTopRadius = input.initialTopRadius;
            coneInput.maxCenterShift = 5.50;
            coneInput.centerShiftPenalty = 0.20;
            const HoleJiheSurfacePouMian::Result cone =
                HoleJiheSurfacePouMian::evaluate(surfaceBoundarySamples, coneInput);
            const bool strongMeasuredCone = cone.valid && cone.holeType == 2
                && cone.bottomValid && cone.validLayers >= 3
                && cone.radiusSlope < -0.60
                && cone.radiusShrink >= 0.75
                && cone.bottomRadius > 0.20
                && cone.depth >= 0.80
                && cone.bottomRadius < 0.75 * cone.topRadius
                && cone.topRadius >= 3.75
                && cone.topRadius <= 5.50;
            if (!strongMeasuredCone) continue;
            const double score = 8.0 * static_cast<double>(cone.validLayers)
                + 5.0 * cone.radiusShrink
                + 2.0 * cone.monotonicRatio
                - 0.08 * cone.centerShift;
            if (score > bestConeScore) {
                bestConeScore = score;
                bestCone = cone;
                bestConeTopW = coneInput.topW;
            }
        }
        if (bestConeScore > -std::numeric_limits<double>::infinity()) {
            const auto& cone = bestCone;
            JiheHouXuan candidate;
            candidate.valid = true;
            candidate.topW = bestConeTopW;
            candidate.centerU = cone.centerU;
            candidate.centerV = cone.centerV;
            candidate.geometry.valid = cone.valid;
            candidate.geometry.surfaceValid = cone.surfaceValid;
            candidate.geometry.profileValid = cone.profileValid;
            candidate.geometry.bottomValid = cone.bottomValid;
            candidate.geometry.holeType = cone.holeType;
            candidate.geometry.inwardPolarity = cone.inwardPolarity;
            candidate.geometry.centerU = cone.centerU;
            candidate.geometry.centerV = cone.centerV;
            candidate.geometry.centerShift = cone.centerShift;
            candidate.geometry.surfaceRadius = cone.surfaceRadius;
            candidate.geometry.surfaceMad = cone.surfaceMad;
            candidate.geometry.surfaceSectors = cone.surfaceSectors;
            candidate.geometry.topRadius = cone.topRadius;
            candidate.geometry.bottomRadius = cone.bottomRadius;
            candidate.geometry.depth = cone.depth;
            candidate.geometry.radiusSlope = cone.radiusSlope;
            candidate.geometry.radiusShrink = cone.radiusShrink;
            candidate.geometry.monotonicRatio = cone.monotonicRatio;
            candidate.geometry.confidence = cone.confidence;
            candidate.geometry.validLayers = cone.validLayers;
            candidate.geometry.reason = cone.reason;
            candidate.geometry.layers.reserve(cone.layers.size());
            for (const auto& layer : cone.layers) {
                candidate.geometry.layers.push_back({layer.depth, layer.radius,
                    layer.coverage, layer.span, layer.points, layer.sectors});
            }
            selected = candidate;
            result.coneCenterRecoveryUsed = true;
            result.coneCenterShift = cone.centerShift;
            result.coneCenterRadius = cone.topRadius;
            result.coneCenterLayers = cone.validLayers;
            result.takeoverMode = "FinalGeometry_BOUNDED_STRONG_CONE_CENTER_RECOVERY";
        }
    }

    copyBaseToResult(result, selected, input);
    result.base = baseCandidate.geometry;
    if (!result.valid) {
        result.reason = "FinalGeometry_NO_VALID_GEOMETRY";
        return result;
    }

    result.axialAnchorUsed = result.axialAnchorUsed
        || std::abs(result.topW - input.topW) >= 0.015;
    if (plane.valid) result.axialCandidateOffset = result.topW - plane.w;

    const bool largeSupportedVisibleMouth = result.voidRecoveryUsed
        && result.voidRadius >= 5.70
        && result.voidAreaCells >= 2300
        && result.voidCircularity >= 0.62
        && result.takeoverMode == "FinalGeometry_LARGE_SUPPORTED_VISIBLE_MOUTH";
    if (result.holeType == 1 && largeSupportedVisibleMouth) {
        result.holeType = 2;
        result.shallowConeHuiFuUsed = true;
        result.shallowConeMode = "FinalGeometry_SUPPORTED_VISIBLE_MOUTH_PARTIAL_CONE";
        result.partialCone = true;
        result.bottomValid = false;
        result.bottomRadius = 0.0;
        result.depth = 0.0;
        result.topRadius = std::max(result.topRadius, result.voidRadius);
        result.confidence = std::max(result.confidence, 0.70);
        result.takeoverMode = result.shallowConeMode;
    }

    if (result.holeType == 1) {
        const bool weakCone = weakPersistentCone(result.base);
        const bool strongHuiFu = strongPersistentInnerWallCone(result.base);
        if (weakCone || strongHuiFu) {
            result.shallowConeHuiFuUsed = true;
            result.holeType = 2;
            result.shallowConeMode = strongHuiFu
                ? "FinalGeometry_STRONG_PersistentInnerWall_SHALLOW_CONE"
                : "FinalGeometry_WEAK_PERSISTENT_SHALLOW_CONE";
            if (strongHuiFu && result.base.huiFuBottomRadius > 0.20
                && result.base.huiFuDepth >= 0.25
                && result.base.huiFuBottomRadius < 0.98 * result.topRadius) {
                result.bottomValid = true;
                result.bottomRadius = result.base.huiFuBottomRadius;
                result.depth = result.base.huiFuDepth;
            } else if (weakCone && !result.base.layers.empty()) {
                const auto& last = result.base.layers.back();
                result.bottomRadius = last.radius;
                result.depth = last.depth;
                result.bottomValid = result.bottomRadius > 0.20
                    && result.bottomRadius < 0.98 * result.topRadius
                    && result.depth >= 0.40;
                if (!result.bottomValid) {
                    result.bottomRadius = 0.0;
                    result.depth = 0.0;
                    result.partialCone = true;
                }
            } else {
                result.bottomValid = false;
                result.bottomRadius = 0.0;
                result.depth = 0.0;
                result.partialCone = true;
            }
            result.confidence = std::max(result.confidence,
                result.partialCone ? 0.68 : 0.75);
            result.takeoverMode = result.shallowConeMode;
        }
    }

    if (result.holeType == 1) {
        result.bottomValid = false;
        result.bottomRadius = result.topRadius;
        result.depth = 0.0;
    } else if (!result.bottomValid) {
        result.partialCone = true;
        result.bottomRadius = 0.0;
        result.depth = 0.0;
    }

    result.centerShift = std::hypot(result.centerU - input.topU,
                                    result.centerV - input.topV);
    result.reason = result.takeoverMode.empty()
        ? std::string("FinalGeometry_RadiusCenterRefine_BASE:") + result.base.reason
        : result.takeoverMode + ":" + result.base.reason;
    return result;
}

}


