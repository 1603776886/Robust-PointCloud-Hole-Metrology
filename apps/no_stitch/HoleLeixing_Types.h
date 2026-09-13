/*
================================================================================
文件：HoleLeixing_Types.h
模块：Hole 公共数据契约

【主要职责】
集中定义 Hole 识别输入、结果、几何证据、类型和 seed 数据结构。

【主要调用关系】
GUI、识别、几何、分析、显示、导出共同包含。

【线程与状态】
仅数据结构，不主动执行算法。

【维护边界】
1. 本文件属于最终稳定结构：日常维护优先整理职责、命名、注释和无语义变化的性能细节，不随意改动已经验证的 Hole 数值判定。
2. Hole 识别阈值、候选排序、ROI、拟合公式、浮点表达式和拼接搜索参数若确需修改，必须单独做生产点云回归，不能夹在结构整理中一起改。
3. 自定义命名遵循“Hole + 拼音 + 基础英文”；Qt/PCL/VTK/Eigen 等第三方官方类型、函数和 API 保持官方名称。
4. 函数注释重点说明“作用、主要调用位置、输入输出/单位、维护风险”；禁止保留只针对历史版本、与当前实现不一致的临时注释。
================================================================================
*/
/*
模块职责：
集中定义孔识别在算法、界面、持久化和导出之间共享的稳定数据类型。
HoleMiaoshu 是单个孔的统一结果载体；HoleShibieResult 是一次识别的结果集合；ShouDongHoleSeed 是用户选孔输入。

主要调用位置：
HoleShibie_Recognition.cpp 负责填写这些字段；ZhuChuangKouWindow、显示模块、PCD 持久化和孔参数输出模块只消费结果。

维护说明：
这里集中保存孔识别结果和几何证据。修改字段时必须同步检查 GUI 显示、PCD 读写和导出接口，避免结果语义不一致。
真正会影响识别范围和拟合行为的参数集中在文件末尾的 ShouDongHoleSeed、CeliangOptions 等配置结构，旁边必须写清单位和调大/调小的影响。
*/
#pragma once
#include "DianYunLeixing_Types.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <QString>
#include <vector>
#include <array>
#include <string>
#include <memory>
// 数据类型
// ═══════════════════════════════════════════════════════

// HoleMiaoshu 是识别结果的统一载体；字段同时服务右侧面板、覆盖层和 PCD 嵌入保存。
/** 【类型导航注释】
 * PerZSeed：Hole 公共数据契约中的自定义 结构体。
 * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PerZSeed {
    float ransacCx = 0, ransacCy = 0; // 该Z层RANSAC圆心
    float voidCx = 0, voidCy = 0;     // 该Z层空区质心
    float z = 0;                       // Z高度
    float comp = 0;                    // 完备度
    int ringPts = 0;                   // RANSAC使用点数
};

struct HoleMiaoshu
{
    Eigen::Vector3f center{ 0, 0, 0 };
    float radius = 0.0f;
    float depth = 0.0f;
    float slopeDeg = 0.0f;
    int type = 0;                      // 0=未知 1=直孔 2=锥孔

    Eigen::Vector3f centerTop{ 0, 0, 0 };
    Eigen::Vector3f centerBot{ 0, 0, 0 };
    float rTop = 0.0f;
    float rBot = 0.0f;
    // 最终半径锁：主几何流程确定规范半径后，后续辅助测量可以继续测量其他半径，但不能覆盖正式显示/导出的权威半径。
    bool finalRadiusLocked = false;
    float finalRadiusLockedValue = 0.0f;
    std::string finalRadiusLockSource;

    // 最终孔型锁：规范局部区域稳定后，由正式孔型阶段负责直孔/锥孔二分类，同时保持已锁定的几何尺寸。
    bool finalTypeLocked = false;
    int finalTypeLockedValue = 0;
    float finalTypeLockedRBot = 0.0f;
    float finalTypeLockedDepth = 0.0f;
    float finalTypeLockedSlopeDeg = 0.0f;
    Eigen::Vector3f finalTypeLockedCenterBot{ 0, 0, 0 };
    std::string finalTypeLockSource;

    // 稀疏锥孔恢复证据：只允许修正特定“锥孔证据不足但可恢复”的孔型语义，不改已经正确的直孔和平台一致路径。
    bool coneHuiFuReviewAttempted = false;
    bool coneHuiFuReviewHuiFuChengGong = false;
    bool coneHuiFuReviewBottomEvidenceStrong = false;
    bool coneHuiFuReviewNormalizedSupportStrong = false;
    bool coneHuiFuReviewMultiShiftConeConsistent = false;
    int coneHuiFuReviewValidProfileCount = 0;
    int coneHuiFuReviewConeProfileCount = 0;
    int coneHuiFuReviewExtraConeProfileCount = 0;
    int coneHuiFuReviewPlatformProfileCount = 0;
    float coneHuiFuReviewBasePointDensityRatio = 0.0f;
    float coneHuiFuReviewAxialPointDensityRatio = 0.0f;
    float coneHuiFuReviewSharedSlope = 0.0f;
    float coneHuiFuReviewSharedRmse = 0.0f;
    float coneHuiFuReviewSlopeSpread = 0.0f;
    float coneHuiFuReviewBottomShrink = 0.0f;
    float coneHuiFuReviewBottomRadiusRatio = 0.0f;
    std::string coneHuiFuReviewMode;
    std::string coneHuiFuReviewReason;

    int supportPts = 0;
    float completeness = 0.0f;
    float circularity = 0.0f;              // 圆形度 0-1（ROI阶段计算）
    bool localVerified = false;           // 局部证据已验证，可用于提高最终判定置信度
    float detectionPlaneZ = 0.0f;         // 粗识别检测平面Z，0=未设置/未知，精识别VLD搜索用
    // 深度证据字段：记录局部孔壁在轴向上的连续性和可测深度。
    float depthMeasured = 0.0f;           // 由有效层真实计算得到的深度（未钳制）
    float depthFinal = 0.0f;              // 最终用于显示的深度（可能被钳制/回退）
    int depthSource = 0;
    bool depthClamped = false;            // depthFinal是否被最小值或截断钳制
    int validLayerCount = 0;              // 有效追踪层数（满足内空+外实+连续条件）

    // 孔型一致性字段：记录直孔/锥孔分类所需的几何一致性证据。
    // detectPath: 候选来源路径；geometryType: 几何孔型分类
    /** 【类型导航注释】
     * DetectPath：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class DetectPath { Unknown=0, RoiTrace=1, CuDingWeiSurface=2, BossInner=3, PlaneCuScan=4 };
    /** 【类型导航注释】
     * JiheType：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class JiheType { Unknown=0, Straight=1, Cone=2, Slope=3, SurfaceSeedUnconfirmed=4 };
    /** 【类型导航注释】
     * HoleKouModel：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class HoleKouModel { Unknown=0, PlanarVoidMouth=1, RingBandMouth=2, BossRingMouth=3, SlopeMouth=4 };
    // 高分FP环带强度分类（RingBandMouth 环带支撑强度验证）
    /** 【类型导航注释】
     * RingQiangDuCategory：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class RingQiangDuCategory { Unknown=0, StrongChordStrongRing=1, StrongChordWeakRing=2, SparseRingArtifact=3, PossibleRealRingHole=4 };
    // 扫描线证据等级（ScanlineZhengJuLevel 分级，仅标签不删除）
    /** 【类型导航注释】
     * ScanlineZhengJuLevel：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class ScanlineZhengJuLevel { Unknown=0, Strong=1, Medium=2, Weak=3, Invalid=4 };
    DetectPath detectPath = DetectPath::Unknown;
    JiheType geometryType = JiheType::Unknown;
    bool coneEvidence = false;            // 浅追踪锥孔证据标记

    // 轴线/半径一致性字段：当前手动识别只填充其中一部分，保留给显示和后续调参。
    float axisRms = 0.0f;                 // 轴线RMS偏差（mm），各层圆心到拟合3D直线的垂直距离RMS
    float axisMaxDev = 0.0f;             // 轴线最大偏差（mm）

    // 手动种子与孔口网格证据：供孔口投票和候选筛选使用。
    std::string manualPoseTag;
    float manualNormalDeltaDeg = 0.0f;
    bool mouthGridExecuted = false;
    Eigen::Vector3f mouthGridCenter{ 0, 0, 0 };

    // 候选语义标签：记录目标孔筛选和一致性判断结果。
    /** 【类型导航注释】
     * TargetYiZhiXing：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class TargetYiZhiXing { Unknown=0, TargetLikely=1, HoleLikeUncertain=2, NonTargetLikely=3 };

    // 表面上下文指标：用于排除凸起、边界和其它孔状非目标结构。
    /** 【类型导航注释】
     * SurfaceContextType：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class SurfaceContextType { Unknown=0, InnerSurface=1, NearBoundary=2,
        StepEdge=3, SlotLike=4, BossEdge=5, DepressionLike=6 };
    float localPlaneNx = 0.0f;             // 局部平面法向量X（内部几何量）
    float localPlaneNy = 0.0f;             // 局部平面法向量Y（内部几何量）
    float localPlaneNz = 1.0f;             // 局部平面法向量Z（默认水平面朝上）
    int inwardPolarity = 0;                 // +1沿localPlaneN入孔，-1反向；禁止用Z正负猜测
    float localPlaneTiltDeg = 0.0f;        // 基础局部坐标系中的表面法向倾角。

    // 最终孔口、孔壁与入孔轴几何证据。所有孔统一执行“孔外支撑面 + 多厚度孔壁截面 + 圆心轨迹”流程，
    // 不按孔号、尺寸、倒角、圆角或倾角建立专用分支。该阶段只修正中心与轴，
    // rTop、type、depth 和 rBot 保持上游已经确定的语义。
    bool refinedSurfaceNormalValid = false;
    float refinedSurfaceNx = 0.0f;
    float refinedSurfaceNy = 0.0f;
    float refinedSurfaceNz = 1.0f;
    float refinedSurfaceTiltDeg = 0.0f;
    int refinedSurfaceSupportPts = 0;
    int refinedSurfaceCoveredSectors = 0;
    float refinedSurfaceCoverage = 0.0f;
    float refinedSurfaceFitRmse = 0.0f;
    std::string refinedSurfaceSource;

    // 均衡外表面法向字段只记录候选评估，不覆盖已经确定的正式法向。
    bool balancedNormalBalancedNormalAttempted = false;
    bool balancedNormalBalancedNormalValid = false;
    float balancedNormalBalancedNormalCoverage = 0.0f;
    float balancedNormalBalancedNormalMaxGapDeg = 360.0f;
    float balancedNormalBalancedNormalResidualMad = 0.0f;
    float balancedNormalBalancedNormalDeltaDeg = 0.0f;
    float balancedNormalBalancedNormalNx = 0.0f;
    float balancedNormalBalancedNormalNy = 0.0f;
    float balancedNormalBalancedNormalNz = 1.0f;
    std::string balancedNormalBalancedNormalDecision;
    float balancedNormalWallCorrectionLimitDeg = 0.0f;
    float balancedNormalWallCorrectionAppliedDeg = 0.0f;

    bool holeAxisInValid = false;
    float holeAxisInNx = 0.0f;
    float holeAxisInNy = 0.0f;
    float holeAxisInNz = -1.0f;
    float holeAxisTiltDeg = 0.0f;
    std::string holeAxisSource;
    int wallAxisValidSlices = 0;
    int wallAxisIterations = 0;
    float wallAxisCenterRmse = 0.0f;
    float wallAxisMeanSliceRmse = 0.0f;
    float wallAxisMeanCoverage = 0.0f;
    float wallAxisRadiusSlope = 0.0f;
    float wallAxisRadiusMonotonicity = 0.0f;
    float wallAxisCorrectionDeg = 0.0f;
    float wallAxisConfidence = 0.0f;

    // 锁定半径后的孔口姿态证据。jointWallCandidatePoints/ValidSlices 记录多厚度孔壁截面，
    // jointCenterSpread 记录圆心轨迹 RMSE；jointRadiusShift 必须保持为 0，避免姿态阶段改写规范半径。
    bool jointGeometryExecuted = false;
    bool jointGeometryAxisValid = false;
    bool jointGeometryMouthValid = false;
    int jointWallCandidatePoints = 0;
    int jointWallNormalPoints = 0;
    int jointWallSectors = 0;
    int jointValidSlices = 0;
    float jointWallCoverage = 0.0f;
    float jointWallSymmetry = 0.0f;
    float jointGlobalPriorWeight = 0.0f;
    float jointCenterEvidenceWeight = 0.0f;
    float jointAxisCorrectionDeg = 0.0f;
    float jointMeanSliceRmse = 0.0f;
    float jointMeanSliceCoverage = 0.0f;
    float jointCenterSpread = 0.0f;
    float jointAllCenterSpread = 0.0f;
    float jointAxisInlierThreshold = 0.0f;
    int jointAxisUsedSlices = 0;
    int jointAxisRejectedSlices = 0;
    int jointTransitionSlices = 0;
    int jointStableStartSlice = 0;
    float jointCenterShift = 0.0f;
    float jointRadiusShift = 0.0f;
    float jointWallRadiusAtZero = 0.0f;
    float jointLockedMouthDepth = 0.0f;
    bool jointSemanticLockPreserved = false;
    float jointRadiusSlope = 0.0f;
    float jointConfidence = 0.0f;

    std::string jointGeometrySource;

    // 真实壁面几何证据。规范 rTop 保持锁定；measuredInnerRadius 是独立的物理量，
    // 用于表示直孔最小内壁及其轮廓，不能伪装成规范上口半径。
    bool coarseCenterExecuted = false;
    bool coarseCenterValid = false;
    float coarseCenterShift = 0.0f;
    float coarseBoundaryRadius = 0.0f;
    float coarseBoundaryMad = 0.0f;
    float coarseBoundaryCoverage = 0.0f;
    float coarseInnerDensity = 0.0f;
    float coarseAnnulusDensity = 0.0f;
    int coarseSurfacePoints = 0;
    int coarseEvaluatedCandidates = 0;
    long long coarseVisitedSurfacePoints = 0;
    int coarseParallelWorkers = 1;
    bool measuredInnerRadiusValid = false;
    float measuredInnerRadius = 0.0f;
    float measuredInnerRadiusMad = 0.0f;
    float measuredInnerDepthStart = 0.0f;
    float measuredInnerDepthEnd = 0.0f;
    float measuredInnerDepthSpan = 0.0f;
    float measuredInnerCoverage = 0.0f;
    float measuredInnerRmse = 0.0f;
    float measuredInnerCenterLineRmse = 0.0f;
    int measuredInnerCandidates = 0;
    int measuredInnerFamilies = 0;
    int measuredInnerUsedSlices = 0;
    int measuredInnerHistogramWindows = 0;
    long long measuredInnerHistogramPointVisits = 0;
    long long measuredInnerFitPointVisits = 0;
    std::string measuredInnerDecision;

    // 独立孔径可观测性字段，只追加证据，不接管规范上口半径、孔型、姿态、下口或深度。
    // 状态值：0=不可观测，1=部分可观测，2=已观测；valid=false 绝不代表“测得半径为零”。
    bool observedBoreRadiusExecuted = false;
    bool observedBoreRadiusValid = false;
    int observedBoreRadiusState = 0;
    float observedBoreRadius = 0.0f;
    float observedBoreRadiusMad = 0.0f;
    float observedBoreDepthStart = 0.0f;
    float observedBoreDepthEnd = 0.0f;
    float observedBoreDepthSpan = 0.0f;
    float observedBoreCoverage = 0.0f;
    float observedBoreRmse = 0.0f;
    float observedBoreCenterLineRmse = 0.0f;
    float observedBoreRadiusSlope = 0.0f;
    float observedBoreAxisCorrectionDeg = 0.0f;
    float observedBoreCenterShift = 0.0f;
    int observedBoreUsedSlices = 0;
    int observedBoreCandidateCount = 0;
    int observedBoreFamilyCount = 0;
    int observedBoreDepthRegions = 0;
    int observedBoreHistogramWindows = 0;
    long long observedBoreHistogramPointVisits = 0;
    long long observedBoreFitPointVisits = 0;
    std::string observedBoreSource = "ObservedBore_NOT_EXECUTED";
    std::string observedBoreDecision = "NOT_EXECUTED";

    bool rawConeWallExecuted = false;
    bool rawConeWallValid = false;
    int rawConeWallCells = 0;
    int rawConeWallSectors = 0;
    int rawConeWallDepthBins = 0;
    int rawConeWallRoiPoints = 0;
    int rawConeWallAxisEvaluations = 0;
    int rawConeWallParallelWorkers = 1;
    float rawConeWallRmse = 0.0f;
    float rawConeWallScale = 0.0f;
    float rawConeWallNoDriftRmse = 0.0f;
    float rawConeWallBaselineEvenRmse = 0.0f;
    float rawConeWallBaselineOddRmse = 0.0f;
    float rawConeWallEvenRmse = 0.0f;
    float rawConeWallOddRmse = 0.0f;
    float rawConeWallEvenImprovement = 0.0f;
    float rawConeWallOddImprovement = 0.0f;
    float rawConeWallImprovement = 0.0f;
    float coneFamilyConeFamilyAngleDeg = 0.0f;
    float rawConeWallAxisCorrectionDeg = 0.0f;
    float rawConeWallCenterShift = 0.0f;
    std::string rawConeWallDecision;

    // 物理孔口交线。rTop 是最终规范截面半径；真实支撑面与孔壁的三维交线在倾斜或圆角入口处
    // 可能不是圆。该交线仅作为内部物理证据，不替代 GUI 主示意圆；
    // GUI 主示意始终按 rTop 绘制，也不在这里回写 type、rTop、rBot 或 depth。
    bool apertureContourExecuted = false;
    bool apertureContourValid = false;
    int apertureContourPoints = 0;
    float apertureContourCoverage = 0.0f;
    float apertureContourDepthMin = 0.0f;
    float apertureContourDepthMax = 0.0f;
    float apertureContourDepthMedian = 0.0f;
    float apertureContourDepthSpan = 0.0f;
    float apertureContourRadiusMin = 0.0f;
    float apertureContourRadiusMax = 0.0f;
    float aperturePlanarRingPlaneResidualMax = 0.0f;
    float apertureContourPlaneResidualMax = 0.0f;
    float apertureContourConfidence = 0.0f;
    float apertureContourCanonicalRadius = 0.0f;
    float apertureContourEvidenceRadius = 0.0f;
    float apertureContourEvidenceScale = 0.0f;
    float apertureContourEvidenceCenterShift = 0.0f;
    int apertureContourEvidenceRawSectors = 0;
    int apertureContourEvidenceUsedSectors = 0;
    int apertureContourEvidenceSupportCount = 0;
    std::array<Eigen::Vector3f, 72> apertureContourWorld{};
    std::array<unsigned char, 72> apertureContourMask{};


    // 全局工艺孔一致性指标：描述孔径族、布局和区域一致性。

    // 全局一致性软标签：只记录一致性，不直接删除候选。
    /** 【类型导航注释】
     * GlobalYiZhiType：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class GlobalYiZhiType { Unknown=0, IsolatedTargetLike=1,
        ClusteredTargetLike=2, NonTargetClusterLike=3,
        OffMainSurfaceLike=4, LayoutUncertain=5 };

    // 扫描线弦缺口验证：利用线激光横截面缺口确认孔口存在性。
    float mouthZ0 = 0.0f;                 // 初始孔口Z层（来自detectionPlaneZ或centerTop.z）
    float mouthZ = 0.0f;                  // 验证用孔口Z层
    float bestMouthZ = 0.0f;              // 自适应搜索后的最优孔口Z
    // 最优层指标
    // 当前使用层指标（单层调用结果，mouthZ=bestMouthZ时与best一致）
    // 孔口环带同高性（rim height coherence）
    // ── 孔口模型证据：分别描述平面空洞与环带支撑 ──
    // ── 环带支撑强度（RingBandMouth 环带支撑强度验证）──
    // 扇区覆盖
    // 支撑线统计
    // 归一化指标
    // 综合评分

    // ── 扫描线证据等级（ScanlineZhengJuLevel 分级）──
    const char* scanlineEvidenceReason = nullptr; // 分级理由（字符串字面量，无需释放）

    // ── 显示层字段（Policy F 策略，仅影响显示不改变检测结果）──
    std::string displayStatus = "Visible";          // 显示状态: Visible/HiddenByWeakScanline/DowngradedByWeakScanline/DowngradedByWeakRing
    std::string displayDecisionReason;              // 显示决策原因
    // ── GUI 显示策略（Plan A：BossInner 零REF源默认降级）──
    std::string guiDisplayStatus = "Visible";       // GUI实际使用的显示状态（不修改原始displayStatus）

    // ── 参数可靠性评估：记录中心、半径、深度和孔型的可信程度 ──
    std::string parameterReliability;
    std::string parameterReliabilityReason;         // 聚合原因（如：半径低置信+深度层数不足）
    std::string centerReliability;
    std::string radiusReliability;
    std::string depthReliability;
    std::string slopeReliability;
    std::string typeReliability;

    // ── 算法最终判定层（独立于 displayStatus/confirmedStatus/guiDisplayStatus）──
    /** 【类型导航注释】
     * FinalHolePanDing：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp、DianYunXianshi_View.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class FinalHolePanDing { Unknown=0, TrueHole=1, FalseHole=2, NeedReview=3 };
    /** 【类型导航注释】
     * FinalJiheType：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp、DianYunXianshi_View.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class FinalJiheType { Unknown=0, Straight=1, Cone=2, ConeWeakHint=3 };
    FinalHolePanDing finalHoleJudgment = FinalHolePanDing::Unknown;
    FinalJiheType finalGeometryType = FinalJiheType::Unknown;
    std::string finalTypeReason;         // 分类依据


    // ── 高可信输出层字段（高可信显示层）──
    /** 【类型导航注释】
     * HighConfidenceStatus：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class HighConfidenceStatus { HighConfidenceHole=0, CandidateHole=1, HiddenWeakCandidate=2 };
    HighConfidenceStatus highConfidenceStatus = HighConfidenceStatus::HiddenWeakCandidate;
    float highConfidenceScore = 0.0f;


    // 候选来源信息。
    std::string sourcePath;                                // 来源路径描述
    float sourceScore = 0.0f;                               // 来源评分
    std::string sourceReason;                               // 来源补充原因

    std::string seedSourcePath;             // 粗候选种子来源路径标签
    std::string rejectReason;               // 最终拒绝原因（后过滤/去重等）

    // 局部区域精识别状态。
    // radiusProfile 汇总
    int radiusProfileN = 0;                  // 有效半径测量层数
    float radiusProfileR2 = 0.0f;            // 线性回归 R²
    bool reliableBottomRadius = false;       // rBot 可靠性
    float rBotProfile = 0.0f;               // radiusProfile 推算的底层半径 (mm)
    float profileBasedDr = 0.0f;
    float profileBasedSlope = 0.0f;
    // 旁路孔型分类（不改正式 geometryType）

    // MicroZVoidSeed 微孔void指标（从CircleCandidate穿透）


    // 局部区域空洞度旁路证据。
    bool roiVoidnessDataValid = false; float roiCoreVoidRatio = 0.0f; float roiCoreFillRatio = 0.0f;
    float roiWallToCoreDensityRatio = 0.0f; float roiWallAnnulusDensity = 0.0f; float roiCoreDensity = 0.0f;
    std::string roiVoidnessRiskCategory; std::string roiVoidnessMissingReason;
    bool roiVoidnessEnabledDemotion = false; std::string roiVoidnessDemotionReason;

    // 孔壁向下追踪与组合分类证据。
    bool wallDownTrackValid = false; int wallDownTrackLayerCount = 0; float wallDownTrackDepthMm = 0.0f;
    float wallDownContinuityScore = 0.0f; float wallDownSupportScore = 0.0f;
    float signedRadiusTrend = 0.0f; float radiusShrinkScore = 0.0f; float radiusExpandScore = 0.0f;
    float coneWallDownEvidenceScore = 0.0f; float convexBumpReverseEvidenceScore = 0.0f;
    std::string wallDownCombinedClass = "UNKNOWN"; std::string wallDownCombinedReason;

    // 物理孔型显示层：只负责给生产界面一个稳定的孔形标签。
    /** 【类型导航注释】
     * WuLiHoleLeixingXianshi：Hole 公共数据契约中的自定义 枚举。
     * 主要使用位置：HoleShibie_Recognition.cpp、ZhuChuangKou_Window.cpp、DianYunXianshi_View.cpp、HoleCanshuShuchu_Export.cpp。
     * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
     */
    enum class WuLiHoleLeixingXianshi { Unknown=0, Cone=1, Straight=2,
        Straight_DeformedWall=3, ConeLike_LowConfidence=4, Artifact=5 };
    float coneWallCompletenessScore = 0.0f;
    WuLiHoleLeixingXianshi physicalHoleTypeDisplay = WuLiHoleLeixingXianshi::Unknown;
    std::string physicalHoleTypeReason;
};

struct HoleShibieResult
{
    using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using CloudPtr = Cloud::Ptr;

    int holes = 0;
    std::vector<HoleMiaoshu> descriptors;

    CloudPtr dianYunView;
    std::vector<CloudPtr> holeVis;
    QString error;
    /** 【函数导航】
     * 作用：执行“ok”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
     * 所属模块：Hole 公共数据契约。
     * 主要引用/调用位置：ZhuChuangKou_Window.cpp。
     * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
     */
    bool ok() const { return error.isEmpty(); }
};
/*
手动选孔种子参数。ZhuChuangKouWindow 在用户点击后构造本结构，再交给正式局部识别入口。
这些数值会直接改变“每个点击点周围看多大范围”和“允许识别多大的孔”，属于生产敏感参数。
*/
struct ShouDongHoleSeed
{
    Eigen::Vector3f point{0, 0, 0};

    // 局部搜索半径，单位毫米。默认 24.0。GUI 会根据“最大孔半径”联动调整。
    // 增大：更不容易漏掉偏离点击点的孔口，但候选点增多、速度下降，也更容易把邻近结构带进来。
    // 减小：速度更快、局部性更强，但用户点偏时更容易裁掉孔口。
    float searchRadiusMm = 25.0f;  // 默认 maxRadiusMm=10 mm 时按 2*Rmax+5 得到 25 mm 搜索半径。

    // 支撑面法向估计半径，单位毫米。默认 18.0。
    // 增大：平面拟合更稳定，但附近台阶、弯曲面或其他结构更容易干扰法向；减小则对噪声更敏感。
    float normalRadiusMm = 18.0f;

    // 允许搜索的孔口最小半径，单位毫米。降低会接纳更小的空洞，也会扩大噪声/伪孔候选范围。
    float minRadiusMm = 1.5f;

    // 允许搜索的孔口最大半径，单位毫米。默认 10.0；GUI 可调。
    // 提高会联动放大局部 ROI 与候选半径范围，因此计算量也会增加。
    float maxRadiusMm = 10.0f;
};
// ═══════════════════════════════════════════════════════
// 孔结果精修和显示辅助接口
// ═══════════════════════════════════════════════════════

/*
孔结果精修参数。当前默认值已经参与既有回归，修改任何一项都可能改变最终几何结果。
*/
/** 【类型导航注释】
 * CeliangOptions：Hole 公共数据契约中的自定义 结构体。
 * 主要使用位置：HoleLeixing_Types.h（本模块内部）。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct CeliangOptions {
    float zSouSuoBanKuan = 2.0f;   // Z 向搜索半宽，单位毫米；增大覆盖更多层，但更容易混入上下表面。
    float zhongXinSouSuo = 3.0f;   // 圆心允许的局部搜索范围，单位毫米；增大可修正更大偏移，同时扩大误匹配空间。
    float cuBuChang = 0.5f;        // 粗搜索步长，单位毫米；减小更细但更慢。
    float xiBuChang = 0.1f;        // 精搜索步长，单位毫米；减小提高搜索分辨率但增加计算量。
    int jiaoDuTong = 72;           // 角度桶数量；增大可描述更细的圆周覆盖，但统计更稀疏、计算量增加。
    float jingXiangTong = 0.10f;   // 径向桶宽/量化尺度；减小提高半径分辨率，同时更容易受噪声影响。
    float zuiDaXiuZheng = 3.0f;    // 单次允许的最大中心修正，单位毫米；增大可能追到邻近结构。
    float fuGaiDuZuiDi = 0.12f;    // 最低圆周覆盖率，范围 0~1；降低会接纳更残缺的孔口证据。
};

// ============================================================================
// 孔轴方向/极性辅助：统一上下口方向和深度正负。
// ============================================================================
/*
模块职责：
孔方向与极性辅助。

主要调用位置：
由孔识别几何链调用，用于统一孔轴正负方向及相关判定。

维护说明：
方向符号会影响深度和上下口定义，任何调整都必须做几何等价性验证。
*/
namespace HoleJiXing {

/** 【函数导航】
 * 作用：执行“normalize”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 公共数据契约。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp、ZhuChuangKou_Window.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline constexpr int normalize(int polarity) noexcept
{
    return polarity == 1 ? 1 : (polarity == -1 ? -1 : 0);
}

/** 【函数导航】
 * 作用：执行“signOrZero”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 公共数据契约。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ZhuChuangKou_Window.cpp、DianYunXianshi_View.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline constexpr float signOrZero(int polarity) noexcept
{
    return static_cast<float>(normalize(polarity));
}

/** 【函数导航】
 * 作用：执行“known”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 公共数据契约。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp、ZhuChuangKou_Window.cpp、DianYunXianshi_View.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline constexpr bool known(int polarity) noexcept
{
    return normalize(polarity) != 0;
}

}

// ============================================================================
// 识别结果来源标识：用于说明结果来自 GUI 几何识别主链。
// ============================================================================
#include <string_view>

namespace HoleShibieSource {

inline constexpr std::string_view kId = "GUI_GEOMETRY_HOLE_RECOGNITION";

} // namespace HoleShibieSource
