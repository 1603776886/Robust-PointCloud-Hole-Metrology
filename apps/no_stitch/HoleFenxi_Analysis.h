/*
================================================================================
文件：HoleFenxi_Analysis.h
模块：Hole 分析接口

【主要职责】
声明候选聚类、Hole 类型共识、深度估计、二次判定、缓存与安全策略。

【主要调用关系】
HoleShibie_Recognition 与部分手动 Hole 支撑流程调用。

【线程与状态】
纯计算/线程局部缓存。

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
集中保存孔候选聚类、孔形分类和深度判定相关的数据结构与算法接口。
这些步骤都属于“已有几何证据之后的孔结果分析”，因此按数据流集中在同一分析模块中。

主要调用位置：
HoleShibie_Recognition 生成几何证据后调用；本文件后半部分的策略区直接复用候选与分类结构。

维护说明：
聚类阈值、孔形分类阈值和深度判定边界属于识别语义。注释或结构整理可以改，数值边界不能在普通维护中顺手修改。
*/

// ============================================================================
// 功能分区：孔候选聚类
// ============================================================================
/*
模块职责：
孔候选聚类子模块。

主要调用位置：
由 HoleShibie_Recognition.cpp 的正式识别链调用，把局部几何证据聚合为稳定候选。

维护说明：
聚类距离、支持点数等阈值会改变候选集合；调整前先写明单位，再做全量等价性验证。
*/

#include <string>
#include <vector>

namespace HoleHouXuanJuLei {

/** 【类型导航注释】
 * Sample：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_JiheJianCe.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Sample {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    int originalIndex = -1;
};

/** 【类型导航注释】
 * HouXuan：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HouXuan {
    bool valid = false;
    int layerIndex = 0;
    double layerW = 0.0;
    double centerU = 0.0;
    double centerV = 0.0;
    double radius = 0.0;
    double circularity = 0.0;
    double coverage = 0.0;
    double residual = 0.0;
    double voidArea = 0.0;
    double wallSupport = 0.0;
    int areaCells = 0;
    int pointCount = 0;
    int sectorCount = 0;
    bool touchesGrid = false;
    double score = 0.0;

    // 开放/残缺孔口不再要求空洞闭合。以下指标只描述扫描可见性，
    // 最终机械孔型仍由后续直孔/锥孔正式审核决定。
    bool openArc = false;
    double interiorCleanRatio = 0.0;       // 孔内核心没有更近点的扇区比例。
    double firstEdgeRatio = 0.0;           // 第一边界一致性证据；候选生成已按第一边界定义，不再重复作硬门。
    double continuousArcCoverage = 0.0;    // 最大连续可见圆弧占整圆比例。
    double circleFitRmse = 0.0;            // 第一边缘径向圆拟合均方根误差，单位 mm。
    bool ellipseValid = false;              // 同一批第一边缘是否形成可直接反推法向的中心约束椭圆。
    double ellipseAxisRatio = 1.0;          // 短轴/长轴；斜切圆柱模型下等于 cos(孔轴与粗法向夹角)。
    double ellipseTiltU = 0.0;              // 椭圆长轴在当前局部 u/v 平面中的无符号方向。
    double ellipseTiltV = 0.0;
    double ellipseModelRmse = 0.0;          // 椭圆径向模型残差，单位 mm。
    int ellipseSectorCount = 0;
    std::string source = "CLOSED_VOID_COMPONENT";
};

/** 【类型导航注释】
 * Cluster：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Cluster {
    int id = -1;
    bool stable = false;
    std::vector<int> candidateIndices;
    int candidateCount = 0;
    int supportLayers = 0;
    double consensusCenterU = 0.0;
    double consensusCenterV = 0.0;
    double consensusTopW = 0.0;
    double consensusTopRadius = 0.0;
    double centerStd = 0.0;
    double radiusStd = 0.0;
    double meanCoverage = 0.0;
    double meanResidual = 0.0;
    double trajectoryContinuity = 0.0;
    double radiusTrendSlope = 0.0;
    double radiusTrendRmse = 0.0;
    double radiusTrendMonotonicity = 0.0;
    double radiusTrendSpan = 0.0;
    int radiusTrendLayers = 0;
    bool taperTrendStable = false;
    int openArcCandidateCount = 0;
    bool openArcStable = false;
    double meanInteriorCleanRatio = 0.0;
    double meanFirstEdgeRatio = 0.0;
    double meanContinuousArcCoverage = 0.0;
    bool openArcEllipseValid = false;
    int openArcEllipseSupportCount = 0;
    double openArcEllipseAxisRatio = 1.0;
    double openArcEllipseAxisRatioStd = 0.0;
    double openArcEllipseDirectionCoherence = 0.0;
    double openArcEllipseTiltU = 0.0;
    double openArcEllipseTiltV = 0.0;
    double openArcEllipseModelRmse = 0.0;
    double openArcCenterDriftU = 0.0;        // 随局部 w 增加时孔心在 u/v 中的漂移斜率，用于消除椭圆方向正负二义性。
    double openArcCenterDriftV = 0.0;
    double openArcCenterDriftMagnitude = 0.0;
    double openArcCenterDriftAlignment = 0.0; // 与椭圆长轴的无符号一致度，1 表示完全一致。
    std::string stabilityMode;
    double score = 0.0;
    double seedBoundaryDistance = 0.0;
    double seedAxialDistance = 0.0;
    double selectionCost = 0.0;
    std::string role = "UNRESOLVED";
    double mouthPlaneDistance = 0.0;
    bool mouthAttachmentValid = false;
    bool deepContinuation = false;
    int parentMouthClusterId = -1;
    std::string roleReason;
};

// 已经通过孔壁/表面剖面确认真实上口以后，重新从该真实上口薄层提取椭圆证据。
// 这一步和开放候选使用同一个“中心约束椭圆”数学模型；区别只是中心/半径来自正式孔壁几何，
// 因而不会把凸台外缘或圆台外缘的椭圆拿去反推孔轴。
/** 【类型导航注释】
 * HoleKouEllipseEvidence：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct HoleKouEllipseEvidence {
    bool valid = false;
    int sectorCount = 0;
    double coverage = 0.0;
    double axisRatio = 1.0;
    double tiltU = 0.0;
    double tiltV = 0.0;
    double modelRmse = 0.0;
};

HoleKouEllipseEvidence fitResolvedMouthEllipse(
    const std::vector<Sample>& samples,
    double centerU, double centerV, double topW, double topRadius);

/** 【类型导航注释】
 * Input：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    double seedU = 0.0;
    double seedV = 0.0;
    double seedW = 0.0;
    // true 表示 seedW 已由孔外局部强支撑面投影得到，只承担“机械外表面轴向锚点”职责。
    // 此时候选支撑面不能落到 seedW 更深处，避免点击锥壁/孔内时把底部半圆当成上口。
    bool seedWIsOuterSurfaceAnchor = false;
    double radialHalfExtent = 24.0;
    double axialHalfExtent = 15.0;
    // GUI 主参数“最大孔半径”，单位 mm。默认 10 mm。
    // 候选半径上限、开放圆弧中心搜索跨度等几何尺度从这里联动；
    // 点密度/残差阈值不随孔径同比放大，避免大孔模式降低质量门槛。
    double maxHoleRadius = 10.0;
    double cellSize = 0.25;
    double layerStep = 0.25;
    double layerBand = 0.40;
    int sectors = 36;
    // true：允许“第一有效边缘”的部分圆弧证据和闭合圆证据进入同一候选系统。
    // 若当前层已经存在明确、贴附真实支撑面的完整圆，可跳过冗余开放弧计算；这只是速度优化，
    // 不改变后续真实孔口、法向、直孔/锥孔和深度的统一算法。
    bool enableOpenArc = false;
};

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    int selectedClusterId = -1;
    int selectedClusterIndex = -1;
    int candidateCount = 0;
    int clusterCount = 0;
    int stableClusterCount = 0;
    double clusterMargin = 0.0;
    double supportPlaneW = 0.0;
    double supportPlaneHistogramW = 0.0;
    double supportPlaneOuterClusterW = 0.0;
    std::string supportPlaneMode;
    int mouthAttachedClusterCount = 0;
    bool rawSeedCandidateGateUsed = false;
    bool rawSeedCandidateScoreUsed = false;
    bool rawSeedClusterGeometryUsed = false;
    bool rawSeedClusterSelectUsed = false;
    bool openArcSearchUsed = false;
    int openArcCandidateCount = 0;
    std::vector<HouXuan> candidates;
    std::vector<Cluster> clusters;
    std::string selectMode;
    std::string exitCode;
    std::string reason;
};

Result evaluate(const std::vector<Sample>& samples, const Input& input);


}

// ============================================================================
// 功能分区：孔形与深度分析
// ============================================================================
/*
模块职责：
孔形分类、锥度轮廓判断和最终孔形裁决。

维护说明：
本文件按职责合并相互紧密的子模块。维护时请按中文功能分区定位逻辑；同一职责优先在现有分区内扩展，避免把连续算法拆成过细文件。
*/

// ============================================================================
// 功能分区：孔形基础截面分类
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/
#include "HoleJihe_Geometry.h"


namespace HoleLeixingPouMianBase {

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    int inputType = 0;
    int holeType = 0;
    bool typeChanged = false;
    bool profileValid = false;
    bool sustainedCone = false;
    bool straightByProfile = false;
    bool robustBaseCone = false;
    bool robustRecoveredWall = false;
    int polarity = 0;
    int layers = 0;
    int distributedDrops = 0;
    double depthSpan = 0.0;
    double firstRadius = 0.0;
    double lastRadius = 0.0;
    double slope = 0.0;
    double shrink = 0.0;
    double shrinkRatio = 0.0;
    double monotonicRatio = 0.0;
    double maxStepDrop = 0.0;
    double requiredShrink = 0.0;

    bool chamferPlatformStraight = false;
    bool segmentedModelValid = false;
    int breakLayer = -1;
    int platformLayers = 0;
    double platformDepthSpan = 0.0;
    double coneWeightedRmse = 0.0;
    double coneBic = 0.0;
    double segmentedWeightedRmse = 0.0;
    double segmentedBic = 0.0;
    double bicImprovement = 0.0;
    double chamferSlope = 0.0;
    double plateauRadius = 0.0;
    double plateauSlope = 0.0;
    double plateauStd = 0.0;
    double plateauCoverage = 0.0;
    double platformEntranceDrop = 0.0;
    double platformConfidence = 0.0;
    std::vector<double> profileDepths;
    std::vector<double> profileRadii;
    std::vector<int> profilePoints;
    std::vector<int> profileSectors;

    std::string mode;
    std::string reason;
};

Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry);

}

// ============================================================================
// 功能分区：孔形多截面一致性分类
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/

namespace HoleLeixingPouMianGongShi {

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    int inputType = 0;
    int baseType = 0;
    int axialType = 0;
    int holeType = 0;
    bool typeChanged = false;
    bool changedFromMultiSectionType = false;

    bool shortStrongBase = false;
    bool baseDirectionalTaper = false;

    bool directionalRecoveryEvidenceStrong = false;
    bool axialConfirmedCone = false;
    bool axialOverCollapse = false;
    bool axialUnstableConeRejected = false;
    bool crossShiftConeConsistent = false;
    bool crossShiftConeHuiFuChengGong = false;
    bool crossShiftPlatformStraight = false;
    bool baseStrongPlatform = false;
    bool axialStrongPlatform = false;

    bool geometryStraightConsensus = false;
    bool wallStraightConsensus = false;

    double axialShift = 0.35;
    double axialTerminalRadius = 0.0;
    double axialTerminalRatio = 0.0;
    double axialShrinkRatio = 0.0;
    double directionalRecoveryMinShrink = 0.0;
    double directionalRecoveryMinShrinkRatio = 0.16;
    double directionalRecoveryMaxTerminalRatio = 0.94;
    double platformReferenceRadius = 0.0;
    double platformCrossShiftDelta = 0.0;
    int crossShiftConeOverlapLayers = 0;
    double crossShiftConeOverlapSpan = 0.0;
    double crossShiftConeRmse = 0.0;
    double crossShiftConeSlopeDelta = 0.0;

    bool affineConeConsistent = false;
    bool affineConeOverruledCollapse = false;

    bool extremeNonWallCollapseConfirmed = false;
    bool affineConeBlockedByExtremeNonWall = false;
    double affineAlignedRmse = 0.0;
    double affineRadiusOffset = 0.0;
    double affineBaseSlope = 0.0;
    double affineAxialSlope = 0.0;
    double affineSharedSlope = 0.0;
    double affineBaseIntercept = 0.0;
    double affineAxialIntercept = 0.0;
    double affineSharedRmse = 0.0;
    double affineSlopeRelativeDifference = 0.0;

    bool axialOverCollapseRaw = false;
    bool nonWallCollapseConfirmed = false;
    bool summaryOnlyNonWallTuoDi = false;
    bool lateSectorDrop = false;
    bool latePointDrop = false;
    bool terminalTrendDeviation = false;
    bool crossShiftShapeMismatch = false;
    bool terminalCollapseEvidence = false;
    bool mouthOvershootEvidence = false;
    int nonWallQualityFailureCount = 0;
    double lateSectorRatio = 1.0;
    double latePointRatio = 1.0;
    double terminalTrendError = 0.0;
    double extremeShrinkThreshold = 0.88;
    double extremeTerminalTrendErrorThreshold = 0.45;

    HoleLeixingPouMianBase::Result base;
    HoleLeixingPouMianBase::Result axial;
    std::string mode;
    std::string reason;
};

Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry);

Result classifyProfiles(const HoleLeixingPouMianBase::Result& base,
                        const HoleLeixingPouMianBase::Result& axial,
                        const HoleJiheFinal::Result& geometry,
                        double axialShift = 0.35);

}

// ============================================================================
// 功能分区：最终孔形分类接口
// ============================================================================
/*
模块职责：
孔形分类子模块。

主要调用位置：
由正式孔识别链读取已经计算好的几何证据，给出直孔/锥孔等分类。

维护说明：
分类阈值属于生产语义；代码整理只能改善结构和注释，不能改变判定边界。
*/
#include <array>

namespace HoleLeixingFinal {

/** 【类型导航注释】
 * ShiftPouMianState：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct ShiftPouMianState {
    double shift = 0.0;
    bool valid = false;
    bool coneLike = false;
    bool platform = false;
    int layers = 0;
    double span = 0.0;
    double slope = 0.0;
    double shrink = 0.0;
    double monotonicRatio = 0.0;
    double pointDensityRatio = 0.0;
};

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool attempted = false;
    bool huiFuChengGong = false;
    bool eligibleConservativeVeto = false;
    bool bottomEvidenceStrong = false;
    bool normalizedSupportStrong = false;
    bool multiShiftConeConsistent = false;

    int validProfileCount = 0;
    int coneProfileCount = 0;
    int extraConeProfileCount = 0;
    int platformProfileCount = 0;

    double basePointDensityRatio = 0.0;
    double axialPointDensityRatio = 0.0;
    double sharedSlope = 0.0;
    double sharedRmse = 0.0;
    double slopeSpread = 0.0;
    double bottomShrink = 0.0;
    double bottomRadiusRatio = 0.0;

    std::array<ShiftPouMianState, 5> profiles{};
    HoleLeixingPouMianGongShi::Result finalType;
    std::string mode;
    std::string reason;
};

Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry,
                const HoleLeixingPouMianGongShi::Result& baseline);
Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry);

Result classifyProfiles(const HoleLeixingPouMianGongShi::Result& baseline,
                        const std::array<HoleLeixingPouMianBase::Result, 3>& extras,
                        const HoleJiheFinal::Result& geometry);

}

// ============================================================================
// 孔深与下口几何：在孔形轮廓基础上计算可测深度和下口参数。
// ============================================================================
/*
模块职责：
孔深度估计子模块。

主要调用位置：
由正式孔识别链在几何候选稳定后调用，估计可观测深度或下口证据。

维护说明：
深度步长、层数和下口门限直接影响输出“可测/不可测”，修改必须专项验证。
*/

namespace HoleShenduGuJi {

/** 【类型导航注释】
 * Layer：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Layer {
    double depth = 0.0;
    double radius = 0.0;
    int points = 0;
    int sectors = 0;
    double mad = 0.0;
};

// 锥孔测量后的物理审核结果。
// 这一审核属于 DepthBottomReview 深度/下口计算的一部分，不是识别末端额外外挂的“最终守门”。
// 只有已经得到可靠下口与深度的锥孔才会执行，用于把明显的直孔局部内缩挡在正常孔型链内。
/** 【类型导航注释】
 * WuLiReview：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct WuLiReview {
    bool evaluated = false;
    bool rejectHole = false;
    bool classifyStraight = false;
    double rawBottomRadius = 0.0;
    double slopeDeg = 0.0;
    double depthRadiusRatio = 0.0;
    std::string reason;
};

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool valid = false;
    bool used = false;
    bool bottomValid = false;
    bool platformDetected = false;
    bool replacedFinalGeometryBottom = false;
    bool recoveredMissingBottom = false;
    int polarity = 0;
    int layers = 0;
    int breakLayer = -1;
    double depth = 0.0;
    double bottomRadius = 0.0;
    double confidence = 0.0;
    double slope = 0.0;
    double rmse = 0.0;
    double monotonicRatio = 0.0;
    double shrink = 0.0;
    double terminalDepth = 0.0;
    double terminalRadius = 0.0;
    double terminalCoverage = 0.0;
    bool terminalCapEvidence = false;
    int terminalCapPoints = 0;
    int terminalCapSectors = 0;
    double finalGeometryDepthDelta = 0.0;
    double finalGeometryBottomRadiusDelta = 0.0;
    // DepthBottomReview 在任何上限裁剪之前保存原始下口半径；物理审核必须看原始值，
    // 不能把 Rbottom>=Rtop 的不可能结果先压小再假装合法。
    double rawBottomRadius = 0.0;
    WuLiReview physicalReview;
    std::vector<Layer> profile;
    std::string mode;
    std::string reason;
};

// 对“已经可靠测得上下口和锥段深度”的锥孔做统一物理审核。
// 规则属于正常 DepthBottomReview 孔审核：
//   1) Rtop <= Rbottom：几何矛盾，整孔拒绝；
//   2) 坡角 < 25°：按直孔；
//   3) 有效锥段深度 < 1.50 mm：按直孔；
//   4) depth/Rtop < 0.20：按直孔。
// 该函数也供 FinalGeometry 下口作为最终测量来源时复用，保证所有正常分支使用同一套限制。
WuLiReview reviewMeasuredCone(double topRadius,
                                  double rawBottomRadius,
                                  double depth);

// 当最终孔型认为是锥孔、但没有可靠下口时，使用已有多层孔壁做统一物理审核。
// 必须同时满足：>=4层、单调率>=0.70、有效锥段>=1.50 mm、depth/Rtop>=0.20、
// 半径向内缩小且由 atan2(缩小量, 有效深度) 得到的坡角>=25°；否则回到直孔。
WuLiReview reviewProfileConeWithoutReliableBottom(
    const HoleLeixingPouMianBase::Result& profile,
    double topRadius);

Result evaluate(const std::vector<HoleJiheFinal::Sample>& samples,
                const HoleJiheFinal::Result& geometry,
                const HoleLeixingPouMianGongShi::Result& typeResult);

}


// ============================================================================
// 功能分区：正式识别策略与缓存政策
// ============================================================================
/*
模块职责：
保存孔分析之后/之间使用的生产路由、第二遍计算、安全救援、缓存和观测策略。与候选聚类、孔形分析同属识别分析层，因此集中在本文件维护。

主要调用位置：
HoleShibie_Recognition.cpp 与 ShouDongHole_WeiziZhicheng.cpp。

维护说明：
这里的条件会决定正式算法路径；普通整理只改结构和注释，不改判定边界。
*/
/*
模块职责：
集中保存正式孔识别中的路径选择、第二遍计算条件、保守救援门、点云指纹缓存和孔径观测策略。
这些规则本身不做 GUI 绘制，也不负责点云文件读写。

主要调用位置：
HoleShibie_Recognition.cpp 和 ShouDongHole_WeiziZhicheng.cpp 在正式识别过程中调用。

维护说明：
这里的布尔门和阈值会决定算法走哪条生产路径。普通结构整理只能改命名、注释和组织方式；若修改判定条件，必须视为识别算法改动。
*/

// ============================================================================
// 功能分区：GUI 识别入口策略
// ============================================================================
/*
模块职责：
根据当前前置结果决定正式孔识别从直接候选路径还是兼容前置结果继续，并汇总一次多种子识别的数量关系。

主要调用位置：
HoleShibie_Recognition.cpp 的识别编排。

维护说明：
路由只根据已有识别状态作选择，不应读取 GUI 控件。改变 directClusterFirst 的条件会改变正式算法路径，不能作为普通界面整理。
*/
#include <algorithm>

namespace GuiEntryCelue {

/** 【类型导航注释】
 * LuYouJueDing：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct LuYouJueDing {
    bool preGateHintExecuted = false;
    bool preGateSucceeded = false;
    bool directClusterFirst = true;
};

/** 【函数导航】
 * 作用：执行“decideRouting”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline LuYouJueDing decideRouting(bool preGateHintEnabled, bool preGateResultValid) noexcept
{
    LuYouJueDing out;
    out.preGateHintExecuted = preGateHintEnabled;
    out.preGateSucceeded = preGateHintEnabled && preGateResultValid;
    out.directClusterFirst = !out.preGateSucceeded;
    return out;
}

/** 【类型导航注释】
 * PiCiTongJi：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct PiCiTongJi {
    int seeds = 0;
    int rawAccepted = 0;
    int uniqueHoles = 0;
    int duplicateSeeds = 0;
    int rejectedSeeds = 0;
};

/** 【函数导航】
 * 作用：执行“summarizeBatch”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline PiCiTongJi summarizeBatch(int seeds, int rawAccepted, int uniqueHoles) noexcept
{
    PiCiTongJi out;
    out.seeds = std::max(0, seeds);
    out.rawAccepted = std::clamp(rawAccepted, 0, out.seeds);
    out.uniqueHoles = std::clamp(uniqueHoles, 0, out.rawAccepted);
    out.duplicateSeeds = out.rawAccepted - out.uniqueHoles;
    out.rejectedSeeds = out.seeds - out.rawAccepted;
    return out;
}

}

// ============================================================================
// 功能分区：第二遍识别策略
// ============================================================================
/*
模块职责：
第二遍识别策略模块。

主要调用位置：
由正式孔识别流程在满足既定条件时决定是否执行第二遍规范几何计算。

维护说明：
触发条件会改变计算量和可能的结果路径，属于需要回归验证的策略参数。
*/

namespace SecondPassCelue {

/** 【类型导航注释】
 * Input：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Input {
    bool pass1TypeValid = false;
    int pass1Type = 0;
    int inputType = 0;
    int baseType = 0;
    int axialType = 0;
    bool baseDirectionalTaper = false;
    bool axialConfirmedCone = false;
    bool depthBottomReviewUsed = false;
    bool depthBottomReviewBottomValid = false;
    bool finalGeometryBottomValid = false;
    bool geometryDisplaced = false;
    bool radialExpansionRequired = false;
};

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool coneCandidate = false;
    bool typeAmbiguous = false;
    bool depthRefinement = false;
    bool required = false;
    std::string mode = "PASS1_STABLE";
};

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result evaluate(const Input& in)
{
    Result out;
    out.coneCandidate = in.pass1TypeValid
        && (in.pass1Type == 2
            || in.inputType == 2
            || in.baseType == 2
            || in.axialType == 2
            || in.baseDirectionalTaper
            || in.axialConfirmedCone);
    out.typeAmbiguous = in.pass1TypeValid
        && in.pass1Type == 1
        && out.coneCandidate;
    out.depthRefinement = in.pass1TypeValid
        && in.pass1Type == 2
        && !(in.depthBottomReviewUsed && in.depthBottomReviewBottomValid)
        && !in.finalGeometryBottomValid;
    out.required = in.geometryDisplaced
        || in.radialExpansionRequired
        || out.typeAmbiguous
        || out.depthRefinement;
    if (in.geometryDisplaced) out.mode = "GEOMETRY_DISPLACED";
    else if (in.radialExpansionRequired) out.mode = "RADIAL_EXPANSION";
    else if (out.typeAmbiguous) out.mode = "TYPE_AMBIGUITY";
    else if (out.depthRefinement) out.mode = "CONE_DEPTH_REFINEMENT";
    return out;
}

}

// ============================================================================
// 功能分区：保守救援安全门
// ============================================================================
/*
模块职责：
保守救援安全门模块。

主要调用位置：
由孔识别链在主候选失败但存在强几何证据时调用。

维护说明：
救援门必须保持孔无关和保守；任何放宽都要特别关注假阳性。
*/

namespace BuJiuSafetyCelue {

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool accepted = false;
    bool clickCompatible = false;
    bool highConsistencyMouth = false;
};

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result evaluate(const HoleHouXuanJuLei::Cluster& cluster) noexcept
{
    Result out;
    out.clickCompatible =
        cluster.seedBoundaryDistance <= 3.35
        && cluster.seedAxialDistance <= 0.75
        && cluster.selectionCost <= 3.50;
    out.highConsistencyMouth =
        cluster.mouthAttachmentValid
        && cluster.stabilityMode == "COMPACT_RADIUS_CLUSTER"
        && cluster.supportLayers >= 3
        && cluster.meanCoverage >= 0.98
        && cluster.meanResidual <= 0.42
        && cluster.centerStd <= 0.08
        && cluster.radiusStd <= 0.08
        && cluster.trajectoryContinuity >= 0.99;
    out.accepted = out.clickCompatible && out.highConsistencyMouth;
    return out;
}

}

// ============================================================================
// 功能分区：点云身份与缓存一致性
// ============================================================================
/*
模块职责：
点云指纹与确定性辅助。

主要调用位置：
由识别链和缓存模块用于确认输入点云身份和缓存一致性。

维护说明：
哈希顺序和字段属于回归契约；普通整理不要改变其字节级定义。
*/
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace DianYunBiaoshi {

struct ZhiWen
{
    const void* cloudObject = nullptr;
    const void* dataPointer = nullptr;
    std::size_t cloudSize = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool dense = false;
    std::uint64_t xyzPointHash = 0;
    std::uint64_t xyzPointHash2 = 0;
};

/** 【函数导航】
 * 作用：执行“floatBits”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：DianYunJichu_Core.cpp、HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::uint32_t floatBits(float value) noexcept
{
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float size mismatch");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/** 【函数导航】
 * 作用：坐标变换“rotateLeft”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::uint64_t rotateLeft(std::uint64_t value, unsigned shift) noexcept
{
    return (value << shift) | (value >> (64U - shift));
}

/** 【函数导航】
 * 作用：执行“avalanche”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::uint64_t avalanche(std::uint64_t value) noexcept
{
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return value;
}

/** 【函数导航】
 * 作用：执行“pointWord”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::uint64_t pointWord(std::uint32_t xBits,
                               std::uint32_t yBits,
                               std::uint32_t zBits,
                               std::size_t index) noexcept
{
    const std::uint64_t xy =
        (static_cast<std::uint64_t>(xBits) << 32U)
        | static_cast<std::uint64_t>(yBits);
    return avalanche(
        xy
        ^ (static_cast<std::uint64_t>(zBits) * 0x9e3779b97f4a7c15ULL)
        ^ (static_cast<std::uint64_t>(index) * 0xd6e8feb86659fd93ULL));
}

/** 【函数导航】
 * 作用：执行“fingerprintDianYun”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp、HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
template <class CloudPtr>
ZhiWen fingerprintDianYun(const CloudPtr& cloud)
{
    ZhiWen out;
    if (!cloud || cloud->empty()) return out;
    out.cloudObject = static_cast<const void*>(cloud.get());
    out.dataPointer = static_cast<const void*>(cloud->points.data());
    out.cloudSize = cloud->size();
    out.width = static_cast<std::uint32_t>(cloud->width);
    out.height = static_cast<std::uint32_t>(cloud->height);
    out.dense = cloud->is_dense;

    std::uint64_t lane0 = 0x243f6a8885a308d3ULL;
    std::uint64_t lane1 = 0x13198a2e03707344ULL;
    std::uint64_t lane2 = 0xa4093822299f31d0ULL;
    std::uint64_t lane3 = 0x082efa98ec4e6c89ULL;
    std::size_t index = 0;
    const std::size_t count = cloud->points.size();
    for (; index + 4U <= count; index += 4U) {
        const auto& p0 = cloud->points[index];
        const auto& p1 = cloud->points[index + 1U];
        const auto& p2 = cloud->points[index + 2U];
        const auto& p3 = cloud->points[index + 3U];
        lane0 = rotateLeft(lane0 ^ pointWord(
            floatBits(p0.x), floatBits(p0.y), floatBits(p0.z), index), 17U)
            * 0x9e3779b185ebca87ULL;
        lane1 = rotateLeft(lane1 ^ pointWord(
            floatBits(p1.x), floatBits(p1.y), floatBits(p1.z), index + 1U), 19U)
            * 0xc2b2ae3d27d4eb4fULL;
        lane2 = rotateLeft(lane2 ^ pointWord(
            floatBits(p2.x), floatBits(p2.y), floatBits(p2.z), index + 2U), 23U)
            * 0x165667b19e3779f9ULL;
        lane3 = rotateLeft(lane3 ^ pointWord(
            floatBits(p3.x), floatBits(p3.y), floatBits(p3.z), index + 3U), 29U)
            * 0x85ebca77c2b2ae63ULL;
    }
    for (; index < count; ++index) {
        const auto& point = cloud->points[index];
        lane0 = rotateLeft(lane0 ^ pointWord(
            floatBits(point.x), floatBits(point.y), floatBits(point.z), index), 17U)
            * 0x9e3779b185ebca87ULL;
    }
    const std::uint64_t countWord = static_cast<std::uint64_t>(count);
    out.xyzPointHash = avalanche(
        lane0 ^ rotateLeft(lane1, 11U) ^ rotateLeft(lane2, 29U)
        ^ rotateLeft(lane3, 47U) ^ countWord);

    out.xyzPointHash2 = avalanche(
        rotateLeft(lane0, 7U) ^ rotateLeft(lane1, 31U)
        ^ rotateLeft(lane2, 43U) ^ lane3
        ^ (countWord * 0xd6e8feb86659fd93ULL));
    return out;
}

/** 【函数导航】
 * 作用：执行“sameFingerprint”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：ShouDongHole_WeiziZhicheng.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool sameFingerprint(const ZhiWen& left,
                            const ZhiWen& right) noexcept
{
    return left.cloudObject == right.cloudObject
        && left.dataPointer == right.dataPointer
        && left.cloudSize == right.cloudSize
        && left.width == right.width
        && left.height == right.height
        && left.dense == right.dense
        && left.xyzPointHash == right.xyzPointHash
        && left.xyzPointHash2 == right.xyzPointHash2;
}

}

// ============================================================================
// 功能分区：识别快速路径
// ============================================================================
/*
模块职责：
孔识别快速路径辅助。

主要调用位置：
由正式孔识别链在满足精确等价条件时减少重复工作。

维护说明：
快速路径必须可回退到完整计算路径，并保持几何与正式工作计数契约；不能用近似结果换速度。
*/

namespace HoleFastPath {

/** 【函数导航】
 * 作用：执行“doubleBits”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::uint64_t doubleBits(double value) noexcept
{
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "double size mismatch");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/** 【函数导航】
 * 作用：执行“quantize1e4”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline long long quantize1e4(float value) noexcept
{
    const double scaled = static_cast<double>(value) * 10000.0;
    return scaled >= 0.0
        ? static_cast<long long>(scaled + 0.5)
        : static_cast<long long>(scaled - 0.5);
}

/** 【函数导航】
 * 作用：执行“exactConfirmedClusterMatch”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleShibie_Recognition.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool exactConfirmedClusterMatch(
    int clusterId,
    double centerU,
    double centerV,
    double topW,
    double topRadius,
    int hintClusterId,
    double hintCenterU,
    double hintCenterV,
    double hintTopW,
    double hintTopRadius) noexcept
{
    return clusterId == hintClusterId
        && doubleBits(centerU) == doubleBits(hintCenterU)
        && doubleBits(centerV) == doubleBits(hintCenterV)
        && doubleBits(topW) == doubleBits(hintTopW)
        && doubleBits(topRadius) == doubleBits(hintTopRadius);
}

}

// ============================================================================
// 功能分区：直孔可观测内径核心
// ============================================================================
/*
模块职责：
直孔可观测内径模块。

主要调用位置：
由正式孔识别链在直孔场景读取真实可见孔壁，计算独立内径证据。

维护说明：
该值与规范孔口半径含义不同；维护时不要互相替代，失败也不能用推算值冒充实测值。
*/
#include <cmath>
#include <limits>

namespace GuanCeBoreRadius {

/** 【类型导航注释】
 * State：Hole 分析接口中的自定义 枚举。
 * 主要使用位置：HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
enum class State : int {
    Unobservable = 0,
    Partial = 1,
    Observed = 2
};

/** 【函数导航】
 * 作用：执行“stateName”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline const char* stateName(State state) {
    switch (state) {
    case State::Observed: return "OBSERVED";
    case State::Partial: return "PARTIAL";
    default: return "UNOBSERVABLE";
    }
}

/** 【类型导航注释】
 * YiYouZhengJu：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleShibie_Recognition.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct YiYouZhengJu {
    bool valid = false;
    double radius = 0.0;
    double radiusMad = 0.0;
    double depthStart = 0.0;
    double depthEnd = 0.0;
    double depthSpan = 0.0;
    double coverage = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double centerLineRmse = std::numeric_limits<double>::infinity();
    int usedSlices = 0;
};

/** 【类型导航注释】
 * Result：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleJihe_Geometry.h、ShouDongHole_WeiziZhicheng.cpp、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、ShouDongHole_Manual.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Result {
    bool executed = false;
    bool valid = false;
    State state = State::Unobservable;
    double radius = 0.0;
    double radiusMad = 0.0;
    double depthStart = 0.0;
    double depthEnd = 0.0;
    double depthSpan = 0.0;
    double coverage = 0.0;
    double rmse = 0.0;
    double centerLineRmse = 0.0;
    double radiusSlope = 0.0;
    double axisCorrectionDegrees = 0.0;
    double centerShift = 0.0;
    int usedSlices = 0;
    int candidateCount = 0;
    int familyCount = 0;
    int depthRegions = 0;
    int histogramWindows = 0;
    long long histogramPointVisits = 0;
    long long fitPointVisits = 0;
    const char* source = "ObservedBore_UNOBSERVABLE";
    const char* decision = "NOT_EXECUTED";
};

/** 【函数导航】
 * 作用：评估/审核“classifyEvidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline State classifyEvidence(int usedSlices, double depthSpan,
                              double coverage, double rmse,
                              double centerLineRmse, double radiusMad,
                              double radiusScale) {

    const double observedSpan = std::max(0.70, 0.11 * radiusScale);
    const double partialSpan = std::max(0.42, 0.065 * radiusScale);
    const double observedMad = std::max(0.10, 0.020 * radiusScale);
    const double partialMad = std::max(0.15, 0.032 * radiusScale);
    if (usedSlices >= 5 && depthSpan >= observedSpan
        && coverage >= 0.40 && rmse <= 0.12
        && centerLineRmse <= std::max(0.20, 0.035 * radiusScale)
        && radiusMad <= observedMad) {
        return State::Observed;
    }
    if (usedSlices >= 3 && depthSpan >= partialSpan
        && coverage >= 0.22 && rmse <= 0.18
        && centerLineRmse <= std::max(0.30, 0.055 * radiusScale)
        && radiusMad <= partialMad) {
        return State::Partial;
    }
    return State::Unobservable;
}

/** 【函数导航】
 * 作用：评估/审核“classifyPromotedExistingEvidence”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline State classifyPromotedExistingEvidence(
    int usedSlices, double depthSpan, double coverage, double rmse,
    double centerLineRmse, double radiusMad, double radiusScale) {

    const State generic = classifyEvidence(
        usedSlices, depthSpan, coverage, rmse, centerLineRmse, radiusMad,
        radiusScale);
    if (generic != State::Unobservable) return generic;

    const bool shortHighCoverage = usedSlices >= 3 && depthSpan >= 0.22
        && coverage >= 0.40 && rmse <= 0.13
        && centerLineRmse <= std::max(0.12, 0.025 * radiusScale)
        && radiusMad <= std::max(0.10, 0.022 * radiusScale);
    return shortHighCoverage ? State::Partial : State::Unobservable;
}

/** 【函数导航】
 * 作用：执行“physicallyInsideMouth”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool physicallyInsideMouth(double radius, double radiusMad,
                                  double canonicalRadius) {
    if (!(radius > 0.45) || !(canonicalRadius > 0.5)
        || !std::isfinite(radius) || !std::isfinite(radiusMad)) return false;
    const double outsideAllowance = std::max(
        0.075, 2.5 * std::max(0.0, radiusMad) + 0.015);
    return radius >= 0.52 * canonicalRadius
        && radius <= canonicalRadius + outsideAllowance;
}

/** 【函数导航】
 * 作用：执行“compatibleWithCanonicalMouthForExisting”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool compatibleWithCanonicalMouthForExisting(
    double radius, double radiusMad, double canonicalRadius) {
    if (!(radius > 0.45) || !(canonicalRadius > 0.5)
        || !std::isfinite(radius) || !std::isfinite(radiusMad)) return false;

    const double outsideAllowance = std::max({
        0.18, 0.060 * canonicalRadius,
        3.5 * std::max(0.0, radiusMad) + 0.030});
    return radius >= 0.52 * canonicalRadius
        && radius <= canonicalRadius + outsideAllowance;
}

/** 【函数导航】
 * 作用：执行“fromExisting”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result fromExisting(const YiYouZhengJu& existing,
                           double canonicalRadius) {
    Result out;
    out.executed = true;
    out.source = "ObservedBore_EXISTING_PERSISTENT_INNER_CYLINDER";
    if (!existing.valid || !compatibleWithCanonicalMouthForExisting(
            existing.radius, existing.radiusMad, canonicalRadius)) {
        out.decision = existing.valid
            ? "REJECT_EXISTING_INNER_OUTSIDE_MOUTH_SEMANTICS"
            : "REJECT_EXISTING_INNER_UNAVAILABLE";
        return out;
    }
    const State state = classifyPromotedExistingEvidence(
        existing.usedSlices, existing.depthSpan, existing.coverage,
        existing.rmse, existing.centerLineRmse, existing.radiusMad,
        canonicalRadius);
    if (state == State::Unobservable) {
        out.decision = "REJECT_EXISTING_INNER_INSUFFICIENT_OBSERVABILITY";
        return out;
    }
    out.valid = true;
    out.state = state;
    out.radius = existing.radius;
    out.radiusMad = existing.radiusMad;
    out.depthStart = existing.depthStart;
    out.depthEnd = existing.depthEnd;
    out.depthSpan = existing.depthSpan;
    out.coverage = existing.coverage;
    out.rmse = existing.rmse;
    out.centerLineRmse = existing.centerLineRmse;
    out.usedSlices = existing.usedSlices;
    out.decision = state == State::Observed
        ? "APPLY_EXISTING_OBSERVED_PERSISTENT_INNER_CYLINDER"
        : (existing.depthSpan >= std::max(0.42, 0.065 * canonicalRadius)
            ? "APPLY_EXISTING_PARTIAL_LONG_SPAN_PERSISTENT_INNER_CYLINDER"
            : "APPLY_EXISTING_PARTIAL_SHORT_HIGH_COVERAGE_INNER_CYLINDER");
    return out;
}

}

// ============================================================================
// 功能分区：直孔可观测内径接口
// ============================================================================
/*
模块职责：
直孔可观测内径模块。

主要调用位置：
由正式孔识别链在直孔场景读取真实可见孔壁，计算独立内径证据。

维护说明：
该值与规范孔口半径含义不同；维护时不要互相替代，失败也不能用推算值冒充实测值。
*/
#include "HoleWeizi_Pose.h"

#include <Eigen/Core>

#include <numeric>
#include <utility>

// 当前生产基线对应的观测内径规则。
//

//

namespace GuanCeBoreRadius {

using HoleWeiziBase::QiePianNiHe;
using HoleWeiziBase::fitCircleRobust;
using HoleWeiziBase::makeAxes;
using HoleWeiziBase::median;
using HoleWeiziBase::normalizeOr;

inline constexpr int kSectors = 72;
inline constexpr int kHistogramBins = 96;

/** 【类型导航注释】
 * TouYingPoint：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：HoleWeizi_Pose.h。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct TouYingPoint {
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
    double radius = 0.0;
};

/** 【类型导航注释】
 * Family：Hole 分析接口中的自定义 结构体。
 * 主要使用位置：ShouDongHole_JiheJianCe.cpp。
 * 维护提示：字段默认值、单位和有效性标志属于调用契约；纯命名/注释整理不得改变字段顺序、默认值或初始化语义。
 */
struct Family {
    bool valid = false;
    State state = State::Unobservable;
    double radius = 0.0;
    double radiusMad = std::numeric_limits<double>::infinity();
    double depthStart = 0.0;
    double depthEnd = 0.0;
    double depthSpan = 0.0;
    double coverage = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double centerLineRmse = std::numeric_limits<double>::infinity();
    double radiusSlope = 0.0;
    double axisCorrectionDegrees = 0.0;
    double centerShift = 0.0;
    double quality = -std::numeric_limits<double>::infinity();
    int usedSlices = 0;
    int depthRegions = 0;
    int totalSupport = 0;
    std::vector<QiePianNiHe> slices;
};

/** 【函数导航】
 * 作用：执行“projectSamples”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::vector<TouYingPoint> projectSamples(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double canonicalRadius,
    Eigen::Vector3d& u,
    Eigen::Vector3d& v,
    double& maximumDepth) {
    axis = normalizeOr(axis, Eigen::Vector3d(0.0, 0.0, -1.0));
    makeAxes(axis, u, v);
    maximumDepth = std::min(12.25, std::max(2.25, 2.05 * canonicalRadius));
    const double radialLow = std::max(0.45, 0.50 * canonicalRadius);
    const double radialHigh = 1.18 * canonicalRadius + 0.35;
    std::vector<TouYingPoint> out;
    out.reserve(samples.size());
    for (const auto& sample : samples) {
        const Eigen::Vector3d point(sample.u, sample.v, sample.w);
        if (!point.allFinite()) continue;
        const Eigen::Vector3d delta = point - center;
        const double depth = delta.dot(axis);
        if (depth < 0.16 || depth > maximumDepth + 0.30) continue;
        const double x = delta.dot(u);
        const double y = delta.dot(v);
        const double radius = std::hypot(x, y);
        if (radius < radialLow || radius > radialHigh) continue;
        out.push_back(TouYingPoint{x, y, depth, radius});
    }
    return out;
}

/** 【函数导航】
 * 作用：执行“generateCandidates”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline std::vector<std::vector<QiePianNiHe>> generateCandidates(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double canonicalRadius,
    int& candidateCount,
    int& histogramWindows,
    long long& histogramPointVisits,
    long long& fitPointVisits) {
    candidateCount = 0;
    histogramWindows = 0;
    histogramPointVisits = 0;
    fitPointVisits = 0;
    Eigen::Vector3d u, v;
    double maximumDepth = 0.0;
    const std::vector<TouYingPoint> projected = projectSamples(
        samples, center, axis, canonicalRadius, u, v, maximumDepth);
    std::vector<std::vector<QiePianNiHe>> byDepth;
    if (projected.size() < 24U) return byDepth;

    static constexpr std::array<double, 3> kHalfWidths{{0.14, 0.22, 0.34}};
    const double radialLow = std::max(0.45, 0.50 * canonicalRadius);
    const double radialHigh = 1.18 * canonicalRadius + 0.35;
    std::vector<double> targetDepths;
    for (double depth = 0.25; depth <= maximumDepth + 1e-9; depth += 0.25)
        targetDepths.push_back(depth);
    byDepth.resize(targetDepths.size());

    for (std::size_t depthIndex = 0; depthIndex < targetDepths.size(); ++depthIndex) {
        const double targetDepth = targetDepths[depthIndex];
        std::vector<QiePianNiHe> depthCandidates;
        for (double halfWidth : kHalfWidths) {
            std::array<int, kHistogramBins> histogram{};
            std::vector<std::size_t> window;
            window.reserve(projected.size() / 8U + 16U);
            for (std::size_t i = 0; i < projected.size(); ++i) {
                if (std::abs(projected[i].depth - targetDepth) > halfWidth) continue;
                window.push_back(i);
                const int bin = std::clamp(static_cast<int>(std::floor(
                    (projected[i].radius - radialLow) / (radialHigh - radialLow)
                    * static_cast<double>(kHistogramBins))), 0, kHistogramBins - 1);
                ++histogram[static_cast<std::size_t>(bin)];
            }
            ++histogramWindows;
            histogramPointVisits += static_cast<long long>(window.size());
            if (window.size() < 18U) continue;

            std::array<int, kHistogramBins> smoothed{};
            int maximumPeak = 0;
            for (int bin = 0; bin < kHistogramBins; ++bin) {
                int value = 0;
                for (int offset = -2; offset <= 2; ++offset) {
                    const int index = std::clamp(bin + offset, 0, kHistogramBins - 1);
                    value += (3 - std::abs(offset))
                        * histogram[static_cast<std::size_t>(index)];
                }
                smoothed[static_cast<std::size_t>(bin)] = value;
                maximumPeak = std::max(maximumPeak, value);
            }
            if (maximumPeak <= 0) continue;
            std::array<int, kHistogramBins> order{};
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](int left, int right) {
                if (smoothed[static_cast<std::size_t>(left)]
                    != smoothed[static_cast<std::size_t>(right)]) {
                    return smoothed[static_cast<std::size_t>(left)]
                        > smoothed[static_cast<std::size_t>(right)];
                }
                return left < right;
            });
            std::vector<int> modes;
            for (int bin : order) {
                if (smoothed[static_cast<std::size_t>(bin)]
                    < std::max(5, static_cast<int>(std::ceil(0.22 * maximumPeak)))) break;
                bool separated = true;
                for (int previous : modes)
                    if (std::abs(previous - bin) < 3) separated = false;
                if (!separated) continue;
                modes.push_back(bin);
                if (modes.size() >= 6U) break;
            }

            for (int bin : modes) {
                const double modeRadius = radialLow
                    + (static_cast<double>(bin) + 0.5)
                    / static_cast<double>(kHistogramBins)
                    * (radialHigh - radialLow);
                const double band = std::clamp(0.038 * modeRadius, 0.11, 0.24);
                std::vector<Eigen::Vector2d> points;
                points.reserve(window.size());
                fitPointVisits += static_cast<long long>(window.size());
                for (std::size_t index : window) {
                    if (std::abs(projected[index].radius - modeRadius) <= band)
                        points.emplace_back(projected[index].x, projected[index].y);
                }
                QiePianNiHe fit = fitCircleRobust(
                    points, targetDepth, halfWidth, 1.20 * canonicalRadius + 0.45);
                if (!fit.valid
                    || fit.radius < 0.52 * canonicalRadius
                    || fit.radius > 1.12 * canonicalRadius + 0.10
                    || fit.center.norm() > std::max(1.45, 0.28 * canonicalRadius)
                    || fit.coverage < 0.18 || fit.rmse > 0.22) continue;
                fit.centerWorld = center + u * fit.center.x()
                    + v * fit.center.y() + normalizeOr(axis,
                        Eigen::Vector3d(0.0, 0.0, -1.0)) * targetDepth;
                fit.source = "ObservedBore_FRAGMENTED_CYLINDER_RADIAL_MODE";
                depthCandidates.push_back(fit);
                ++candidateCount;
            }
        }

        std::sort(depthCandidates.begin(), depthCandidates.end(),
            [](const QiePianNiHe& left, const QiePianNiHe& right) {
                if (left.radius != right.radius) return left.radius < right.radius;
                if (left.rmse != right.rmse) return left.rmse < right.rmse;
                return left.halfThickness < right.halfThickness;
            });
        std::vector<QiePianNiHe> deduplicated;
        const double duplicateTolerance = std::max(0.075, 0.016 * canonicalRadius);
        for (const QiePianNiHe& fit : depthCandidates) {
            int duplicate = -1;
            for (std::size_t i = 0; i < deduplicated.size(); ++i) {
                if (std::abs(deduplicated[i].radius - fit.radius) <= duplicateTolerance) {
                    duplicate = static_cast<int>(i);
                    break;
                }
            }
            const double quality = fit.coverage - 1.8 * fit.rmse
                + 0.0015 * std::sqrt(static_cast<double>(std::max(1, fit.supportCount)))
                - 0.08 * fit.halfThickness;
            if (duplicate < 0) {
                deduplicated.push_back(fit);
            } else {
                const QiePianNiHe& old = deduplicated[static_cast<std::size_t>(duplicate)];
                const double oldQuality = old.coverage - 1.8 * old.rmse
                    + 0.0015 * std::sqrt(static_cast<double>(std::max(1, old.supportCount)))
                    - 0.08 * old.halfThickness;
                if (quality > oldQuality)
                    deduplicated[static_cast<std::size_t>(duplicate)] = fit;
            }
        }
        byDepth[depthIndex] = std::move(deduplicated);
    }
    return byDepth;
}

/** 【函数导航】
 * 作用：执行“countDepthRegions”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline int countDepthRegions(const std::vector<QiePianNiHe>& slices) {
    if (slices.empty()) return 0;
    int regions = 1;
    for (std::size_t i = 1; i < slices.size(); ++i)
        if (slices[i].depth - slices[i - 1].depth > 0.75) ++regions;
    return regions;
}

/** 【函数导航】
 * 作用：评估/审核“evaluateFamily”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleWeizi_Pose.h。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline bool evaluateFamily(std::vector<QiePianNiHe> slices,
                           double canonicalRadius,
                           Family& out) {
    if (slices.size() < 3U) return false;
    std::sort(slices.begin(), slices.end(), [](const QiePianNiHe& left,
                                               const QiePianNiHe& right) {
        return left.depth < right.depth;
    });
    const double depthSpan = slices.back().depth - slices.front().depth;
    if (depthSpan < std::max(0.42, 0.065 * canonicalRadius)) return false;

    std::vector<double> radii, rmses;
    radii.reserve(slices.size());
    rmses.reserve(slices.size());
    double coverage = 0.0;
    int support = 0;
    for (const QiePianNiHe& slice : slices) {
        radii.push_back(slice.radius);
        rmses.push_back(slice.rmse);
        coverage += slice.coverage;
        support += slice.supportCount;
    }
    const double radius = median(radii);
    std::vector<double> radiusDeviation;
    radiusDeviation.reserve(radii.size());
    for (double value : radii) radiusDeviation.push_back(std::abs(value - radius));
    const double radiusMad = 1.4826 * median(radiusDeviation);
    coverage /= static_cast<double>(slices.size());
    const double rmse = median(rmses);

    double sumD = 0.0, sumR = 0.0, sumDD = 0.0, sumDR = 0.0;
    double sumX = 0.0, sumY = 0.0, sumDX = 0.0, sumDY = 0.0;
    for (const QiePianNiHe& slice : slices) {
        sumD += slice.depth;
        sumR += slice.radius;
        sumDD += slice.depth * slice.depth;
        sumDR += slice.depth * slice.radius;
        sumX += slice.center.x();
        sumY += slice.center.y();
        sumDX += slice.depth * slice.center.x();
        sumDY += slice.depth * slice.center.y();
    }
    const double n = static_cast<double>(slices.size());
    const double determinant = n * sumDD - sumD * sumD;
    if (std::abs(determinant) < 1e-9) return false;
    const double slope = (n * sumDR - sumD * sumR) / determinant;
    const double bx = (n * sumDX - sumD * sumX) / determinant;
    const double by = (n * sumDY - sumD * sumY) / determinant;
    const double ax = (sumX - bx * sumD) / n;
    const double ay = (sumY - by * sumD) / n;
    double centerSquared = 0.0;
    for (const QiePianNiHe& slice : slices) {
        const double dx = slice.center.x() - (ax + bx * slice.depth);
        const double dy = slice.center.y() - (ay + by * slice.depth);
        centerSquared += dx * dx + dy * dy;
    }
    const double centerLineRmse = std::sqrt(centerSquared / n);
    const double centerShift = std::hypot(ax, ay);
    const double axisCorrectionDegrees = std::atan(std::hypot(bx, by))
        * 180.0 / HoleWeiziBase::kPi;
    const int regions = countDepthRegions(slices);

    const double mouthSeparation = canonicalRadius - radius;
    if (!physicallyInsideMouth(radius, radiusMad, canonicalRadius)

        || mouthSeparation < std::max(0.08, 0.018 * canonicalRadius)
        || std::abs(slope) > std::max(0.10, 0.022 * canonicalRadius)
        || centerShift > std::max(1.20, 0.24 * canonicalRadius)
        || axisCorrectionDegrees > 8.0) return false;

    const State state = classifyEvidence(
        static_cast<int>(slices.size()), depthSpan, coverage, rmse,
        centerLineRmse, radiusMad, canonicalRadius);
    if (state == State::Unobservable) return false;

    if (regions > 2 || (regions > 1 && (slices.size() < 4U || depthSpan < 1.0)))
        return false;

    out.valid = true;
    out.state = state;
    out.radius = radius;
    out.radiusMad = radiusMad;
    out.depthStart = slices.front().depth;
    out.depthEnd = slices.back().depth;
    out.depthSpan = depthSpan;
    out.coverage = coverage;
    out.rmse = rmse;
    out.centerLineRmse = centerLineRmse;
    out.radiusSlope = slope;
    out.axisCorrectionDegrees = axisCorrectionDegrees;
    out.centerShift = centerShift;
    out.usedSlices = static_cast<int>(slices.size());
    out.depthRegions = regions;
    out.totalSupport = support;
    out.slices = std::move(slices);
    out.quality = 1.25 * coverage
        + 0.12 * std::min(10.0, n)
        + 0.08 * std::min(4.0, depthSpan)
        + 0.0012 * std::sqrt(static_cast<double>(std::max(1, support)))
        - 2.2 * rmse - 1.5 * radiusMad
        - 0.8 * centerLineRmse - 0.10 * std::abs(slope)
        - 0.02 * axisCorrectionDegrees;
    return true;
}

/** 【函数导航】
 * 作用：估计“estimateFragmentedCylinder”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleFenxi_Analysis.h（本文件内部调用/实现）。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result estimateFragmentedCylinder(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const Eigen::Vector3d& center,
    Eigen::Vector3d axis,
    double canonicalRadius) {
    Result out;
    out.executed = true;
    out.source = "ObservedBore_FRAGMENTED_PERSISTENT_CYLINDER";
    if (samples.empty() || !center.allFinite() || !axis.allFinite()
        || !(canonicalRadius > 0.5)) {
        out.decision = "REJECT_RAW_INVALID_INPUT";
        return out;
    }
    int candidateCount = 0, histogramWindows = 0;
    long long histogramPointVisits = 0, fitPointVisits = 0;
    const auto byDepth = generateCandidates(
        samples, center, axis, canonicalRadius, candidateCount,
        histogramWindows, histogramPointVisits, fitPointVisits);
    out.candidateCount = candidateCount;
    out.histogramWindows = histogramWindows;
    out.histogramPointVisits = histogramPointVisits;
    out.fitPointVisits = fitPointVisits;
    if (candidateCount < 3) {
        out.decision = "REJECT_RAW_TOO_FEW_CIRCLE_CANDIDATES";
        return out;
    }

    std::vector<Family> families;
    const double radiusTolerance = std::max(0.13, 0.026 * canonicalRadius);
    for (const auto& depthCandidates : byDepth) {
        for (const QiePianNiHe& seed : depthCandidates) {
            double targetRadius = seed.radius;
            std::vector<QiePianNiHe> selected;
            for (int iteration = 0; iteration < 4; ++iteration) {
                selected.clear();
                for (const auto& candidates : byDepth) {
                    const QiePianNiHe* best = nullptr;
                    double bestQuality = -std::numeric_limits<double>::infinity();
                    for (const QiePianNiHe& candidate : candidates) {
                        if (std::abs(candidate.radius - targetRadius) > radiusTolerance) continue;
                        const double quality = candidate.coverage - 1.8 * candidate.rmse
                            + 0.0015 * std::sqrt(static_cast<double>(
                                std::max(1, candidate.supportCount)))
                            - 0.08 * candidate.halfThickness
                            - 0.8 * std::abs(candidate.radius - targetRadius)
                                / radiusTolerance;
                        if (!best || quality > bestQuality) {
                            best = &candidate;
                            bestQuality = quality;
                        }
                    }
                    if (best) selected.push_back(*best);
                }
                if (selected.size() < 3U) break;
                std::vector<double> radii;
                radii.reserve(selected.size());
                for (const QiePianNiHe& fit : selected) radii.push_back(fit.radius);
                targetRadius = median(radii);
            }
            if (selected.size() < 3U) continue;
            Family family;
            if (evaluateFamily(selected, canonicalRadius, family))
                families.push_back(std::move(family));
        }
    }
    out.familyCount = static_cast<int>(families.size());
    if (families.empty()) {
        out.decision = "REJECT_RAW_NO_STABLE_CYLINDER_FAMILY";
        return out;
    }

    std::sort(families.begin(), families.end(), [](const Family& left,
                                                   const Family& right) {
        if (left.quality != right.quality) return left.quality > right.quality;
        if (left.state != right.state)
            return static_cast<int>(left.state) > static_cast<int>(right.state);
        if (left.radiusMad != right.radiusMad) return left.radiusMad < right.radiusMad;
        return left.radius < right.radius;
    });
    const Family& best = families.front();
    if (families.size() > 1U) {
        const Family& second = families[1];
        const double conflictTolerance = std::max(0.18, 0.035 * canonicalRadius);
        if (second.quality >= best.quality - 0.18
            && std::abs(second.radius - best.radius) > conflictTolerance) {
            out.decision = "REJECT_RAW_COMPETING_CYLINDER_FAMILIES";
            return out;
        }
    }

    out.valid = true;
    out.state = best.state;
    out.radius = best.radius;
    out.radiusMad = best.radiusMad;
    out.depthStart = best.depthStart;
    out.depthEnd = best.depthEnd;
    out.depthSpan = best.depthSpan;
    out.coverage = best.coverage;
    out.rmse = best.rmse;
    out.centerLineRmse = best.centerLineRmse;
    out.radiusSlope = best.radiusSlope;
    out.axisCorrectionDegrees = best.axisCorrectionDegrees;
    out.centerShift = best.centerShift;
    out.usedSlices = best.usedSlices;
    out.depthRegions = best.depthRegions;
    out.decision = best.state == State::Observed
        ? "APPLY_RAW_OBSERVED_FRAGMENTED_PERSISTENT_CYLINDER"
        : "APPLY_RAW_PARTIAL_FRAGMENTED_PERSISTENT_CYLINDER";
    return out;
}

/** 【函数导航】
 * 作用：评估/审核“evaluate”对应的本模块子步骤；输入、输出和单位以函数签名及相邻结构体字段说明为准。
 * 所属模块：Hole 分析接口。
 * 主要引用/调用位置：HoleJihe_Geometry.h、HoleFenxi_Analysis.cpp、HoleShibie_Recognition.cpp、HoleJihe_Geometry.cpp。
 * 阅读提示：先看参数与返回值，再结合本文件上方功能分区理解前置条件；若修改数值计算，必须做对应生产数据回归。
 */
inline Result evaluate(
    const std::vector<HoleJiheFinal::Sample>& samples,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& axis,
    double canonicalRadius,
    const YiYouZhengJu& existing,
    bool allowRawEstimator) {
    Result existingResult = fromExisting(existing, canonicalRadius);
    if (existingResult.valid) return existingResult;
    if (!allowRawEstimator) {
        existingResult.executed = true;
        existingResult.valid = false;
        existingResult.state = State::Unobservable;
        existingResult.radius = 0.0;
        existingResult.source = "ObservedBore_UNOBSERVABLE";
        existingResult.decision = "UNOBSERVABLE_RAW_NOT_JUSTIFIED_BY_LEGACY_EVIDENCE";
        return existingResult;
    }

    Result raw = estimateFragmentedCylinder(
        samples, center, axis, canonicalRadius);
    if (raw.valid) return raw;

    Result out = raw;
    out.executed = true;
    out.valid = false;
    out.state = State::Unobservable;
    out.radius = 0.0;
    out.source = "ObservedBore_UNOBSERVABLE";
    if (existing.valid) {
        if (raw.decision && std::string(raw.decision).find("REJECT_RAW_") == 0U)
            out.decision = "UNOBSERVABLE_EXISTING_INCONSISTENT_AND_RAW_WEAK";
        else
            out.decision = "UNOBSERVABLE_EXISTING_INCONSISTENT";
    } else {
        out.decision = "UNOBSERVABLE_NO_PERSISTENT_CYLINDER_EVIDENCE";
    }
    return out;
}

}
