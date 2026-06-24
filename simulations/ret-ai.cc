/* 

Modular simulation for AI-driven antenna tilt optimization using ns-3-ai
3 gNBs in equilateral triangle topology (facing origin)
Integrated with Samsung LLM Agent for tilt decisions

Parameters:
- NUM_GNB: Fixed at 3 (equilateral triangle)
- simDuration: Simulation duration (seconds)
- sampleIntervalS: Metric sampling interval (seconds)
- decisionEpochS: AI decision interval (seconds)
- isdM: Inter-site distance (meters)
- gnbHeightM: gNB antenna height (meters)
- coverageTputThresholdMbps: Coverage threshold (Mbps)

Output:
- ret_summary.csv: Time-series summary metrics
- ret_per_ue.csv: Per-UE time-series metrics
- ue_positions.csv: UE positions and serving gNB
- log.txt: Configuration log

Usage:
  ./run.sh ret-ai

*/

#include "ns3/ai-module.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <fstream>
#include <numeric>
#include <vector>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>  // for mkdir

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRetAiSimulation");

// ── Simulation parameters ─────────────────────────────────────────────────────
static constexpr uint32_t NUM_GNB                       = 3;  // Fixed: 3 gNBs in equilateral triangle

// Runtime-configurable global parameters
static uint32_t g_numUE                        = 50;
static double   g_simDuration                  = 40.0;
static double   g_isdM                         = 500.0;
static double   g_gnbHeightM                   = 25.0;
static double   g_sampleIntervalS              = 1.0;
static double   g_decisionEpochS               = 5.0;  // AI decision interval
static double   g_coverageTputThresholdMbps    = 0.5;

// UE placement bounds
static double   g_ueBoundsXMin                 = -150.0;
static double   g_ueBoundsXMax                 = 150.0;
static double   g_ueBoundsYMin                 = -150.0;
static double   g_ueBoundsYMax                 = 150.0;

// Ring placement parameters (for Edge scenario)
static double   g_ueRingInnerRadiusM           = 120.0;
static double   g_ueRingOuterRadiusM           = 160.0;
static bool     g_ueRingPlacement              = false;

// Output directory configuration
static std::string   g_runName = "ret-ai";
static std::string   g_outputDir = ".";
static std::string   g_outputSubdir = "";
static std::string   g_csvSummaryFileName = "";
static std::string   g_csvPerUeFileName = "";
static std::string   g_uePositionsFileName = "";
static std::string   g_logFileName = "";
static std::ofstream g_logFile;

// Scenario preset
std::string g_scenario = "custom";

// Per-gNB sector bearing (degrees) - facing toward origin
static const std::array<double, NUM_GNB> BEARING_DEG = {270.0, 30.0, 150.0};

// ── Global state ──────────────────────────────────────────────────────────────
std::vector<Ptr<UniformPlanarArray>> g_gnbAntennas(NUM_GNB, nullptr);
std::array<double, NUM_GNB>          g_currentTiltDeg;

// Measurement state per UE
std::vector<Ptr<PacketSink>> g_sinks;
std::vector<uint64_t>        g_prevBytes;
std::vector<double>          g_latestSinr_dB;
std::vector<double>          g_latestRsrp_dBm;

// CSV output handles
std::ofstream g_csvSummary;
std::ofstream g_csvPerUe;
std::ofstream g_uePositions;

// AI Environment state
static Ptr<OpenGymEnv> g_retEnv = nullptr;
static bool            g_aiReady = false;
static bool            g_useAI = true;  // Toggle to disable AI and use fixed tilts
static double          g_fixedTilt = 0.0;  // Fallback tilt if AI disabled

// Latest metrics for AI observation
static double          g_latestTime = 0.0;
static double          g_latestTotalTput = 0.0;
static double          g_latestAvgTput = 0.0;
static double          g_latestMinTput = 0.0;
static double          g_latestP5Tput = 0.0;
static double          g_latestMaxTput = 0.0;
static double          g_latestJfi = 1.0;
static double          g_latestCoverage = 0.0;

// ── SINR trace callback ───────────────────────────────────────────────────────
void DlSinrCallback(uint32_t ueIdx,
                    uint16_t /*cellId*/,
                    uint16_t /*rnti*/,
                    double   sinrLinear,
                    uint16_t /*bwpId*/)
{
    if (sinrLinear > 0.0)
        g_latestSinr_dB[ueIdx] = 10.0 * std::log10(sinrLinear);
}

// ── RSRP trace callback ───────────────────────────────────────────────────────
void RsrpCallback(uint32_t ueIdx,
                  uint16_t /*cellId*/,
                  uint16_t /*imsi*/,
                  uint16_t /*rnti*/,
                  double   rsrp_dBm,
                  uint8_t  /*bwpId*/)
{
    g_latestRsrp_dBm[ueIdx] = rsrp_dBm;
}

// ── RET actuator — true electrical tilt via beamforming vector ───────────────
void ApplyRet(uint32_t gnbIdx, double newTiltDeg)
{
    NS_ASSERT_MSG(g_gnbAntennas[gnbIdx], "Null UPA handle for gNB " << gnbIdx);

    const double bearingDeg = BEARING_DEG[gnbIdx];
    const double zenithDeg  = 90.0 - newTiltDeg;

    NS_LOG_INFO("[RET] t=" << Simulator::Now().GetSeconds()
                << "s  gNB[" << gnbIdx << "]"
                << "  elec_tilt=" << newTiltDeg << "deg"
                << "  bearing="   << bearingDeg  << "deg"
                << "  (zenith="   << zenithDeg   << "deg)");

    PhasedArrayModel::ComplexVector bfv =
        CreateDirectionalBfvAz(g_gnbAntennas[gnbIdx], bearingDeg, zenithDeg);
    g_gnbAntennas[gnbIdx]->SetBeamformingVector(bfv);
    g_currentTiltDeg[gnbIdx] = newTiltDeg;
}

// ── Harvest UPA handles + set mechanical bearing ──────────────────────────────
void HarvestAntennaHandles(const NetDeviceContainer& gnbDevs,
                           Ptr<NrHelper> nrHelper)
{
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        Ptr<NrGnbPhy>      phy     = nrHelper->GetGnbPhy(gnbDevs.Get(i), 0);
        Ptr<NrSpectrumPhy> specPhy = phy->GetSpectrumPhy();
        Ptr<Object>        antObj  = specPhy->GetAntenna();
        Ptr<UniformPlanarArray> upa = antObj->GetObject<UniformPlanarArray>();

        NS_ABORT_MSG_IF(!upa, "UniformPlanarArray null for gNB " << i);

        upa->SetAlpha(DegreesToRadians(BEARING_DEG[i]));
        upa->SetBeta(0.0);

        g_gnbAntennas[i] = upa;
        NS_LOG_INFO("[INIT] gNB[" << i << "] UPA acquired"
                    << "  mech_bearing=" << BEARING_DEG[i] << "deg");
    }
}

// ── Equilateral triangle gNB placement (facing origin) ───────────────────────
void PlaceGnbs(NodeContainer& gnbs)
{
    const double radius = g_isdM / std::sqrt(3.0);
    const std::array<double, NUM_GNB> positionAngles = {90.0, 210.0, 330.0};
    
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        double angleRad = DegreesToRadians(positionAngles[i]);
        gnbs.Get(i)->GetObject<MobilityModel>()->SetPosition(
            Vector(radius * std::cos(angleRad),
                   radius * std::sin(angleRad),
                   g_gnbHeightM));
    }
    NS_LOG_INFO("[gNB] Placed "<<NUM_GNB<<" gNBs in equilateral triangle (radius=" << radius << "m, ISD=" << g_isdM << "m)");
}

// ── Ring-based UE placement for cell-edge testing ─────────────────────────────
void PlaceUesInRings(NodeContainer& ues, const NodeContainer& gnbs)
{
    Ptr<UniformRandomVariable> radiusVar = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> angleVar  = CreateObject<UniformRandomVariable>();
    
    double innerR2 = g_ueRingInnerRadiusM * g_ueRingInnerRadiusM;
    double outerR2 = g_ueRingOuterRadiusM * g_ueRingOuterRadiusM;
    
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        uint32_t gnbIdx = i % NUM_GNB;
        Vector gnbPos = gnbs.Get(gnbIdx)->GetObject<MobilityModel>()->GetPosition();
        
        double r = std::sqrt(radiusVar->GetValue(innerR2, outerR2));
        double theta = angleVar->GetValue(0.0, 2.0 * M_PI);
        
        double x = gnbPos.x + r * std::cos(theta);
        double y = gnbPos.y + r * std::sin(theta);
        double z = 1.5;
        
        ues.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(x, y, z));
    }
    
    NS_LOG_INFO("[UE] Placed " << ues.GetN() << " UEs in rings around gNBs "
                << "(inner=" << g_ueRingInnerRadiusM << "m, outer=" << g_ueRingOuterRadiusM << "m)");
}

// ── AI Decision Handler - Request new tilts from Python/LLM ───────────────────
void RequestAiDecision()
{
    if (!g_useAI || !g_retEnv)
    {
        // Fallback to fixed tilt
        for (uint32_t i = 0; i < NUM_GNB; ++i)
        {
            ApplyRet(i, g_fixedTilt);
        }
        return;
    }

    NS_LOG_INFO("[AI] Requesting decision at t=" << g_latestTime << "s");
    
    // Notify Python to get action (this blocks until Python responds)
    g_retEnv->Notify();
    
    // Next decision epoch
    double nextDecisionTime = g_latestTime + g_decisionEpochS;
    if (nextDecisionTime < g_simDuration)
    {
        Simulator::Schedule(Seconds(nextDecisionTime), &RequestAiDecision);
    }
}

// ── Metrics Sampling with AI Integration ──────────────────────────────────────
void SampleMetrics()
{
    double now = Simulator::Now().GetSeconds();

    // ── Per-UE throughput calculation ────────────────────────────────────────
    std::vector<double> tput(g_numUE, 0.0);
    std::vector<double> activeTputs;

    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        if (!g_sinks[u]) continue;
        uint64_t bytes   = g_sinks[u]->GetTotalRx();
        uint64_t delta   = bytes - g_prevBytes[u];
        g_prevBytes[u]   = bytes;
        tput[u]          = static_cast<double>(delta) * 8.0 / (g_sampleIntervalS * 1e6);

        if (bytes > 0)
        {
            activeTputs.push_back(tput[u]);
        }
    }

    uint32_t activeUeCount = activeTputs.size();
    if (activeUeCount == 0)
    {
        if (now + g_sampleIntervalS < g_simDuration)
            Simulator::Schedule(Seconds(g_sampleIntervalS), &SampleMetrics);
        return;
    }

    // ── Aggregate metrics ──────────────────────────────────────────────────
    double sumT  = std::accumulate(activeTputs.begin(), activeTputs.end(), 0.0);
    double sumT2 = std::inner_product(activeTputs.begin(), activeTputs.end(), activeTputs.begin(), 0.0);
    double avgT  = sumT / activeUeCount;
    double jfi = (sumT2 > 0.0) ? (sumT * sumT) / (activeUeCount * sumT2) : 1.0;

    uint32_t covered = 0;
    for (double t : activeTputs) if (t >= g_coverageTputThresholdMbps) ++covered;
    double coveragePct = 100.0 * covered / activeUeCount;

    std::vector<double> sorted = activeTputs;
    std::sort(sorted.begin(), sorted.end());
    double p5  = sorted[std::max(0, static_cast<int>(0.05 * activeUeCount + 0.5) - 1)];
    double minT = sorted.front();
    double maxT = sorted.back();

    // ── Update latest metrics for AI observation ────────────────────────────
    g_latestTime = now;
    g_latestTotalTput = sumT;
    g_latestAvgTput = avgT;
    g_latestMinTput = minT;
    g_latestP5Tput = p5;
    g_latestMaxTput = maxT;
    g_latestJfi = jfi;
    g_latestCoverage = coveragePct;

    // ── Current tilt snapshot ───────────────────────────────────────────────
    std::string tiltSnap;
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        tiltSnap += std::to_string(static_cast<int>(g_currentTiltDeg[i]));
        if (i < NUM_GNB - 1) tiltSnap += "|";
    }

    // ── Write summary CSV row ───────────────────────────────────────────────
    g_csvSummary << now
                 << "," << sumT
                 << "," << avgT
                 << "," << minT
                 << "," << p5
                 << "," << maxT
                 << "," << jfi
                 << "," << coveragePct;
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_csvSummary << "," << g_currentTiltDeg[i];
    g_csvSummary << "\n";

    // ── Write per-UE CSV rows ──────────────────────────────────────────────
    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        g_csvPerUe << now
                   << "," << u
                   << "," << tput[u]
                   << "," << g_latestSinr_dB[u]
                   << "," << g_latestRsrp_dBm[u];
        for (uint32_t i = 0; i < NUM_GNB; ++i)
            g_csvPerUe << "," << g_currentTiltDeg[i];
        g_csvPerUe << "\n";
    }

    NS_LOG_INFO("[METRIC] t=" << now << "s"
                << "  totalTput=" << std::fixed << std::setprecision(2) << sumT << "Mbps"
                << "  avgTput="   << avgT  << "Mbps"
                << "  minTput="   << minT  << "Mbps"
                << "  p5Tput="    << p5    << "Mbps"
                << "  JFI="       << std::setprecision(3) << jfi
                << "  coverage="  << std::setprecision(0) << coveragePct << "%"
                << "  tilts=["    << tiltSnap << "]");

    if (now + g_sampleIntervalS < g_simDuration)
        Simulator::Schedule(Seconds(g_sampleIntervalS), &SampleMetrics);
}

// ── AI Environment Class ──────────────────────────────────────────────────────
class RetAiEnv : public OpenGymEnv
{
  public:
    RetAiEnv();
    ~RetAiEnv() override;
    static TypeId GetTypeId();
    void DoDispose() override;

    // OpenGym interfaces:
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    bool GetGameOver() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;
};

RetAiEnv::RetAiEnv()
{
    SetOpenGymInterface(OpenGymInterface::Get());
    
    // Initialize tilts to default
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        g_currentTiltDeg[i] = 7.0;
    }
}

RetAiEnv::~RetAiEnv()
{
}

TypeId
RetAiEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::RetAiEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

void
RetAiEnv::DoDispose()
{
}

Ptr<OpenGymSpace>
RetAiEnv::GetActionSpace()
{
    // Action: 3 tilt angles (one per gNB), range 0-15 degrees
    std::vector<uint32_t> shape = {3};
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 15, shape, dtype);
    return box;
}

Ptr<OpenGymSpace>
RetAiEnv::GetObservationSpace()
{
    // Observation: 11 values (time, total, avg, min, p5, max, jfi, coverage, tilt0, tilt1, tilt2)
    std::vector<uint32_t> shape = {11};
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 1000, shape, dtype);
    return box;
}

bool
RetAiEnv::GetGameOver()
{
    return false;  // No terminal state in this continuous optimization
}

Ptr<OpenGymDataContainer>
RetAiEnv::GetObservation()
{
    // Build observation vector: [time, total_tput, avg_tput, min_tput, p5_tput, max_tput, jfi, coverage, tilt0, tilt1, tilt2]
    std::vector<uint32_t> shape = {11};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);

    box->AddValue(static_cast<float>(g_latestTime));
    box->AddValue(static_cast<float>(g_latestTotalTput));
    box->AddValue(static_cast<float>(g_latestAvgTput));
    box->AddValue(static_cast<float>(g_latestMinTput));
    box->AddValue(static_cast<float>(g_latestP5Tput));
    box->AddValue(static_cast<float>(g_latestMaxTput));
    box->AddValue(static_cast<float>(g_latestJfi));
    box->AddValue(static_cast<float>(g_latestCoverage));
    box->AddValue(static_cast<float>(g_currentTiltDeg[0]));
    box->AddValue(static_cast<float>(g_currentTiltDeg[1]));
    box->AddValue(static_cast<float>(g_currentTiltDeg[2]));

    NS_LOG_INFO("[AI-OBS] Sending observation: time=" << g_latestTime 
                << " total=" << g_latestTotalTput 
                << " jfi=" << g_latestJfi
                << " tilts=[" << g_currentTiltDeg[0] << "," << g_currentTiltDeg[1] << "," << g_currentTiltDeg[2] << "]");

    return box;
}

float
RetAiEnv::GetReward()
{
    // Reward = weighted combination of throughput and fairness
    // Normalize: throughput ~100 Mbps, JFI ~1.0
    float reward = static_cast<float>(g_latestTotalTput / 100.0) + static_cast<float>(g_latestJfi);
    return reward;
}

std::string
RetAiEnv::GetExtraInfo()
{
    return "";
}

bool
RetAiEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    Ptr<OpenGymBoxContainer<uint32_t>> box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
    
    uint32_t tilt0 = box->GetValue(0);
    uint32_t tilt1 = box->GetValue(1);
    uint32_t tilt2 = box->GetValue(2);
    
    NS_LOG_INFO("[AI-ACTION] Received action: tilts=[" << tilt0 << "," << tilt1 << "," << tilt2 << "]");
    
    // Apply new tilts to all gNBs
    ApplyRet(0, static_cast<double>(tilt0));
    ApplyRet(1, static_cast<double>(tilt1));
    ApplyRet(2, static_cast<double>(tilt2));
    
    g_aiReady = true;
    return true;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    // Set default values
    g_numUE                        = 10;
    g_simDuration                  = 15.0;
    g_isdM                         = 400.0;
    g_gnbHeightM                   = 25.0;
    g_sampleIntervalS              = 1.0;
    g_decisionEpochS               = 3.0;
    g_coverageTputThresholdMbps    = 20;

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUE", "Number of UEs", g_numUE);
    cmd.AddValue("simDuration", "Simulation duration (s)", g_simDuration);
    cmd.AddValue("isdM", "Inter-site distance (m)", g_isdM);
    cmd.AddValue("gnbHeightM", "gNB height (m)", g_gnbHeightM);
    cmd.AddValue("sampleIntervalS", "Sample interval (s)", g_sampleIntervalS);
    cmd.AddValue("decisionEpochS", "AI decision interval (s)", g_decisionEpochS);
    cmd.AddValue("coverageTputThresholdMbps", "Coverage threshold (Mbps)", g_coverageTputThresholdMbps);
    cmd.AddValue("useAI", "Use AI for tilt optimization (true/false)", g_useAI);
    cmd.AddValue("fixedTilt", "Fixed tilt if AI disabled", g_fixedTilt);
    cmd.AddValue("runName", "Run name for output directory", g_runName);
    cmd.AddValue("outputDir", "Parent output directory", g_outputDir);
    cmd.AddValue("scenario", "Scenario preset: Edge, Urban, Rural", g_scenario);
    cmd.AddValue("ueXMin", "UE X min bound (m)", g_ueBoundsXMin);
    cmd.AddValue("ueXMax", "UE X max bound (m)", g_ueBoundsXMax);
    cmd.AddValue("ueYMin", "UE Y min bound (m)", g_ueBoundsYMin);
    cmd.AddValue("ueYMax", "UE Y max bound (m)", g_ueBoundsYMax);

    cmd.Parse(argc, argv);

    LogComponentEnable("NrRetAiSimulation", LOG_LEVEL_INFO);

    // Generate timestamped output subdirectory
    std::time_t now = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    g_outputSubdir = g_outputDir + "/" + g_runName + "_" + timestamp;
    
    mkdir(g_outputSubdir.c_str(), 0755);
    
    g_csvSummaryFileName = g_outputSubdir + "/ret_summary.csv";
    g_csvPerUeFileName = g_outputSubdir + "/ret_per_ue.csv";
    g_uePositionsFileName = g_outputSubdir + "/ue_positions.csv";
    g_logFileName = g_outputSubdir + "/log.txt";
    
    g_logFile.open(g_logFileName);
    g_logFile << "========================================\n";
    g_logFile << "   AI-Driven RET Simulation Log\n";
    g_logFile << "========================================\n\n";
    g_logFile << "Timestamp: " << timestamp << "\n";
    g_logFile << "AI Enabled: " << (g_useAI ? "Yes" : "No") << "\n";
    g_logFile << "Decision Epoch: " << g_decisionEpochS << "s\n\n";
    
    // Apply scenario preset
    if (g_scenario == "Edge") {
        g_numUE = 50;
        g_isdM = 200.0;
        g_gnbHeightM = 25.0;
        g_coverageTputThresholdMbps = 20;
        g_ueRingPlacement = true;
        g_ueRingInnerRadiusM = 70;
        g_ueRingOuterRadiusM = 110.0;
    } else if (g_scenario == "Urban") {
        g_numUE = 50;
        g_isdM = 500.0;
        g_gnbHeightM = 30.0;
        g_ueBoundsXMin = -150.0; g_ueBoundsXMax = 150.0;
        g_ueBoundsYMin = -150.0; g_ueBoundsYMax = 150.0;
        g_coverageTputThresholdMbps = 50;
    } else if (g_scenario == "Rural") {
        g_numUE = 50;
        g_isdM = 1500.0;
        g_gnbHeightM = 40.0;
        g_ueBoundsXMin = -650.0; g_ueBoundsXMax = 650.0;
        g_ueBoundsYMin = -650.0; g_ueBoundsYMax = 650.0;
        g_coverageTputThresholdMbps = 10;
    }

    // Initialize UE state vectors
    g_sinks.resize(g_numUE);
    g_prevBytes.resize(g_numUE, 0);
    g_latestSinr_dB.resize(g_numUE, 0.0);
    g_latestRsrp_dBm.resize(g_numUE, -140.0);

    // Initialize tilts
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        g_currentTiltDeg[i] = g_fixedTilt;
    }

    // Open CSV files
    g_csvSummary.open(g_csvSummaryFileName);
    g_csvSummary << "time_s"
                 << ",total_tput_Mbps,avg_tput_Mbps,min_tput_Mbps"
                 << ",p5_tput_Mbps,max_tput_Mbps"
                 << ",jfi,coverage_pct";
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_csvSummary << ",gnb" << i << "_tilt_deg";
    g_csvSummary << "\n";

    g_csvPerUe.open(g_csvPerUeFileName);
    g_csvPerUe << "time_s,ue_id,tput_Mbps,sinr_dB,rsrp_dBm";
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_csvPerUe << ",gnb" << i << "_tilt_deg";
    g_csvPerUe << "\n";

    // Nodes
    NodeContainer gnbNodes;  gnbNodes.Create(NUM_GNB);
    NodeContainer ueNodes;   ueNodes.Create(g_numUE);

    // Mobility
    MobilityHelper gnbMob;
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMob.Install(gnbNodes);
    PlaceGnbs(gnbNodes);

    MobilityHelper ueMob;
    ueMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    if (g_ueRingPlacement)
    {
        ueMob.Install(ueNodes);
        PlaceUesInRings(ueNodes, gnbNodes);
    }
    else
    {
        std::ostringstream xVar, yVar;
        xVar << "ns3::UniformRandomVariable[Min=" << g_ueBoundsXMin << "|Max=" << g_ueBoundsXMax << "]";
        yVar << "ns3::UniformRandomVariable[Min=" << g_ueBoundsYMin << "|Max=" << g_ueBoundsYMax << "]";
        
        ueMob.SetPositionAllocator(
            "ns3::RandomBoxPositionAllocator",
            "X", StringValue(xVar.str()),
            "Y", StringValue(yVar.str()),
            "Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));
        ueMob.Install(ueNodes);
    }

    // NR helpers
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));
    bfHelper->SetAttribute("BeamformingPeriodicity",
                           TimeValue(Seconds(g_simDuration + 1000.0)));

    // Channel helper
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");

    // Band / BWP
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 100e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaPF::GetTypeId());

    // Antenna config
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaHorizontalSpacing", DoubleValue(0.5));
    nrHelper->SetGnbAntennaAttribute("AntennaVerticalSpacing",   DoubleValue(0.5));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<ThreeGppAntennaModel>()));

    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(43.0));
    nrHelper->SetUePhyAttribute ("TxPower", DoubleValue(23.0));

    // Install devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs  = nrHelper->InstallUeDevice(ueNodes,   allBwps);

    // Harvest UPA handles
    HarvestAntennaHandles(gnbDevs, nrHelper);

    // IP stack
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Attach
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);
    
    std::string mapping_UE_To_gNB = "";
    for(uint32_t i = 0; i < g_numUE; i++) {
        Ptr<NrUeNetDevice> ueNetDevice = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<const NrGnbNetDevice> targetGnb = ueNetDevice->GetTargetGnb();
        uint32_t gnbIdx = 0;
        for (uint32_t g = 0; g < NUM_GNB; g++) {
            Ptr<NetDevice> gnbDev = gnbDevs.Get(g);
            if (gnbDev == targetGnb) {
                gnbIdx = g;
                break;
            }
        }
        mapping_UE_To_gNB += std::to_string(gnbIdx) + ",";
    }
    NS_LOG_INFO("[INIT] UE Mapping: " << mapping_UE_To_gNB);

    // Write UE positions CSV
    g_uePositions.open(g_uePositionsFileName);
    g_uePositions << "ue_id,x_m,y_m,z_m,gnb_id\n";
    for(uint32_t i = 0; i < g_numUE; i++) {
        Ptr<MobilityModel> ueMobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector uePos = ueMobility->GetPosition();
        
        Ptr<NrUeNetDevice> ueNetDevice = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<const NrGnbNetDevice> targetGnb = ueNetDevice->GetTargetGnb();
        
        uint32_t gnbIdx = 0;
        for (uint32_t g = 0; g < NUM_GNB; g++) {
            Ptr<NetDevice> gnbDev = gnbDevs.Get(g);
            if (gnbDev == targetGnb) {
                gnbIdx = g;
                break;
            }
        }
        
        g_uePositions << i << "," << uePos.x << "," << uePos.y << "," << uePos.z << "," << gnbIdx << "\n";
    }
    g_uePositions.close();

    // Remote host + routing
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    InternetStackHelper remoteStack;
    remoteStack.Install(remoteHostContainer);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("5ms"));
    NetDeviceContainer internetDevs =
        p2p.Install(epcHelper->GetPgwNode(), remoteHostContainer.Get(0));

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4h.Assign(internetDevs);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(
            remoteHostContainer.Get(0)->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // DL UDP traffic + PacketSink installation
    uint16_t dlPort = 1234;

    Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable>();
    RV->SetAttribute("Min", DoubleValue(1.0));
    RV->SetAttribute("Max", DoubleValue(2.0));

    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval",   TimeValue(Seconds(0.0001)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1000));

        double randomStart = RV->GetValue();

        ApplicationContainer dlApps = dlClient.Install(remoteHostContainer.Get(0));
        dlApps.Start(Seconds(randomStart));
        dlApps.Stop (Seconds(g_simDuration));

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApps = dlSink.Install(ueNodes.Get(u));
        sinkApps.Start(Seconds(randomStart - 1));
        sinkApps.Stop (Seconds(g_simDuration));

        g_sinks[u] = DynamicCast<PacketSink>(sinkApps.Get(0));
    }

    // Connect PHY traces
    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(u), 0);

        uePhy->TraceConnectWithoutContext(
            "DlDataSinr",
            MakeBoundCallback(&DlSinrCallback, u));

        uePhy->TraceConnectWithoutContext(
            "ReportRsrp",
            MakeBoundCallback(&RsrpCallback, u));
    }

    // ── Create AI Environment ───────────────────────────────────────────────
    if (g_useAI)
    {
        g_retEnv = CreateObject<RetAiEnv>();
        std::cout << "[AI-DEBUG] Created RetAiEnv - waiting for Python agent connection..." << std::endl;
    }

    // Apply initial tilts at t=1ms
    NS_LOG_INFO("=== Initial Tilt Configuration ===");
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        Simulator::Schedule(MilliSeconds(1), &ApplyRet, i, g_fixedTilt);
    }

    // Start metric sampling at t=2.5s
    Simulator::Schedule(Seconds(2.5), &SampleMetrics);

    // Schedule first AI decision at t=5s (after initial metrics collected)
    if (g_useAI)
    {
        Simulator::Schedule(Seconds(g_decisionEpochS), &RequestAiDecision);
        std::cout << "[AI-DEBUG] First AI decision scheduled at t=" << g_decisionEpochS << "s" << std::endl;
    }

    // Run simulation
    Simulator::Stop(Seconds(g_simDuration));
    std::cout << "[AI-DEBUG] Starting Simulator::Run() with " << g_numUE << " UEs for " << g_simDuration << "s" << std::endl;
    std::time_t start = std::time(nullptr);
    char buf1[64];
    std::strftime(buf1, sizeof(buf1), "%Y-%m-%d %H:%M:%S", std::localtime(&start));
    NS_LOG_INFO("Running: " 
            << "AI=" << (g_useAI ? "Enabled" : "Disabled")
            << "  " << NUM_GNB << " gNBs, " 
            << g_numUE << " UEs, "
            << "duration=" << g_simDuration << "s, " 
            << "ISD="<< g_isdM << " m, "
            << "decisionEpoch=" << g_decisionEpochS << "s\n"
            <<"  Output dir: "<<g_outputSubdir<<"\n"
            <<"  Start Time:" <<buf1);
    
    Simulator::Run();
    Simulator::Destroy();

    // Close files
    g_csvSummary.close();
    g_csvPerUe.close();

    NS_LOG_INFO("Done. Results written to "<< g_csvSummaryFileName<<" and "<<g_csvPerUeFileName);
    
    now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    NS_LOG_INFO("Completed at time " << buf);
    
    g_logFile << "\n========================================\n";
    g_logFile << "   Simulation Completed\n";
    g_logFile << "========================================\n\n";
    g_logFile << "Completion Time: " << buf << "\n";
    g_logFile.close();

    // Notify simulation end to Python
    if (g_retEnv)
    {
        g_retEnv->NotifySimulationEnd();
    }

    return 0;
}
