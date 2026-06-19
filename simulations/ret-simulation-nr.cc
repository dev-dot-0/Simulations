/* 

Modular simulation,
set NUM_UE, SIM_DURATION_S, ISD_M, GNB_HEIGHT_M, SAMPLE_INTERVAL_S and
COVERAGE_TPUT_THRESHOLD_MBPS for a simulation
select scenario to simulate (Edge, Urban, Rural)
also log file output path

*/

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

NS_LOG_COMPONENT_DEFINE("NrRetSimulation");

// ── Simulation parameters ─────────────────────────────────────────────────────
static constexpr uint32_t NUM_GNB                       = 5;  // Fixed, cannot be changed via CLI

// Runtime-configurable global parameters (defaults below, can be overridden via CLI)
static uint32_t g_numUE                        = 10;
static double   g_simDuration                  = 40.0;
static double   g_isdM                         = 500.0;
static double   g_gnbHeightM                   = 25.0;
static double   g_sampleIntervalS              = 1.0;
static double   g_coverageTputThresholdMbps    = 0.5;  // coverage gate

// UE placement bounds (defaults: ±350m square)
static double   g_ueBoundsXMin                 = -350.0;
static double   g_ueBoundsXMax                 = 350.0;
static double   g_ueBoundsYMin                 = -350.0;
static double   g_ueBoundsYMax                 = 350.0;

// Ring placement parameters (for Edge scenario - cell-edge UEs)
static double   g_ueRingInnerRadiusM           = 120.0;  // Inner boundary of ring around gNB
static double   g_ueRingOuterRadiusM           = 160.0;  // Outer boundary of ring around gNB
static bool     g_ueRingPlacement              = false;  // true = ring placement, false = rectangular

// Output directory configuration
static std::string   g_runName = "run";         // Run identifier for output directory
static std::string   g_outputDir = ".";         // Parent output directory (default: current dir)
static std::string   g_outputSubdir = "";       // Generated subdir with timestamp
static std::string   g_csvSummaryFileName = ""; // Full path to summary CSV
static std::string   g_csvPerUeFileName = "";   // Full path to per-UE CSV
static std::string   g_uePositionsFileName = "";// Full path to UE positions CSV

// Scenario preset: "custom", "Edge", "Urban", "Rural"
std::string g_scenario = "custom";
// Per-gNB initial electrical downtilt (degrees)
static const std::array<double, NUM_GNB> INITIAL_TILT_DEG = {4.0, 6.0, 5.0, 7.0, 3.0};

// Per-gNB sector bearing (degrees)
static const std::array<double, NUM_GNB> BEARING_DEG = {180.0, 252.0, 324.0, 36.0, 108.0};

// Per-gNB independent RET schedule: { time_s, new_electrical_tilt_deg }
static const std::array<std::vector<std::pair<double, double>>, NUM_GNB> RET_SCHEDULE = {{
    /* gNB 0 */ {{ 8.0, 10.0}, {20.0,  5.0}, {32.0, 12.0}},
    /* gNB 1 */ {{ 5.0,  2.0}, {15.0,  8.0}, {28.0,  4.0}},
    /* gNB 2 */ {{10.0, 14.0}, {22.0,  6.0}},
    /* gNB 3 */ {{ 6.0,  9.0}, {18.0,  3.0}, {30.0, 11.0}},
    /* gNB 4 */ {{12.0,  7.0}, {25.0, 13.0}},
}};

// ── Global state ──────────────────────────────────────────────────────────────
std::vector<Ptr<UniformPlanarArray>> g_gnbAntennas(NUM_GNB, nullptr);
std::array<double, NUM_GNB>          g_currentTiltDeg;   // live tilt per gNB

// Measurement state per UE (will be resized in main() after parsing CLI)
std::vector<Ptr<PacketSink>> g_sinks;
std::vector<uint64_t>        g_prevBytes;
std::vector<double>          g_latestSinr_dB;
std::vector<double>          g_latestRsrp_dBm;

// CSV output handles
std::ofstream g_csvSummary;
std::ofstream g_csvPerUe;
std::ofstream g_uePositions;

// ── SINR trace callback ───────────────────────────────────────────────────────
// Signature: DlDataSinrTracedCallback(cellId, rnti, sinr_linear, bwpId)
// We bind ueIdx at connection time.
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
// Signature: TracedCallback<uint16_t cellId, uint16_t imsi, uint16_t rnti,
//                           double rsrp_dBm, uint8_t bwpId>
void RsrpCallback(uint32_t ueIdx,
                  uint16_t /*cellId*/,
                  uint16_t /*imsi*/,
                  uint16_t /*rnti*/,
                  double   rsrp_dBm,
                  uint8_t  /*bwpId*/)
{
    g_latestRsrp_dBm[ueIdx] = rsrp_dBm;
}

// ── Metric sampling ───────────────────────────────────────────────────────────
void SampleMetrics()
{
    double now = Simulator::Now().GetSeconds();

    // ── Per-UE throughput ────────────────────────────────────────────────────
    std::vector<double> tput(g_numUE, 0.0);
    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        if (!g_sinks[u]) continue;
        uint64_t bytes   = g_sinks[u]->GetTotalRx();
        uint64_t delta   = bytes - g_prevBytes[u];
        g_prevBytes[u]   = bytes;
        tput[u]          = static_cast<double>(delta) * 8.0 / (g_sampleIntervalS * 1e6); // Mbps
    }

    // ── Aggregate metrics ────────────────────────────────────────────────────
    double sumT  = std::accumulate(tput.begin(), tput.end(), 0.0);
    double sumT2 = std::inner_product(tput.begin(), tput.end(), tput.begin(), 0.0);
    double avgT  = sumT / g_numUE;

    // JFI = (Σx)² / (n·Σx²)  — 1.0 = perfectly fair, 1/n = worst case
    double jfi = (sumT2 > 0.0) ? (sumT * sumT) / (g_numUE * sumT2) : 1.0;

    // Coverage: fraction of UEs above threshold
    uint32_t covered = 0;
    for (double t : tput) if (t >= g_coverageTputThresholdMbps) ++covered;
    double coveragePct = 100.0 * covered / g_numUE;

    // 5th-percentile throughput (edge UE proxy)
    std::vector<double> sorted = tput;
    std::sort(sorted.begin(), sorted.end());
    double p5  = sorted[std::max(0, static_cast<int>(0.05 * g_numUE + 0.5) - 1)];
    double minT = sorted.front();
    double maxT = sorted.back();

    // ── Current tilt snapshot ─────────────────────────────────────────────────
    std::string tiltSnap;
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        tiltSnap += std::to_string(static_cast<int>(g_currentTiltDeg[i]));
        if (i < NUM_GNB - 1) tiltSnap += "|";
    }

    // ── Write summary CSV row ─────────────────────────────────────────────────
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

    // ── Write per-UE CSV rows ─────────────────────────────────────────────────
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

    // ── Console log ──────────────────────────────────────────────────────────
    NS_LOG_INFO("[METRIC] t=" << now << "s"
                << "  totalTput=" << std::fixed << std::setprecision(2) << sumT << "Mbps"
                << "  avgTput="   << avgT  << "Mbps"
                << "  minTput="   << minT  << "Mbps"
                << "  p5Tput="    << p5    << "Mbps"
                << "  JFI="       << std::setprecision(3) << jfi
                << "  coverage="  << std::setprecision(0) << coveragePct << "%"
                << "  tilts=["    << tiltSnap << "]");

    // Schedule next sample (stop before simulation end to avoid overshoot)
    if (now + g_sampleIntervalS < g_simDuration)
        Simulator::Schedule(Seconds(g_sampleIntervalS), &SampleMetrics);
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

        // Set mechanical bearing via SetAlpha() — bypasses the [-π,π] checker
        // which rejects exactly 180° (= π rad) at the boundary.
        // Mechanical tilt stays at 0; all tilt is done electrically.
        upa->SetAlpha(DegreesToRadians(BEARING_DEG[i]));
        upa->SetBeta(0.0);

        g_gnbAntennas[i] = upa;
        NS_LOG_INFO("[INIT] gNB[" << i << "] UPA acquired"
                    << "  mech_bearing=" << BEARING_DEG[i] << "deg");
    }


}

// ── Pentagon gNB placement ────────────────────────────────────────────────────
void PlaceGnbs(NodeContainer& gnbs)
{
    const double radius = g_isdM / (2.0 * std::sin(M_PI / NUM_GNB));
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        double angle = (2.0 * M_PI / NUM_GNB) * i - M_PI / 2.0;
        gnbs.Get(i)->GetObject<MobilityModel>()->SetPosition(
            Vector(radius * std::cos(angle),
                   radius * std::sin(angle),
                   g_gnbHeightM));
    }
    NS_LOG_INFO("[gNB] Placed "<<NUM_GNB<<" gNBs in N-gon");
}

// ── Ring-based UE placement for cell-edge testing ─────────────────────────────
void PlaceUesInRings(NodeContainer& ues, const NodeContainer& gnbs)
{
    // Create random variables for ring placement
    Ptr<UniformRandomVariable> radiusVar = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> angleVar  = CreateObject<UniformRandomVariable>();
    
    // For uniform distribution within ring, use sqrt() on area
    double innerR2 = g_ueRingInnerRadiusM * g_ueRingInnerRadiusM;
    double outerR2 = g_ueRingOuterRadiusM * g_ueRingOuterRadiusM;
    
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        // Each UE is assigned to a gNB (round-robin by closest attachment logic)
        uint32_t gnbIdx = i % NUM_GNB;
        Vector gnbPos = gnbs.Get(gnbIdx)->GetObject<MobilityModel>()->GetPosition();
        
        // Random radius within ring (sqrt for uniform area distribution)
        double r = std::sqrt(radiusVar->GetValue(innerR2, outerR2));
        
        // Random angle [0, 2π)
        double theta = angleVar->GetValue(0.0, 2.0 * M_PI);
        
        // Position relative to gNB
        double x = gnbPos.x + r * std::cos(theta);
        double y = gnbPos.y + r * std::sin(theta);
        double z = 1.5;  // UE height
        
        ues.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(x, y, z));
    }
    
    NS_LOG_INFO("[UE] Placed " << ues.GetN() << " UEs in rings around gNBs "
                << "(inner=" << g_ueRingInnerRadiusM << "m, outer=" << g_ueRingOuterRadiusM << "m)");
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    // Set default values for global parameters
    g_numUE                        = 10;
    g_simDuration                  = 40.0;
    g_isdM                         = 500.0;
    g_gnbHeightM                   = 25.0;
    g_sampleIntervalS              = 1.0;
    g_coverageTputThresholdMbps    = 0.5;

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUE", "Number of UEs", g_numUE);
    cmd.AddValue("simDuration", "Simulation duration (s)", g_simDuration);
    cmd.AddValue("isdM", "Inter-site distance (m)", g_isdM);
    cmd.AddValue("gnbHeightM", "gNB height (m)", g_gnbHeightM);
    cmd.AddValue("sampleIntervalS", "Sample interval (s)", g_sampleIntervalS);
    cmd.AddValue("coverageTputThresholdMbps", "Coverage threshold (Mbps)", g_coverageTputThresholdMbps);

    // Output directory configuration
    cmd.AddValue("runName", "Run name for output directory", g_runName);
    cmd.AddValue("outputDir", "Parent output directory", g_outputDir);

    // Scenario preset: "custom", "Edge", "Urban", "Rural"
    cmd.AddValue("scenario", "Scenario preset: Edge, Urban, Rural (overrides other params)", g_scenario);

    // UE bounds (can be set directly or via scenario preset)
    cmd.AddValue("ueXMin", "UE X min bound (m)", g_ueBoundsXMin);
    cmd.AddValue("ueXMax", "UE X max bound (m)", g_ueBoundsXMax);
    cmd.AddValue("ueYMin", "UE Y min bound (m)", g_ueBoundsYMin);
    cmd.AddValue("ueYMax", "UE Y max bound (m)", g_ueBoundsYMax);

    cmd.Parse(argc, argv);

    LogComponentEnable("NrRetSimulation", LOG_LEVEL_INFO);
    // Config::SetDefault("ns3::NrQosFlowToRlcMapping", EnumValue(ns3::NrGnbRrc::RLC_UM_ALWAYS));
    // Config::SetDefault(ns3::ThreeGppChannelConditionModel::m_updatePeriod)
    
    
    // Generate timestamped output subdirectory
    std::time_t now = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    g_outputSubdir = g_outputDir + "/" + g_runName + "_" + timestamp;
    
    // Create output directory
    mkdir(g_outputSubdir.c_str(), 0755);
    
    // Build full file paths
    g_csvSummaryFileName = g_outputSubdir + "/ret_summary.csv";
    g_csvPerUeFileName = g_outputSubdir + "/ret_per_ue.csv";
    g_uePositionsFileName = g_outputSubdir + "/ue_positions.csv";

    // Apply scenario preset (overrides command-line values)
    if (g_scenario == "Edge") {
        g_numUE = 50;
        g_isdM = 200.0;
        g_gnbHeightM = 25.0;
        g_ueBoundsXMin = -250.0; g_ueBoundsXMax = 250.0;  // Cell edge only
        g_ueBoundsYMin = -250.0; g_ueBoundsYMax = 250.0;
        g_coverageTputThresholdMbps = 0.2;
        g_ueRingPlacement = true;  // Enable ring placement for Edge scenario
        g_ueRingInnerRadiusM = 120.0;  // 70% of gNB radius (170m)
        g_ueRingOuterRadiusM = 160.0;  // 94% of gNB radius
    } else if (g_scenario == "Urban") {
        g_numUE = 100;
        g_isdM = 500.0;
        g_gnbHeightM = 30.0;
        g_ueBoundsXMin = -500.0; g_ueBoundsXMax = 500.0;  // Full coverage
        g_ueBoundsYMin = -500.0; g_ueBoundsYMax = 500.0;
        g_coverageTputThresholdMbps = 0.3;
    } else if (g_scenario == "Rural") {
        g_numUE = 25;
        g_isdM = 1000.0;
        g_gnbHeightM = 40.0;
        g_ueBoundsXMin = -800.0; g_ueBoundsXMax = 800.0;  // Wide area
        g_ueBoundsYMin = -800.0; g_ueBoundsYMax = 800.0;
        g_coverageTputThresholdMbps = 0.1;
    }
    // else "custom" - use CLI values

    // Initialize UE state vectors based on command-line specified numUE
    g_sinks.resize(g_numUE, nullptr);
    g_prevBytes.resize(g_numUE, 0);
    g_latestSinr_dB.resize(g_numUE, 0.0);
    g_latestRsrp_dBm.resize(g_numUE, -140.0);

    // Initialise current-tilt tracker
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_currentTiltDeg[i] = INITIAL_TILT_DEG[i];

    // ── Open CSV files ────────────────────────────────────────────────────────
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

    // ── Nodes ─────────────────────────────────────────────────────────────────
    NodeContainer gnbNodes;  gnbNodes.Create(NUM_GNB);
    NodeContainer ueNodes;   ueNodes.Create(g_numUE);

    // ── Mobility ──────────────────────────────────────────────────────────────
    MobilityHelper gnbMob;
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMob.Install(gnbNodes);
    PlaceGnbs(gnbNodes);

    MobilityHelper ueMob;
    ueMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    if (g_ueRingPlacement)
    {
        // Ring-based placement for Edge scenario (cell-edge UEs)
        ueMob.Install(ueNodes);  // Install mobility model first
        PlaceUesInRings(ueNodes, gnbNodes);  // Then place UEs in rings around gNBs
    }
    else
    {
        // Rectangular uniform random placement for Urban/Rural/Custom scenarios
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

   

    // ── NR helpers ────────────────────────────────────────────────────────────
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));
    // One-shot beamforming: period >> simDuration so it never re-runs after t=0
    bfHelper->SetAttribute("BeamformingPeriodicity",
                           TimeValue(Seconds(g_simDuration + 1000.0)));

    // ── Channel helper ────────────────────────────────────────────────────────
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");

    // ── Band / BWP ────────────────────────────────────────────────────────────
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // ── Scheduler ─────────────────────────────────────────────────────────────
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    // ── Antenna config ────────────────────────────────────────────────────────
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

    // ── Install devices ───────────────────────────────────────────────────────
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs  = nrHelper->InstallUeDevice(ueNodes,   allBwps);

    // ── Harvest UPA handles + mechanical orientation ──────────────────────────
    HarvestAntennaHandles(gnbDevs, nrHelper);

    // ── IP stack ──────────────────────────────────────────────────────────────
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // ── Attach (triggers one-shot beamforming at t=0) ─────────────────────────
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);
    
    // Print which UE is attached to which gNB
    std::string mapping_UE_To_gNB = "";
    for(uint32_t i = 0; i < g_numUE; i++) {
        Ptr<MobilityModel> ueMobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        // Vector uePos = ueMobility->GetPosition();

        Ptr<NrUeNetDevice> ueNetDevice = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<const NrGnbNetDevice> targetGnb = ueNetDevice->GetTargetGnb();
         // Find which gNB node this corresponds to
        uint32_t gnbIdx = 0;
        for (uint32_t g = 0; g < NUM_GNB; g++) {
            Ptr<NetDevice> gnbDev = gnbDevs.Get(g);
            if (gnbDev == targetGnb) {
                gnbIdx = g;
                break;
            }
        }
        mapping_UE_To_gNB += std::to_string(gnbIdx) + ",";
        NS_LOG_INFO("[INIT] UE "<<i<<"  Attached to gNB "<<gnbIdx);
    }
    NS_LOG_INFO("[INIT] UE Mappping is "<<mapping_UE_To_gNB);

    // ── Write UE positions CSV ─────────────────────────────────────────────────
    g_uePositions.open(g_uePositionsFileName);
    g_uePositions << "ue_id,x_m,y_m,z_m,gnb_id\n";
    for(uint32_t i = 0; i < g_numUE; i++) {
        Ptr<MobilityModel> ueMobility = ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector uePos = ueMobility->GetPosition();
        
        Ptr<NrUeNetDevice> ueNetDevice = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<const NrGnbNetDevice> targetGnb = ueNetDevice->GetTargetGnb();
        
        // Find which gNB node this corresponds to
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
    NS_LOG_INFO("[INIT] UE positions written to " << g_uePositionsFileName);

    // ── Remote host + routing ──────────────────────────────────────────────────
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
    (void)internetIfaces.GetAddress(1); // remoteHostAddr — suppress warning

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(
            remoteHostContainer.Get(0)->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ── DL UDP traffic + PacketSink installation ──────────────────────────────
    uint16_t dlPort = 1234;

    Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable>();
    RV->SetAttribute("Min", DoubleValue(1.0));
    RV->SetAttribute("Max", DoubleValue(5.0));

    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval",   TimeValue(Seconds(0.0001)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1000));

        // Start Applications at random intervals to not overload the Scheduler
        double randomStart = RV->GetValue();


        ApplicationContainer dlApps = dlClient.Install(remoteHostContainer.Get(0));
        dlApps.Start(Seconds(randomStart));
        dlApps.Stop (Seconds(g_simDuration));

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApps = dlSink.Install(ueNodes.Get(u));
        sinkApps.Start(Seconds(randomStart - 1));
        sinkApps.Stop (Seconds(g_simDuration));

        // Store sink pointer for throughput polling
        g_sinks[u] = DynamicCast<PacketSink>(sinkApps.Get(0));
    }

    // ── Connect PHY traces for SINR and RSRP ─────────────────────────────────
    for (uint32_t u = 0; u < g_numUE; ++u)
    {
        Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(u), 0);

        // DlDataSinr: (cellId, rnti, sinr_linear, bwpId) — bind ueIdx
        uePhy->TraceConnectWithoutContext(
            "DlDataSinr",
            MakeBoundCallback(&DlSinrCallback, u));

        // ReportRsrp: (cellId, imsi, rnti, rsrp_dBm, bwpId) — bind ueIdx
        uePhy->TraceConnectWithoutContext(
            "ReportRsrp",
            MakeBoundCallback(&RsrpCallback, u));
    }

    // ── Initial electrical tilts at t=1ms (after BF run at t=0) ─────────────
    NS_LOG_INFO("=== Initial Electrical Tilt ===");
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        Simulator::Schedule(MilliSeconds(1), &ApplyRet, i, INITIAL_TILT_DEG[i]);
        NS_LOG_INFO("  gNB[" << i << "]"
                    << "  bearing="    << BEARING_DEG[i]      << "deg"
                    << "  elec_tilt="  << INITIAL_TILT_DEG[i] << "deg");
    }

    // ── RET schedule ──────────────────────────────────────────────────────────
    NS_LOG_INFO("=== RET Event Schedule ===");
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        for (const auto& [t, tilt] : RET_SCHEDULE[i])
        {
            Simulator::Schedule(Seconds(t), &ApplyRet, i, tilt);
            NS_LOG_INFO("  gNB[" << i << "]  t=" << t << "s  tilt=" << tilt << "deg");
        }
    // ── Start metric sampling at t=5.5s Staggered Traffic times ────────────
    Simulator::Schedule(Seconds(5.5), &SampleMetrics);

    // ── Enable ns-3/NR built-in traces (RxPacketTrace, etc.) ─────────────────
    // nrHelper->EnableTraces();

    // ── Run ───────────────────────────────────────────────────────────────────
    Simulator::Stop(Seconds(g_simDuration));
    std::time_t start = std::time(nullptr);
    char buf1[64];
    std::strftime(buf1, sizeof(buf1), "%Y-%m-%d %H:%M:%S", std::localtime(&start));
    NS_LOG_INFO("Running: " 
            << "Scenario=" << g_scenario
            << "\n  " << NUM_GNB << " gNBs, " 
            << g_numUE << " UEs, "
            << "duration=" << g_simDuration << "s, " 
            << "ISD="<< g_isdM << " m, "
            << "gNB Height="<<g_gnbHeightM<< " m, "
            << "UE Bounds X=[" << g_ueBoundsXMin << ", " << g_ueBoundsXMax << "], "
            << "Y=[" << g_ueBoundsYMin << ", " << g_ueBoundsYMax << "], "
            << "Sample Interval="<<g_sampleIntervalS<< " s, "
            << "Coverage Threshold="<<g_coverageTputThresholdMbps<< " Mbps\n"
            <<"  Output dir: "<<g_outputSubdir<<"\n"
            <<"  Writing to: "<<g_csvSummaryFileName<<" (summary), "<<g_csvPerUeFileName<<" (per-UE), "<<g_uePositionsFileName<<" (UE positions)\n"
            <<"  Start Time:" <<buf1
            );
    Simulator::Run();
    Simulator::Destroy();

    // ── Close CSV files ───────────────────────────────────────────────────────
    g_csvSummary.close();
    g_csvPerUe.close();

    NS_LOG_INFO("Done. Results written to "<< g_csvSummaryFileName<<" and "<<g_csvPerUeFileName);
    
    now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    NS_LOG_INFO("Completed at time " << buf);

    return 0;
}
