/* =============================================================================
 * Multi-Cell Remote Electrical Tilt (RET) Simulation — with measurements
 * ns-3.46 + 5G-LENA v4.1.1
 * 5 gNBs | 10 UEs | Independent per-tower electrical tilt control
 *
 * Metrics collected every SAMPLE_INTERVAL_S seconds:
 *   • Per-UE DL throughput (Mbps)
 *   • Total / average / min / 5th-pct / max throughput
 *   • Jain's Fairness Index  JFI = (Σx)² / (n·Σx²)
 *   • Coverage %  (UEs above COVERAGE_TPUT_THRESHOLD_MBPS)
 *   • Latest DL Data SINR per UE (dB)   — via NrUePhy::DlDataSinr trace
 *   • Latest RSRP per UE (dBm)          — via NrUePhy::ReportRsrp trace
 *
 * Output files (written to working directory):
 *   ret_summary.csv   — one row per sample interval
 *   ret_per_ue.csv    — per-UE row per sample interval
 * =============================================================================
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRetSimulation");

// ── Simulation parameters ─────────────────────────────────────────────────────
static constexpr uint32_t NUM_GNB                       = 5;
static constexpr uint32_t NUM_UE                        = 10;
static constexpr double   SIM_DURATION_S                = 40.0;
static constexpr double   ISD_M                         = 500.0;
static constexpr double   GNB_HEIGHT_M                  = 25.0;
static constexpr double   SAMPLE_INTERVAL_S             = 1.0;
static constexpr double   COVERAGE_TPUT_THRESHOLD_MBPS  = 0.5;  // coverage gate

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

// Measurement state per UE
std::vector<Ptr<PacketSink>> g_sinks(NUM_UE, nullptr);
std::vector<uint64_t>        g_prevBytes(NUM_UE, 0);
std::vector<double>          g_latestSinr_dB(NUM_UE, 0.0);   // last DlDataSinr
std::vector<double>          g_latestRsrp_dBm(NUM_UE, -140.0); // last RSRP

// CSV output handles
std::ofstream g_csvSummary;
std::ofstream g_csvPerUe;

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
    std::vector<double> tput(NUM_UE, 0.0);
    for (uint32_t u = 0; u < NUM_UE; ++u)
    {
        if (!g_sinks[u]) continue;
        uint64_t bytes   = g_sinks[u]->GetTotalRx();
        uint64_t delta   = bytes - g_prevBytes[u];
        g_prevBytes[u]   = bytes;
        tput[u]          = static_cast<double>(delta) * 8.0 / (SAMPLE_INTERVAL_S * 1e6); // Mbps
    }

    // ── Aggregate metrics ────────────────────────────────────────────────────
    double sumT  = std::accumulate(tput.begin(), tput.end(), 0.0);
    double sumT2 = std::inner_product(tput.begin(), tput.end(), tput.begin(), 0.0);
    double avgT  = sumT / NUM_UE;

    // JFI = (Σx)² / (n·Σx²)  — 1.0 = perfectly fair, 1/n = worst case
    double jfi = (sumT2 > 0.0) ? (sumT * sumT) / (NUM_UE * sumT2) : 1.0;

    // Coverage: fraction of UEs above threshold
    uint32_t covered = 0;
    for (double t : tput) if (t >= COVERAGE_TPUT_THRESHOLD_MBPS) ++covered;
    double coveragePct = 100.0 * covered / NUM_UE;

    // 5th-percentile throughput (edge UE proxy)
    std::vector<double> sorted = tput;
    std::sort(sorted.begin(), sorted.end());
    double p5  = sorted[std::max(0, static_cast<int>(0.05 * NUM_UE + 0.5) - 1)];
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
    for (uint32_t u = 0; u < NUM_UE; ++u)
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
    if (now + SAMPLE_INTERVAL_S < SIM_DURATION_S)
        Simulator::Schedule(Seconds(SAMPLE_INTERVAL_S), &SampleMetrics);
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
    const double radius = ISD_M / (2.0 * std::sin(M_PI / NUM_GNB));
    for (uint32_t i = 0; i < NUM_GNB; ++i)
    {
        double angle = (2.0 * M_PI / NUM_GNB) * i - M_PI / 2.0;
        gnbs.Get(i)->GetObject<MobilityModel>()->SetPosition(
            Vector(radius * std::cos(angle),
                   radius * std::sin(angle),
                   GNB_HEIGHT_M));
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    double simDuration = SIM_DURATION_S;
    CommandLine cmd(__FILE__);
    cmd.AddValue("simDuration", "Simulation duration (s)", simDuration);
    cmd.Parse(argc, argv);

    LogComponentEnable("NrRetSimulation", LOG_LEVEL_INFO);

    // Initialise current-tilt tracker
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_currentTiltDeg[i] = INITIAL_TILT_DEG[i];

    // ── Open CSV files ────────────────────────────────────────────────────────
    g_csvSummary.open("ret_summary.csv");
    g_csvSummary << "time_s"
                 << ",total_tput_Mbps,avg_tput_Mbps,min_tput_Mbps"
                 << ",p5_tput_Mbps,max_tput_Mbps"
                 << ",jfi,coverage_pct";
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_csvSummary << ",gnb" << i << "_tilt_deg";
    g_csvSummary << "\n";

    g_csvPerUe.open("ret_per_ue.csv");
    g_csvPerUe << "time_s,ue_id,tput_Mbps,sinr_dB,rsrp_dBm";
    for (uint32_t i = 0; i < NUM_GNB; ++i)
        g_csvPerUe << ",gnb" << i << "_tilt_deg";
    g_csvPerUe << "\n";

    // ── Nodes ─────────────────────────────────────────────────────────────────
    NodeContainer gnbNodes;  gnbNodes.Create(NUM_GNB);
    NodeContainer ueNodes;   ueNodes.Create(NUM_UE);

    // ── Mobility ──────────────────────────────────────────────────────────────
    MobilityHelper gnbMob;
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMob.Install(gnbNodes);
    PlaceGnbs(gnbNodes);

    MobilityHelper ueMob;
    ueMob.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Bounds",   RectangleValue(Rectangle(-400, 400, -400, 400)),
        "Speed",    StringValue("ns3::ConstantRandomVariable[Constant=3.0]"),
        "Distance", DoubleValue(50.0));
    ueMob.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=-350|Max=350]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=-350|Max=350]"),
        "Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));
    ueMob.Install(ueNodes);

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
                           TimeValue(Seconds(simDuration + 1000.0)));

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
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));
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
    for (uint32_t u = 0; u < NUM_UE; ++u)
    {
        UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval",   TimeValue(MilliSeconds(10)));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1000));
        ApplicationContainer dlApps = dlClient.Install(remoteHostContainer.Get(0));
        dlApps.Start(Seconds(1.0));
        dlApps.Stop (Seconds(simDuration));

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApps = dlSink.Install(ueNodes.Get(u));
        sinkApps.Start(Seconds(0.5));
        sinkApps.Stop (Seconds(simDuration));

        // Store sink pointer for throughput polling
        g_sinks[u] = DynamicCast<PacketSink>(sinkApps.Get(0));
    }

    // ── Connect PHY traces for SINR and RSRP ─────────────────────────────────
    for (uint32_t u = 0; u < NUM_UE; ++u)
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

    // ── Start metric sampling at t=1.5s (traffic starts at 1.0s) ────────────
    Simulator::Schedule(Seconds(1.5), &SampleMetrics);

    // ── Enable ns-3/NR built-in traces (RxPacketTrace, etc.) ─────────────────
    nrHelper->EnableTraces();

    // ── Run ───────────────────────────────────────────────────────────────────
    Simulator::Stop(Seconds(simDuration));
    NS_LOG_INFO("Running: " << NUM_GNB << " gNBs, " << NUM_UE
                << " UEs, duration=" << simDuration << "s");
    Simulator::Run();
    Simulator::Destroy();

    // ── Close CSV files ───────────────────────────────────────────────────────
    g_csvSummary.close();
    g_csvPerUe.close();

    NS_LOG_INFO("Done. Results written to ret_summary.csv and ret_per_ue.csv");
    return 0;
}