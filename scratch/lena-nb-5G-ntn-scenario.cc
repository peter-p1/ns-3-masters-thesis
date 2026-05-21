/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC), 
 * Copyright (c) 2022 Communication Networks Institute at TU Dortmund University
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Jaume Nin <jaume.nin@cttc.cat>
 *          Manuel Requena <manuel.requena@cttc.es>
 *          Tim Gebauer <tim.gebauer@tu-dortmund.de> (NB-IoT Extension)
 *          Pascal Jörke <pascal.joerke@tu-dortmund.de>        
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/lte-module.h"
#include "ns3/nb-iot-rrc-sap.h"
#include "ns3/channel-condition-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/mobility-model.h"
#include "ns3/application.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/satellite-orbit-mobility-model.h"
#include "ns3/julian-date.h"
#include "nprach-detection-model.h"
#include "ntn-geometric-handover.h"
#include "ns3/lte-enb-phy.h"
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <ctime>    
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <unordered_map>

using namespace ns3;



// Global trace file for logging
std::ofstream* g_asciiTrace = nullptr;
static void WriteOut(std::ofstream* traceFile, const std::string& s, bool console)
{
  if (traceFile)
  {
    *traceFile << s << std::endl;
  }
  if (console)
  {
    std::cout << s << std::endl;
  }
}

static std::unordered_map<uint64_t,double> g_lastRsrp;
static std::vector<NbIotRrcSap::NprachParametersNb> g_ceParams;
static double g_ce1LowerBound = -110.0;
static double g_ce2LowerBound = -120.0;
static std::unordered_map<uint32_t,uint64_t> g_cellRntiToImsi;
static std::unordered_map<uint64_t,uint32_t> g_raStartCount;
static std::unordered_map<uint64_t,uint32_t> g_raSuccessCount;
static std::unordered_map<uint64_t,uint32_t> g_raTimeoutCount;
static std::unordered_map<uint64_t,double> g_imsiRttMs;
static std::unordered_map<uint64_t,double> g_imsiDistanceKm;

// NPRACH detection model state (populated from main before Simulator::Run)
static int g_nprachSetId = 3;                 // derived from enbAntennaGainDb
static int g_nprachNrep  = 1;                 // nprachRepsPerAttempt (default NRep=1)
static Ptr<UniformRandomVariable> g_nprachRng;
static Ptr<GeocentricConstantPositionMobilityModel> g_nprachEnbMob;
static std::unordered_map<uint64_t, Ptr<GeocentricConstantPositionMobilityModel>> g_imsiUeMob;
static std::unordered_map<uint64_t,uint32_t> g_nprachDetectedCount;
static std::unordered_map<uint64_t,uint32_t> g_nprachMissedCount;
static bool g_ntnHoEnabled = false;

// Per-IMSI timing for connection-procedure analysis
static std::unordered_map<uint64_t,double> g_raFirstStartSec;     // first RA start
static std::unordered_map<uint64_t,double> g_connectedSec;        // RRC ConnectionEstablished
static std::unordered_map<uint64_t,uint32_t> g_preambleTxCounter; // last reported preambleTxCounter

// Per-IMSI ping (UDP echo) RTT samples in ms; pendingTxTimes is a FIFO of TX timestamps per UE
static std::unordered_map<uint64_t, std::vector<double>> g_pingRttMs;
static std::unordered_map<uint32_t, std::vector<double>> g_pendingTxByApp;   // appId -> FIFO of tx times (s)
static std::unordered_map<uint32_t, uint64_t> g_appIdToImsi;
static std::unordered_map<uint64_t, uint32_t> g_pingTxCount;
static std::unordered_map<uint64_t, uint32_t> g_pingRxCount;

// NPRACH preamble-collision proxy: bucket attempts into NPRACH opportunities
// and count cases where 2+ UEs select the same subcarrier in the same window.
static double g_nprachOpportunityMs = 80.0;   // Format 2, NRep=1 ~ 80 ms period
static int g_nprachNumSubcarriers = 12;       // typical NPRACH subcarrier pool
static uint64_t g_collisionEventCount = 0;    // # opportunities with >=1 collision
static uint64_t g_collidedAttemptCount = 0;   // # UE attempts that collided
static std::unordered_map<int64_t, std::unordered_map<int, std::vector<uint64_t>>> g_collisionWindow;
static std::unordered_map<uint64_t,uint32_t> g_perImsiCollisions;
static inline uint32_t MakeCrKey(uint16_t cellId, uint16_t rnti){ return (uint32_t(cellId) << 16) | rnti; }
static NbIotRrcSap::NprachParametersNb::NumRepetitionsPerPreambleAttempt MapNumRepetitionsPerPreambleAttempt(int v){
  using N = NbIotRrcSap::NprachParametersNb::NumRepetitionsPerPreambleAttempt;
  switch(v){
    case 1: return N::n1;
    case 2: return N::n2;
    case 4: return N::n4;
    case 8: return N::n8;
    case 16: return N::n16;
    case 32: return N::n32;
    case 64: return N::n64;
    case 128: return N::n128;
    default: return N::n1;
  }
}

static int GetCeIndex(double rsrp)
{
  if (rsrp < g_ce2LowerBound) return 2;
  if (rsrp < g_ce1LowerBound) return 1;
  return 0;
}

static double ComputeLatitudeForRttMs(double rttMs, double satAltM, double ueAltM, double satLatDeg)
{
  double c = 299792458.0;
  double distance = (rttMs / 1000.0) * c / 2.0;
  double r = 6371000.0 + ueAltM;
  double rs = 6371000.0 + satAltM;
  double cosTheta = (r * r + rs * rs - distance * distance) / (2.0 * r * rs);
  if (cosTheta > 1.0) cosTheta = 1.0;
  if (cosTheta < -1.0) cosTheta = -1.0;
  double theta = std::acos(cosTheta);
  double lat = satLatDeg + (theta * 180.0 / M_PI);
  if (lat > 90.0) lat = 90.0;
  if (lat < -90.0) lat = -90.0;
  return lat;
}

void CaptureCurrentCellRsrpSinr(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId)
{
  uint32_t key = MakeCrKey(cellId, rnti);
  auto it = g_cellRntiToImsi.find(key);
  if (it != g_cellRntiToImsi.end())
  {
    double rsrpDbm = 10.0 * std::log10(rsrp / 1e-3);
    g_lastRsrp[it->second] = rsrpDbm;
  }
}

void LogRarRepetitionsRuntime(uint64_t imsi, int ceIdx)
{
  NbIotRrcSap::DciN0 d0;
  NbIotRrcSap::DciN1 d1;
  if (ceIdx == 0)
  {
    d0.dciRepetitions = NbIotRrcSap::DciN0::DciRepetitions::r2;
    d1.dciRepetitions = NbIotRrcSap::DciN1::DciRepetitions::r2;
  }
  else if (ceIdx == 1)
  {
    d0.dciRepetitions = NbIotRrcSap::DciN0::DciRepetitions::r32;
    d1.dciRepetitions = NbIotRrcSap::DciN1::DciRepetitions::r32;
  }
  else
  {
    d0.dciRepetitions = NbIotRrcSap::DciN0::DciRepetitions::r256;
    d1.dciRepetitions = NbIotRrcSap::DciN1::DciRepetitions::r256;
  }
  if (g_asciiTrace)
  {
    *g_asciiTrace << "[RAR_REPETITIONS] Time: " << Simulator::Now().GetSeconds()
                  << "s, IMSI: " << imsi
                  << ", CE: " << ceIdx
                  << ", DCI-N0: " << NbIotRrcSap::ConvertDciN0Repetitions2int(d0)
                  << ", DCI-N1: " << NbIotRrcSap::ConvertDciN1Repetitions2int(d1)
                  << ", NPRACH: " << NbIotRrcSap::ConvertNumRepetitionsPerPreambleAttempt2int(g_ceParams[ceIdx])
                  << ", NPDCCH-RA: " << NbIotRrcSap::ConvertNpdcchNumRepetitionsRa2int(g_ceParams[ceIdx])
                  << std::endl;
  }
}

// Custom logging functions to mimic LENA-NB behavior
void LogConnectionEstablished (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[CONNECTION_ESTABLISHED] Time: " << Simulator::Now().GetSeconds()
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
  g_cellRntiToImsi[MakeCrKey(cellId, rnti)] = imsi;
  if (g_connectedSec.find(imsi) == g_connectedSec.end())
  {
    g_connectedSec[imsi] = Simulator::Now().GetSeconds();
  }
  if (g_ntnHoEnabled) NtnGeometricHandover::OnConnected(imsi, cellId);
}

void LogStateTransition (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, 
                        LteUeRrc::State oldState, LteUeRrc::State newState)
{
  if (g_asciiTrace) *g_asciiTrace << "[STATE_TRANSITION] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti 
                                   << ", OldState: " << oldState << ", NewState: " << newState << std::endl;
  g_cellRntiToImsi[MakeCrKey(cellId, rnti)] = imsi;
}

void LogSchedulingRequest (std::string context, uint64_t imsi, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[SCHEDULING_REQUEST] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", RNTI: " << rnti << std::endl;
}

void LogRandomAccessSuccessful (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[RANDOM_ACCESS_SUCCESS] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
  g_raSuccessCount[imsi]++;
}

void LogRandomAccessError (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[RANDOM_ACCESS_ERROR] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
}

void LogRandomAccessStart (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, 
                          LteUeRrc::State oldState, LteUeRrc::State newState)
{
  if (newState == LteUeRrc::IDLE_RANDOM_ACCESS)
  {
    if (g_asciiTrace) *g_asciiTrace << "[RANDOM_ACCESS_START] Time: " << Simulator::Now().GetSeconds()
                                     << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
    g_raStartCount[imsi]++;
    if (g_raFirstStartSec.find(imsi) == g_raFirstStartSec.end())
    {
      g_raFirstStartSec[imsi] = Simulator::Now().GetSeconds();
    }
    g_cellRntiToImsi[MakeCrKey(cellId, rnti)] = imsi;
    int ceIdx = 0;
    std::unordered_map<uint64_t,double>::iterator it = g_lastRsrp.find(imsi);
    if (it != g_lastRsrp.end())
    {
      ceIdx = GetCeIndex(it->second);
    }
    LogRarRepetitionsRuntime(imsi, ceIdx);
    // NPRACH detection now enforced at eNB PHY layer (LteEnbPhy::s_rachPreambleFilter)
  }
}

void LogRaResponseTimeout (uint64_t imsi, bool contention, uint8_t preambleTxCounter, uint8_t maxPreambleTxLimit)
{
  if (g_asciiTrace) *g_asciiTrace << "[RA_RESPONSE_TIMEOUT] Time: " << Simulator::Now().GetSeconds()
                                   << "s, IMSI: " << imsi << ", Contention: " << (contention ? "Yes" : "No")
                                   << ", PreambleTxCounter: " << (uint16_t)preambleTxCounter
                                   << ", MaxPreambleTxLimit: " << (uint16_t)maxPreambleTxLimit << std::endl;
  g_raTimeoutCount[imsi]++;
  g_preambleTxCounter[imsi] = std::max<uint32_t>(g_preambleTxCounter[imsi], preambleTxCounter);
}

// Free helpers for Simulator::Schedule of the overloaded LteHelper::Attach.
static void DoAttachConstellation(Ptr<LteHelper> h, Ptr<NetDevice> ue)
{
  h->Attach(ue);
}
static void DoAttachToEnb(Ptr<LteHelper> h, Ptr<NetDevice> ue, Ptr<NetDevice> enb)
{
  h->Attach(ue, enb);
}

// UDP echo Tx/Rx tracing for ping-style RTT measurement
static void OnPingTx(uint32_t appId, Ptr<const Packet> /*p*/)
{
  g_pendingTxByApp[appId].push_back(Simulator::Now().GetSeconds());
  auto it = g_appIdToImsi.find(appId);
  if (it != g_appIdToImsi.end()) g_pingTxCount[it->second]++;
}

static void OnPingRx(uint32_t appId, Ptr<const Packet> /*p*/)
{
  auto it = g_appIdToImsi.find(appId);
  if (it == g_appIdToImsi.end()) return;
  uint64_t imsi = it->second;
  auto& fifo = g_pendingTxByApp[appId];
  if (fifo.empty()) return;
  double rttMs = (Simulator::Now().GetSeconds() - fifo.front()) * 1000.0;
  fifo.erase(fifo.begin());
  g_pingRttMs[imsi].push_back(rttMs);
  g_pingRxCount[imsi]++;
  if (g_asciiTrace) *g_asciiTrace << "[PING_RTT] Time: " << Simulator::Now().GetSeconds()
                                   << "s, IMSI: " << imsi << ", RTT: " << std::fixed
                                   << std::setprecision(2) << rttMs << " ms" << std::endl;
}

void LogCellSelectionOk (std::string context, uint64_t imsi, uint16_t cellId)
{
  if (g_asciiTrace) *g_asciiTrace << "[CELL_SELECTION_OK] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << std::endl;
}

void LogSib1Received (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[SIB1_RECEIVED] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
}

void LogTxOpportunity (std::string context, uint64_t imsi, uint16_t rnti, uint32_t bytes)
{
  if (g_asciiTrace) *g_asciiTrace << "[TX_OPPORTUNITY] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", RNTI: " << rnti << ", Bytes: " << bytes << std::endl;
}

void LogUeMeasurements (std::string context, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, bool isServingCell, uint8_t componentCarrierId)
{
  if (g_asciiTrace) *g_asciiTrace << "[UE_MEASUREMENTS] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, RNTI: " << rnti << ", CellId: " << cellId 
                                   << ", RSRP: " << rsrp << ", RSRQ: " << rsrq 
                                   << ", ServingCell: " << (isServingCell ? "Yes" : "No") << std::endl;
}

void LogEnbConnectionEstablished (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[ENB_CONNECTION_ESTABLISHED] Time: " << Simulator::Now().GetSeconds() 
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId << ", RNTI: " << rnti << std::endl;
}

void LogHandoverStart (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t otherCid)
{
  if (g_asciiTrace) *g_asciiTrace << "[HANDOVER_START] Time: " << Simulator::Now().GetSeconds()
                                   << "s, IMSI: " << imsi << ", SourceCellId: " << cellId
                                   << ", TargetCellId: " << otherCid << ", RNTI: " << rnti << std::endl;
}

void LogHandoverEndOk (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  if (g_asciiTrace) *g_asciiTrace << "[HANDOVER_END_OK] Time: " << Simulator::Now().GetSeconds()
                                   << "s, IMSI: " << imsi << ", CellId: " << cellId
                                   << ", RNTI: " << rnti << std::endl;
  g_cellRntiToImsi[MakeCrKey(cellId, rnti)] = imsi;
  if (g_ntnHoEnabled) NtnGeometricHandover::OnConnected(imsi, cellId);
}


void CalculateAndLogUeToEnbDelay(NodeContainer ueNodes, NodeContainer enbNodes, std::ofstream* traceFile)
{
  WriteOut(traceFile, "\nUE to eNB Propagation Delays (ConstantSpeedPropagationDelayModel)", true);
  
  for (uint32_t ue = 0; ue < ueNodes.GetN(); ++ue)
    {
      Ptr<GeocentricConstantPositionMobilityModel> ueMobility = ueNodes.Get(ue)->GetObject<GeocentricConstantPositionMobilityModel>();
      Vector uePos = ueMobility->GetGeocentricPosition();
      
      for (uint32_t enb = 0; enb < enbNodes.GetN(); ++enb)
        {
          Ptr<GeocentricConstantPositionMobilityModel> enbMobility = enbNodes.Get(enb)->GetObject<GeocentricConstantPositionMobilityModel>();
          Vector enbPos = enbMobility->GetGeocentricPosition();
          
          double dx = uePos.x - enbPos.x;
          double dy = uePos.y - enbPos.y;
          double dz = uePos.z - enbPos.z;
          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
          Time oneWayDelay = Seconds(distance / 299792458.0);
          
          double elevationAngle = ueMobility->GetElevationAngle(enbMobility);
          
          std::stringstream ss;
          ss << "UE " << ue << " -> eNB " << enb << ": Distance = " << std::fixed << std::setprecision(2) << distance/1000.0 << " km, "
             << "One-way delay = " << oneWayDelay.GetMilliSeconds() << " ms, "
             << "Round-trip delay = " << (oneWayDelay.GetMilliSeconds() * 2) << " ms, "
             << "Elevation angle = " << std::setprecision(3) << elevationAngle << "\u00b0";
          WriteOut(traceFile, ss.str(), true);
        }
    }
  
  WriteOut(traceFile, "", false);
}

static void LogSatellitePositionPeriodic(NodeContainer satellites, NodeContainer ueNodes, Time period)
{
  std::ostringstream ss;
  ss << "[SAT_POSITION] Time: " << Simulator::Now().GetSeconds() << "s";
  for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
      Ptr<GeocentricConstantPositionMobilityModel> m = satellites.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
      if (!m) continue;
      Vector g = m->GetGeographicPosition();
      ss << ", sat" << i << "=(lat=" << std::fixed << std::setprecision(4) << g.x
         << ", lon=" << g.y << ", alt=" << std::setprecision(1) << g.z/1000.0 << "km)";
      if (ueNodes.GetN() > 0)
        {
          Ptr<GeocentricConstantPositionMobilityModel> ueMob = ueNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>();
          if (ueMob)
            {
              double el = ueMob->GetElevationAngle(m);
              ss << std::setprecision(2) << " el=" << el << "°";
            }
        }
    }
  if (g_asciiTrace) *g_asciiTrace << ss.str() << std::endl;
  Simulator::Schedule(period, &LogSatellitePositionPeriodic, satellites, ueNodes, period);
}

void PrintSatelliteInfo (NodeContainer satellites, NodeContainer groundStations, std::ofstream* traceFile, bool console)
{
  WriteOut(traceFile, "\nNTN Satellite Configuration:", console);
  for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
      Ptr<GeocentricConstantPositionMobilityModel> satMobility = satellites.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
      Vector geo = satMobility->GetGeographicPosition();
      std::stringstream ss;
      ss << "Satellite " << i << " (eNB): Geographic (lat=" << geo.x << ", lon=" << geo.y << ", alt=" << geo.z << ")";
      WriteOut(traceFile, ss.str(), console);
    }
  WriteOut(traceFile, "\nGround UE Positions:", console);
  for (uint32_t i = 0; i < groundStations.GetN(); ++i)
    {
      Ptr<GeocentricConstantPositionMobilityModel> ueMob = groundStations.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
      Vector geo = ueMob->GetGeographicPosition();
      std::stringstream ss;
      ss << "UE " << i << ": Geographic (lat=" << geo.x << ", lon=" << geo.y << ", alt=" << geo.z << ")";
      WriteOut(traceFile, ss.str(), console);
    }
}


NS_LOG_COMPONENT_DEFINE ("LenaNb5GNTN");

struct TleEntry {
  std::string name;
  std::string line1;
  std::string line2;
};

// Parse a classic 3-line-per-satellite TLE file (NAME + line 1 + line 2).
// Blank lines and lines starting with '#' are ignored.
static std::vector<TleEntry> LoadTleFile(const std::string& path)
{
  std::vector<TleEntry> out;
  std::ifstream f(path);
  if (!f.is_open())
  {
    NS_FATAL_ERROR("Cannot open TLE file: " << path);
  }
  std::string line;
  std::vector<std::string> buf;
  while (std::getline(f, line))
  {
    if (line.empty() || line[0] == '#') continue;
    // Trim trailing CR for Windows line endings
    if (!line.empty() && line.back() == '\r') line.pop_back();
    buf.push_back(line);
    if (buf.size() == 3)
    {
      TleEntry e;
      e.name  = buf[0];
      // strip trailing whitespace from name
      while (!e.name.empty() && std::isspace(static_cast<unsigned char>(e.name.back()))) e.name.pop_back();
      e.line1 = buf[1];
      e.line2 = buf[2];
      out.push_back(e);
      buf.clear();
    }
  }
  return out;
}

int
main (int argc, char *argv[])
{
  Time simTime = Seconds(100);
  int seed = 21;
  std::string simName = "NTN-sim";
  // double cellsize = 2500; // in meters
  int num_ues = 1;
  int packetsize = 49; // in bytes
  Time packetinterval = Seconds(10);
  double ce1LowerBoundDbm = -110.0;
  double ce2LowerBoundDbm = -120.0;
  int nprachRepsPerAttempt = 0; // 0 means no override
  bool rttSweep = false;
  double rttSweepMinMs = 8.0;
  double rttSweepMaxMs = 40.0;
  double rttSweepStepMs = 4.0;
  bool useFixedRss = false;
  double fixedRssDbm = -120.0;

  int nprachFormat = 2;  // 1 = Format 1 (3.75 kHz, CP 266 us), 2 = Format 2 (1.25 kHz, CP 800 us, NTN)
  double carrierFrequencyHz = 2e9; // default 2 GHz (3GPP TR 38.821, aligned with NPRACH detection model)
  double enbAntennaGainDb = 24.0; // 3GPP TR 38.821 (Set-2)
  double ueAntennaGainDb = 0.0;   // 3GPP TR 38.821
  std::string channelConditionModel = "AlwaysLos"; // or "ThreeGppRma" - terrestrial rural-macro model

  double satelliteAltitude = 600 * 1000; // first value is in meters
  // 0 - 90; 1.598 - 80; 3.273 - 70; 5.118 - 60; 7.255 - 50; 9.862 - 40; 13.217 - 30; 17.744 - 20; 24.033 - 10
  double satelliteLatitude = 8.5; // Equatorial orbit, change this
  double satelliteLongitude = 0.0; // Prime meridian

  // Orbital (SGP4) mobility configuration. When useOrbitalModel is true, the
  // satellite eNB position is driven by SGP4 from the TLE below and the
  // satelliteLatitude/Longitude/Altitude values above are ignored.
  bool useOrbitalModel = true;
  std::string tleName   = "SATELIOT_1";
  // Default TLE is a real Sateliot NB-IoT NTN satellite fetched from celestrak
  // (epoch 2026-04-24). Sun-synchronous orbit, ~550 km altitude, 97.68° incl.
  // Replace via CLI (--tleLine1 --tleLine2) or load from scratch/tle-data/sateliot.tle
  // for alternative satellites or a 3GPP TR 38.821 reference orbit.
  std::string tleLine1  = "1 60550U 24149CL  26114.18615092  .00001182  00000+0  10549-3 0  9995";
  std::string tleLine2  = "2 60550  97.6763 190.9896 0005148 253.9991 106.0664 14.97828949 91985";
  // Absolute simulation start time (UTC). If empty, TLE epoch is used.
  std::string simStartTimeUtc = "";
  double orbitUpdateIntervalMs = 100.0;

  // Constellation / handover configuration
  // Default: 4-satellite Sateliot constellation, 24 dBi (Set-2) eNB antenna gain.
  bool useConstellation = true;
  std::string tleFile = "scratch/tle-data/sateliot.tle";
  int constellationSize = 4;

  // Per-UE start jitter so UEs don't all transmit simultaneously
  double maxStartJitterMs = 1000.0;

  // UDP-echo packets per UE. Default 1e6 = effectively unlimited (one ping
  // every packetInterval until simTime expires). Set to 1 for "single-shot"
  // tests that minimise sustained RRC churn at high UE counts.
  uint32_t maxPacketsPerUe = 1000000;

  // Heavy LTE module text-stats files (Dl/Ul {Phy,Mac,Rlc,Pdcp,RsrpSinr...}
  // Stats.txt) are not consumed by the UE-scaling analysis. DlRsrpSinrStats
  // alone reaches 200+ MB at 250 UEs / 60 s. Off by default; pass
  // --enableLteStatsLogs=true to emit them when manually inspecting a run.
  bool enableLteStatsLogs = false;
  // A3 RSRP is terrestrial-only and ill-suited for the high relative velocity
  // of LEO eNBs (ping-pong / late HO). When ntnHoEnabled=true the LTE module
  // HO algorithm is replaced by NoOp and a custom geometric (location/time-
  // based) trigger drives the X2 handover from the scenario, mirroring the
  // D1/D2/T1 conditions of 3GPP Rel-17 NTN CHO (TR 38.821 §7.3.2.1).
  std::string handoverAlgorithm = "ns3::A3RsrpHandoverAlgorithm";
  double handoverHysteresisDb = 3.0;
  double handoverTimeToTriggerMs = 256.0;
  bool useIdealRrc = true;
  bool ntnHoEnabled = false;
  double ntnHoThresh1Km = 1500.0;     // CondEvent D2 Thresh1 (Ml1 > Thresh1 -> leave)
  double ntnHoThresh2Km = 1200.0;     // CondEvent D2 Thresh2 (Ml2 < Thresh2 -> enter)
  double ntnHoEvalPeriodMs = 500.0;
  double ntnHoTimeToTriggerMs = 0.0;
  
  // UE geographic coordinates (lat, lon, alt). If alt<8km, valid as ground UE.
  double ueLatitudeDeg = 0.0;
  double ueLongitudeDeg = 0.0; 
  double ueAltitudeM = 1.5;
  
  // Command line arguments
  CommandLine cmd (__FILE__);
  cmd.AddValue ("simTime", "Total duration of the simulation", simTime);
  cmd.AddValue ("simName", "Simulation name for logging", simName);
  cmd.AddValue ("randomSeed", "Random seed", seed);
  cmd.AddValue ("numUes", "Number of UEs", num_ues);
  cmd.AddValue ("packetSize", "Packet size in bytes", packetsize);
  cmd.AddValue ("packetInterval", "Packet interval", packetinterval);
  cmd.AddValue ("frequency", "Carrier frequency in Hz", carrierFrequencyHz);
  cmd.AddValue ("enbAntennaGainDb", "eNB isotropic antenna gain in dB", enbAntennaGainDb);
  cmd.AddValue ("ueAntennaGainDb", "UE isotropic antenna gain in dB", ueAntennaGainDb);
  cmd.AddValue ("channelConditionModel", "Channel condition model: AlwaysLos or ThreeGppRma", channelConditionModel);
  cmd.AddValue ("satelliteAltitude", "Satellite altitude in meters (used only if useOrbitalModel=false)", satelliteAltitude);
  cmd.AddValue ("satelliteLatitude", "Satellite latitude in degrees (used only if useOrbitalModel=false)", satelliteLatitude);
  cmd.AddValue ("satelliteLongitude", "Satellite longitude in degrees (used only if useOrbitalModel=false)", satelliteLongitude);
  cmd.AddValue ("useOrbitalModel", "Drive satellite position from TLE via SGP4 (true) or use fixed lat/lon/alt (false)", useOrbitalModel);
  cmd.AddValue ("tleName", "Satellite name (for logging)", tleName);
  cmd.AddValue ("tleLine1", "TLE line 1", tleLine1);
  cmd.AddValue ("tleLine2", "TLE line 2", tleLine2);
  cmd.AddValue ("simStartTimeUtc", "Absolute simulation start time in UTC (e.g. '2024-01-01 00:00:00'); empty = TLE epoch", simStartTimeUtc);
  cmd.AddValue ("orbitUpdateIntervalMs", "SGP4 position update period in ms", orbitUpdateIntervalMs);
  cmd.AddValue ("useConstellation", "Load multiple satellites from tleFile (requires useOrbitalModel=true)", useConstellation);
  cmd.AddValue ("tleFile", "Path to a 3-line-per-satellite TLE file", tleFile);
  cmd.AddValue ("constellationSize", "Number of satellites to load from tleFile (0 = all)", constellationSize);
  cmd.AddValue ("handoverAlgorithm", "LTE handover algorithm type", handoverAlgorithm);
  cmd.AddValue ("handoverHysteresisDb", "A3 handover hysteresis in dB", handoverHysteresisDb);
  cmd.AddValue ("handoverTimeToTriggerMs", "A3 handover time-to-trigger in ms", handoverTimeToTriggerMs);
  cmd.AddValue ("useIdealRrc", "Use ideal RRC (required for LTE handover)", useIdealRrc);
  cmd.AddValue ("ntnHoEnabled", "Use 3GPP NTN CondEvent D2 geometric handover instead of A3 RSRP", ntnHoEnabled);
  cmd.AddValue ("ntnHoThresh1Km", "D2 Thresh1: Ml1 (UE-PCell distance, km) above which to leave", ntnHoThresh1Km);
  cmd.AddValue ("ntnHoThresh2Km", "D2 Thresh2: Ml2 (UE-candidate distance, km) below which to enter", ntnHoThresh2Km);
  cmd.AddValue ("ntnHoEvalPeriodMs", "D2 evaluation period (ms)", ntnHoEvalPeriodMs);
  cmd.AddValue ("ntnHoTimeToTriggerMs", "Time-to-trigger (ms)", ntnHoTimeToTriggerMs);
  cmd.AddValue ("ueLatitude", "UE latitude in degrees", ueLatitudeDeg);
  cmd.AddValue ("ueLongitude", "UE longitude in degrees", ueLongitudeDeg);
  cmd.AddValue ("ueAltitude", "UE altitude in meters", ueAltitudeM);
  cmd.AddValue ("ce1LowerBoundDbm", "RSRP threshold (dBm) for CE1", ce1LowerBoundDbm);
  cmd.AddValue ("ce2LowerBoundDbm", "RSRP threshold (dBm) for CE2", ce2LowerBoundDbm);
  cmd.AddValue ("nprachRepsPerAttempt", "NPRACH repetitions per preamble attempt (1,2,4,8,16,32,64,128)", nprachRepsPerAttempt);
  cmd.AddValue ("nprachFormat", "NPRACH preamble format: 1 (3.75 kHz, CP 266 us) or 2 (1.25 kHz, CP 800 us, NTN)", nprachFormat);
  cmd.AddValue ("rttSweep", "Enable RTT sweep using multiple UEs", rttSweep);
  cmd.AddValue ("rttSweepMinMs", "Minimum RTT in ms for sweep", rttSweepMinMs);
  cmd.AddValue ("rttSweepMaxMs", "Maximum RTT in ms for sweep", rttSweepMaxMs);
  cmd.AddValue ("rttSweepStepMs", "RTT step in ms for sweep", rttSweepStepMs);
  cmd.AddValue ("useFixedRss", "Use FixedRssLossModel for constant RSRP", useFixedRss);
  cmd.AddValue ("fixedRssDbm", "Fixed RSS in dBm for FixedRssLossModel", fixedRssDbm);
  cmd.AddValue ("maxStartJitterMs", "Max random ms added per-UE to app start time (0 = synchronous)", maxStartJitterMs);
  cmd.AddValue ("maxPacketsPerUe", "UDP-echo MaxPackets per UE (1 = single ping, default = unlimited)", maxPacketsPerUe);
  cmd.AddValue ("enableLteStatsLogs", "Emit ns-3 LTE module text-stats files (Dl/Ul Phy/Mac/Rlc/Pdcp/RsrpSinr); off by default", enableLteStatsLogs);
  cmd.AddValue ("nprachOpportunityMs", "NPRACH opportunity window in ms for collision tracking", g_nprachOpportunityMs);
  cmd.AddValue ("nprachNumSubcarriers", "Number of NPRACH subcarriers per CE level (collision pool size)", g_nprachNumSubcarriers);
  cmd.Parse (argc, argv);

  g_ce1LowerBound = ce1LowerBoundDbm;
  g_ce2LowerBound = ce2LowerBoundDbm;

  RngSeedManager::SetSeed(seed);

  std::vector<double> rttSweepMs;
  double rttSweepActualMinMs = rttSweepMinMs;
  double rttSweepActualMaxMs = rttSweepMaxMs;
  if (rttSweep)
  {
    if (rttSweepStepMs <= 0.0)
    {
      rttSweepStepMs = 1.0;
    }
    double minRttMs = (2.0 * (satelliteAltitude - ueAltitudeM) / 299792458.0) * 1000.0;
    if (minRttMs < 0.0)
    {
      minRttMs = 0.0;
    }
    rttSweepActualMinMs = rttSweepMinMs < minRttMs ? minRttMs : rttSweepMinMs;
    for (double v = rttSweepActualMinMs; v <= rttSweepMaxMs + 1e-9; v += rttSweepStepMs)
    {
      rttSweepMs.push_back(v);
    }
    if (!rttSweepMs.empty())
    {
      rttSweepActualMaxMs = rttSweepMs.back();
    }
    if (!rttSweepMs.empty())
    {
      num_ues = static_cast<int>(rttSweepMs.size());
    }
    simName += "-rttSweep";
  }

  // add satelliteAltitude to the log folder name
  simName += std::string("-") + std::to_string(static_cast<int>(std::lround(satelliteAltitude / 1000.0))) + std::string("km");
  simName += std::string("-seed") + std::to_string(seed);
  
  
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  

  // Create LTE Helper with NTN configuration
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);
    
  lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
  lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel");
  lteHelper->SetEnbAntennaModelAttribute("Gain", DoubleValue(enbAntennaGainDb));
  lteHelper->SetUeAntennaModelAttribute("Gain", DoubleValue(ueAntennaGainDb));
  
  if (useFixedRss)
  {
    lteHelper->SetPathlossModelType(TypeId::LookupByName("ns3::FixedRssLossModel"));
    lteHelper->SetPathlossModelAttribute("Rss", DoubleValue(fixedRssDbm));
  }
  else
  {
    lteHelper->SetPathlossModelType(TypeId::LookupByName("ns3::ThreeGppNTNUrbanPropagationLossModel"));
    lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(carrierFrequencyHz));
    if (channelConditionModel == std::string("ThreeGppRma"))
    {
      Ptr<ThreeGppRmaChannelConditionModel> rma = CreateObject<ThreeGppRmaChannelConditionModel>();
      lteHelper->SetPathlossModelAttribute("ChannelConditionModel", PointerValue(rma));
    }
    else
    {
      lteHelper->SetPathlossModelAttribute(
          "ChannelConditionModel",
          PointerValue(CreateObject<AlwaysLosChannelConditionModel>()));
    }
  }
  
  Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(useIdealRrc));
  Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(true));
  Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(true));

  // Handover algorithm (must be configured before InstallEnbDevice).
  if (useConstellation)
    {
      if (ntnHoEnabled)
        {
          // Disable RSRP-driven HO; geometric trigger drives X2 HO from the scenario.
          lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
        }
      else
        {
          lteHelper->SetHandoverAlgorithmType(handoverAlgorithm);
          if (handoverAlgorithm == "ns3::A3RsrpHandoverAlgorithm")
            {
              lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(handoverHysteresisDb));
              lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger",
                  TimeValue(MilliSeconds(static_cast<int64_t>(handoverTimeToTriggerMs))));
            }
        }
    }

  Ptr<Node> pgw = epcHelper->GetPgwNode();
  
  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  InternetStackHelper internet;
  internet.Install(remoteHostContainer);

  // Create the Internet (fixed backhaul values)
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
  p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

  // Create satellite eNB node(s) with NTN mobility model.
  Vector referencePoint(ueLatitudeDeg, ueLongitudeDeg, ueAltitudeM);

  // Resolve the list of TLEs to instantiate.
  std::vector<TleEntry> tles;
  if (useOrbitalModel && useConstellation)
    {
      tles = LoadTleFile(tleFile);
      if (tles.empty())
        {
          NS_FATAL_ERROR("TLE file contained no valid entries: " << tleFile);
        }
      if (constellationSize > 0 && static_cast<int>(tles.size()) > constellationSize)
        {
          tles.resize(constellationSize);
        }
    }
  else if (useOrbitalModel)
    {
      tles.push_back({tleName, tleLine1, tleLine2});
    }
  // else: no TLEs — single static sat, handled below.

  const uint32_t numSats = useOrbitalModel ? tles.size() : 1;
  NodeContainer enbNodes;
  enbNodes.Create(numSats);

  // Pre-parse the start time once (shared by all sats in the constellation).
  JulianDate startTime;
  bool startTimeSet = false;
  if (!simStartTimeUtc.empty())
    {
      // JulianDate's sscanf expects "YYYY-MM-DD HH:MM:SS" with a space;
      // accept ISO 8601 'T' separator too by normalizing it.
      std::string iso = simStartTimeUtc;
      std::replace(iso.begin(), iso.end(), 'T', ' ');
      startTime = JulianDate(iso);
      startTimeSet = true;
    }

  if (useOrbitalModel)
    {
      for (uint32_t i = 0; i < numSats; ++i)
        {
          Ptr<SatelliteOrbitMobilityModel> orbitMobility = CreateObject<SatelliteOrbitMobilityModel>();
          orbitMobility->SetUpdateInterval(MilliSeconds(static_cast<int64_t>(orbitUpdateIntervalMs)));
          if (startTimeSet)
            {
              orbitMobility->SetStartTime(startTime);
            }
          if (!orbitMobility->SetTle(tles[i].name, tles[i].line1, tles[i].line2))
            {
              NS_FATAL_ERROR("Failed to parse TLE for '" << tles[i].name << "'");
            }
          orbitMobility->SetCoordinateTranslationReferencePoint(referencePoint);
          enbNodes.Get(i)->AggregateObject(orbitMobility);
        }
    }
  else
    {
      Ptr<GeocentricConstantPositionMobilityModel> satMobility = CreateObject<GeocentricConstantPositionMobilityModel>();
      satMobility->SetGeographicPosition(Vector(satelliteLatitude, satelliteLongitude, satelliteAltitude));
      satMobility->SetCoordinateTranslationReferencePoint(referencePoint);
      enbNodes.Get(0)->AggregateObject(satMobility);
    }

  // Create UE nodes
  NodeContainer ueNodes;
  ueNodes.Create(num_ues);
  
  // Set UE positions using geographic coordinates
  MobilityHelper mobilityUe;
  mobilityUe.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
  mobilityUe.Install(ueNodes);
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    Ptr<GeocentricConstantPositionMobilityModel> ueMob = ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
    if (!ueMob)
    {
      ueMob = CreateObject<GeocentricConstantPositionMobilityModel>();
      ueNodes.Get(i)->AggregateObject(ueMob);
    }
    ueMob->SetCoordinateTranslationReferencePoint(referencePoint);
    double ueLat = ueLatitudeDeg;
    double ueLon = ueLongitudeDeg;
    if (rttSweep && i < rttSweepMs.size())
    {
      ueLat = ComputeLatitudeForRttMs(rttSweepMs[i], satelliteAltitude, ueAltitudeM, satelliteLatitude);
    }
    else if (ueNodes.GetN() == 1 && useOrbitalModel)
    {
      // Place the single UE under sat 0's sub-satellite point so it sees a satellite at t=0.
      Ptr<GeocentricConstantPositionMobilityModel> sat0 = enbNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>();
      if (sat0)
      {
        Vector geo = sat0->GetGeographicPosition();
        ueLat = geo.x;
        ueLon = geo.y;
      }
    }
    else if (ueNodes.GetN() > 1)
    {
      // Cluster UEs around the constellation's sub-satellite points so a fair
      // share start with a visible satellite. Each UE is offset around one of
      // the satellite footprints (round-robin), within ~±5° of the sub-sat
      // point — close enough for elevation > 10° at LEO, far enough that UEs
      // don't all collide on the same cell at t=0.
      Ptr<GeocentricConstantPositionMobilityModel> satMob =
          enbNodes.Get(i % enbNodes.GetN())->GetObject<GeocentricConstantPositionMobilityModel>();
      if (useOrbitalModel && satMob)
      {
        Vector geo = satMob->GetGeographicPosition();
        // Deterministic angular offset based on UE index so seeds remain reproducible.
        double angle = (i * 137.5) * M_PI / 180.0; // golden-angle spread
        double radiusDeg = 2.0 + (i % 5) * 0.7;     // 2..5 degrees
        ueLat = geo.x + radiusDeg * std::cos(angle);
        ueLon = geo.y + radiusDeg * std::sin(angle);
        if (ueLat > 80.0)  ueLat = 80.0;
        if (ueLat < -80.0) ueLat = -80.0;
      }
      else
      {
        double frac = static_cast<double>(i) / static_cast<double>(ueNodes.GetN() - 1);
        ueLat = -40.0 + frac * 80.0;
        ueLon = -180.0 + frac * 360.0;
      }
    }
    ueMob->SetGeographicPosition(Vector(ueLat, ueLon, ueAltitudeM));
  }

  // Install LTE Devices

  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  // X2 interface between every pair of eNBs — needed for LTE handover.
  if (useConstellation && enbNodes.GetN() >= 2)
    {
      lteHelper->AddX2Interface(enbNodes);
    }

  // Populate NtnGeometricHandover registry (cellId -> enb dev/mob, imsi -> ue dev/mob).
  if (ntnHoEnabled)
    {
      for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
        {
          Ptr<LteEnbNetDevice> enbDev = enbLteDevs.Get(i)->GetObject<LteEnbNetDevice>();
          if (!enbDev) continue;
          NtnGeometricHandover::RegisterEnb(
              enbDev->GetCellId(), enbDev,
              enbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>());
        }
      for (uint32_t u = 0; u < ueLteDevs.GetN(); ++u)
        {
          Ptr<LteUeNetDevice> ueDev = ueLteDevs.Get(u)->GetObject<LteUeNetDevice>();
          if (!ueDev) continue;
          NtnGeometricHandover::RegisterUe(
              ueDev->GetImsi(), ueDev,
              ueNodes.Get(u)->GetObject<GeocentricConstantPositionMobilityModel>());
        }
    }

  // Install the IP stack on the UEs
  internet.Install(ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
  
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get(u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
      ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<LteUeNetDevice> ueNetDev = ueLteDevs.Get(u)->GetObject<LteUeNetDevice>();
    uint64_t imsi = ueNetDev->GetImsi();
    Ptr<GeocentricConstantPositionMobilityModel> ueMob = ueNodes.Get(u)->GetObject<GeocentricConstantPositionMobilityModel>();
    Ptr<GeocentricConstantPositionMobilityModel> enbMob = enbNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>();
    Vector uePos = ueMob->GetGeocentricPosition();
    Vector enbPos = enbMob->GetGeocentricPosition();
    double dx = uePos.x - enbPos.x;
    double dy = uePos.y - enbPos.y;
    double dz = uePos.z - enbPos.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    double rttMs = (distance / 299792458.0) * 2.0 * 1000.0;
    g_imsiDistanceKm[imsi] = distance / 1000.0;
    g_imsiRttMs[imsi] = rttMs;
    g_imsiUeMob[imsi] = ueMob;
  }

  Ptr<UniformRandomVariable> RaUeUniformVariable = CreateObject<UniformRandomVariable>();

  // NPRACH detection model wiring
  g_nprachEnbMob = enbNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>();
  g_nprachRng    = RaUeUniformVariable;
  g_nprachSetId  = NprachDetectionModel::SelectSetId(enbAntennaGainDb);
  g_nprachNrep   = (nprachRepsPerAttempt > 0) ? nprachRepsPerAttempt : 1;

  // Set RACH preamble filter on eNB PHY — blocks preamble when detection model says NO
  LteEnbPhy::s_rachPreambleFilter = [](Ptr<Node> ueNode, Ptr<Node> enbNode) -> bool {
    auto ueMob = ueNode->GetObject<GeocentricConstantPositionMobilityModel>();
    auto enbMob = enbNode->GetObject<GeocentricConstantPositionMobilityModel>();
    if (!ueMob || !enbMob) return true; // fallback: accept

    double elDeg = ueMob->GetElevationAngle(enbMob);
    NprachDetectionModel::Result res;
    bool det = NprachDetectionModel::IsDetected(g_nprachSetId, g_nprachNrep,
                                                elDeg, g_nprachRng, &res);
    // Determine IMSI for logging (lookup from mobility pointer)
    uint64_t imsi = 0;
    for (const auto& kv : g_imsiUeMob) {
      if (kv.second == ueMob) { imsi = kv.first; break; }
    }
    if (det) g_nprachDetectedCount[imsi]++; else g_nprachMissedCount[imsi]++;

    // Passive NPRACH preamble-collision tracking: bin attempts into the current
    // NPRACH opportunity window and assign a random subcarrier from the pool.
    // If two UEs in the same window pick the same subcarrier, count a collision.
    int64_t bucket = static_cast<int64_t>(Simulator::Now().GetMilliSeconds() / g_nprachOpportunityMs);
    int sub = static_cast<int>(g_nprachRng->GetInteger(0, g_nprachNumSubcarriers - 1));
    auto& subMap = g_collisionWindow[bucket];
    auto& list   = subMap[sub];
    list.push_back(imsi);
    if (list.size() == 2)
    {
      g_collisionEventCount++;
      g_collidedAttemptCount += 2;
      g_perImsiCollisions[imsi]++;
      g_perImsiCollisions[list.front()]++;
      if (g_asciiTrace)
      {
        *g_asciiTrace << "[NPRACH_COLLISION] Time: " << Simulator::Now().GetSeconds()
                      << "s, Window: " << bucket << ", Subcarrier: " << sub
                      << ", IMSIs: " << list.front() << "," << imsi << std::endl;
      }
    }
    else if (list.size() > 2)
    {
      g_collidedAttemptCount++;
      g_perImsiCollisions[imsi]++;
    }
    if (g_asciiTrace)
    {
      *g_asciiTrace << "[NPRACH_DETECTION] Time: " << Simulator::Now().GetSeconds()
                    << "s, IMSI: " << imsi
                    << ", Set: " << g_nprachSetId
                    << ", NRep: " << g_nprachNrep
                    << ", Elevation: " << std::fixed << std::setprecision(2) << elDeg << " deg"
                    << ", SlantDist: " << std::setprecision(2) << (res.slantDistanceM / 1000.0) << " km"
                    << ", PL: " << std::setprecision(2) << res.pathLossDb << " dB"
                    << ", SINR: " << std::setprecision(2) << res.sinrDb << " dB"
                    << ", P_det: " << std::setprecision(4) << res.pDet
                    << (res.saturatedHigh ? " (saturated)" : "")
                    << (res.sigmoidValid ? "" : " (NO_FIT)")
                    << ", Detected: " << (det ? "YES -> PREAMBLE ACCEPTED" : "NO -> PREAMBLE DROPPED")
                    << std::endl;
    }
    return det;
  };

  // Install and start applications on UEs and remote host
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;

  // Set up data transmission. The "jitter" applies to BOTH the LTE attach
  // (which triggers cell selection + RACH) and the UDP-echo client start.
  // Spreading the attach is essential — applying jitter only to the UDP
  // client lets all 250 UEs RACH at sim init and overwhelm the LTE state
  // machines (UE-RRC IDLE_CONNECTING race in lte-ue-rrc.cc:699 at ~0.27 s).
  Ptr<UniformRandomVariable> startJitterRng = CreateObject<UniformRandomVariable>();
  startJitterRng->SetAttribute("Min", DoubleValue(0.0));
  startJitterRng->SetAttribute("Max", DoubleValue(maxStartJitterMs / 1000.0));
  bool useCellSelection = (useConstellation && enbNodes.GetN() >= 2);
  for (uint16_t i = 0; i < num_ues; i++)
    {
      // Per-UE attach delay (and matching app-start grace).
      double attachDelaySec;
      if (rttSweep)
        {
          attachDelaySec = static_cast<double>(i) * 1.0;
        }
      else
        {
          attachDelaySec = (maxStartJitterMs > 0.0) ? startJitterRng->GetValue() : 0.0;
        }

      Ptr<NetDevice> ueDev = ueLteDevs.Get(i);
      if (useCellSelection)
        {
          Simulator::Schedule(Seconds(attachDelaySec),
                              &DoAttachConstellation, lteHelper, ueDev);
        }
      else
        {
          Simulator::Schedule(Seconds(attachDelaySec),
                              &DoAttachToEnb, lteHelper, ueDev, enbLteDevs.Get(0));
        }

      ++ulPort;
      UdpEchoServerHelper server(ulPort);
      serverApps.Add(server.Install(remoteHost));

      UdpEchoClientHelper ulClient(remoteHostAddr, ulPort);
      ulClient.SetAttribute("Interval", TimeValue(packetinterval));
      ulClient.SetAttribute("MaxPackets", UintegerValue(maxPacketsPerUe));
      ulClient.SetAttribute("PacketSize", UintegerValue(packetsize));
      clientApps.Add(ulClient.Install(ueNodes.Get(i)));

      // Wire UDP echo Tx/Rx traces for per-packet RTT measurement.
      Ptr<Application> clientApp = clientApps.Get(i);
      Ptr<LteUeNetDevice> ueDevTyped = ueDev->GetObject<LteUeNetDevice>();
      uint64_t imsi = ueDevTyped ? ueDevTyped->GetImsi() : 0;
      uint32_t appId = clientApp->GetNode()->GetId();
      g_appIdToImsi[appId] = imsi;
      clientApp->TraceConnectWithoutContext("Tx", MakeBoundCallback(&OnPingTx, appId));
      clientApp->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnPingRx, appId));

      // Server can start anytime; client starts 2 s after this UE's attach so
      // RRC has time to come up before the first packet.
      serverApps.Get(i)->SetStartTime(Seconds(1.0));
      clientApps.Get(i)->SetStartTime(Seconds(attachDelaySec + 2.0));
    }

  auto start = std::chrono::system_clock::now(); 
  std::time_t start_time = std::chrono::system_clock::to_time_t(start);
  
  
  // Create simplified log directory structure
  std::string logdir = "logs/";
  logdir += simName;
  logdir += "/";
  
  // Create timestamp subdirectory
  auto tm = *std::localtime(&start_time);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  logdir += ss.str();
  logdir += "/";
  
  // Create the log directory
  std::string makedir = "mkdir -p " + logdir;
  int result = std::system(makedir.c_str());
  if (result != 0) {
    std::cerr << "Warning: Failed to create log directory" << std::endl;
  }  

  

  // Print NTN configuration will be emitted after opening trace file

  // Logging for MAC and RRC layers will be emitted after opening trace file
  
  // Heavy LTE text-stats files: opt-in (see enableLteStatsLogs above).
  if (enableLteStatsLogs)
  {
    lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnablePdcpTraces();
  }
  
  // Set up custom trace sinks for detailed logging
  std::string traceFile = logdir + "ntn-lena-nb-traces.txt";
  std::ofstream asciiTrace;
  asciiTrace.open(traceFile.c_str());
  NbiotScheduler::SetGlobalLogDir(logdir);
  NbiotScheduler::SetGlobalNprachFormat(nprachFormat);

  WriteOut(nullptr, "", true);
  WriteOut(&asciiTrace, "NTN NB-IoT Simulation:", true);
  WriteOut(&asciiTrace, std::string("Number of UEs: ") + std::to_string(num_ues), true);
  WriteOut(&asciiTrace, std::string("Packet Size: ") + std::to_string(packetsize) + " bytes", true);
  WriteOut(&asciiTrace, std::string("Packet Interval: ") + std::to_string(packetinterval.GetSeconds()) + " seconds", true);
  if (rttSweep)
  {
    WriteOut(&asciiTrace, std::string("RTT sweep (ms): ") + std::to_string(rttSweepActualMinMs) + " to " + std::to_string(rttSweepActualMaxMs) + " step " + std::to_string(rttSweepStepMs), true);
  }
  if (useFixedRss)
  {
    WriteOut(&asciiTrace, std::string("Fixed RSS (dBm): ") + std::to_string(fixedRssDbm), true);
  }
  {
    std::string startStr = std::ctime(&start_time);
    if (!startStr.empty() && startStr.back()=='\n') startStr.pop_back();
    WriteOut(&asciiTrace, std::string("Started simulation at ") + startStr, true);
  }

  PrintSatelliteInfo(enbNodes, ueNodes, &asciiTrace, true);

  WriteOut(&asciiTrace, std::string("\nMAC and RRC layer logging in directory: ") + logdir, true);
  {
    g_ceParams.resize(3);
    g_ceParams[0].coverageEnhancementLevel = NbIotRrcSap::NprachParametersNb::CoverageEnhancementLevel::zero;
    g_ceParams[0].numRepetitionsPerPreambleAttempt = NbIotRrcSap::NprachParametersNb::NumRepetitionsPerPreambleAttempt::n1;
    g_ceParams[0].npdcchNumRepetitionsRA = NbIotRrcSap::NprachParametersNb::NpdcchNumRepetitionsRA::r1;
    g_ceParams[1].coverageEnhancementLevel = NbIotRrcSap::NprachParametersNb::CoverageEnhancementLevel::one;
    g_ceParams[1].numRepetitionsPerPreambleAttempt = NbIotRrcSap::NprachParametersNb::NumRepetitionsPerPreambleAttempt::n64;
    g_ceParams[1].npdcchNumRepetitionsRA = NbIotRrcSap::NprachParametersNb::NpdcchNumRepetitionsRA::r32;
    g_ceParams[2].coverageEnhancementLevel = NbIotRrcSap::NprachParametersNb::CoverageEnhancementLevel::two;
    g_ceParams[2].numRepetitionsPerPreambleAttempt = NbIotRrcSap::NprachParametersNb::NumRepetitionsPerPreambleAttempt::n128;
    g_ceParams[2].npdcchNumRepetitionsRA = NbIotRrcSap::NprachParametersNb::NpdcchNumRepetitionsRA::r256;

    if (nprachRepsPerAttempt > 0)
    {
      auto r = MapNumRepetitionsPerPreambleAttempt(nprachRepsPerAttempt);
      g_ceParams[0].numRepetitionsPerPreambleAttempt = r;
      g_ceParams[1].numRepetitionsPerPreambleAttempt = r;
      g_ceParams[2].numRepetitionsPerPreambleAttempt = r;
      NbiotScheduler::SetGlobalCeLevels(g_ceParams[0], g_ceParams[1], g_ceParams[2]);
    }

  }
  
  // Calculate and log UE to eNB propagation delays
  CalculateAndLogUeToEnbDelay(ueNodes, enbNodes, &asciiTrace);
  
  g_asciiTrace = &asciiTrace;

  // Log active NPRACH detection model configuration + run self-test
  {
    std::ostringstream ss;
    ss << "NPRACH detection model: Set=" << g_nprachSetId
       << " (gain " << NprachDetectionModel::GainForSet(g_nprachSetId) << " dBi)"
       << ", NRep=" << g_nprachNrep
       << ", Format=" << nprachFormat
       << " (" << (nprachFormat == 2 ? "1.25 kHz, CP 800 us" : "3.75 kHz, CP 266 us") << ")";
    WriteOut(&asciiTrace, ss.str(), true);
  }
  // Connect to RRC layer trace sources
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", 
                   MakeCallback(LogConnectionEstablished));
  
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition", 
                   MakeCallback(LogStateTransition));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition", 
                   MakeCallback(LogRandomAccessStart));
  
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful", 
                   MakeCallback(LogRandomAccessSuccessful));
  
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessError", 
                   MakeCallback(LogRandomAccessError));
  
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/InitialCellSelectionEndOk", 
                   MakeCallback(LogCellSelectionOk));
  
  // Connect to eNB RRC traces
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback(LogEnbConnectionEstablished));

  // Handover traces (emitted only when a handover algorithm is configured).
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback(LogHandoverStart));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback(LogHandoverEndOk));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUeMac/RaResponseTimeout", 
                                MakeCallback(LogRaResponseTimeout));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                                MakeCallback(CaptureCurrentCellRsrpSinr));
 
  // removed console output: trace file created

  Simulator::Stop(simTime);

  if (useOrbitalModel)
    {
      Simulator::Schedule(Seconds(0), &LogSatellitePositionPeriodic, enbNodes, ueNodes, Seconds(1));
    }

  if (ntnHoEnabled)
    {
      g_ntnHoEnabled = true;
      NtnGeometricHandover::Config hoCfg;
      hoCfg.thresh1M = ntnHoThresh1Km * 1000.0;
      hoCfg.thresh2M = ntnHoThresh2Km * 1000.0;
      hoCfg.evalPeriod = MilliSeconds(static_cast<int64_t>(ntnHoEvalPeriodMs));
      hoCfg.timeToTrigger = MilliSeconds(static_cast<int64_t>(ntnHoTimeToTriggerMs));
      hoCfg.trace = &asciiTrace;
      NtnGeometricHandover::Configure(lteHelper, hoCfg);
      WriteOut(&asciiTrace,
               std::string("3GPP NTN CondEvent D2 enabled (TS 38.331 §5.5.4.15a): Thresh1=")
                   + std::to_string(ntnHoThresh1Km) + " km, Thresh2="
                   + std::to_string(ntnHoThresh2Km) + " km, eval="
                   + std::to_string(ntnHoEvalPeriodMs) + " ms, TTT="
                   + std::to_string(ntnHoTimeToTriggerMs) + " ms",
               true);
      NtnGeometricHandover::Start();
    }

  Simulator::Run();
  
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  
  {
    std::string endStr = std::ctime(&end_time);
    if (!endStr.empty() && endStr.back()=='\n') endStr.pop_back();
    WriteOut(&asciiTrace, std::string("\nSimulation completed at: ") + endStr, true);
    WriteOut(&asciiTrace, std::string("Elapsed time: ") + std::to_string(elapsed_seconds.count()) + "s", true);
  }
  if (g_ntnHoEnabled)
  {
    WriteOut(&asciiTrace,
             std::string("CondEvent D2 triggers: ")
                 + std::to_string(NtnGeometricHandover::GetTriggerCount()),
             true);
  }
  if (rttSweep && g_asciiTrace)
  {
    WriteOut(&asciiTrace, std::string("\nRTT sweep summary:"), true);
    for (std::unordered_map<uint64_t,double>::iterator it = g_imsiRttMs.begin(); it != g_imsiRttMs.end(); ++it)
    {
      uint64_t imsi = it->first;
      double rttMs = it->second;
      double distanceKm = g_imsiDistanceKm[imsi];
      uint32_t starts = g_raStartCount[imsi];
      uint32_t ok = g_raSuccessCount[imsi];
      uint32_t timeout = g_raTimeoutCount[imsi];
      std::ostringstream line;
      line << "IMSI " << imsi << ": RTT " << std::fixed << std::setprecision(2) << rttMs
           << " ms, Distance " << std::fixed << std::setprecision(2) << distanceKm
           << " km, RA starts " << starts << ", RA success " << ok << ", RA timeouts " << timeout;
      WriteOut(&asciiTrace, line.str(), true);
    }
  }

  // ===== UE-scaling analysis summary =====
  WriteOut(&asciiTrace, "\n========== UE-scaling analysis summary ==========", true);
  {
    std::ostringstream cfg;
    cfg << "numUes=" << num_ues << ", satellites=" << numSats
        << ", maxStartJitterMs=" << maxStartJitterMs
        << ", nprachOpportunityMs=" << g_nprachOpportunityMs
        << ", nprachNumSubcarriers=" << g_nprachNumSubcarriers
        << ", nprachSet=" << g_nprachSetId << " (gain " << enbAntennaGainDb << " dBi)";
    WriteOut(&asciiTrace, cfg.str(), true);
  }

  // Per-IMSI table
  WriteOut(&asciiTrace,
           "IMSI | distKm | rttGeomMs | RAstarts | RAsuccess | RAtimeouts | preambleTx | "
           "connDelayMs | connected | nprachDet | nprachMiss | collisions | pingTx | pingRx | meanPingRtt",
           true);
  uint32_t connectedUes = 0;
  std::vector<double> allConnDelayMs;
  std::vector<double> allMeanRttMs;
  uint32_t totalRaStarts = 0;
  uint32_t totalRaTimeouts = 0;
  uint32_t totalPingTx = 0;
  uint32_t totalPingRx = 0;
  for (uint32_t u = 0; u < ueLteDevs.GetN(); ++u)
  {
    Ptr<LteUeNetDevice> ueDev = ueLteDevs.Get(u)->GetObject<LteUeNetDevice>();
    if (!ueDev) continue;
    uint64_t imsi = ueDev->GetImsi();
    double connDelayMs = -1.0;
    auto itStart = g_raFirstStartSec.find(imsi);
    auto itConn  = g_connectedSec.find(imsi);
    bool connected = (itConn != g_connectedSec.end());
    if (connected && itStart != g_raFirstStartSec.end())
    {
      connDelayMs = (itConn->second - itStart->second) * 1000.0;
      allConnDelayMs.push_back(connDelayMs);
    }
    if (connected) connectedUes++;
    double meanRtt = -1.0;
    auto itRtt = g_pingRttMs.find(imsi);
    if (itRtt != g_pingRttMs.end() && !itRtt->second.empty())
    {
      double sum = 0.0;
      for (double v : itRtt->second) sum += v;
      meanRtt = sum / itRtt->second.size();
      allMeanRttMs.push_back(meanRtt);
    }
    totalRaStarts   += g_raStartCount[imsi];
    totalRaTimeouts += g_raTimeoutCount[imsi];
    totalPingTx     += g_pingTxCount[imsi];
    totalPingRx     += g_pingRxCount[imsi];
    std::ostringstream row;
    row << imsi
        << " | " << std::fixed << std::setprecision(1) << g_imsiDistanceKm[imsi]
        << " | " << std::setprecision(2) << g_imsiRttMs[imsi]
        << " | " << g_raStartCount[imsi]
        << " | " << g_raSuccessCount[imsi]
        << " | " << g_raTimeoutCount[imsi]
        << " | " << g_preambleTxCounter[imsi]
        << " | " << (connDelayMs < 0 ? std::string("-") : std::to_string(connDelayMs))
        << " | " << (connected ? "Y" : "N")
        << " | " << g_nprachDetectedCount[imsi]
        << " | " << g_nprachMissedCount[imsi]
        << " | " << g_perImsiCollisions[imsi]
        << " | " << g_pingTxCount[imsi]
        << " | " << g_pingRxCount[imsi]
        << " | " << (meanRtt < 0 ? std::string("-") : std::to_string(meanRtt));
    WriteOut(&asciiTrace, row.str(), true);
  }

  auto mean = [](const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0; for (double x : v) s += x; return s / v.size();
  };
  double connectRatePct = ueLteDevs.GetN() > 0 ?
      100.0 * connectedUes / ueLteDevs.GetN() : 0.0;

  // ===== Aggregate / scaling-friendly KPIs (parseable single lines) =====
  std::ostringstream kpi;
  kpi << std::fixed;
  WriteOut(&asciiTrace, "\n========== Aggregate KPIs ==========", true);
  kpi.str(""); kpi << "[KPI] numUes=" << num_ues;                          WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] connectedUes=" << connectedUes;               WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] connectRatePct=" << std::setprecision(2) << connectRatePct; WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] meanConnDelayMs=" << std::setprecision(2) << mean(allConnDelayMs); WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] meanPingRttMs=" << std::setprecision(2) << mean(allMeanRttMs);     WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] totalRaStarts=" << totalRaStarts;             WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] totalRaTimeouts=" << totalRaTimeouts;         WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] meanRaTimeoutsPerUe=" << std::setprecision(3)
                   << (ueLteDevs.GetN() ? double(totalRaTimeouts)/ueLteDevs.GetN() : 0.0);
  WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] collisionEvents=" << g_collisionEventCount;       WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] collidedAttempts=" << g_collidedAttemptCount;     WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] pingTx=" << totalPingTx << ", pingRx=" << totalPingRx;
  WriteOut(&asciiTrace, kpi.str(), true);
  kpi.str(""); kpi << "[KPI] pingLossPct=" << std::setprecision(2)
                   << (totalPingTx ? 100.0 * (totalPingTx - totalPingRx) / totalPingTx : 0.0);
  WriteOut(&asciiTrace, kpi.str(), true);

  asciiTrace.close();
  
  
  // removed console output: trace file saved path
  
  // Move all trace files to the logs directory
  std::vector<std::string> traceFiles = {
    "DlMacStats.txt", "UlMacStats.txt",
    "DlRlcStats.txt", "UlRlcStats.txt",
    "DlPdcpStats.txt", "UlPdcpStats.txt",
    "DlTxPhyStats.txt", "UlTxPhyStats.txt", "UlRxPhyStats.txt",
    "DlRsrpSinrStats.txt", "UlSinrStats.txt", "UlInterferenceStats.txt"
  };
  
  for (const auto& file : traceFiles) {
    std::string moveCmd = "mv " + file + " " + logdir + " 2>/dev/null";
    std::system(moveCmd.c_str());
  }
  
  // removed console output: all logs directory
  
  Simulator::Destroy();
  return 0;
}
