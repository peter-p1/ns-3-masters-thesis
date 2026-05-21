/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// NTN geometric (location-based) handover trigger for ns-3 LTE NB-IoT.
//
// Implements 3GPP TS 38.331 §5.5.4.15a — CondEvent D2:
//
//   "Distance between UE and the serving cell moving reference location is
//    above threshold1 AND distance between UE and a moving reference
//    location is below threshold2."
//
// Reference locations are taken as the live 3D position of the corresponding
// satellite eNB (moving with its mobility model). Ml1, Ml2 are 3D Euclidean
// distances in geocentric coordinates (TS 38.305 / 38.331 ASN.1 distance
// units are metres). Trigger condition (entering, simplified, no Hys):
//
//     Ml1 > Thresh1   AND   Ml2 < Thresh2
//
// must hold continuously for timeToTrigger. CondEvent T1 (time-based) and
// hysteresis-based form are not implemented.
//
// On trigger, the standard LTE X2 handover procedure is invoked through
// LteHelper::HandoverRequest(). The scenario must therefore (a) install X2
// interfaces between candidate eNBs, and (b) replace the LTE module's HO
// algorithm with NoOpHandoverAlgorithm so A3 RSRP does not also fire.
//
// Usage:
//
//   #include "ntn-geometric-handover.h"
//   ...
//   NtnGeometricHandover::Config cfg;
//   cfg.thresh1M = 1500e3;   // serving-cell distance above this -> D2 leave
//   cfg.thresh2M = 1200e3;   // candidate-cell distance below this -> D2 enter
//   cfg.evalPeriod = MilliSeconds(500);
//   cfg.timeToTrigger = MilliSeconds(200);
//   cfg.trace = &asciiTrace;
//   NtnGeometricHandover::Configure(lteHelper, cfg);
//   for (each eNB i) NtnGeometricHandover::RegisterEnb(cellId, enbDev, enbMob);
//   for (each UE)   NtnGeometricHandover::RegisterUe(imsi, ueDev, ueMob);
//   // on RRC ConnectionEstablished and HandoverEndOk:
//   NtnGeometricHandover::OnConnected(imsi, cellId);
//   NtnGeometricHandover::Start();
//
#ifndef NTN_GEOMETRIC_HANDOVER_H
#define NTN_GEOMETRIC_HANDOVER_H

#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

#include <fstream>
#include <iomanip>
#include <limits>
#include <unordered_map>

namespace ns3 {

struct NtnGeometricHandoverConfig
{
  // 3GPP TS 38.331 §5.5.4.15a CondEvent D2 thresholds (metres).
  double thresh1M = 1500e3;            // Ml1 (serving) > Thresh1  -> leave
  double thresh2M = 1200e3;            // Ml2 (candidate) < Thresh2 -> enter
  Time   evalPeriod = MilliSeconds(500);
  Time   timeToTrigger = MilliSeconds(0);
  std::ofstream* trace = nullptr;      // optional trace file
};

class NtnGeometricHandover
{
public:
  using Config = NtnGeometricHandoverConfig;

  static void Configure(Ptr<LteHelper> lteHelper, const Config& cfg)
  {
    s_lteHelper = lteHelper;
    s_cfg = cfg;
  }

  static void RegisterEnb(uint16_t cellId,
                          Ptr<LteEnbNetDevice> enbDev,
                          Ptr<GeocentricConstantPositionMobilityModel> enbMob)
  {
    s_cellIdToEnbDev[cellId] = enbDev;
    s_cellIdToEnbMob[cellId] = enbMob;
  }

  static void RegisterUe(uint64_t imsi,
                         Ptr<LteUeNetDevice> ueDev,
                         Ptr<GeocentricConstantPositionMobilityModel> ueMob)
  {
    s_imsiToUeDev[imsi] = ueDev;
    s_imsiToUeMob[imsi] = ueMob;
  }

  // Call from RRC trace handlers (ConnectionEstablished / HandoverEndOk).
  static void OnConnected(uint64_t imsi, uint16_t cellId)
  {
    s_imsiToServingCellId[imsi] = cellId;
  }

  static void Start()
  {
    if (!s_lteHelper)
      {
        return;
      }
    s_enabled = true;
    Simulator::Schedule(s_cfg.evalPeriod, &NtnGeometricHandover::Evaluate);
  }

  static uint32_t GetTriggerCount() { return s_triggerCount; }
  static const Config& GetConfig()  { return s_cfg; }

private:
  struct Pending { uint16_t candCellId = 0; Time startTime; };

  static void Evaluate()
  {
    if (!s_enabled || !s_lteHelper)
      {
        return;
      }
    Time now = Simulator::Now();
    for (auto& kv : s_imsiToServingCellId)
      {
        uint64_t imsi = kv.first;
        uint16_t servingCellId = kv.second;
        if (servingCellId == 0)
          {
            continue; // HO in flight or not yet connected
          }

        auto ueMobIt = s_imsiToUeMob.find(imsi);
        auto srcMobIt = s_cellIdToEnbMob.find(servingCellId);
        auto srcDevIt = s_cellIdToEnbDev.find(servingCellId);
        auto ueDevIt  = s_imsiToUeDev.find(imsi);
        if (ueMobIt == s_imsiToUeMob.end() || srcMobIt == s_cellIdToEnbMob.end()
            || srcDevIt == s_cellIdToEnbDev.end() || ueDevIt == s_imsiToUeDev.end())
          {
            continue;
          }

        // Ml1 = distance UE <-> serving moving reference location (3D).
        double ml1 = ueMobIt->second->GetDistanceFrom(srcMobIt->second);
        if (ml1 <= s_cfg.thresh1M)
          {
            s_pending.erase(imsi);
            continue;
          }

        // Best candidate: smallest Ml2 below thresh2M.
        uint16_t bestCand = 0;
        double bestMl2 = std::numeric_limits<double>::infinity();
        for (auto& cand : s_cellIdToEnbMob)
          {
            if (cand.first == servingCellId)
              {
                continue;
              }
            double ml2 = ueMobIt->second->GetDistanceFrom(cand.second);
            if (ml2 < s_cfg.thresh2M && ml2 < bestMl2)
              {
                bestMl2 = ml2;
                bestCand = cand.first;
              }
          }
        if (bestCand == 0)
          {
            s_pending.erase(imsi);
            continue;
          }

        Pending& p = s_pending[imsi];
        if (p.candCellId != bestCand)
          {
            p.candCellId = bestCand;
            p.startTime = now;
            Trace("[condEvent D2 pending]", now, imsi, servingCellId, bestCand, ml1, bestMl2);
          }
        if (now - p.startTime >= s_cfg.timeToTrigger)
          {
            Trace("[condEvent D2 fulfilled]", now, imsi, servingCellId, bestCand, ml1, bestMl2);
            s_lteHelper->HandoverRequest(MilliSeconds(0), ueDevIt->second,
                                         srcDevIt->second, bestCand);
            // Mark in-flight; OnConnected (HandoverEndOk) restores it.
            s_imsiToServingCellId[imsi] = 0;
            s_pending.erase(imsi);
            ++s_triggerCount;
          }
      }
    Simulator::Schedule(s_cfg.evalPeriod, &NtnGeometricHandover::Evaluate);
  }

  static void Trace(const char* tag, Time now, uint64_t imsi,
                    uint16_t srcCellId, uint16_t candCellId,
                    double ml1, double ml2)
  {
    if (!s_cfg.trace)
      {
        return;
      }
    *s_cfg.trace << tag << " Time: " << now.GetSeconds()
                 << "s, IMSI: " << imsi
                 << ", PCell: " << srcCellId << ", CandCell: " << candCellId
                 << ", Ml1: " << std::fixed << std::setprecision(1) << (ml1 / 1000.0)
                 << " km, Ml2: " << (ml2 / 1000.0) << " km"
                 << ", Thresh1: " << (s_cfg.thresh1M / 1000.0)
                 << " km, Thresh2: " << (s_cfg.thresh2M / 1000.0) << " km"
                 << std::endl;
  }

  inline static Ptr<LteHelper> s_lteHelper;
  inline static Config s_cfg;
  inline static bool s_enabled = false;
  inline static uint32_t s_triggerCount = 0;
  inline static std::unordered_map<uint16_t, Ptr<LteEnbNetDevice>> s_cellIdToEnbDev;
  inline static std::unordered_map<uint16_t, Ptr<GeocentricConstantPositionMobilityModel>>
      s_cellIdToEnbMob;
  inline static std::unordered_map<uint64_t, Ptr<LteUeNetDevice>> s_imsiToUeDev;
  inline static std::unordered_map<uint64_t, Ptr<GeocentricConstantPositionMobilityModel>>
      s_imsiToUeMob;
  inline static std::unordered_map<uint64_t, uint16_t> s_imsiToServingCellId;
  inline static std::unordered_map<uint64_t, Pending> s_pending;
};

} // namespace ns3

#endif // NTN_GEOMETRIC_HANDOVER_H
