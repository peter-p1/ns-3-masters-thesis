#ifndef NPRACH_DETECTION_MODEL_H
#define NPRACH_DETECTION_MODEL_H

#include "ns3/random-variable-stream.h"
#include "ns3/ptr.h"
#include <cmath>

namespace ns3 {

// NPRACH detection model based on Matlab link-level simulation results.
// Physical setup: LEO 600/1200 km, f=2 GHz. Sigmoid fits parametrised per
// (setId, NRep) where setId encodes the satellite antenna gain configuration:
//   Set 1 -> 30.0  dBi
//   Set 2 -> 24.0  dBi
//   Set 3 -> 16.2  dBi
class NprachDetectionModel
{
public:
  struct Result
  {
    double slantDistanceM;
    double fsplDb;
    double pathLossDb;
    double sinrDb;
    double pDet;
    bool   detected;
    bool   sigmoidValid;   // false if no fit exists for (setId, nrep)
    bool   saturatedHigh;  // true if fit is saturated (treated as P_det = 1)
  };

  // Physical constants
  static constexpr double kEarthRadiusM = 6371.0e3;
  static constexpr double kSatAltM      = 600.0e3;
  static constexpr double kFreqHz       = 2.0e9; // 2000 MHz
  static constexpr double kC            = 299792458.0;

  // enbAntennaGainDb -> setId mapping. Tolerant of floating-point values.
  static int SelectSetId(double enbAntennaGainDb)
  {
    if (enbAntennaGainDb >= 27.0) return 1;  // 30 dBi
    if (enbAntennaGainDb >= 20.0) return 2;  // 24 dBi
    return 3;                                // 16.2 dBi
  }

  static double GainForSet(int setId)
  {
    switch (setId)
    {
      case 1: return 30.0;
      case 2: return 24.0;
      case 3: return 16.2;
      default: return 16.2;
    }
  }

  // Slant distance from satellite (LEO) to ground UE given elevation angle.
  // d = sqrt((Re+hs)^2 - (Re*cos(el))^2) - Re*sin(el)
  static double SlantDistance(double elevationDeg)
  {
    double el = elevationDeg * M_PI / 180.0;
    double Re = kEarthRadiusM;
    double hs = kSatAltM;
    double a  = (Re + hs) * (Re + hs);
    double b  = (Re * std::cos(el)) * (Re * std::cos(el));
    return std::sqrt(a - b) - Re * std::sin(el);
  }

  static double Fspl(double distanceM)
  {
    return 20.0 * std::log10(4.0 * M_PI * distanceM * kFreqHz / kC);
  }

  // PL = FSPL - sat_gain + 3 + 2.2 + 0.1 + 3  (scintillation + atmos + polar + shadowing)
  static double PathLoss(double fsplDb, double satGainDb)
  {
    return fsplDb - satGainDb + 3.0 + 2.2 + 0.1 + 3.0;
  }

  // Empirical SINR model from link-level fit: SINR = -1.35*PL + 198 + 10*log10(15/1.25)
  static double SinrFromPathLoss(double plDb)
  {
    return -1.35 * plDb + 198.0 + 10.0 * std::log10(15.0 / 1.25);
  }

  struct SigmoidParams
  {
    bool   valid;
    bool   saturatedHigh;
    double sinr0Db;
    double k;
  };

  // Sigmoid fit parameters from sigmoid_fit_NS3.csv (Matlab results).
  // Set 1 NRep=2 and NRep=4 are "saturated_high" -> always-detected region.
  static SigmoidParams GetSigmoid(int setId, int nrep)
  {
    if (setId == 1 && nrep == 1)  return {true, false,  7.210, 0.547};
    if (setId == 1 && nrep == 2)  return {true, true,   0.0,   0.0};
    if (setId == 1 && nrep == 4)  return {true, true,   0.0,   0.0};
    if (setId == 2 && nrep == 1)  return {true, false,  5.307, 0.361};
    if (setId == 2 && nrep == 2)  return {true, false,  1.021, 0.487};
    if (setId == 2 && nrep == 4)  return {true, false, -0.417, 0.735};
    if (setId == 3 && nrep == 1)  return {true, false,  4.670, 0.394};
    if (setId == 3 && nrep == 2)  return {true, false,  0.553, 0.469};
    if (setId == 3 && nrep == 4)  return {true, false, -2.111, 0.548};
    if (setId == 3 && nrep == 16) return {true, false, -6.268, 0.738};
    return {false, false, 0.0, 0.0};
  }

  // Deterministic evaluation (no random draw) — useful for logging and tests.
  static Result Evaluate(int setId, int nrep, double elevationAngleDeg)
  {
    Result r{};
    r.slantDistanceM = SlantDistance(elevationAngleDeg);
    r.fsplDb         = Fspl(r.slantDistanceM);
    r.pathLossDb     = PathLoss(r.fsplDb, GainForSet(setId));
    r.sinrDb         = SinrFromPathLoss(r.pathLossDb);

    SigmoidParams s  = GetSigmoid(setId, nrep);
    r.sigmoidValid   = s.valid;
    r.saturatedHigh  = s.saturatedHigh;

    if (!s.valid)
    {
      r.pDet = 0.0;
      r.detected = false;
    }
    else if (s.saturatedHigh)
    {
      r.pDet = 1.0;
      r.detected = true;
    }
    else
    {
      r.pDet = 1.0 / (1.0 + std::exp(-s.k * (r.sinrDb - s.sinr0Db)));
      r.detected = false; // pending Bernoulli draw
    }
    return r;
  }

  // Full detection: compute SINR and P_det, then Bernoulli draw against rng.
  static bool IsDetected(int setId, int nrep, double elevationAngleDeg,
                         Ptr<UniformRandomVariable> rng, Result* outRes = nullptr)
  {
    Result r = Evaluate(setId, nrep, elevationAngleDeg);
    if (!r.sigmoidValid)
    {
      r.detected = false;
    }
    else if (r.saturatedHigh || r.pDet >= 1.0)
    {
      r.detected = true;
    }
    else
    {
      double u = rng->GetValue(0.0, 1.0);
      r.detected = (u < r.pDet);
    }
    if (outRes) *outRes = r;
    return r.detected;
  }
};

} // namespace ns3

#endif // NPRACH_DETECTION_MODEL_H
