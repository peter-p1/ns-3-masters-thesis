/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#ifndef SATELLITE_ORBIT_MOBILITY_MODEL_H
#define SATELLITE_ORBIT_MOBILITY_MODEL_H

#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/type-id.h"

#include "julian-date.h"
#include "satellite.h"

#include <string>

namespace ns3 {

/**
 * \ingroup satellite
 * \brief Mobility model that propagates a satellite orbit using SGP4/SDP4.
 *
 * Inherits from GeocentricConstantPositionMobilityModel so it remains
 * compatible with NTN propagation loss models and existing scenarios that
 * rely on geographic/geocentric APIs and GetElevationAngle().
 *
 * The position is recomputed periodically (every UpdateInterval) from a TLE
 * via the Satellite class. Each update writes into the parent class via
 * SetGeographicPosition(), which triggers NotifyCourseChange() and invalidates
 * any cached propagation state.
 */
class SatelliteOrbitMobilityModel : public GeocentricConstantPositionMobilityModel
{
public:
  static TypeId GetTypeId (void);

  SatelliteOrbitMobilityModel ();
  virtual ~SatelliteOrbitMobilityModel ();

  /**
   * \brief Configure the satellite from a TLE (two-line element) set.
   * \return true if the TLE was parsed successfully.
   *
   * If the simulation start time has not been explicitly set, it defaults to
   * the TLE epoch.
   */
  bool SetTle (const std::string& name,
               const std::string& line1,
               const std::string& line2);

  Ptr<Satellite> GetSatellite (void) const;

  void SetStartTime (const JulianDate& t);
  JulianDate GetStartTime (void) const;

  void SetUpdateInterval (Time interval);
  Time GetUpdateInterval (void) const;

protected:
  virtual void DoInitialize (void) override;
  virtual void DoDispose (void) override;

private:
  void UpdatePosition (void);

  Ptr<Satellite> m_satellite;
  JulianDate m_startTime;
  bool m_startTimeSet;
  Time m_updateInterval;
  EventId m_updateEvent;
};

} // namespace ns3

#endif /* SATELLITE_ORBIT_MOBILITY_MODEL_H */
