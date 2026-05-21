/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#include "satellite-orbit-mobility-model.h"

#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/vector.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatelliteOrbitMobilityModel");
NS_OBJECT_ENSURE_REGISTERED (SatelliteOrbitMobilityModel);

TypeId
SatelliteOrbitMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatelliteOrbitMobilityModel")
    .SetParent<GeocentricConstantPositionMobilityModel> ()
    .SetGroupName ("Satellite")
    .AddConstructor<SatelliteOrbitMobilityModel> ()
    .AddAttribute ("UpdateInterval",
                   "Period between SGP4 position updates.",
                   TimeValue (MilliSeconds (100)),
                   MakeTimeAccessor (&SatelliteOrbitMobilityModel::SetUpdateInterval,
                                     &SatelliteOrbitMobilityModel::GetUpdateInterval),
                   MakeTimeChecker ());
  return tid;
}

SatelliteOrbitMobilityModel::SatelliteOrbitMobilityModel ()
  : m_startTimeSet (false),
    m_updateInterval (MilliSeconds (100))
{
}

SatelliteOrbitMobilityModel::~SatelliteOrbitMobilityModel ()
{
}

bool
SatelliteOrbitMobilityModel::SetTle (const std::string& name,
                                     const std::string& line1,
                                     const std::string& line2)
{
  Ptr<Satellite> sat = CreateObject<Satellite> ();
  sat->SetName (name);
  if (!sat->SetTleInfo (line1, line2))
    {
      NS_LOG_ERROR ("Invalid TLE for satellite '" << name << "'");
      return false;
    }
  m_satellite = sat;
  if (!m_startTimeSet)
    {
      m_startTime = m_satellite->GetTleEpoch ();
    }
  UpdatePosition ();
  return true;
}

Ptr<Satellite>
SatelliteOrbitMobilityModel::GetSatellite (void) const
{
  return m_satellite;
}

void
SatelliteOrbitMobilityModel::SetStartTime (const JulianDate& t)
{
  m_startTime = t;
  m_startTimeSet = true;
}

JulianDate
SatelliteOrbitMobilityModel::GetStartTime (void) const
{
  return m_startTime;
}

void
SatelliteOrbitMobilityModel::SetUpdateInterval (Time interval)
{
  m_updateInterval = interval;
}

Time
SatelliteOrbitMobilityModel::GetUpdateInterval (void) const
{
  return m_updateInterval;
}

void
SatelliteOrbitMobilityModel::DoInitialize (void)
{
  if (m_satellite)
    {
      UpdatePosition ();
    }
  GeocentricConstantPositionMobilityModel::DoInitialize ();
}

void
SatelliteOrbitMobilityModel::DoDispose (void)
{
  Simulator::Cancel (m_updateEvent);
  m_satellite = nullptr;
  GeocentricConstantPositionMobilityModel::DoDispose ();
}

void
SatelliteOrbitMobilityModel::UpdatePosition (void)
{
  if (!m_satellite)
    {
      return;
    }

  JulianDate now = m_startTime + Simulator::Now ();
  Vector3D geo = m_satellite->GetGeographicPosition (now);

  SetGeographicPosition (Vector (geo.x, geo.y, geo.z));

  NS_LOG_DEBUG ("Sat '" << m_satellite->GetName ()
                << "' t=" << Simulator::Now ().GetSeconds ()
                << "s lat=" << geo.x << " lon=" << geo.y << " alt=" << geo.z);

  m_updateEvent = Simulator::Schedule (m_updateInterval,
                                       &SatelliteOrbitMobilityModel::UpdatePosition,
                                       this);
}

} // namespace ns3
