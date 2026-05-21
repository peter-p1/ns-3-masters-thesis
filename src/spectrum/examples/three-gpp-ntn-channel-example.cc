/*
 * Copyright (c) 2023 SIGNET Lab, Department of Information Engineering,
 * University of Padova
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
 */

/**
 * This example is a modified version of "three-gpp-channel-example", to include
 * the 3GPP NTN channel model.
 * The preconfigured parameters are the one provided by 3GPP in TR 38.821,
 * more specifically scenario 10 in down-link mode.
 * Two static nodes, one on the ground and one in orbit, communicates using
 * circular aperture antennas. The carrier frequency is set at 20GHz with 400MHz bandwidth.
 * The result is the SNR of the signal and the pathloss, saved in the ntn-snr-trace.txt file.
 */

#include "ns3/channel-condition-model.h"
#include "ns3/circular-aperture-antenna-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/lte-spectrum-value-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/parabolic-antenna-model.h"
#include "ns3/simple-net-device.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uniform-planar-array.h"
#include <ns3/antenna-model.h>

#include <fstream>

NS_LOG_COMPONENT_DEFINE("NTNChannelExample");

using namespace ns3;

static Ptr<ThreeGppPropagationLossModel>
    m_propagationLossModel; //!< the PropagationLossModel object
static Ptr<ThreeGppSpectrumPropagationLossModel>
    m_spectrumLossModel; //!< the SpectrumPropagationLossModel object

Ptr<SpectrumValue>
CreateTxPowerSpectralDensity(double fc, double pwr, double BW, double RB_WIDTH)
{
    unsigned int numRbs = std::floor(BW / RB_WIDTH);
    double f = fc - (numRbs * RB_WIDTH / 2.0);
    double powerTx = pwr; // dBm power

    Bands rbs;              // A vector representing each resource block
    std::vector<int> rbsId; // A vector representing the resource block IDs
    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += RB_WIDTH / 2;
        rb.fc = f;
        f += RB_WIDTH / 2;
        rb.fh = f;

        rbs.push_back(rb);
        rbsId.push_back(numrb);
    }
    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(model);

    double powerTxW = std::pow(10., (powerTx - 30) / 10); // Get Tx power in Watts
    double txPowerDensity = (powerTxW / BW);

    for (auto rbId : rbsId)
    {
        (*txPsd)[rbId] = txPowerDensity;
    }

    return txPsd; // [W/Hz]
}

Ptr<SpectrumValue>
CreateNoisePowerSpectralDensity(double fc, double noiseFigure, double BW, double RB_WIDTH)
{
    unsigned int numRbs = std::floor(BW / RB_WIDTH);
    double f = fc - (numRbs * RB_WIDTH / 2.0);
    double noiseFigureDb = noiseFigure; // dB noise figure

    Bands rbs;              // A vector representing each resource block
    std::vector<int> rbsId; // A vector representing the resource block IDs
    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += RB_WIDTH / 2;
        rb.fc = f;
        f += RB_WIDTH / 2;
        rb.fh = f;

        rbs.push_back(rb);
        rbsId.push_back(numrb);
    }
    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(model);

    // see "LTE - From theory to practice"
    // Section 22.4.4.2 Thermal Noise and Receiver Noise Figure
    const double kT_dBm_Hz = -174.0;                          // dBm/Hz
    double kT_W_Hz = std::pow(10.0, (kT_dBm_Hz - 30) / 10.0); // W/Hz
    double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);

    double noisePowerSpectralDensity = kT_W_Hz * noiseFigureLinear;

    for (auto rbId : rbsId)
    {
        (*txPsd)[rbId] = noisePowerSpectralDensity;
    }

    return txPsd; // W/Hz
}

/**
 * \brief A structure that holds the parameters for the
 * ComputeSnr function. In this way the problem with the limited
 * number of parameters of method Schedule is avoided.
 */
struct ComputeSnrParams
{
    Ptr<MobilityModel> txMob;        //!< the tx mobility model
    Ptr<MobilityModel> rxMob;        //!< the rx mobility model
    double txPow;                    //!< the tx power in dBm
    double noiseFigure;              //!< the noise figure in dB
    Ptr<PhasedArrayModel> txAntenna; //!< the tx antenna array
    Ptr<PhasedArrayModel> rxAntenna; //!< the rx antenna array
    double frequency;                //!< the carrier frequency in Hz
    double bandwidth;                //!< the total bandwidth in Hz
    double resourceBlockBandwidth;   //!< the Resource Block bandwidth in Hz

    /**
     * \brief Constructor
     * \param pTxMob the tx mobility model
     * \param pRxMob the rx mobility model
     * \param pTxPow the tx power in dBm
     * \param pNoiseFigure the noise figure in dB
     * \param pTxAntenna the tx antenna array
     * \param pRxAntenna the rx antenna array
     * \param pFrequency the carrier frequency in Hz
     * \param pBandwidth the total bandwidth in Hz
     * \param pResourceBlockBandwidth the Resource Block bandwidth in Hz
     */
    ComputeSnrParams(Ptr<MobilityModel> pTxMob,
                     Ptr<MobilityModel> pRxMob,
                     double pTxPow,
                     double pNoiseFigure,
                     Ptr<PhasedArrayModel> pTxAntenna,
                     Ptr<PhasedArrayModel> pRxAntenna,
                     double pFrequency,
                     double pBandwidth,
                     double pResourceBlockBandwidth)
    {
        txMob = pTxMob;
        rxMob = pRxMob;
        txPow = pTxPow;
        noiseFigure = pNoiseFigure;
        txAntenna = pTxAntenna;
        rxAntenna = pRxAntenna;
        frequency = pFrequency;
        bandwidth = pBandwidth;
        resourceBlockBandwidth = pResourceBlockBandwidth;
    }
};

/**
 * Perform the beamforming using the DFT beamforming method
 * \param thisDevice the device performing the beamforming
 * \param thisAntenna the antenna object associated to thisDevice
 * \param otherDevice the device towards which point the beam
 */
static void
DoBeamforming(Ptr<NetDevice> thisDevice,
              Ptr<PhasedArrayModel> thisAntenna,
              Ptr<NetDevice> otherDevice)
{
    // retrieve the position of the two devices
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

    // compute the azimuth and the elevation angles
    Angles completeAngle(bPos, aPos);
    double hAngleRadian = completeAngle.GetAzimuth();

    double vAngleRadian = completeAngle.GetInclination(); // the elevation angle

    // retrieve the number of antenna elements and resize the vector
    uint64_t totNoArrayElements = thisAntenna->GetNumberOfElements();
    PhasedArrayModel::ComplexVector antennaWeights(totNoArrayElements);

    // the total power is divided equally among the antenna elements
    double power = 1.0 / sqrt(totNoArrayElements);

    // compute the antenna weights
    const double sinVAngleRadian = sin(vAngleRadian);
    const double cosVAngleRadian = cos(vAngleRadian);
    const double sinHAngleRadian = sin(hAngleRadian);
    const double cosHAngleRadian = cos(hAngleRadian);

    for (uint64_t ind = 0; ind < totNoArrayElements; ind++)
    {
        Vector loc = thisAntenna->GetElementLocation(ind);
        double phase = -2 * M_PI *
                       (sinVAngleRadian * cosHAngleRadian * loc.x +
                        sinVAngleRadian * sinHAngleRadian * loc.y + cosVAngleRadian * loc.z);
        antennaWeights[ind] = exp(std::complex<double>(0, phase)) * power;
    }

    // store the antenna weights
    thisAntenna->SetBeamformingVector(antennaWeights);
}

/**
 * Compute the average SNR
 * \param params A structure that holds the parameters that are needed to perform calculations in
 * ComputeSnr
 */
static void
ComputeSnr(ComputeSnrParams& params)
{
    Ptr<SpectrumValue> txPsd = CreateTxPowerSpectralDensity(params.frequency,
                                                            params.txPow,
                                                            params.bandwidth,
                                                            params.resourceBlockBandwidth);
    Ptr<SpectrumValue> rxPsd = txPsd->Copy();
    NS_LOG_DEBUG("Average tx power " << 10 * log10(Sum(*txPsd) * params.resourceBlockBandwidth)
                                     << " dB");

    // create the noise PSD
    Ptr<SpectrumValue> noisePsd = CreateNoisePowerSpectralDensity(params.frequency,
                                                                  params.noiseFigure,
                                                                  params.bandwidth,
                                                                  params.resourceBlockBandwidth);
    NS_LOG_DEBUG("Average noise power "
                 << 10 * log10(Sum(*noisePsd) * params.resourceBlockBandwidth) << " dB");

    // apply the pathloss
    double propagationGainDb = m_propagationLossModel->CalcRxPower(0, params.txMob, params.rxMob);
    NS_LOG_DEBUG("Pathloss " << -propagationGainDb << " dB");
    double propagationGainLinear = std::pow(10.0, (propagationGainDb) / 10.0);
    *(rxPsd) *= propagationGainLinear;

    NS_ASSERT_MSG(params.txAntenna, "params.txAntenna is nullptr!");
    NS_ASSERT_MSG(params.rxAntenna, "params.rxAntenna is nullptr!");

    Ptr<SpectrumSignalParameters> rxSsp = Create<SpectrumSignalParameters>();
    rxSsp->psd = rxPsd;
    rxSsp->txAntenna =
        ConstCast<AntennaModel, const AntennaModel>(params.txAntenna->GetAntennaElement());

    // apply the fast fading and the beamforming gain
    rxPsd = m_spectrumLossModel->CalcRxPowerSpectralDensity(rxSsp,
                                                            params.txMob,
                                                            params.rxMob,
                                                            params.txAntenna,
                                                            params.rxAntenna);
    NS_LOG_DEBUG("Average rx power " << 10 * log10(Sum(*rxPsd) * params.bandwidth) << " dB");

    // compute the SNR
    NS_LOG_DEBUG("Average SNR " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " dB");

    // print the SNR and pathloss values in the ntn-snr-trace.txt file
    std::ofstream f;
    f.open("ntn-snr-trace.txt", std::ios::out | std::ios::app);
    f << Simulator::Now().GetSeconds() << " " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " "
      << propagationGainDb << std::endl;
    f.close();
}

int
main(int argc, char* argv[])
{
    uint32_t simTime = 1000; // simulation time in milliseconds
    uint32_t timeRes = 10;   // time resolution in milliseconds

    // SCENARIO 10 DL of TR 38.321
    // This parameters can be set accordingly to 3GPP TR 38.821 or arbitrarily modified

    std::string scenario = "NTN-Suburban"; // 3GPP propagation scenario
    // All available NTN scenarios: DenseUrban, Urban, Suburban, Rural.

    double frequency = 20e9;     // operating frequency in Hz
    double bandwidth = 400e6;    // Hz
    double RB_bandwidth = 120e3; // Hz

    // Satellite parameters
    double satEIRPDensity = 40;    // dBW/MHz
    double satAntennaGain = 58.5;  // dB
    double satAntennaDiameter = 5; // meters

    // UE Parameters
    double vsatAntennaGain = 39.7;       // dB
    double vsatAntennaDiameter = 0.6;    // meters
    double vsatAntennaNoiseFigure = 1.2; // dB

    // Calculate transmission power in dBm using EIRPDensity + 10*log10(Bandwidth) - AntennaGain +
    // 30
    double txPow = (satEIRPDensity + 10 * log10(bandwidth / 1e6) - satAntennaGain) + 30;

    NS_LOG_DEBUG("Transmitting power: " << txPow << "dBm, (" << pow(10., (txPow - 30) / 10)
                                        << "W)");

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(1))); // update the channel at each iteration
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(0.0))); // do not update the channel condition

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // create and configure the factories for the channel condition and propagation loss models
    ObjectFactory propagationLossModelFactory;
    ObjectFactory channelConditionModelFactory;

    // propagationLossModelFactory.SetTypeId (ThreeGppRmaPropagationLossModel::GetTypeId ());
    if (scenario == "NTN-DenseUrban")
    {
        propagationLossModelFactory.SetTypeId(
            ThreeGppNTNDenseUrbanPropagationLossModel::GetTypeId());
        channelConditionModelFactory.SetTypeId(
            ThreeGppNTNDenseUrbanChannelConditionModel::GetTypeId());
    }
    else if (scenario == "NTN-Urban")
    {
        propagationLossModelFactory.SetTypeId(ThreeGppNTNUrbanPropagationLossModel::GetTypeId());
        channelConditionModelFactory.SetTypeId(ThreeGppNTNUrbanChannelConditionModel::GetTypeId());
    }
    else if (scenario == "NTN-Suburban")
    {
        propagationLossModelFactory.SetTypeId(ThreeGppNTNSuburbanPropagationLossModel::GetTypeId());
        channelConditionModelFactory.SetTypeId(
            ThreeGppNTNSuburbanChannelConditionModel::GetTypeId());
    }
    else if (scenario == "NTN-Rural")
    {
        propagationLossModelFactory.SetTypeId(ThreeGppNTNRuralPropagationLossModel::GetTypeId());
        channelConditionModelFactory.SetTypeId(ThreeGppNTNRuralChannelConditionModel::GetTypeId());
    }
    else
    {
        NS_FATAL_ERROR("Unknown NTN scenario");
    }

    // create the propagation loss model
    m_propagationLossModel = propagationLossModelFactory.Create<ThreeGppPropagationLossModel>();
    m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
    m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(true));

    // create the spectrum propagation loss model
    m_spectrumLossModel = CreateObject<ThreeGppSpectrumPropagationLossModel>();
    m_spectrumLossModel->SetChannelModelAttribute("Frequency", DoubleValue(frequency));
    m_spectrumLossModel->SetChannelModelAttribute("Scenario", StringValue(scenario));

    // create the channel condition model and associate it with the spectrum and
    // propagation loss model
    Ptr<ChannelConditionModel> condModel =
        channelConditionModelFactory.Create<ThreeGppChannelConditionModel>();
    m_spectrumLossModel->SetChannelModelAttribute("ChannelConditionModel", PointerValue(condModel));
    m_propagationLossModel->SetChannelConditionModel(condModel);

    // create the tx and rx nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create the tx and rx devices
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();

    // associate the nodes and the devices
    nodes.Get(0)->AddDevice(txDev);
    txDev->SetNode(nodes.Get(0));
    nodes.Get(1)->AddDevice(rxDev);
    rxDev->SetNode(nodes.Get(1));

    // create the tx and rx mobility models, set the positions
    Ptr<GeocentricConstantPositionMobilityModel> txMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();
    Ptr<GeocentricConstantPositionMobilityModel> rxMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();

    txMob->SetGeographicPosition(Vector(45.40869, 11.89448, 35786000)); // GEO over Padova
    rxMob->SetGeographicPosition(Vector(45.40869, 11.89448, 14.0));     // Padova Coordinates

    // This is not strictly necessary, but is useful to have "sensible" values when using
    // GetPosition()
    txMob->SetCoordinateTranslationReferencePoint(
        Vector(45.40869, 11.89448, 0.0)); // Padova Coordinates without altitude
    rxMob->SetCoordinateTranslationReferencePoint(
        Vector(45.40869, 11.89448, 0.0)); // Padova Coordinates without altitude

    NS_LOG_DEBUG("TX Position: " << txMob->GetPosition());
    NS_LOG_DEBUG("RX Position: " << rxMob->GetPosition());

    // assign the mobility models to the nodes
    nodes.Get(0)->AggregateObject(txMob);
    nodes.Get(1)->AggregateObject(rxMob);

    // create the antenna objects and set their proprieties
    Ptr<PhasedArrayModel> txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(1),
        "NumRows",
        UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<CircularApertureAntennaModel>(
            "AntennaMaxGainDb",
            DoubleValue(satAntennaGain),
            "AntennaCircularApertureRadius",
            DoubleValue(satAntennaDiameter / 2),
            "OperatingFrequency",
            DoubleValue(frequency),
            "AntennaInclination",
            DoubleValue(DegreesToRadians(180))) // Points the ground
                     ));

    Ptr<PhasedArrayModel> rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(1),
        "NumRows",
        UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<CircularApertureAntennaModel>(
            "AntennaMaxGainDb",
            DoubleValue(vsatAntennaGain),
            "AntennaCircularApertureRadius",
            DoubleValue(vsatAntennaDiameter / 2),
            "OperatingFrequency",
            DoubleValue(frequency),
            "AntennaInclination",
            DoubleValue(DegreesToRadians(0))) // Points the sky
                     ));

    // set the beamforming vectors
    DoBeamforming(rxDev, rxAntenna, txDev);
    DoBeamforming(txDev, txAntenna, rxDev);

    for (int i = 0; i < floor(simTime / timeRes); i++)
    {
        Simulator::Schedule(MilliSeconds(timeRes * i),
                            &ComputeSnr,
                            ComputeSnrParams(txMob,
                                             rxMob,
                                             txPow,
                                             vsatAntennaNoiseFigure,
                                             txAntenna,
                                             rxAntenna,
                                             frequency,
                                             bandwidth,
                                             RB_bandwidth));
    }

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
