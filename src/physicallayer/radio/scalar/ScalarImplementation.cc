//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "ScalarImplementation.h"
#include "Modulation.h"
#include "IRadioChannel.h"

Define_Module(ScalarRadioSignalAttenuation);
Define_Module(ScalarRadioBackgroundNoise);
Define_Module(ScalarRadioSignalReceiver);
Define_Module(ScalarRadioSignalTransmitter);

void ScalarRadioSignalTransmission::printToStream(std::ostream &stream) const
{
    RadioSignalTransmissionBase::printToStream(stream);
    stream << ", power = " << power << ", carrier frequency = " << carrierFrequency << ", bandwidth = " << bandwidth;
}

void ScalarRadioSignalReception::printToStream(std::ostream &stream) const
{
    RadioSignalReceptionBase::printToStream(stream);
    stream << ", power = " << power << ", carrier frequency = " << carrierFrequency << ", bandwidth = " << bandwidth;
}

W ScalarRadioSignalNoise::computeMaxPower(simtime_t startTime, simtime_t endTime) const
{
    W noisePower = W(0);
    W maxNoisePower = W(0);
    for (std::map<simtime_t, W>::const_iterator it = powerChanges->begin(); it != powerChanges->end(); it++)
    {
        noisePower += it->second;
        if (noisePower > maxNoisePower && startTime <= it->first && it->first <= endTime)
            maxNoisePower = noisePower;
    }
    return maxNoisePower;
}

const IRadioSignalReception *ScalarRadioSignalAttenuation::computeReception(const IRadio *receiverRadio, const IRadioSignalTransmission *transmission) const
{
    const IRadioChannel *channel = receiverRadio->getChannel();
    const IRadio *transmitterRadio = transmission->getTransmitter();
    const IRadioAntenna *receiverAntenna = receiverRadio->getAntenna();
    const IRadioAntenna *transmitterAntenna = transmitterRadio->getAntenna();
    const ScalarRadioSignalTransmission *scalarTransmission = check_and_cast<const ScalarRadioSignalTransmission *>(transmission);
    const IRadioSignalArrival *arrival = channel->getArrival(receiverRadio, transmission);
    const simtime_t receptionStartTime = arrival->getStartTime();
    const simtime_t receptionEndTime = arrival->getEndTime();
    const Coord receptionStartPosition = arrival->getStartPosition();
    const Coord receptionEndPosition = arrival->getEndPosition();
    const EulerAngles receptionStartOrientation = arrival->getStartOrientation();
    const EulerAngles receptionEndOrientation = arrival->getEndOrientation();
    const EulerAngles transmissionDirection = computeTransmissionDirection(transmission, arrival);
    const EulerAngles transmissionAntennaDirection = transmission->getStartOrientation() - transmissionDirection;
    const EulerAngles receptionAntennaDirection = transmissionDirection - arrival->getStartOrientation();
    double transmitterAntennaGain = transmitterAntenna->computeGain(transmissionAntennaDirection);
    double receiverAntennaGain = receiverAntenna->computeGain(receptionAntennaDirection);
    m distance = m(receptionStartPosition.distance(transmission->getStartPosition()));
    double pathLossFactor = channel->getPathLoss()->computePathLoss(channel->getPropagation()->getPropagationSpeed(), scalarTransmission->getCarrierFrequency(), distance);
    W transmissionPower = scalarTransmission->getPower();
    W receptionPower = transmissionPower * std::min(1.0, transmitterAntennaGain * receiverAntennaGain * pathLossFactor);
    return new ScalarRadioSignalReception(receiverRadio, transmission, receptionStartTime, receptionEndTime, receptionStartPosition, receptionEndPosition, receptionStartOrientation, receptionEndOrientation, scalarTransmission->getCarrierFrequency(), scalarTransmission->getBandwidth(), receptionPower);
}

void ScalarRadioBackgroundNoise::initialize(int stage)
{
    cModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        power = mW(FWMath::dBm2mW(par("power")));
    }
}

const IRadioSignalNoise *ScalarRadioBackgroundNoise::computeNoise(const IRadioSignalListening *listening) const
{
    const BandRadioSignalListening *bandListening = check_and_cast<const BandRadioSignalListening *>(listening);
    simtime_t startTime = listening->getStartTime();
    simtime_t endTime = listening->getEndTime();
    std::map<simtime_t, W> *powerChanges = new std::map<simtime_t, W>();
    powerChanges->insert(std::pair<simtime_t, W>(startTime, power));
    powerChanges->insert(std::pair<simtime_t, W>(endTime, -power));
    return new ScalarRadioSignalNoise(startTime, endTime, bandListening->getCarrierFrequency(), bandListening->getBandwidth(), powerChanges);
}

void ScalarRadioSignalListeningDecision::printToStream(std::ostream &stream) const
{
    RadioSignalListeningDecision::printToStream(stream);
    stream << ", maximum power = " << powerMax;
}

void ScalarRadioSignalTransmitter::initialize(int stage)
{
    FlatRadioSignalTransmitterBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        bitrate = bps(par("bitrate"));
        power = W(par("power"));
    }
}

void ScalarRadioSignalTransmitter::printToStream(std::ostream &stream) const
{
    stream << "scalar radio signal transmitter, "
           << "headerBitLength = " << headerBitLength << ", "
           << "bitrate = " << bitrate << ", "
           << "power = " <<  power << ", "
           << "carrierFrequency = " << carrierFrequency << ", "
           << "bandwidth = " << bandwidth;
}

const IRadioSignalTransmission *ScalarRadioSignalTransmitter::createTransmission(const IRadio *transmitter, const cPacket *macFrame, const simtime_t startTime) const
{
    RadioTransmissionRequest *controlInfo = dynamic_cast<RadioTransmissionRequest *>(macFrame->getControlInfo());
    W transmissionPower = controlInfo && !isNaN(controlInfo->getPower().get()) ? controlInfo->getPower() : power;
    bps transmissionBitrate = controlInfo && !isNaN(controlInfo->getBitrate().get()) ? controlInfo->getBitrate() : bitrate;
    const simtime_t duration = (macFrame->getBitLength() + headerBitLength) / transmissionBitrate.get();
    const simtime_t endTime = startTime + duration;
    IMobility *mobility = transmitter->getAntenna()->getMobility();
    const Coord startPosition = mobility->getCurrentPosition();
    const Coord endPosition = mobility->getCurrentPosition();
    const EulerAngles startOrientation = mobility->getCurrentAngularPosition();
    const EulerAngles endOrientation = mobility->getCurrentAngularPosition();
    return new ScalarRadioSignalTransmission(transmitter, macFrame, startTime, endTime, startPosition, endPosition, startOrientation, endOrientation, modulation, headerBitLength, macFrame->getBitLength(), carrierFrequency, bandwidth, transmissionBitrate, transmissionPower);
}

void ScalarRadioSignalReceiver::initialize(int stage)
{
    SNIRRadioSignalReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        energyDetection = mW(FWMath::dBm2mW(par("energyDetection")));
        sensitivity = mW(FWMath::dBm2mW(par("sensitivity")));
        carrierFrequency = Hz(par("carrierFrequency"));
        bandwidth = Hz(par("bandwidth"));
        const char *modulationName = par("modulation");
        if (strcmp(modulationName, "NULL")==0)
            modulation = new NullModulation();
        else if (strcmp(modulationName, "BPSK")==0)
            modulation = new BPSKModulation();
        else if (strcmp(modulationName, "16-QAM")==0)
            modulation = new QAM16Modulation();
        else if (strcmp(modulationName, "256-QAM")==0)
            modulation = new QAM256Modulation();
        else
            throw cRuntimeError(this, "Unknown modulation '%s'", modulationName);
    }
}

void ScalarRadioSignalReceiver::printToStream(std::ostream &stream) const
{
    stream << "scalar radio signal receiver, "
           << "energyDetection = " << energyDetection << ", "
           << "sensitivity = " <<  sensitivity << ", "
           << "carrierFrequency = " << carrierFrequency << ", "
           << "bandwidth = " << bandwidth;
}

const IRadioSignalListening *ScalarRadioSignalReceiver::createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition) const
{
    return new BandRadioSignalListening(radio, startTime, endTime, startPosition, endPosition, carrierFrequency, bandwidth);
}

bool ScalarRadioSignalReceiver::computeIsReceptionPossible(const IRadioSignalTransmission *transmission) const
{
    // TODO: check if modulation matches?
    const ScalarRadioSignalTransmission *scalarTransmission = check_and_cast<const ScalarRadioSignalTransmission *>(transmission);
    if (carrierFrequency == scalarTransmission->getCarrierFrequency() && bandwidth == scalarTransmission->getBandwidth())
        return true;
    else if (areOverlappingBands(carrierFrequency, bandwidth, scalarTransmission->getCarrierFrequency(), scalarTransmission->getBandwidth()))
        throw cRuntimeError("Overlapping bands are not supported");
    else
        return false;
}

// TODO: this is not purely functional, see interface comment
bool ScalarRadioSignalReceiver::computeIsReceptionPossible(const IRadioSignalReception *reception) const
{
    const ScalarRadioSignalReception *scalarReception = check_and_cast<const ScalarRadioSignalReception *>(reception);
    if (carrierFrequency == scalarReception->getCarrierFrequency() && bandwidth == scalarReception->getBandwidth())
        return scalarReception->getPower() >= sensitivity;
    else if (areOverlappingBands(carrierFrequency, bandwidth, scalarReception->getCarrierFrequency(), scalarReception->getBandwidth()))
        throw cRuntimeError("Overlapping bands are not supported");
    else
        return false;
}

const IRadioSignalNoise *ScalarRadioSignalReceiver::computeNoise(const IRadioSignalListening *listening, const std::vector<const IRadioSignalReception *> *receptions, const IRadioSignalNoise *backgroundNoise) const
{
    const BandRadioSignalListening *bandListening = check_and_cast<const BandRadioSignalListening *>(listening);
    Hz carrierFrequency = bandListening->getCarrierFrequency();
    Hz bandwidth = bandListening->getBandwidth();
    simtime_t noiseStartTime = SimTime::getMaxTime();
    simtime_t noiseEndTime = 0;
    std::map<simtime_t, W> *powerChanges = new std::map<simtime_t, W>();
    for (std::vector<const IRadioSignalReception *>::const_iterator it = receptions->begin(); it != receptions->end(); it++)
    {
        const ScalarRadioSignalReception *reception = check_and_cast<const ScalarRadioSignalReception *>(*it);
        if (carrierFrequency == reception->getCarrierFrequency() && bandwidth == reception->getBandwidth())
        {
            W power = reception->getPower();
            simtime_t startTime = reception->getStartTime();
            simtime_t endTime = reception->getEndTime();
            if (startTime < noiseStartTime)
                noiseStartTime = startTime;
            if (endTime > noiseEndTime)
                noiseEndTime = endTime;
            std::map<simtime_t, W>::iterator itStartTime = powerChanges->find(startTime);
            if (itStartTime != powerChanges->end())
                itStartTime->second += power;
            else
                powerChanges->insert(std::pair<simtime_t, W>(startTime, power));
            std::map<simtime_t, W>::iterator itEndTime = powerChanges->find(endTime);
            if (itEndTime != powerChanges->end())
                itEndTime->second -= power;
            else
                powerChanges->insert(std::pair<simtime_t, W>(endTime, -power));
        }
        else if (areOverlappingBands(carrierFrequency, bandwidth, reception->getCarrierFrequency(), reception->getBandwidth()))
            throw cRuntimeError("Overlapping bands are not supported");
    }
    if (backgroundNoise)
    {
        const ScalarRadioSignalNoise *scalarBackgroundNoise = check_and_cast<const ScalarRadioSignalNoise *>(backgroundNoise);
        if (carrierFrequency == scalarBackgroundNoise->getCarrierFrequency() && bandwidth == scalarBackgroundNoise->getBandwidth())
        {
            const std::map<simtime_t, W> *backgroundNoisePowerChanges = check_and_cast<const ScalarRadioSignalNoise *>(backgroundNoise)->getPowerChanges();
            for (std::map<simtime_t, W>::const_iterator it = backgroundNoisePowerChanges->begin(); it != backgroundNoisePowerChanges->end(); it++)
            {
                std::map<simtime_t, W>::iterator jt = powerChanges->find(it->first);
                if (jt != powerChanges->end())
                    jt->second += it->second;
                else
                    powerChanges->insert(std::pair<simtime_t, W>(it->first, it->second));
            }
        }
        else if (areOverlappingBands(carrierFrequency, bandwidth, scalarBackgroundNoise->getCarrierFrequency(), scalarBackgroundNoise->getBandwidth()))
            throw cRuntimeError("Overlapping bands are not supported");
    }
    return new ScalarRadioSignalNoise(noiseStartTime, noiseEndTime, carrierFrequency, bandwidth, powerChanges);
}

const IRadioSignalListeningDecision *ScalarRadioSignalReceiver::computeListeningDecision(const IRadioSignalListening *listening, const std::vector<const IRadioSignalReception *> *interferingReceptions, const IRadioSignalNoise *backgroundNoise) const
{
    const IRadioSignalNoise *noise = computeNoise(listening, interferingReceptions, backgroundNoise);
    const ScalarRadioSignalNoise *scalarNoise = check_and_cast<const ScalarRadioSignalNoise *>(noise);
    W maxPower = scalarNoise->computeMaxPower(listening->getStartTime(), listening->getEndTime());
    delete noise;
    return new ScalarRadioSignalListeningDecision(listening, maxPower >= energyDetection, maxPower);
}

// TODO: this is not purely functional, see interface comment
bool ScalarRadioSignalReceiver::computeHasBitError(double minSNIR, int bitLength, double bitrate) const
{
    double ber = modulation->calculateBER(minSNIR, bandwidth.get(), bitrate);
    if (ber == 0.0)
        return false;
    else
    {
        double pErrorless = pow(1.0 - ber, bitLength);
        return dblrand() > pErrorless;
    }
}

bool ScalarRadioSignalReceiver::computeIsReceptionSuccessful(const IRadioSignalReception *reception, const RadioReceptionIndication *indication) const
{
    const ScalarRadioSignalTransmission *scalarTransmission = check_and_cast<const ScalarRadioSignalTransmission *>(reception->getTransmission());
    return SNIRRadioSignalReceiverBase::computeIsReceptionSuccessful(reception, indication) &&
           !computeHasBitError(indication->getMinSNIR(), scalarTransmission->getPayloadBitLength(), scalarTransmission->getBitrate().get());
}

const IRadioSignalReceptionDecision *ScalarRadioSignalReceiver::computeReceptionDecision(const IRadioSignalListening *listening, const IRadioSignalReception *reception, const std::vector<const IRadioSignalReception *> *interferingReceptions, const IRadioSignalNoise *backgroundNoise) const
{
    const BandRadioSignalListening *bandListening = check_and_cast<const BandRadioSignalListening *>(listening);
    const ScalarRadioSignalReception *scalarReception = check_and_cast<const ScalarRadioSignalReception *>(reception);
    if (bandListening->getCarrierFrequency() == scalarReception->getCarrierFrequency() && bandListening->getBandwidth() == scalarReception->getBandwidth())
        return SNIRRadioSignalReceiverBase::computeReceptionDecision(listening, reception, interferingReceptions, backgroundNoise);
    else if (areOverlappingBands(bandListening->getCarrierFrequency(), bandListening->getBandwidth(), scalarReception->getCarrierFrequency(), scalarReception->getBandwidth()))
        throw cRuntimeError("Overlapping bands are not supported");
    else
        return new RadioSignalReceptionDecision(reception, new RadioReceptionIndication(), false, false, false);
}

double ScalarRadioSignalReceiver::computeMinSNIR(const IRadioSignalReception *reception, const IRadioSignalNoise *noise) const
{
    const ScalarRadioSignalNoise *scalarNoise = check_and_cast<const ScalarRadioSignalNoise *>(noise);
    const ScalarRadioSignalReception *scalarReception = check_and_cast<const ScalarRadioSignalReception *>(reception);
    return unit(scalarReception->getPower() / scalarNoise->computeMaxPower(reception->getStartTime(), reception->getEndTime())).get();
}
