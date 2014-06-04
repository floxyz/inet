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

#ifndef __INET_SCALARIMPLEMENTATION_H_
#define __INET_SCALARIMPLEMENTATION_H_

#include "ImplementationBase.h"
#include "GenericImplementation.h"
#include "IModulation.h"
#include "Radio.h"

class INET_API ScalarRadioSignalTransmission : public RadioSignalTransmissionBase
{
    protected:
        const IModulation *modulation;
        const int headerBitLength;
        const int payloadBitLength;
        const bps bitrate;
        const W power;
        const Hz carrierFrequency;
        const Hz bandwidth;

    public:
        ScalarRadioSignalTransmission(const IRadio *transmitter, const cPacket *macFrame, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition, const IModulation *modulation, int headerBitLength, int payloadBitLength, bps bitrate, W power, Hz carrierFrequency, Hz bandwidth) :
            RadioSignalTransmissionBase(transmitter, macFrame, startTime, endTime, startPosition, endPosition),
            modulation(modulation),
            headerBitLength(headerBitLength),
            payloadBitLength(payloadBitLength),
            bitrate(bitrate),
            power(power),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual void printToStream(std::ostream &stream) const;

        virtual const IModulation *getModulation() const { return modulation; }
        virtual int getHeaderBitLength() const { return headerBitLength; }
        virtual int getPayloadBitLength() const { return payloadBitLength; }
        virtual bps getBitrate() const { return bitrate; }
        virtual W getPower() const { return power; }
        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual Hz getBandwidth() const { return bandwidth; }
};

class INET_API ScalarRadioSignalListening : public RadioSignalListeningBase
{
    protected:
        const Hz carrierFrequency;
        const Hz bandwidth;

    public:
        ScalarRadioSignalListening(const IRadio *radio, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition, Hz carrierFrequency, Hz bandwidth) :
            RadioSignalListeningBase(radio, startTime, endTime, startPosition, endPosition),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual Hz getBandwidth() const { return bandwidth; }
};

class INET_API ScalarRadioSignalReception : public RadioSignalReceptionBase
{
    protected:
        const W power;
        const Hz carrierFrequency;
        const Hz bandwidth;

    public:
        ScalarRadioSignalReception(const IRadio *radio, const IRadioSignalTransmission *transmission, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition, W power, Hz carrierFrequency, Hz bandwidth) :
            RadioSignalReceptionBase(radio, transmission, startTime, endTime, startPosition, endPosition),
            power(power),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual void printToStream(std::ostream &stream) const;

        virtual W getPower() const { return power; }
        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual Hz getBandwidth() const { return bandwidth; }
};

class INET_API ScalarRadioSignalNoise : public RadioSignalNoiseBase
{
    protected:
        const std::map<simtime_t, W> *powerChanges;
        const Hz carrierFrequency;
        const Hz bandwidth;

    public:
        ScalarRadioSignalNoise(simtime_t startTime, simtime_t endTime, const std::map<simtime_t, W> *powerChanges, Hz carrierFrequency, Hz bandwidth) :
            RadioSignalNoiseBase(startTime, endTime),
            powerChanges(powerChanges),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual ~ScalarRadioSignalNoise() { delete powerChanges; }

        virtual void printToStream(std::ostream &stream) const { stream << "scalar noise"; }
        virtual const std::map<simtime_t, W> *getPowerChanges() const { return powerChanges; }
        virtual W computeMaxPower(simtime_t startTime, simtime_t endTime) const;
        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual Hz getBandwidth() const { return bandwidth; }
};

class INET_API ScalarRadioSignalAttenuationBase : public virtual IRadioSignalAttenuation
{
    public:
        virtual const IRadioSignalReception *computeReception(const IRadio *radio, const IRadioSignalTransmission *transmission) const;

        virtual double computeLoss(const IRadioSignalTransmission *transmission, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition) const = 0;
};

class INET_API ScalarRadioSignalFreeSpaceAttenuation : public RadioSignalFreeSpaceAttenuationBase, public ScalarRadioSignalAttenuationBase
{
    public:
        ScalarRadioSignalFreeSpaceAttenuation() :
            RadioSignalFreeSpaceAttenuationBase()
        {}

        ScalarRadioSignalFreeSpaceAttenuation(double alpha) :
            RadioSignalFreeSpaceAttenuationBase(alpha)
        {}

        virtual void printToStream(std::ostream &stream) const { stream << "scalar free space attenuation"; }

        virtual double computeLoss(const IRadioSignalTransmission *transmission, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition) const;
};

class INET_API ScalarRadioSignalCompoundAttenuation : public CompoundAttenuationBase, public ScalarRadioSignalAttenuationBase
{
    public:
        ScalarRadioSignalCompoundAttenuation(const std::vector<const IRadioSignalAttenuation *> *elements) :
            CompoundAttenuationBase(elements)
        {}

        virtual double computeLoss(const IRadioSignalTransmission *transmission, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition) const;
};

class INET_API ScalarRadioBackgroundNoise : public cSimpleModule, public IRadioBackgroundNoise
{
    protected:
        W power;

    protected:
        virtual void initialize(int stage);

    public:
        ScalarRadioBackgroundNoise() :
            power(W(sNaN))
        {}

        ScalarRadioBackgroundNoise(W power) :
            power(power)
        {}

        virtual void printToStream(std::ostream &stream) const { stream << "scalar background noise"; }

        virtual W getPower() const { return power; }

        virtual const IRadioSignalNoise *computeNoise(const IRadioSignalListening *listening) const;
        virtual const IRadioSignalNoise *computeNoise(const IRadioSignalReception *reception) const;
};

class INET_API ScalarRadioSignalListeningDecision : public RadioSignalListeningDecision
{
    protected:
        const W powerMax;

    public:
        ScalarRadioSignalListeningDecision(const IRadioSignalListening *listening, bool isListeningPossible, W powerMax) :
            RadioSignalListeningDecision(listening, isListeningPossible),
            powerMax(powerMax)
        {}

        virtual void printToStream(std::ostream &stream) const;

        virtual W getPowerMax() const { return powerMax; }
};

class INET_API ScalarRadioSignalTransmitter: public RadioSignalTransmitterBase
{
    protected:
        const IModulation *modulation;
        int headerBitLength;
        bps bitrate;
        W power;
        Hz carrierFrequency;
        Hz bandwidth;

    protected:
        virtual void initialize(int stage);

    public:
        ScalarRadioSignalTransmitter() :
            modulation(NULL),
            headerBitLength(-1),
            bitrate(sNaN),
            power(W(sNaN)),
            carrierFrequency(Hz(sNaN)),
            bandwidth(Hz(sNaN))
        {}

        ScalarRadioSignalTransmitter(const IModulation *modulation, int headerBitLength, bps bitrate, W power, Hz carrierFrequency, Hz bandwidth) :
            modulation(modulation),
            headerBitLength(headerBitLength),
            bitrate(bitrate),
            power(power),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual void printToStream(std::ostream &stream) const;

        virtual const IRadioSignalTransmission *createTransmission(const IRadio *radio, const cPacket *packet, const simtime_t startTime) const;

        virtual const IModulation *getModulation() const { return modulation; }

        virtual int getHeaderBitLength() const { return headerBitLength; }
        virtual void setHeaderBitLength(int headerBitLength) { this->headerBitLength = headerBitLength; }

        virtual bps getBitrate() const { return bitrate; }
        virtual void setBitrate(bps bitrate) { this->bitrate = bitrate; }

        virtual W getPower() const { return power; }
        virtual void setPower(W power) { this->power = power; }

        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual void setCarrierFrequency(Hz carrierFrequency) { this->carrierFrequency = carrierFrequency; }

        virtual Hz getBandwidth() const { return bandwidth; }
        virtual void setBandwidth(Hz bandwidth) { this->bandwidth = bandwidth; }
};

class INET_API ScalarRadioSignalReceiver : public SNIRRadioSignalReceiverBase
{
    protected:
        const IModulation *modulation;
        W energyDetection;
        W sensitivity;
        Hz carrierFrequency;
        Hz bandwidth;

    protected:
        virtual void initialize(int stage);

        virtual bool areOverlappingBands(Hz carrierFrequency1, Hz bandwidth1, Hz carrierFrequency2, Hz bandwidth2) const;
        virtual bool computeIsReceptionPossible(const IRadioSignalTransmission *transmission) const;
        virtual bool computeIsReceptionPossible(const IRadioSignalReception *reception) const;
        virtual bool computeIsReceptionSuccessful(const IRadioSignalReception *reception, const RadioReceptionIndication *indication) const;

        virtual const IRadioSignalNoise *computeNoise(const IRadioSignalListening *listening, const std::vector<const IRadioSignalReception *> *receptions, const IRadioSignalNoise *backgroundNoise) const;
        virtual double computeMinSNIR(const IRadioSignalReception *reception, const IRadioSignalNoise *noise) const;
        virtual bool computeHasBitError(double minSNIR, int bitLength, double bitrate) const;

    public:
        ScalarRadioSignalReceiver() :
            SNIRRadioSignalReceiverBase(),
            modulation(NULL),
            energyDetection(W(sNaN)),
            sensitivity(W(sNaN)),
            carrierFrequency(Hz(sNaN)),
            bandwidth(Hz(sNaN))
        {}

        ScalarRadioSignalReceiver(const IModulation *modulation, double snirThreshold, W energyDetection, W sensitivity, Hz carrierFrequency, Hz bandwidth) :
            SNIRRadioSignalReceiverBase(snirThreshold),
            modulation(modulation),
            energyDetection(energyDetection),
            sensitivity(sensitivity),
            carrierFrequency(carrierFrequency),
            bandwidth(bandwidth)
        {}

        virtual ~ScalarRadioSignalReceiver() { delete modulation; }

        virtual void printToStream(std::ostream &stream) const;

        virtual const IRadioSignalListening *createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition) const;

        virtual const IRadioSignalListeningDecision *computeListeningDecision(const IRadioSignalListening *listening, const std::vector<const IRadioSignalReception *> *interferingReceptions, const IRadioSignalNoise *backgroundNoise) const;
        virtual const IRadioSignalReceptionDecision *computeReceptionDecision(const IRadioSignalListening *listening, const IRadioSignalReception *reception, const std::vector<const IRadioSignalReception *> *interferingReceptions, const IRadioSignalNoise *backgroundNoise) const;

        virtual const IModulation *getModulation() const { return modulation; }

        virtual Hz getCarrierFrequency() const { return carrierFrequency; }
        virtual void setCarrierFrequency(Hz carrierFrequency) { this->carrierFrequency = carrierFrequency; }

        virtual Hz getBandwidth() const { return bandwidth; }
        virtual void setBandwidth(Hz bandwidth) { this->bandwidth = bandwidth; }
};

#endif
