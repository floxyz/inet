#ifndef GRIDNEIGHBORCACHE_H_
#define GRIDNEIGHBORCACHE_H_

#include "RadioChannel.h"

class INET_API GridNeighborCache : public RadioChannel::INeighborCache, public cListener, public cSimpleModule
{
    public:
        typedef std::vector<const IRadio *> Radios;
        typedef std::vector<Radios > RadioGrid;

    protected:
        RadioGrid grid;
        Radios radios;
        RadioChannel *radioChannel;

        unsigned int numberOfCells;
        Coord constraintAreaMin, constraintAreaMax;
        double range;
        double maxSpeed;
        cMessage *fillRectanglesTimer;
        double refillPeriod;

        Coord splittingUnits;
        Coord dimension;
        Coord sideLengths;

    protected:
        virtual int numInitStages() const { return NUM_INIT_STAGES; }
        virtual void initialize(int stage);
        virtual void handleMessage(cMessage *msg);
        void fillCubeVector();

        Coord calculateSideLength();
        Coord calculateDimension();
        unsigned int calculateNumberOfCells();
        unsigned int posToCubeId(Coord pos);

        unsigned int rowmajorIndex(unsigned int xIndex, unsigned int yIndex, unsigned int zIndex);
        Coord decodeRowmajorIndex(unsigned int ind);

    public:
        void addRadio(const IRadio *radio);
        void removeRadio(const IRadio *radio);
        void sendToNeighbors(IRadio *transmitter, const IRadioFrame *frame);

        GridNeighborCache();
        virtual ~GridNeighborCache();

};

#endif /* GRIDNEIGHBORCACHE_H_ */
