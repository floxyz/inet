#ifndef GRIDNEIGHBORCACHE_H_
#define GRIDNEIGHBORCACHE_H_

#include "RadioChannel.h"

class INET_API GridNeighborCache : public RadioChannel::INeighborCache, public cSimpleModule
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
        cMessage *refillCellsTimer;
        double refillPeriod;
        bool useMaxDimension;

        Coord splittingUnits;
        Coord sideLengths;
        int dimension[3];

    protected:
        virtual int numInitStages() const { return NUM_INIT_STAGES; }
        virtual void initialize(int stage);
        virtual void handleMessage(cMessage *msg);

        void fillCubeVector();
        void init();
        Coord calculateSideLength();
        void calculateDimension(int *dim);
        unsigned int calculateNumberOfCells();
        unsigned int posToCubeId(Coord pos);

        unsigned int rowmajorIndex(unsigned int xIndex, unsigned int yIndex, unsigned int zIndex);
        Coord decodeRowmajorIndex(unsigned int ind);

    public:
        void addRadio(const IRadio *radio);
        void removeRadio(const IRadio *radio);
        void sendToNeighbors(IRadio *transmitter, const IRadioFrame *frame);

        GridNeighborCache() : refillCellsTimer(NULL) {};
        virtual ~GridNeighborCache();

};

#endif /* GRIDNEIGHBORCACHE_H_ */
