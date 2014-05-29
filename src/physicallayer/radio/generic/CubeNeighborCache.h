#ifndef CUBENEIGHBORCACHE_H_
#define CUBENEIGHBORCACHE_H_

#include "RadioChannel.h"

class INET_API CubeNeighborCache : public INeighborCache, public cListener, public cSimpleModule
{
    public:
        typedef std::vector<const IRadio *> Radios;
        typedef std::vector<Radios > RadioGrid;

    protected:
        RadioGrid cube;
        Radios radios;

        int numberOfRectangles;
        Coord constraintAreaMin, constraintAreaMax;
        m range;
        ms maxSpeed;
        cMessage *fillRectanglesTimer;
        simtime_t refillPeriod;

    protected:
        virtual void initialize(int stage);
        void fillCubeVector();
        int pos2CubeId(Coord pos);
        virtual void handleSelfMessage(cMessage *msg);

    public:
        CubeNeighborCache(int numberOfRectangles);
        virtual ~CubeNeighborCache();
};

#endif /* CUBENEIGHBORCACHE_H_ */
