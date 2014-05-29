#include "CubeNeighborCache.h"

CubeNeighborCache::CubeNeighborCache(int numberOfRectangles) :
                                      fillRectanglesTimer(NULL)
{
    if (numberOfRectangles <= 0)
        throw cRuntimeError("numberOfRectangles must be a strictly positive integer");

    cube.resize(numberOfRectangles);
    fillRectanglesTimer = new cMessage("fillRectanglesTimer");
}


void CubeNeighborCache::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {
        refillPeriod = par("refillPeriod");
    }
    else if (stage == INITSTAGE_LINK_LAYER_2)
    {
        fillCubeVector();
        scheduleAt(simTime() + refillPeriod, fillRectanglesTimer);
    }
}

void CubeNeighborCache::fillCubeVector()
{
    for (int i = 0; i < numberOfRectangles; i++)
        cube[i].clear();

    for (unsigned int i = 0; i < radios.size(); i++)
    {
        const IRadio * radio = radios[i];
        Coord radioPos = radio->getAntenna()->getMobility()->getCurrentPosition();
        cube[pos2CubeId(radioPos)].push_back(radio);
    }
}

int CubeNeighborCache::pos2CubeId(Coord pos)
{
    double xD = 3.0;
    double yD = 3.0;
    double zD = 3.0;

    double stepX = (constraintAreaMax.x - constraintAreaMin.x) / xD;
    double stepY = (constraintAreaMax.y - constraintAreaMin.y) / yD;
    double stepZ = (constraintAreaMax.z - constraintAreaMin.z) / zD;

    return (yD * zD) * floor(pos.x / stepX)  + zD * floor(pos.y / stepY) + floor(pos.z / stepZ);

}

void CubeNeighborCache::handleSelfMessage(cMessage *msg)
{
    if (!msg->isSelfMessage())
        throw cRuntimeError("This module only handles self messages");

    fillCubeVector();

    scheduleAt(simTime() + refillPeriod, msg);
}

CubeNeighborCache::~CubeNeighborCache()
{
    cancelAndDelete(fillRectanglesTimer);
}

