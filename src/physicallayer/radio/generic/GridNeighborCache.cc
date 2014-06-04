#include "GridNeighborCache.h"
#include <algorithm>
Define_Module(GridNeighborCache);

GridNeighborCache::GridNeighborCache() :
                                      fillRectanglesTimer(NULL)
{

}



void GridNeighborCache::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {
        // TODO:
        radioChannel = check_and_cast<RadioChannel*>(getParentModule());

        constraintAreaMin.x = par("constraintAreaMinX");
        constraintAreaMin.y = par("constraintAreaMinY");
        constraintAreaMin.z = par("constraintAreaMinZ");
        constraintAreaMax.x = par("constraintAreaMaxX");
        constraintAreaMax.y = par("constraintAreaMaxY");
        constraintAreaMax.z = par("constraintAreaMaxZ");

        splittingUnits.x = par("xSplittingUnit");
        splittingUnits.y = par("ySplittingUnit");
        splittingUnits.z = par("zSplittingUnit");

        range = par("range");
        maxSpeed = par("maxSpeed");
        refillPeriod = par("refillPeriod");

        // todo: the order of the method calls is crucial
        sideLengths = calculateSideLength();
        dimension = calculateDimension();
        numberOfCells = calculateNumberOfCells();
        grid.resize(numberOfCells);

    }
    else if (stage == INITSTAGE_LINK_LAYER_2)
    {
        fillCubeVector();
        fillRectanglesTimer = new cMessage("fillRectanglesTimer");
        scheduleAt(simTime() + refillPeriod, fillRectanglesTimer);
    }
}

void GridNeighborCache::fillCubeVector()
{
    for (unsigned int i = 0; i < numberOfCells; i++)
        grid[i].clear();

    for (unsigned int i = 0; i < radios.size(); i++)
    {
        const IRadio * radio = radios[i];
        Coord radioPos = radio->getAntenna()->getMobility()->getCurrentPosition();
        grid[posToCubeId(radioPos)].push_back(radio);
    }
}

unsigned int GridNeighborCache::posToCubeId(Coord pos)
{
    // map coordinates to indices

    unsigned int xIndex = pos.x * dimension.x / sideLengths.x;
    unsigned int yIndex = pos.y * dimension.y / sideLengths.y;
    unsigned int zIndex = pos.z * dimension.z / sideLengths.z;

    return rowmajorIndex(xIndex, yIndex, zIndex);
}

unsigned int GridNeighborCache::rowmajorIndex(unsigned int xIndex, unsigned int yIndex, unsigned int zIndex)
{
    unsigned int coord[3] = {xIndex, yIndex, zIndex};
    unsigned int dim[3] = {(unsigned int)dimension.x, (unsigned int)dimension.y, (unsigned int)dimension.z};
    unsigned int ind = 0;

    for (unsigned int k = 0; k < 3; k++)
    {
        unsigned int prodDim = 1;

        for (unsigned int l = k + 1; l < 3; l++)
            if (dim[l] > 0)
                prodDim *= dim[l];

        ind += prodDim * coord[k];
    }

    return ind;
}

Coord GridNeighborCache::decodeRowmajorIndex(unsigned int ind)
{
    unsigned int dim[3] = {(unsigned int)dimension.x, (unsigned int)dimension.y, (unsigned int)dimension.z};
    int coord[3];

    for (unsigned int k = 0; k < 3; k++)
    {
        unsigned int prodDim = 1;

        for (unsigned int l = k + 1; l < 3; l++)
            if (dim[l] > 0)
                prodDim *= dim[l];

        coord[k] = ind / prodDim;
        ind %= prodDim;
    }
    return Coord(coord[0], coord[1], coord[2]);
}

void GridNeighborCache::handleMessage(cMessage *msg)
{
    if (!msg->isSelfMessage())
        throw cRuntimeError("This module only handles self messages");

    EV_DETAIL << "Update neighbors" << endl;

    fillCubeVector();

    scheduleAt(simTime() + refillPeriod, msg);
}

void GridNeighborCache::addRadio(const IRadio *radio)
{
    radios.push_back(radio);
}

void GridNeighborCache::removeRadio(const IRadio *radio)
{
    Radios::iterator it = find(radios.begin(), radios.end(), radio);
    if (it != radios.end())
        radios.erase(it);
    else
    {
        // error?
    }
}



void GridNeighborCache::sendToNeighbors(IRadio* transmitter, const IRadioFrame* frame)
{
    std::vector<int> neighborCubeIDs;
    Coord transmitterPos = transmitter->getAntenna()->getMobility()->getCurrentPosition();
    fillNeighborVector(transmitterPos, neighborCubeIDs);
    for (unsigned int i = 0; i < neighborCubeIDs.size(); i++)
    {
        std::cout << "id: " << neighborCubeIDs[i] << endl;
        Radios *neighborCube = &grid[neighborCubeIDs[i]];
        for (unsigned int j = 0; j < neighborCube->size(); j++)
            radioChannel->sendToRadio(transmitter, (*neighborCube)[j], frame);
    }
}

void GridNeighborCache::fillNeighborVector(Coord transmitterPos, std::vector<int>& neighborCubeIDs)
{
    double radius = range + (maxSpeed * refillPeriod);

    int xCells = sideLengths.x == 0 ? 0 : ceil((radius * dimension.x) / sideLengths.x);
    int yCells = sideLengths.y == 0 ? 0 : ceil((radius * dimension.y) / sideLengths.y);
    int zCells = sideLengths.z == 0 ? 0 : ceil((radius * dimension.z) / sideLengths.z);

    Coord transmitterMatCoord = decodeRowmajorIndex(posToCubeId(transmitterPos));

    // decodes the row-major index to matrix indices
    // for easier calculations

    int transmitterMatICoord = transmitterMatCoord.x;
    int transmitterMatJCoord = transmitterMatCoord.y;
    int transmitterMatKCoord = transmitterMatCoord.z;

    int iStart = transmitterMatICoord - xCells < 0 ? 0 : transmitterMatICoord - xCells;
    int iEnd = transmitterMatICoord + xCells >= dimension.x ? dimension.x - 1 : transmitterMatICoord + xCells;

    int jStart = transmitterMatJCoord - yCells < 0 ? 0 : transmitterMatJCoord - yCells;
    int jEnd = transmitterMatJCoord + yCells >= dimension.y ? dimension.y - 1 : transmitterMatJCoord + yCells;

    int kStart = transmitterMatKCoord - zCells < 0 ? 0 : transmitterMatKCoord - zCells;
    int kEnd = transmitterMatKCoord + zCells >= dimension.z ? dimension.z - 1: transmitterMatKCoord + zCells;

    if (iEnd < 0) iEnd = 0;
    if (jEnd < 0) jEnd = 0;
    if (kEnd < 0) kEnd = 0;

    // collecting the neighbor cells
    for (int i = iStart; i <= iEnd; i++)
        for (int j = jStart; j <= jEnd; j++)
            for (int k = kStart; k <= kEnd; k++)
                neighborCubeIDs.push_back(rowmajorIndex(i,j,k));

}

Coord GridNeighborCache::calculateDimension()
{
    int xDim = sideLengths.x / splittingUnits.x;
    int yDim = sideLengths.y / splittingUnits.y;
    int zDim = sideLengths.z / splittingUnits.z;

    if (true) // todo:
    {
        int maxDim = std::max(std::max(xDim,yDim),zDim);

        xDim = xDim == 0 ? xDim : maxDim;
        yDim = yDim == 0 ? yDim : maxDim;
        zDim = zDim == 0 ? zDim : maxDim;
    }
    return Coord(xDim, yDim, zDim);
}

Coord GridNeighborCache::calculateSideLength()
{
    return Coord(constraintAreaMax.x - constraintAreaMin.x, constraintAreaMax.y - constraintAreaMin.y,
            constraintAreaMax.z - constraintAreaMin.z);
}

unsigned int GridNeighborCache::calculateNumberOfCells()
{
    int xDim = dimension.x != 0 ? dimension.x : 1;
    int yDim = dimension.y != 0 ? dimension.y : 1;
    int zDim = dimension.z != 0 ? dimension.z : 1;
    return (unsigned int)(xDim * yDim * zDim);
}

GridNeighborCache::~GridNeighborCache()
{
    cancelAndDelete(fillRectanglesTimer);
}

