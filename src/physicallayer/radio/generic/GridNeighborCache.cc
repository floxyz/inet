#include "GridNeighborCache.h"

Define_Module(GridNeighborCache);

void GridNeighborCache::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
    {
        // TODO: NED parameter?
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
        useMaxDimension = par("useMaxDimension");

        init();

    }
    else if (stage == INITSTAGE_LINK_LAYER_2) // TODO: is it the correct stage to do this?
    {
        fillCubeVector();
        refillCellsTimer = new cMessage("refillCellsTimer");
        scheduleAt(simTime() + refillPeriod, refillCellsTimer);
    }
}

void GridNeighborCache::fillCubeVector()
{
    for (unsigned int i = 0; i < numberOfCells; i++)
        grid[i].clear();

    for (unsigned int i = 0; i < radios.size(); i++)
    {
        const IRadio *radio = radios[i];
        Coord radioPos = radio->getAntenna()->getMobility()->getCurrentPosition();
        grid[posToCubeId(radioPos)].push_back(radio);
    }
}

unsigned int GridNeighborCache::posToCubeId(Coord pos)
{
    // map coordinates to indices

    unsigned int xIndex = pos.x * dimension[0] / sideLengths.x;
    unsigned int yIndex = pos.y * dimension[1] / sideLengths.y;
    unsigned int zIndex = pos.z * dimension[2] / sideLengths.z;

    return rowmajorIndex(xIndex, yIndex, zIndex);
}

unsigned int GridNeighborCache::rowmajorIndex(unsigned int xIndex, unsigned int yIndex, unsigned int zIndex)
{
    unsigned int coord[3] = {xIndex, yIndex, zIndex};
    unsigned int ind = 0;

    for (unsigned int k = 0; k < 3; k++)
    {
        unsigned int prodDim = 1;

        for (unsigned int l = k + 1; l < 3; l++)
            if (dimension[l] > 0)
                prodDim *= dimension[l];

        ind += prodDim * coord[k];
    }

    return ind;
}

Coord GridNeighborCache::decodeRowmajorIndex(unsigned int ind)
{
    int coord[3] = {-1, -1, -1};
    for (unsigned int k = 0; k < 3; k++)
    {
        unsigned int prodDim = 1;

        for (unsigned int l = k + 1; l < 3; l++)
            if (dimension[l] > 0)
                prodDim *= dimension[l];

        coord[k] = ind / prodDim;
        ind %= prodDim;
    }
    return Coord(coord[0], coord[1], coord[2]);
}

void GridNeighborCache::handleMessage(cMessage *msg)
{
    if (!msg->isSelfMessage())
        throw cRuntimeError("This module only handles self messages");

    EV_DETAIL << "Updating the grid cells" << endl;

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
        // TODO: is it an error?
    }
}

void GridNeighborCache::sendToNeighbors(IRadio *transmitter, const IRadioFrame *frame)
{
    double radius = range + (maxSpeed * refillPeriod);
    Coord transmitterPos = transmitter->getAntenna()->getMobility()->getCurrentPosition();

    // we have to measure the radius in cells
    int xCells = sideLengths.x == 0 ? 0 : ceil((radius * dimension[0]) / sideLengths.x);
    int yCells = sideLengths.y == 0 ? 0 : ceil((radius * dimension[1]) / sideLengths.y);
    int zCells = sideLengths.z == 0 ? 0 : ceil((radius * dimension[2]) / sideLengths.z);

    // decode the row-major index to matrix indices
    // for easier calculations
    Coord transmitterMatCoord = decodeRowmajorIndex(posToCubeId(transmitterPos));

    int transmitterMatICoord = transmitterMatCoord.x;
    int transmitterMatJCoord = transmitterMatCoord.y;
    int transmitterMatKCoord = transmitterMatCoord.z;

    // the start and the end positions of the smallest rectangle which contains our ball with
    // the radius calculated above
    int iStart = transmitterMatICoord - xCells < 0 ? 0 : transmitterMatICoord - xCells;
    int iEnd = transmitterMatICoord + xCells >= dimension[0] ? dimension[0] - 1 : transmitterMatICoord + xCells;

    int jStart = transmitterMatJCoord - yCells < 0 ? 0 : transmitterMatJCoord - yCells;
    int jEnd = transmitterMatJCoord + yCells >= dimension[1] ? dimension[1] - 1 : transmitterMatJCoord + yCells;

    int kStart = transmitterMatKCoord - zCells < 0 ? 0 : transmitterMatKCoord - zCells;
    int kEnd = transmitterMatKCoord + zCells >= dimension[2] ? dimension[2] - 1: transmitterMatKCoord + zCells;

    // dimension[X] - 1 equals to 1 if dimension[X] = 0
    if (iEnd < 0) iEnd = 0;
    if (jEnd < 0) jEnd = 0;
    if (kEnd < 0) kEnd = 0;

    // sending frame to the neighbor cells

    for (int i = iStart; i <= iEnd; i++)
    {
        for (int j = jStart; j <= jEnd; j++)
        {
            for (int k = kStart; k <= kEnd; k++)
            {
                int cellID = rowmajorIndex(i,j,k);
                Radios *neighborCube = &grid[cellID];
                unsigned int cellSize = neighborCube->size();

                for (unsigned int l = 0; l < cellSize; l++)
                    radioChannel->sendToRadio(transmitter, (*neighborCube)[l], frame);
            }
        }
    }

}

void GridNeighborCache::calculateDimension(int *dim)
{
    int xDim = sideLengths.x / splittingUnits.x;
    int yDim = sideLengths.y / splittingUnits.y;
    int zDim = sideLengths.z / splittingUnits.z;

    if (useMaxDimension)
    {
        int maxDim = std::max(std::max(xDim,yDim),zDim);

        xDim = xDim == 0 ? xDim : maxDim;
        yDim = yDim == 0 ? yDim : maxDim;
        zDim = zDim == 0 ? zDim : maxDim;
    }

    dim[0] = xDim;
    dim[1] = yDim;
    dim[2] = zDim;
}

Coord GridNeighborCache::calculateSideLength()
{
    return Coord(constraintAreaMax.x - constraintAreaMin.x, constraintAreaMax.y - constraintAreaMin.y,
            constraintAreaMax.z - constraintAreaMin.z);
}

unsigned int GridNeighborCache::calculateNumberOfCells()
{
    unsigned int prodDim = 1;
    for (int i = 0; i < 3; i++)
        if (dimension[i] != 0)
            prodDim *= dimension[i];

    return prodDim;
}

void GridNeighborCache::init()
{
    sideLengths = calculateSideLength();
    calculateDimension(dimension);
    numberOfCells = calculateNumberOfCells();
    grid.resize(numberOfCells);
}

GridNeighborCache::~GridNeighborCache()
{
    cancelAndDelete(refillCellsTimer);
}

