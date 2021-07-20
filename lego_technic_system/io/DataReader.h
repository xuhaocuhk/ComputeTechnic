//
// Created by student on 4/25/2019.
//

#ifndef LEGO_TECHNIC_MAIN_DATAREADER_H
#define LEGO_TECHNIC_MAIN_DATAREADER_H

#include <vector>
#include <bricks/BeamTemplate.h>

using namespace std;

namespace DataReader {

    std::vector<BeamTemplate *> readBeamsFromFile(string path);

    vector<Connector *> readConnectorsFromFile(string path);

    BeamTemplate *getBeamByID(string Id);

    Connector *getConnectorByID(string Id);

}

#endif //LEGO_TECHNIC_MAIN_DATAREADER_H
