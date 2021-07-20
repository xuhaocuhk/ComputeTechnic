//
// Created by 徐豪 on 2017/10/27.
//

#ifndef LEGOGENERATION_MAIN1_BEAMFACTORY_H
#define LEGOGENERATION_MAIN1_BEAMFACTORY_H


#include "BeamTemplate.h"

class BrickFactory {

public:
    BrickFactory();


    const vector<BeamTemplate> &getAllBeams() const;

    const vector<Connector> getAllConnectors() const;

    const vector<Connector> &getAllStraightConnectors() const;

    const vector<Connector> &getAllNonStraightConnectors() const;


private:
    void initAllBeams();

    void initStraightConnectors();

    void initNonStraightConnectors();

private:

    vector<BeamTemplate> beam_list;
    vector<Connector> straight_conns;
    vector<Connector> nonStraight_conns;

};


#endif //LEGOGENERATION_MAIN1_BEAMFACTORY_H
