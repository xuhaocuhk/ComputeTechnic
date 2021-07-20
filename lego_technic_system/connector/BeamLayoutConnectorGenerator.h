//
// Created by 徐豪 on 2018-10-15.
//

#ifndef LEGO_TECHNIC_MAIN_BEAMLAYOUTCONNECTORGENERATOR_H
#define LEGO_TECHNIC_MAIN_BEAMLAYOUTCONNECTORGENERATOR_H


#include "core/ConstructionGraph.h"
#include "core/UnitEdgeGroup.h"

class ConstructionGraph;

class BeamLayoutConnectorGenerator {

public:

    BeamLayoutConnectorGenerator(ConstructionGraph*);

    void genStraightConnectors(); //generate "straight" connectors within a component

    vector<UnitEdgeGroup*> genConnectorsCoverUnCoveredEdges(); // return number of unconnected groups

    ////find all possible connectors that bridge two given set
    vector<Connector> findAllPossibleConnector(set<BeamHole*>& comp_hole_1, set<BeamHole*>& comp_hole_2); // find all possible connectors between two beamholes groups

    ////find all possible connectors that covers the set
    vector<Connector> findAllPossibleConnector(set<BeamHole*>& beamhole_set);

    ////Collect all uncovered unit edges
    static set<UnitEdge*> getAllUnCoveredEdges();

    ////Group all uncovered unit edges
    vector<UnitEdgeGroup*> groupAllUnCoveredEdges(set<UnitEdge*> & ucEdges);

    void eraseBeam(BeamInstance*);
    void addBeam(BeamInstance*);

    void updateStatus(OperationBeams&);

    int getCollisionNum();


public:
    ConstructionGraph* constructionGraph = NULL;

    set< BeamHole*  > beamholes; // all beamhole from beams
    map<LayerHole*, vector<BeamHole*> >  beamholesOnLayerHole;

    map< BeamHole*, set<BeamHole*> > collisionHoles;

    vector< UnitEdgeGroup* > uncoverd_uedges_groups;
    map< UnitEdge*, UnitEdgeGroup* > belongedGroup; //UnitEdgeGroup of the given unitedge belongs


    set< Connector* > connectors; // connectors on current component
    set< Connector* > straight_conns; // straight connectors
    map< BeamHole*,  vector<Connector*> > connectorsOnHole;

private:
    void initBeamHoles();

    bool allPinsAvailable(Connector& connector, set<BeamHole*>&, set<BeamHole*>& );
    bool allPinsAvailable(Connector& connector, set<BeamHole*>& );
    bool noCollision_non_pin_components(Connector& connector, set<BeamHole*>&, set<BeamHole*>& );//返回当前的Connector的非pin的部分是不是与其他现有的部分有collision
    bool noCollision_non_pin_components(Connector& connector, set<BeamHole*>& );//返回当前的Connector的非pin的部分是不是与其他现有的部分有collision
    bool bridgeTwoComponents(Connector &connector, set<BeamHole *> &current_holes, set<BeamHole *> &adj_holes); // whether the connector bridges two adjacent components


};


#endif //LEGO_TECHNIC_MAIN_BEAMLAYOUTCONNECTORGENERATOR_H
